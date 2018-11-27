#include "FSM.h"
#include <math.h>
#include <map>
#include <algorithm>

// Constructor. Initialize current state and current lane
FSM::FSM(States state, Lanes lane) {
  current_state_ = state;
  current_lane_ = lane;
}

FSM::~FSM() {

}


// Check Safety function. Returns 0 if no vehicle in the lane and returns 1 if the vehicle is in lane and is close.
int FSM::CheckSafety(Lanes lane, int &id, bool isLaneChange) {
  id = 1;
  for (int i = 0; i < other_cars_.size(); ++i) {
    if (IsInLane(i, lane)) {
      if (IsClose(i, isLaneChange)) {
        id = i;
        return 1;
      }
    }
  }
  return 0;
}

// IsClose function. Returns as close if car is in 30m range ahead. And in case of lane change this function also checks for the vehicle behind in the range of 20m
bool FSM::IsClose(int id, bool isLaneChange) {
  double s = other_cars_[id][5];
  double vx = other_cars_[id][3];
  double vy = other_cars_[id][4];
  double speed = sqrt(vx*vx + vy*vy);

  s += (timestep * speed);
  double car_s = car_[0];
  if (!isLaneChange)
    return (((s >= car_s) && (s - car_s) < 30.0));
  else 
    return (((s >= car_s) && (s - car_s) < 30.0) || ((s < car_s) && (car_s - s) < 20.0));
}


// GetNextState function based on the the sensor fusion and ego car stats.
States FSM::GetNextState(std::vector<std::vector<double>> sensor_fusion, double s, double d, double v) {
  other_cars_ = sensor_fusion;
  car_.clear();
  car_.push_back(s);
  car_.push_back(d);
  car_.push_back(v);

  int ahead_id = 0;
  int id = 0;
  // Priority is given for keep lane. If it is safe, keep lane is maintained
  if (CheckSafety(current_lane_, ahead_id) == 0)
    return KEEP_LANE;
  else {
    // condition for if MIDDLE is the current lane
    if (current_lane_ == MIDDLE) {
      int left_id, right_id;
      int ret_left = CheckSafety(LEFT, left_id, true);
      int ret_right = CheckSafety(RIGHT, right_id, true);
      if (ret_left == 0 && ret_right == 0) {
        float left_s, right_s;
        bool l = GetVehicleAhead(LEFT, left_id, left_s);
        bool r = GetVehicleAhead(RIGHT, right_id, right_s);
        if (l && r) {
          if (left_s < right_s) {
            return SHIFT_RIGHT;
          }
          else {
            return SHIFT_LEFT;
          }
        }
        if (l) {
          return SHIFT_RIGHT;
        }
        else {
          return SHIFT_LEFT;
        }
      }
      else if (ret_left == 0) {
        return SHIFT_LEFT;
      }
      else if (ret_right == 0) {
        return SHIFT_RIGHT;
      }
    }

    // condition if the current lane is RIGHT
    else if (current_lane_ == RIGHT) {
      if (CheckSafety(MIDDLE, id, true) == 0) {
        return SHIFT_LEFT;
      }
    }

    // condition if the current lane is LEFT
    else if (current_lane_ == LEFT) {
      if (CheckSafety(MIDDLE, id, true) == 0) {
        return SHIFT_RIGHT;
      }
    }
  }
  // calculate the speed of the vehicle ahead in case of TOO_CLOSE state
  float vx = other_cars_[ahead_id][3];
  float vy = other_cars_[ahead_id][4];
  float other_v = sqrt(vx*vx + vy*vy);
  new_velocity_ = other_v*2.24;
  return TOO_CLOSE;
}

// IsInLane function. Checks whether the vehicle with the given id is in lane or not.
bool FSM::IsInLane(int id, Lanes lane) {
  float d = other_cars_[id][6];
  return ((d > LANE_WIDTH*lane) && (d < LANE_WIDTH*(lane + 1)));
}


bool FSM::GetVehicleAhead(Lanes lane, int &id, float &min_s) {
  min_s = max_s;
  bool found_vehicle = false;
  id = -1;
  for (int i = 0; i < other_cars_.size(); ++i) {
    float s = other_cars_[i][5];
    float vx = other_cars_[i][3];
    float vy = other_cars_[i][4];
    float speed = sqrt(vx*vx + vy*vy);

    s += (timestep * speed);
    if (IsInLane(i, lane) && (car_[0] < s && s < min_s)) {
      min_s = s;
      id = i;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}