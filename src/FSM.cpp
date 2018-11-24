#include "FSM.h"
#include <math.h>

FSM::FSM(States state, Lanes lane) {
  current_state_ = state;
  current_lane_ = lane;
}

FSM::~FSM() {

}


int FSM::CheckSafety(Lanes lane, int &id) {
  id = 1;
  for (int i = 0; i < other_cars_.size(); ++i) {
    if (IsInLane(i, lane)) {
      if (IsClose(i)) {
        id = i;
        return 1;
      }
      // else if (IsSpeedMaintainable(i)) {
      //   return 2;
      // }
    }
  }
  return 0;
}


bool FSM::IsClose(int id) {
  double s = other_cars_[id][5];
  double vx = other_cars_[id][3];
  double vy = other_cars_[id][4];
  double speed = sqrt(vx*vx + vy*vy);

  s += (/*prev_size_ **/ timestep * speed);
  return (((s > car_[0]) && (s - car_[0]) < 30.0) /*|| ((s <= car_[0]) && (car_[0] - s) < 40.0)*/);
}

bool FSM::IsSpeedMaintainable(int id) {
  float s = other_cars_[id][5];
  float vx = other_cars_[id][3];
  float vy = other_cars_[id][4];
  float speed = sqrt(vx*vx + vy*vy);

  s += (prev_size_ * 0.02 * speed);
  return ((s > car_[0]) && (s - car_[0] >= 30) && (s - car_[0]) < 35);
}




States FSM::GetNextState(std::vector<std::vector<double>> sensor_fusion, double s, double d, int prev_size) {
  other_cars_ = sensor_fusion;
  car_.clear();
  car_.push_back(s);
  car_.push_back(d);

  int ahead_id = 0;
  if (CheckSafety(current_lane_, ahead_id) == 0)
    return KEEP_LANE;
  else {
    if (current_lane_ == MIDDLE) {
      if (CheckSafety(LEFT, ahead_id) == 0) {
        return SHIFT_LEFT;
      }
      else if (CheckSafety(RIGHT, ahead_id) == 0) {
        return SHIFT_RIGHT;
      }
    }

    else if (current_lane_ == RIGHT) {
      if (CheckSafety(MIDDLE, ahead_id) == 0) {
        return SHIFT_LEFT;
      }
    }

    else if (current_lane_ == LEFT) {
      if (CheckSafety(MIDDLE, ahead_id) == 0) {
        return SHIFT_RIGHT;
      }
    }
  }
  // float vx = other_cars_[ahead_id][3];
  // float vy = other_cars_[ahead_id][4];
  // float v = sqrt(vx*vx + vy*vy);
  // float max_velocity_in_front = (other_cars_[ahead_id][5] - car_[0]) + v - 0.5 * (car_[3]);
  // float max_velocity_accel_limit = max_acceleration/timestep + car_[2];
  // new_velocity_ = std::min(std::min(max_velocity_in_front, max_velocity_accel_limit), target_speed);
  return TOO_CLOSE;
 
  // States state;
  // switch (current_state_) {
  //   case KEEP_LANE:
  //   {
  //     int ret = CheckSafety(current_lane_);
  //     if (ret == 1) {
  //       if (current_lane_ == MIDDLE) {
  //         if (CheckSafety(LEFT) == 0) {
  //           state = SHIFT_LEFT;
  //         }
  //         else if (CheckSafety(RIGHT) == 0) {
  //           state = SHIFT_RIGHT;
  //         }
  //         else {
  //           state == TOO_CLOSE;
  //         }
  //       }
  //       else if (current_lane_ == RIGHT) {
  //         if (CheckSafety(MIDDLE) == 0) {
  //           state = SHIFT_LEFT;
  //         }
  //         else {
  //           state = TOO_CLOSE;
  //         }
  //       }

  //       else if (current_lane_ == LEFT) {
  //         if (CheckSafety(MIDDLE) == 0) {
  //           state = SHIFT_RIGHT;
  //         }
  //         else {
  //           state = TOO_CLOSE;
  //         }
  //       }
  //     }
  //     else if (ret == 2) {
  //       if (current_lane_ == MIDDLE) {
  //         if (CheckSafety(LEFT) == 0) {
  //           state = SHIFT_LEFT;
  //         }
  //         else if (CheckSafety(RIGHT) == 0) {
  //           state = SHIFT_RIGHT;
  //         }
  //         else {
  //           state == MAINTAIN_SPEED;
  //         }
  //       }
  //       else if (current_lane_ == RIGHT) {
  //         if (CheckSafety(MIDDLE) == 0) {
  //           state = SHIFT_LEFT;
  //         }
  //         else {
  //           state = MAINTAIN_SPEED;
  //         }
  //       }

  //       else if (current_lane_ == LEFT) {
  //         if (CheckSafety(MIDDLE) == 0) {
  //           state = SHIFT_RIGHT;
  //         }
  //         else {
  //           state = MAINTAIN_SPEED;
  //         }
  //       }
  //     }
  //     else {
  //       state = KEEP_LANE;
  //     }
  //     break;
  //   }

  //   case SHIFT_LEFT:
  //     state = MAINTAIN_SPEED;
  //     break;

  //   case SHIFT_RIGHT:
  //     state = MAINTAIN_SPEED;
  //     break;

  //   case TOO_CLOSE:
  //     {
  //       int ret = CheckSafety(current_lane_);
  //       if (ret == 0) {
  //         state = KEEP_LANE;
  //       }
  //       else if (ret == 2) {
  //         state = MAINTAIN_SPEED;
  //       }
  //       else {
  //         state = TOO_CLOSE;
  //       }
  //     }
  //     break;

  //   case MAINTAIN_SPEED:
  //     {
  //       int ret = CheckSafety(current_lane_);
  //       if (ret == 0) {
  //         state = KEEP_LANE;
  //       }
  //       else if (ret == 2) {
  //         if (current_lane_ == MIDDLE) {
  //           if (CheckSafety(LEFT) == 0) {
  //             state = SHIFT_LEFT;
  //           }
  //           else if (CheckSafety(RIGHT) == 0) {
  //             state = SHIFT_RIGHT;
  //           }
  //           else {
  //             state == MAINTAIN_SPEED;
  //           }
  //         }
  //         else if (current_lane_ == RIGHT) {
  //           if (CheckSafety(MIDDLE) == 0) {
  //             state = SHIFT_LEFT;
  //           }
  //           else {
  //             state = MAINTAIN_SPEED;
  //           }
  //         }

  //         else if (current_lane_ == LEFT) {
  //           if (CheckSafety(MIDDLE) == 0) {
  //             state = SHIFT_RIGHT;
  //           }
  //           else {
  //             state = MAINTAIN_SPEED;
  //           }
  //         }
  //       }
  //       else {
  //         state = TOO_CLOSE;
  //       }
  //     }
  //     break;
  // }
  // return state;
}


bool FSM::IsInLane(int id, Lanes lane) {
  float d = other_cars_[id][6];
  return ((d > LANE_WIDTH*lane) && (d < LANE_WIDTH*(lane + 1)));
}

std::vector<States> FSM::SuccessorStates() {
  std::vector<States> states;
  switch (current_state_) {
    case KEEP_LANE:
      states.push_back(KEEP_LANE);
      if (current_lane_ == MIDDLE) {
        states.push_back(SHIFT_LEFT);
        states.push_back(SHIFT_RIGHT);
      }
      else if (current_lane_ == LEFT) {
        states.push_back(SHIFT_RIGHT);
      }
      else {
        states.push_back(SHIFT_LEFT);
      }
      break;

    case SHIFT_LEFT:
    case SHIFT_RIGHT:
      states.push_back(KEEP_LANE);
      break;
  }
}

/*
States FSM::GetNextState(std::vector<std::vector<double>> sensor_fusion, double s, double d, int prev_size) {
  // Get all successor states
  std::vector<States> states = SuccessorStates();

  for (int i = 0; i < states.size(); ++i) {
    // Get the trajectory of each state
    if (states[i] == KEEP_LANE) {
      // Generate keep lane trajectory
    }
    else if (states[i] == SHIFT_LEFT) {
      // Generate shift left trajectory
    }

    else if (states[i] == SHIFT_RIGHT) {
      // Generate shift right trajectory
    }

    // Calculate cost of each state
  }

  // return the state with lowest cost
}
*/

std::vector<float> FSM::GetKinematics(Lanes lane) {
  float max_velocity_accel_limit = max_acceleration/timestep + car_[2];
  float new_position;
  float new_velocity;
  float new_accel;

  int ahead_id = 1;
  int behind_id = -1;
  if (GetVehicleAhead(lane, ahead_id)) {
    float vx = other_cars_[ahead_id][3];
    float vy = other_cars_[ahead_id][4];
    float v = sqrt(vx*vx + vy*vy);
    if (GetVehicleBehind(lane, behind_id)) {       
      new_velocity = v; //must travel at the speed of traffic, regardless of preferred buffer
    } 
    else {
      float max_velocity_in_front = (other_cars_[ahead_id][5] - car_[0] - 0.5) + v - 0.5 * (car_[3]);
      new_velocity = std::min(std::min(max_velocity_in_front, max_velocity_accel_limit), target_speed);
    }
  } 
  else {
    new_velocity = std::min(max_velocity_accel_limit, target_speed);
  }
    
  new_accel = (new_velocity - car_[2])/timestep; //Equation: (v_1 - v_0)/t = acceleration
  new_position = car_[0] + new_velocity*timestep + new_accel*timestep*timestep / 2.0;
  return {new_position, new_velocity, new_accel};
}


bool FSM::GetVehicleAhead(Lanes lane, int &id) {
  int min_s = max_s;
  bool found_vehicle = false;
  id = -1;
  for (int i = 0; i < other_cars_.size(); ++i) {
    float s = other_cars_[id][5];
    float vx = other_cars_[id][3];
    float vy = other_cars_[id][4];
    float speed = sqrt(vx*vx + vy*vy);

    s += (prev_size_ * timestep * speed);
    if (IsInLane(i, lane) && (s > car_[0] && s < min_s /*&& (s - car_[0]) < 30*/)) {
      min_s = s;
      id = i;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}


bool FSM::GetVehicleBehind(Lanes lane, int &id) {
  int maxs = -1;
  bool found_vehicle = false;
  id = -1;
  for (int i = 0; i < other_cars_.size(); ++i) {
    float s = other_cars_[id][5];
    float vx = other_cars_[id][3];
    float vy = other_cars_[id][4];
    float speed = sqrt(vx*vx + vy*vy);

    s += (prev_size_ * timestep * speed);
    if (IsInLane(i, lane) && (car_[0] > s && s > maxs /*&& (car_[0] - s) < 30*/)) {
      maxs = s;
      id = i;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}