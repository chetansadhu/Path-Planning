#ifndef _FSM_H
#define _FSM_H_

#include <vector>


#define LANE_WIDTH 4

constexpr float max_s = 6945.554;
constexpr float target_speed = 49.5f;
constexpr float timestep = 0.02f;

// enumeration for Lanes
enum Lanes {
  LEFT,
  MIDDLE,
  RIGHT
};

// enumeration for states
enum States {
  KEEP_LANE,
  SHIFT_RIGHT,
  SHIFT_LEFT,
  TOO_CLOSE
};

class FSM {
  public:    
    FSM(States state, Lanes lane);
    ~FSM();

    States GetNextState(std::vector<std::vector<double>> sensor_fusion, double s, double d, double v);
    double GetVelocity() const { return new_velocity_; }

  private:
    int CheckSafety(Lanes lane, int &id, bool isLaneChange = false);
    bool IsInLane(int vehicle_id, Lanes lane);
    bool IsClose(int vehicle_id, bool isLaneChange);
    bool GetVehicleAhead(Lanes lane, int &vehicle_id, float &vehicle_s);

  private:
    States current_state_;
    Lanes current_lane_;
    std::vector<std::vector<double> > other_cars_;
    std::vector<double> car_;
    double new_velocity_;
};

#endif