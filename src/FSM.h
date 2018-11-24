#ifndef _FSM_H
#define _FSM_H_

#include <vector>


#define LANE_WIDTH 4

constexpr double max_s = 6945.554;
constexpr float target_speed = 49.5f;
constexpr float timestep = 0.02f;
constexpr float max_acceleration = 2.0f;

enum Lanes {
  LEFT,
  MIDDLE,
  RIGHT
};

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

    States GetNextState(std::vector<std::vector<double>> sensor_fusion, double s, double d, int prev_size);
    double GetVelocity() const { return new_velocity_; }

  private:
    int CheckSafety(Lanes lane, int &id);
    bool IsInLane(int vehicle_id, Lanes lane);
    bool IsClose(int vehicle_id);
    bool IsSpeedMaintainable(int vehicle_id);
    std::vector<States> SuccessorStates();
    std::vector<float> GetKinematics(Lanes lane);
    bool GetVehicleAhead(Lanes lane, int &vehicle_id);
    bool GetVehicleBehind(Lanes lane, int &vehicle_id);

  private:
    States current_state_;
    Lanes current_lane_;
    std::vector<std::vector<double> > other_cars_;
    std::vector<double> car_;
    int prev_size_;
    double new_velocity_;
};

#endif