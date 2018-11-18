#include "State.h"

class DriveState : public State {
public:

  double left_pos, right_pos, left_vel, right_vel, yaw;

  DriveState();

  void UpdateLeftPos(double pos);
  void UpdateRightPos(double pos);
  void UpdateLeftVel(double vel);
  void UpdateRightVel(double vel);
  void UpdateYaw(double yaw);

  double GetLeftPos();
  double GetRightPos();
  double GetLeftVel();
  double GetRightVel();
  double GetYaw();

};
