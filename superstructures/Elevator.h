#include "control/PIDController.h"
#include "hardware/Motor.h"

namespace Superstructure {
class Elevator {
public:
  Hardware::Motor *motor;
  PIDController *controller;

  enum class State { UNINITIALIZED, ZEROING, RUNNING };

  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  double offset = 0.0;
  // Control loop time step
  double kDt = 0.01;

  State state = State::UNINITIALIZED;

  Elevator();

  // target: Where are we trying to get to
  double Update(double encoder, double target);
  // Hall effect sensor at the bottom of the elevator
  bool hallEffect();

  double kMaxVoltage = 12.0;
  double kMaxZeroingVoltage = 4.0;
  double kZeroingVelocity = 0.05;
  double kMaxHeight = 0.45;
  double kMinHeight = -0.45;
};
} // namespace Superstructure
