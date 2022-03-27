#include "Elevator.h"
#include <algorithm>
#include <math.h>

namespace Superstructure {

double numMotors = 2.0;
double stallTorque = 2.402;
double stallCurrent = 126.145;
double freeSpeed = 5015.562;
double freeSpeedCurrent = 1.170;
double loadMass = 20.0;
double gearRatio = 72.0 / 12.0 * 22.0 / 16.0;
double r = 0.25 * 0.0254 * 22.0 / M_PI / 2.0;

Elevator::Elevator() {
  motor = new Hardware::Motor(numMotors, stallCurrent, stallTorque, freeSpeed,
                              freeSpeedCurrent, r, gearRatio, loadMass);
  controller = new PIDController();

  controller->kP = 100.0;
  controller->kD = 1.0;
  controller->kMaxGoal = kMaxHeight;
  controller->kMinGoal = kMinHeight;
}

// If the elevator position is below 0.05, then
bool Elevator::hallEffect() { return position > -0.01 && position < 0.0; }

double Elevator::Update(double encoder, double target) {

  switch (state) {
  case Elevator::State::UNINITIALIZED:
    state = Elevator::State::ZEROING;
    controller->SetGoal(encoder);
    break;
  case Elevator::State::ZEROING:
    controller->SetGoal(controller->goal - kDt * kZeroingVelocity);
    if (hallEffect()) {
      state = Elevator::State::RUNNING;
      offset = -encoder;
    }
    break;
  case Elevator::State::RUNNING:
    controller->SetGoal(target);
    break;
  }

  double v = controller->Compute(encoder + offset);
  double max_voltage =
      state == Elevator::State::RUNNING && (v < 0 && hallEffect())
          ? kMaxZeroingVoltage
          : kMaxVoltage;
  return std::min(max_voltage, std::max(v, -max_voltage));
}
} // namespace Superstructure
