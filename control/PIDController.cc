#include "PIDController.h"
#include <algorithm>

void PIDController::SetGoal(double goal_) {
  goal = std::min(kMaxGoal, std::max(kMinGoal, goal_));
}

double PIDController::Compute(double current) {
  double error = goal - current;
  totalError += error;

  double vel = (error - lastError) / kDt;

  double out = (kP * error) + (kI * totalError) + (kD * vel);

  lastError = error;

  return out;
}
