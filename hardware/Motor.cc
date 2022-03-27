#include "Motor.h"
#include <math.h>

namespace Hardware {
Motor::Motor(double numMotors, double stallCurrent, double stallTorque,
             double freeSpeed, double freeCurrent, double r, double gearRatio,
             double loadMass) {
  R = V / stallCurrent; // R = V / I -- Max volts & Max current = R
  this->r = r;
  G = gearRatio;
  M = loadMass;

  kT = (numMotors * stallTorque) / stallCurrent;
  // V = IR + w_m/kV (w_m = angular velocity of motor)
  kV = ((freeSpeed / 60.0 * 2.0 * M_PI) / (V - R * freeCurrent));
}

double Motor::GetAcceleration(double voltage, double velocity) {
  return -kT * G * G / (kV * R * r * r * M) * velocity +
         G * kT / (R * r * M) * voltage;
}
} // namespace Hardware
