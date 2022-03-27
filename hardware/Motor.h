namespace Hardware {
class Motor {
public:
  double V = 12;  // Voltage (Volts) -- constant 12 volts
  double R = 0.0; // Resistance (Ohms)

  double kT = 0.0;
  double kV = 0.0;

  double G = 0.0; // Gear ratio (Nm)
  double M = 0.0; // Load mass (kg) float r; // Radius of axel (m)
  double r = 0.0; // The ratio between the motor axel and the load axel (pulley,
                  // radius, etc) (m)

  // Using:
  // V = IR + w_m/kV (w_m = angular velocity of motor)
  // G * T_m = T_l (T_m = torque on motor, T_l = torque on load)
  // T_m/kT = I
  // F_l = T_l / r
  // w_m / G = w_l (w_m = ang. vel. of motor, w_l = ang. vel. of the load)
  // w_l * r = v_l (v_l = velocity of load)

  // Then:
  // V = T_m*R/kT + w_m/kV
  // V = (T_l*R)/(kT * G) + w_m/kV
  // V = (F_l*r*R)/(kT * G) + w_m/kV
  // V = (F_l*r*R)/(kT*G) + (v_m*G)/(r*kV) -- where v_m is vel of motor
  // V = (m_l*A_l*r*R)/(kT*G) + (v_m*G)/(r*kV) -- where A_l is acc of load
  // A_l = (G*kT)/(R*r*m_l)*(V-(G*v_l)/(r*kV))

  Motor(double numMotors, double stallCurrent, double stallTorque,
        double freeSpeed, double freeCurrent, double r, double gearRatio,
        double loadMass);

  // Get the load acceleration given voltage and the velocity of the load
  double GetAcceleration(double voltage, double velocity);
};
} // namespace Hardware
