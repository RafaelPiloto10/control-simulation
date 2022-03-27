class PIDController {
public:
  double kP;
  double kI;
  double kD;

  double goal;
  double lastError;
  double totalError;

  double kMaxGoal;
  double kMinGoal;

  const double kDt = 0.01;

  void SetGoal(double goal);
  double Compute(double current);
};
