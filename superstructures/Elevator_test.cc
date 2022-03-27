#include "Elevator.h"
#include <gtest/gtest.h>
#include <iostream>

using Superstructure::Elevator;

double current_time = 0.0;
double offset = -0.1;

double encoder(Elevator *elevator) { return elevator->position + offset; }

void SimulateTime(Elevator *elevator, double voltage, double time) {
  const double max_voltage = elevator->state == Elevator::State::RUNNING
                                 ? elevator->kMaxVoltage
                                 : elevator->kMaxZeroingVoltage;
  EXPECT_GE(voltage, -max_voltage);
  EXPECT_LE(voltage, max_voltage);
  const double kSimTime = 0.001;
  const double starting_time = time;
  // Euler integration
  while (time > 0) {
    const double current_dt = std::min(time, kSimTime);
    elevator->acceleration =
        elevator->motor->GetAcceleration(voltage, elevator->velocity);
    elevator->position += elevator->velocity * current_dt;
    elevator->velocity += elevator->acceleration * current_dt;
    time -= kSimTime;

    // Hardcode hardstop which is 1 cm away from softstop
    EXPECT_LE(elevator->position, elevator->kMaxHeight + 0.01);
    EXPECT_GE(elevator->position, elevator->kMinHeight - 0.01);
  }

  current_time += starting_time;
}

TEST(ElevatorLoop, Zeros) {
  Elevator *elevator = new Elevator();

  elevator->position = 0.1;

  double target = 0.3;
  EXPECT_EQ(elevator->motor->V, 12);

  double total_time = 500; // seconds
  FILE *fd = fopen("/tmp/dump", "w");
  fprintf(fd, "# time, position, voltage, velocity \n");

  for (int i = 0; i < total_time; i++) {
    double voltage = elevator->Update(encoder(elevator), target);
    SimulateTime(elevator, voltage, elevator->kDt);

    fprintf(fd, "%f, %f, %f, %f\n", current_time, elevator->position, voltage,
            elevator->velocity);
  }

  fclose(fd);

  EXPECT_EQ(elevator->state, Elevator::State::RUNNING);
  EXPECT_NEAR(elevator->position, target, 0.01);
}

TEST(ElevatorLoop, TooHigh) {
  Elevator *elevator = new Elevator();

  elevator->position = 0.1;

  double target = 10.0;
  EXPECT_EQ(elevator->motor->V, 12);

  double total_time = 500; // seconds
  FILE *fd = fopen("/tmp/dump", "w");
  fprintf(fd, "# time, position, voltage, velocity \n");

  for (int i = 0; i < total_time; i++) {
    double voltage = elevator->Update(encoder(elevator), target);
    SimulateTime(elevator, voltage, elevator->kDt);

    fprintf(fd, "%f, %f, %f, %f\n", current_time, elevator->position, voltage,
            elevator->velocity);
  }

  fclose(fd);

  EXPECT_EQ(elevator->state, Elevator::State::RUNNING);
  EXPECT_NEAR(elevator->position, elevator->kMaxHeight, 0.01);
}

TEST(ElevatorLoop, TooLow) {
  Elevator *elevator = new Elevator();

  elevator->position = 0.1;

  double target = -10.0;
  EXPECT_EQ(elevator->motor->V, 12);

  double total_time = 500; // seconds
  FILE *fd = fopen("/tmp/dump", "w");
  fprintf(fd, "# time, position, voltage, velocity \n");

  for (int i = 0; i < total_time; i++) {
    double voltage = elevator->Update(encoder(elevator), target);
    SimulateTime(elevator, voltage, elevator->kDt);

    fprintf(fd, "%f, %f, %f, %f\n", current_time, elevator->position, voltage,
            elevator->velocity);
  }

  fclose(fd);

  EXPECT_EQ(elevator->state, Elevator::State::RUNNING);
  EXPECT_NEAR(elevator->position, elevator->kMinHeight, 0.01);
}
