#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <string>
#include <limits>

#define TIME_STEP 64
#define MAX_SPEED 6.28

using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();

  // --- Motors ---
  Motor *left_Motor = robot->getMotor("left wheel motor");
  Motor *right_Motor = robot->getMotor("right wheel motor");

  left_Motor->setPosition(INFINITY);
  right_Motor->setPosition(INFINITY);

  left_Motor->setVelocity(0.0);
  right_Motor->setVelocity(0.0);

  // --- Distance sensors ---
  DistanceSensor *prox_sensors[8];
  for (int ind = 0; ind < 8; ++ind) {
    std::string ps_name = "ps" + std::to_string(ind);
    prox_sensors[ind] = robot->getDistanceSensor(ps_name);
    prox_sensors[ind]->enable(TIME_STEP);
  }

  double left_speed = MAX_SPEED;
  double right_speed = MAX_SPEED;

  while (robot->step(TIME_STEP) != -1) {
    bool left_wall = prox_sensors[5]->getValue() > 80.0;
    bool left_conner = prox_sensors[6]->getValue() > 80.0;
    bool front_wall = prox_sensors[7]->getValue() > 80.0;

    if (front_wall) {
      // turn right
      left_speed = MAX_SPEED;
      right_speed = -MAX_SPEED;
    } else {
      if (left_wall) {
        // go straight
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED;
      } else {
        // turn left slightly
        left_speed = MAX_SPEED / 8.0;
        right_speed = MAX_SPEED;
      }
      
      if(left_conner)
      {
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED/8;
      
      }
    }

    left_Motor->setVelocity(left_speed);
    right_Motor->setVelocity(right_speed);
  }

  delete robot;
  return 0;
}
