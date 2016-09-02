
#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include "pwm.h"
#include <unistd.h>

#define FREQ 1000

#define PORT_PIN_1 13  // for the wiringPi library
#define PORT_PIN_2 14  // -"-


class RobotController
{
  PWM::pwm_ptr p;
  int speed;
  void doMove(const int leftSpeed,const int rightSpeed);
public:
  enum TurningAngle{REGULAR,SHARP};
  enum WheelSide{LEFT_WHEELS,RIGHT_WHEELS};

  RobotController(int speed);
  ~RobotController();

  void SetSpeed(int newSpeed);
  void Forward();
  void Reverse();
  void Stop();
  void Right(TurningAngle value);
  void Left(TurningAngle value);

};

#endif // ROBOTCONTROLLER_H
