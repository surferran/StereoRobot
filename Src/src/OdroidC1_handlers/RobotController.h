
#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include "pwm.h"
#ifdef COMPILING_ON_ROBOT
#include <unistd.h>
#endif

#define FREQ			1000
#define MAX_HW_SPEED	800

//#define PORT_PIN_1 13  // for the wiringPi library
//#define PORT_PIN_2 14  // -"-


class RobotController
{
private:
  PWM::pwm_ptr PWM_pointeer ;
  //int speed;
  void basicMove(const int leftSpeed,const int rightSpeed);

public:
  //enum TurningAngle{REGULAR,SHARP};
  enum WheelSide{LEFT_WHEELS,RIGHT_WHEELS};

  RobotController();
  ~RobotController();

  void SetSpeed(int newSpeed);

  void Forward(double thrust_percent, double angle, double turn_ratio);
  // void Reverse();	//not supported by Hardware.
  void Stop();

};

#endif // ROBOTCONTROLLER_H
