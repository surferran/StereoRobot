
#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include "pwm.h"
#ifdef COMPILING_ON_ROBOT
#include <unistd.h>
#endif

#include <thread>
#include <mutex>
#include <atomic>

using namespace std;

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
  ///enum WheelSide{LEFT_WHEELS,RIGHT_WHEELS};
  enum WheelSide{RIGHT_WHEELS, LEFT_WHEELS};

#ifdef COMPILING_ON_ROBOT
  RobotController();
  ~RobotController();
  
  void Forward(double thrust_percent, double angle);
  // void Reverse();	//not supported by Hardware.
  void Stop();
#else

  RobotController(){};
  ~RobotController(){};

  void Forward(double thrust_percent, double angle){};
  // void Reverse();	//not supported by Hardware.
  void Stop(){};
#endif



};

#endif // ROBOTCONTROLLER_H
