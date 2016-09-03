
#include "RobotController.h"

//#include "/root/github/wiringPi/wiringPi/wiringPi.h" //RAN addition 27/12/15
//#include "wiringPi.h"

RobotController::RobotController(){
	PWM_pointeer = PWM::create();
	Stop();
}

void RobotController::basicMove(const int leftSpeed,const int rightSpeed){
	PWM_pointeer->enable(RobotController::LEFT_WHEELS ,true);
	PWM_pointeer->enable(RobotController::RIGHT_WHEELS,true);

	PWM_pointeer->set_freq(RobotController::LEFT_WHEELS ,FREQ);
	PWM_pointeer->set_freq(RobotController::RIGHT_WHEELS,FREQ);

	PWM_pointeer->set_duty_cycle(RobotController::LEFT_WHEELS ,leftSpeed);
	PWM_pointeer->set_duty_cycle(RobotController::RIGHT_WHEELS,rightSpeed);
}

// turn angle = positive angle is from straight forward to the right direction.
//				negative is to the left.
//	thrust_percent - 0~100
//	angle - [rad]
// turn_ratio - 0 (go straight with minor turn) to 1 (full turn, with only side-effect forward)
void RobotController::Forward(double thrust_percent, double angle, double turn_ratio)  // TODO: add turn_rate_ratio option
{
	const double	action_dt		= 0.05;//0.01 ; // [sec]
	///double			angle_turn_rate = angle / action_dt ;
	const double	l_ref = MAX_HW_SPEED;   //10.0;
	double			Tcommon			= MAX_HW_SPEED * thrust_percent * 1/turn_ratio ;
	double			delta_thrust	= l_ref * sin(angle) * turn_ratio ;    // tan() also option.
	double			thrustR, thrustL;
	if (delta_thrust > 0)
	{
		thrustL = Tcommon + delta_thrust;
		thrustR = Tcommon ;
	}
	else
	{
		thrustL = Tcommon ;
		thrustR = Tcommon - delta_thrust;	
	}
	basicMove(thrustL, thrustR);
}
 
void RobotController::Stop(){
  PWM_pointeer->enable(RobotController::RIGHT_WHEELS,false);
  PWM_pointeer->enable(RobotController::LEFT_WHEELS	,false);
}

RobotController::~RobotController(){
    Stop();
}
