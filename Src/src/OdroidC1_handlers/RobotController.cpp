
#ifdef COMPILING_ON_ROBOT

#include "RobotController.h"


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
	if ((thrust_percent>10) && (thrust_percent<50))	//elevate the values
		thrust_percent=50;
	if (thrust_percent>100)
		thrust_percent=100;
	double			Tcommon			= thrust_percent/100. * (1 - turn_ratio) ;
	double			delta_thrust	= l_ref * (angle) * turn_ratio ;    // tan() also option.//sin()
	double			thrustR, thrustL;
	int min_thrust = 5 * MAX_HW_SPEED; //[%]

	if ((Tcommon>10) && (Tcommon<50))	//elevate the values
		Tcommon=50;
	Tcommon=MAX_HW_SPEED * Tcommon;

	if (delta_thrust > 0)
	{
		if (delta_thrust + Tcommon < min_thrust)
			{ Stop(); return; }
		thrustL = Tcommon + delta_thrust;
		thrustR = Tcommon ;
	}
	else
	{
		if (-delta_thrust + Tcommon < min_thrust)
			{ Stop(); return; }
		thrustL = Tcommon ;
		thrustR = Tcommon - delta_thrust;	
	}
	basicMove(thrustL, thrustR);

	std::this_thread::sleep_for(std::chrono::milliseconds( 10 ) ); //*action_dt//100 delay
	//Stop();
}
 
void RobotController::Stop(){
  PWM_pointeer->enable(RobotController::RIGHT_WHEELS,false);
  PWM_pointeer->enable(RobotController::LEFT_WHEELS	,false);
}

RobotController::~RobotController(){
    Stop();
}

#endif