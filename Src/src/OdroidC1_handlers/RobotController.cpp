
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
void RobotController::Forward(double thrust_percent, double angle)
{
	//const int minimumCommonThrust 	= 50;   	// similar to initialUserFwdThrust_percent in mainApp
	const int 		minThrust 			= 20;//30;
	const double 	minAngleToReact 	= 0.01;//0.03;
	const double	l_ref 				= MAX_HW_SPEED;   //10.0;
	const int 		maxDeltaThrust		= 95;//50 //MAX_HW_SPEED*50/800 ~16%?
	double			thrustR, thrustL;

	if ((angle>0) && (angle <  minAngleToReact))
			angle = 0;
	if ((angle<0) && (angle > -minAngleToReact))
			angle = 0;

	if ((thrust_percent>3) && (thrust_percent<minThrust))	//elevate the values
		thrust_percent=minThrust;
	if (thrust_percent>100)
		thrust_percent=100;
	double			Tcommon			= thrust_percent ;///* (1 - turn_ratio) ;
	double 			tmp4dbg 		= (angle) ;///* turn_ratio ;				// it 'shouldn't' be above 1
	double			delta_thrust	= l_ref * tmp4dbg ; ///* (angle) * turn_ratio ;    // tan() also option.//sin()

	int 	min_TotalThrust 		= 5./100. * MAX_HW_SPEED; //[%] to fraction

	if ((delta_thrust>0) && (delta_thrust>maxDeltaThrust))
		delta_thrust = maxDeltaThrust;
	if ((delta_thrust<0) && (delta_thrust<-maxDeltaThrust))
		delta_thrust = -maxDeltaThrust;

///	if ((Tcommon> (minimumCommonThrust * 0.3) ) && (Tcommon < minimumCommonThrust))	//elevate the values
	///	Tcommon = minimumCommonThrust ;
	Tcommon *= 1/100.0 * MAX_HW_SPEED;  //[%] to [fraction]

	if (delta_thrust > 0)
	{
		if (delta_thrust + Tcommon < min_TotalThrust)
			{ Stop(); return; }
		if (delta_thrust + Tcommon > MAX_HW_SPEED)
			Tcommon = MAX_HW_SPEED - delta_thrust ;
		thrustL = Tcommon + delta_thrust;
		thrustR = Tcommon - delta_thrust;
	}
	else
	{
		if (-delta_thrust + Tcommon < min_TotalThrust)
			{ Stop(); return; }
		if (-delta_thrust + Tcommon > MAX_HW_SPEED)
			Tcommon = MAX_HW_SPEED + delta_thrust ;
		thrustL = Tcommon + delta_thrust;
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