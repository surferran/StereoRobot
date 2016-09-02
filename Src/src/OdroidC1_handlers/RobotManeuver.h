/*
 * RobotManeuver.h
 *
 *  Created on: Aug 4, 2015
 *      Author: root
 */

#ifndef ROBOTMANEUVER_H_
#define ROBOTMANEUVER_H_

#include <iostream>
#include <functional>
#include <chrono>
#include <future>
#include <cstdio>
#include "RobotController.h"
using namespace std;


class RobotManeuver
{
	RobotController robot;
	int speed ;
	typedef void (RobotManeuver::*TurnFunc)();
	void Maneuver(TurnFunc firstFunc,TurnFunc secondFunc);
	void Maneuver2(TurnFunc firstFunc,TurnFunc secondFunc);

public:
	enum DirectionEnum{
		RIGHT,
		LEFT
	};
	RobotManeuver(int speed=400);
	void Forward();
	void Reverse();
	void Stop();
	void Right();
	void Left();
	void SmoothRight();
	void SmoothLeft();
	void Turn90Deg(DirectionEnum dir);
	void ForwardStep(double factor = 1);
	void ManeuverRight();
	void ManeuverLeft();


};



#endif /* ROBOTMANEUVER_H_ */
