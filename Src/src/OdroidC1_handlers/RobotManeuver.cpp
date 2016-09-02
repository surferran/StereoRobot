/*
 * RobotManeuver.cpp
 *
 *  Created on: Aug 8, 2015
 *      Author: root
 */
#include "RobotManeuver.h"

void RobotManeuver::Maneuver(TurnFunc firstFunc,TurnFunc secondFunc){
	(this->*firstFunc)();
	std::this_thread::sleep_for(std::chrono::milliseconds((int)(200000/speed)));
	robot.Forward();
    std::this_thread::sleep_for(std::chrono::milliseconds(380000/speed));
    (this->*secondFunc)();
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(400000/speed)));
    robot.Forward();
    std::this_thread::sleep_for(std::chrono::milliseconds(380000/speed));
	(this->*firstFunc)();
	std::this_thread::sleep_for(std::chrono::milliseconds((int)(200000/speed)));
    robot.Forward();
}

void RobotManeuver::Maneuver2(TurnFunc firstFunc,TurnFunc secondFunc){
	(this->*firstFunc)();
	std::this_thread::sleep_for(std::chrono::milliseconds((int)(250000/speed)));
	robot.Forward();
    std::this_thread::sleep_for(std::chrono::milliseconds(380000/speed));
    (this->*secondFunc)();
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(250000/speed)));
    robot.Forward();
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(119000/speed)));
    (this->*secondFunc)();
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(250000/speed)));
    robot.Forward();
    std::this_thread::sleep_for(std::chrono::milliseconds(380000/speed));
	(this->*firstFunc)();
	std::this_thread::sleep_for(std::chrono::milliseconds((int)(250000/speed)));
    robot.Forward();
}

RobotManeuver::RobotManeuver(int speed ):robot(RobotController(speed)),speed(speed) {
}

void RobotManeuver::Forward() {
	robot.Forward();
}

void RobotManeuver::Reverse() {
	robot.Reverse();
}

void RobotManeuver::Stop() {
	robot.Stop();
}

void RobotManeuver::Right() {
	robot.Right(RobotController::SHARP);
}

void RobotManeuver::Left() {
	robot.Left(RobotController::SHARP);
}

void RobotManeuver::SmoothRight() {
	robot.Right(RobotController::REGULAR);
}

void RobotManeuver::SmoothLeft() {
	robot.Left(RobotController::REGULAR);
}

void RobotManeuver::ManeuverRight() {
	Maneuver2((TurnFunc) &RobotManeuver::SmoothRight,(TurnFunc) &RobotManeuver::SmoothLeft);
}

void RobotManeuver::ManeuverLeft() {
	Maneuver2((TurnFunc) &RobotManeuver::SmoothLeft,(TurnFunc) &RobotManeuver::SmoothRight);
}

void RobotManeuver::Turn90Deg(DirectionEnum dir) {
	robot.Stop();
	if(dir == RIGHT)
	{
		robot.Right(RobotController::SHARP);
	}else{
		robot.Left(RobotController::SHARP);
	}
	std::this_thread::sleep_for(std::chrono::milliseconds((int)(400000/speed)));
	robot.Stop();
}

void RobotManeuver::ForwardStep(double factor) {
	robot.Forward();
	std::this_thread::sleep_for(std::chrono::milliseconds((int)(factor*350000/speed)));
	robot.Stop();
}

