
#include "RobotController.h"

//#include "/root/github/wiringPi/wiringPi/wiringPi.h" //RAN addition 27/12/15
//#include "wiringPi.h"

RobotController::RobotController(int speed):speed(speed){
  p=PWM::create();

  //pinMode(PORT_PIN_1, OUTPUT); compilation problem

  //static const xstring gpio_path="/sys/class/gpio/export";
  //std::ofstream f(gpio_path);
  //f << value << std::endl;
  //gpio_path ;//<< std::endl;

  //pin 21 is gpio.bit 9 as 106
  //pin 23 is gpio.bit 8 as 105
  //system("sudo -c ""echo 106 > /sys/class/gpio/export"" ");
  //system("sudo -c ""echo out > /sys/class/gpio/gpio106/direction"" ");
  //system("sudo -c ""echo 1  > /sys/class/gpio/gpio106/value"" ");
  system("echo 106 > /sys/class/gpio/export");
  system("chmod a+x /sys/class/gpio");
  system("chmod -R a+rw /sys/class/gpio");
  system("chmod a+rw /sys/class/gpio/gpio106/value");
  system("echo out > /sys/class/gpio/gpio106/direction");
  system("echo 1  > /sys/class/gpio/gpio106/value");


}
void RobotController::Forward(){
  doMove(speed,speed);

}
void RobotController::Reverse(){

	//doMove(speed,speed);

}
void RobotController::Stop(){
  p->enable(RobotController::RIGHT_WHEELS,false);
  p->enable(RobotController::LEFT_WHEELS,false);
}

RobotController::~RobotController(){
    Stop();
}
void RobotController::Right(TurningAngle angle){

  
  int leftSpeed  = speed - speed;
  if (angle == RobotController::REGULAR)
    leftSpeed = (int)(speed*0.25);
  
  doMove(leftSpeed,speed);

  
}
void RobotController::Left(TurningAngle angle){
  int rightSpeed  = speed - speed;
  if (angle == RobotController::REGULAR)
    rightSpeed = (int)(speed*0.25);
    
  doMove(speed,rightSpeed);
}

void RobotController::doMove(const int leftSpeed,const int rightSpeed){
  p->enable(RobotController::LEFT_WHEELS,true);
  p->enable(RobotController::RIGHT_WHEELS,true);

  p->set_freq(RobotController::LEFT_WHEELS,FREQ);
  p->set_freq(RobotController::RIGHT_WHEELS,FREQ);

    
  p->set_duty_cycle(RobotController::LEFT_WHEELS,leftSpeed);
  p->set_duty_cycle(RobotController::RIGHT_WHEELS,rightSpeed);

}

void RobotController::SetSpeed(int newSpeed){
	speed = newSpeed;
}

