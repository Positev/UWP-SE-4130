#ifndef SRC_CONTROLLERS_WHEELDRIVER_H_
#define SRC_CONTROLLERS_WHEELDRIVER_H_

#include "DCMotorController.h";
#include "RoteryEncoderMonitor.h";
#include "PIDDriver.h";

enum DriveType {POWER, VELOCITY, DISPLACEMENT};

/*
 * Track and report end stop state
 */
class WheelDriver
{

private:

    DCMotorController *motor;
    RoteryEncoderMonitor *encoder;
    PIDDRIVER pid;

    
    float currentVelocity; // feet per second
    float targetVelocity; // feet per second
    float wheelSize; // inches
    float direction; 



public:

	WheelDriver( DCMotorController *motor, RoteryEncoderMonitor *encoder );


    void setTargetVelocity();
    void spinForDisatance();
    void setDirection();
    void setPower();
    
    void stop();

    void handleTimerEvent();






};


#endif /* SRC_CONTROLLERS_WHEELDRIVER_H_ */