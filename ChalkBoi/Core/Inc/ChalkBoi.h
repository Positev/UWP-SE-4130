/*
 * ChalkBoi.cpp
 *
 * Used https://www.modernescpp.com/index.php/thread-safe-initialization-of-a-singleton
 * as an example for thread safe singleton.
 *
 *  Created on: Apr 1, 2021
 *      Author: Tkgn1
 */
#include "DCMotorController.h"
#include "RoteryEncoderMonitor.h"
#include "WheelDriver.h"
#include "DriveTrainController.h"
#include "ServoController.h"
#include "main.h"



class ChalkBoi
{

public:
	static ChalkBoi &getInstance();

	//void pwmPulse(int motor);

	void encoderTick();

	void setForward(int);
	void setTurn(MotorDirection, int);
	void startPID1();
	void startPID2();
	void startPID3();
void updateMotor1State();
void updateMotor2State();
void updateMotor3State();
	void stopPID1();
	void stopPID2();
	void stopPID3();
	RoteryEncoderMonitor *getEncoder1();
	RoteryEncoderMonitor *getEncoder2();
	RoteryEncoderMonitor *getEncoder3();
	ServoController *getServo();
	PIDDriver *getPID1();
	PIDDriver *getPID2();
	PIDDriver *getPID3();

private:
	PIDDriver *pid1;
	PIDDriver *pid2;
	PIDDriver *pid3;

	

	// DCMotorController *motor1;
	// DCMotorController *motor2;
	// DCMotorController *motor3;

	RoteryEncoderMonitor *encoder1;
	RoteryEncoderMonitor *encoder2;
	RoteryEncoderMonitor *encoder3;

	ServoController * servo;

	// WheelDriver *wheel1;
	// WheelDriver *wheel2;
	// WheelDriver *wheel3;

	// DriveTrainController *driveBase;

	ChalkBoi();
	ChalkBoi &operator=(const ChalkBoi &);
};
