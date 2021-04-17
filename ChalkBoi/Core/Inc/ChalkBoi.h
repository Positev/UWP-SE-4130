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
#include "main.h"


class ChalkBoi{


public:
	static ChalkBoi& getInstance();

	void pwmPulse(int motor);

	void encoderTick();

private:

	DCMotorController *motor1;
	DCMotorController *motor2;
	DCMotorController *motor3;

	RoteryEncoderMonitor *encoder1;
	RoteryEncoderMonitor *encoder2;
	RoteryEncoderMonitor *encoder3;

	WheelDriver *wheel1;
	WheelDriver *wheel2;
	WheelDriver *wheel3;

	DriveTrainController *driveBase;

	ChalkBoi();
  ChalkBoi& operator=(const ChalkBoi&);
};
