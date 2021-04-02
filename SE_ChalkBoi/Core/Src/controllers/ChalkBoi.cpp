/*
 * ChalkBoi.cpp
 *
 * Used https://www.modernescpp.com/index.php/thread-safe-initialization-of-a-singleton
 * as an example for thread safe singleton.
 *
 *  Created on: Apr 1, 2021
 *      Author: Tkgn1
 */
#include "DCMotorController.h";
#include "RoteryEncoderMonitor.h";
#include "WheelDriver.h";
#include "DriveTrainController.h";
#include "main.h"


class ChalkBoi{
	static ChalkBoi *instance;


public:
	static ChalkBoi& getInstance(){
		static ChalkBoi instance;
		return instance;
	}

	void pwmPulse(int motor){
		if (motor == 1){
			return motor1->pwmPulse();
		}
		else if (motor == 2){
			return motor2->pwmPulse();
		}
		else if (motor == 3){
			return motor3->pwmPulse();
		}
	}

	void encoderTick(int wheel, int channel, int value){
			if (wheel == 1){
				return encoder1->handleChange(channel, value);
			}
			else if (wheel == 2){
				return encoder2->handleChange(channel, value);
			}
			else if (wheel == 3){
				return encoder3->handleChange(channel, value);
			}
	}

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

	DriveTrainController *driveBase = new DriveTrainController(wheel1, wheel2, wheel3);

	ChalkBoi(){
		motor1 = new DCMotorController(Motor1PWM_GPIO_Port, Motor1PWM_Pin, Motor1Dir1_GPIO_Port, Motor1Dir1_PinMotor1Dir2_GPIO_Port, Motor1Dir2_Pin);
		motor2 = new DCMotorController(Motor2PWM_GPIO_Port, Motor2PWM_Pin, Motor2Dir1_GPIO_Port, Motor2Dir1_PinMotor2Dir2_GPIO_Port, Motor2Dir2_Pin);
		motor3 = new DCMotorController(Motor3PWM_GPIO_Port, Motor3PWM_Pin, Motor3Dir1_GPIO_Port, Motor3Dir1_PinMotor3Dir2_GPIO_Port, Motor3Dir2_Pin);

		encoder1 = new RoteryEncoderMonitor(Motor1Enc1_GPIO_Port,Motor1Enc1_Pin,Motor1Enc2_GPIO_Port,Motor1Enc2_Pin);
		encoder2 = new RoteryEncoderMonitor(Motor2Enc1_GPIO_Port,Motor2Enc1_Pin,Motor2Enc2_GPIO_Port,Motor2Enc2_Pin);
		encoder3 = new RoteryEncoderMonitor(Motor3Enc1_GPIO_Port, Motor3Enc1_Pin,Motor3Enc2_GPIO_Port,Motor3Enc2_Pin);

		wheel1 = new WheelDriver(motor1, encoder1);
		wheel2 = new WheelDriver(motor2, encoder2);
		wheel3 = new WheelDriver(motor3, encoder3);
		driveBase = new DriveTrainController(wheel1, wheel2, wheel3);
	};
	  ~ChalkBoi()= default;
	  ChalkBoi(const ChalkBoi&)= delete;
	  ChalkBoi& operator=(const ChalkBoi&)= delete;
};
