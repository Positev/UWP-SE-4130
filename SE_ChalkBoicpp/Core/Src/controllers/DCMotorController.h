/*
 * DCMotorController.h
 *
 *  Created on: Mar 31, 2021
 *      Author: Trevor Keegan
 */

#ifndef SRC_CONTROLLERS_DCMOTORCONTROLLER_H_
#define SRC_CONTROLLERS_DCMOTORCONTROLLER_H_

#include "cmsis_os.h"
#include "main.h"

/*
 * DC Motor controller will provide a set of functions for interacting with a
 * motor via PWM signal sent through the output at (port,pin)
 */
class DCMotorController
{

enum MotorDirection {Clockwise, CounterClockwise};
private:

	//---Board implementation specific--------------------
uint16_t pwmPort;
uint16_t pwmPin;

uint16_t dir1Pin;
uint16_t dir1Port;

uint16_t dir2Pin;
uint16_t dir2Port;
	//---Board implementation specific--------------------


	//---Motor parameters for desired behavior-----------

	int frequency; // in milliseconds, motor specific possibly
	float power;
	MotorDirection direction; // True should spin clockwise, false should s
	//---Motor parameters for desired behavior-----------

public:

	/*
	 * IN - PORT is the port used to identify the signal pin that is driven high and low for pwm
	 * IN - PIN is the pin used to identify the signal pin that is driven high and low for pwm
	 */
	DCMotorController(uint16_t port, uint16_t pin, uint16_t dir1Port, uint16_t dir1Pin, uint16_t dir2Port, uint16_t dir2Pin);

	/*
	 * IN - POWER should be a range limited float between 0 and 1
	 *
	 */
	void setPower(float power);

	/*
	 *
	 * IN - DIRECTION should be a MOTORDIRECTION enumeration that will change the spinning
	 * 		direction of the motor
	 */
	void setDirection(MotorDirection direction);

	/*
	 * bring motor to full stop
	 */
	void stop();

	/*
	 * drive pin at port high then low based on duty cycle determined by power
	 */
	void pwmPulse();


};


#endif /* SRC_CONTROLLERS_DCMOTORCONTROLLER_H_ */
