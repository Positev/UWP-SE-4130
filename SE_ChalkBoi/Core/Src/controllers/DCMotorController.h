/*
 * DCMotorController.h
 *
 *  Created on: Mar 31, 2021
 *      Author: Trevor Keegan
 */

#ifndef SRC_CONTROLLERS_DCMOTORCONTROLLER_H_
#define SRC_CONTROLLERS_DCMOTORCONTROLLER_H_

enum MotorDirection {Clockwise, CounterClockwise};

/*
 * DC Motor controller will provide a set of functions for interacting with a
 * motor via PWM signal sent through the output at (port,pin)
 */
class DCMotorController
{

private:

	//---Board implementation specific--------------------
	int pwmPort;
	int pwmPin;

	int dir1Pin;
	int dir1Port;
	
	int dir2Pin; 
	int dir2Port;
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
	DCMotorController(int port, int pin, int dir1Port, int dir1Pin, int dir2Port, int dir2Pin);

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
