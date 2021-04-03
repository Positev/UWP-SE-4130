/*
 * ServoController.h
 *
 *  Created on: Mar 31, 2021
 *      Author: trevorkeegan
 */

#ifndef SRC_CONTROLLERS_SERVOCONTROLLER_H_
#define SRC_CONTROLLERS_SERVOCONTROLLER_H_



/*
 * Control a 180 noncontinuous pwm servo
 */
class ServoController
{

private:

	//---Board implementation specific--------------------
	int Port;
	int Pin;
	//---Board implementation specific--------------------


	//---Motor parameters for desired behavior-----------

	float angle;
    int frequency;
	//---Motor parameters for desired behavior-----------

public:

	/*
	 * IN - PORT is the port used to identify the signal pin that is driven high and low for pwm
	 * IN - PIN is the pin used to identify the signal pin that is driven high and low for pwm
	 */
	ServoController(int portA, int pinA);

    /*
     * Rotate servo to angle, angle is range limited in degrees between 0 and 180
     */
    void goToAngle(float angle); 

    /*
	 * drive pin at port high then low based on duty cycle determined by power
	 */
    void pwmPulse();


};



#endif /* SRC_CONTROLLERS_SERVOCONTROLLER_H_ */
