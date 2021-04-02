/*
 * EndStopController.h
 *
 *  Created on: Mar 31, 2021
 *      Author: trevorkeegan
 */

#ifndef SRC_CONTROLLERS_ENDSTOPCONTROLLER_H_
#define SRC_CONTROLLERS_ENDSTOPCONTROLLER_H_


/*
 * Track and report end stop state
 */
class EndStopController
{

private:

	//---Board implementation specific--------------------
	int signalPort;
	int signalPin;
	//---Board implementation specific--------------------


	//---Motor parameters for desired behavior-----------

    int lastReading; // 1 or 0 - high or low
	//---Motor parameters for desired behavior-----------

public:

	/*
	 * IN - PORT is the port used to identify the signal pin that is driven high and low for pwm
	 * IN - PIN is the pin used to identify the signal pin that is driven high and low for pwm
	 */
	EndStopController(int portA, int pinA, int portB, int pinB );

    // handle pin interrupt
	void handleChange();

    // get last reading
    int isPressed();



};


#endif /* SRC_CONTROLLERS_ENDSTOPCONTROLLER_H_ */
