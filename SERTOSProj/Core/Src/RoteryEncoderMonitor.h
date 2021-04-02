/*
 * RoteryEncoderMonitor.h
 *
 *  Created on: Mar 31, 2021
 *      Author: trevorkeegan
 */

#ifndef SRC_CONTROLLERS_ROTERYENCODERMONITOR_H_
#define SRC_CONTROLLERS_ROTERYENCODERMONITOR_H_

/*
 * RoteryEncoderMonitor will be used to handle value change events for the pins. 
 * A rotery encoder uses two pins to track rotation and direction so provide two pins to use this.
 */
class RoteryEncoderMonitor
{

private:

	//---Board implementation specific--------------------
	GPIO_TypeDef* signalAPort;
	int signalAPin;
	GPIO_TypeDef signalBPort;
	int signalBPin;
	//---Board implementation specific--------------------


	//---Motor parameters for desired behavior-----------

	int tickCount; // negative number implies reverse direction. No range limit. 
  int lastReadingA; // 1 or 0 - high or low
  int lastReadingB; // 1 or 0 - high or low
	//---Motor parameters for desired behavior-----------

public:

	/*
	 * IN - PORT is the port used to identify the signal pin that is driven high and low for pwm
	 * IN - PIN is the pin used to identify the signal pin that is driven high and low for pwm
	 */
	RoteryEncoderMonitor(GPIO_TypeDef portA, uint16_t pinA, GPIO_TypeDef portB, uint16_t pinB );

	void handleChange(int channel, int newValue){};

    // retrieve count
    int getCount();

    // set count to 0
    void reset();


};


#endif /* SRC_CONTROLLERS_ROTERYENCODERMONITOR_H_ */
