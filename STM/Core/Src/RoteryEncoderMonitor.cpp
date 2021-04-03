#include"RoteryEncoderMonitor.h"
#include"main.h"
RoteryEncoderMonitor::RoteryEncoderMonitor(GPIO_TypeDef* portA, uint16_t pinA, GPIO_TypeDef* portB, uint16_t pinB ){
    this->signalAPort = portA;
    this->signalAPin = pinA;
    this->signalBPort = portB;
    this->signalBPin = pinB;

    this->tickCount = 0;
}

void RoteryEncoderMonitor::handleChange(){
    int newReadingA = HAL_GPIO_ReadPin(this->signalAPort, this->signalAPin);
    int newReadingB = HAL_GPIO_ReadPin(this->signalBPort, this->signalBPin);

    if (newReadingA != this->lastReadingA  && this->lastReadingA == 1){

    		// If the DT state is different than the CLK state then
    		// the encoder is rotating CCW so decrement
    		if (newReadingB != this->lastReadingB) {
    			this->tickCount --;
    			this->direction = RoteryEncoderMonitor::CounterClockwise;
    		} else {
    			// Encoder is rotating CW so increment
    			this->tickCount ++;
    			this->direction = RoteryEncoderMonitor::Clockwise;
    		}

    	}

    this->lastReadingA = newReadingA;
    this->lastReadingB = newReadingB;


}

int RoteryEncoderMonitor::getCount(){
    return this->tickCount;
}

void RoteryEncoderMonitor::reset(){
    this->tickCount = 0;
}
