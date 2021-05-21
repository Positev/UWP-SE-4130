#include "ServoController.h"
#include"main.h"
#include "cmsis_os.h"

ServoController::ServoController(GPIO_TypeDef* port, uint16_t pin){
  this->pwmPort = port;
  this->pwmPin = pin;
  this->angle = 0;
}


void ServoController::goToAngle(float angle){
  this->angle = angle;
}

void ServoController::pwmPulse(){
  
  int frequency = 20;
  float pos = (float) this->angle / 180;
  float power =  (2.0 -.91) * pos + .91; // micro seconds
  float on =  power;
  float off = frequency-power;

  HAL_GPIO_WritePin(pwmPort, pwmPin, GPIO_PIN_SET);
  osDelay(on);

  HAL_GPIO_WritePin(pwmPort, pwmPin, GPIO_PIN_RESET);
  osDelay(off);
}

