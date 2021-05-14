#include "ServoController.h"
#include"main.h"

ServoController::ServoController(GPIO_TypeDef* port, uint16_t pin){
  this->pwmPort = port;
  this->pwmPin = pin;
  this->angle = 90;
}


void ServoController::goToAngle(float angle){
  this->angle = angle;
}

void ServoController::pwmPulse(){
  
  int frequency = 2;
  float angle = 0 / 180;
  float power = angle; // micro seconds


  HAL_GPIO_WritePin(pwmPort, pwmPin, GPIO_PIN_SET);
  HAL_Delay(frequency * power );

  HAL_GPIO_WritePin(pwmPort, pwmPin, GPIO_PIN_RESET);
  HAL_Delay(frequency* (1 - power));
}