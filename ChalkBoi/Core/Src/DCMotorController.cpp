#include "DCMotorController.h"
#include  "main.h"

DCMotorController::DCMotorController(GPIO_TypeDef* port, uint16_t pin, GPIO_TypeDef* dir1Port, uint16_t dir1Pin, GPIO_TypeDef* dir2Port, uint16_t dir2Pin){
  this->pwmPort = port;
  this->pwmPin = pin;
  this->dir1Port = dir1Port;
  this->dir1Pin = dir1Pin;
  this->dir2Port = dir2Port;
  this->dir2Pin = dir2Pin;

  this->frequency = 20;
  this->power = 0;
  this->direction = DCMotorController::Clockwise;
} 

void DCMotorController::setPower(float power){
  if (power > 1){
    this->power = 1;
  }
  else if (power < 0){
    this->power = 0;
  }
  else{
    this->power = power;
  }
  
}

void DCMotorController::pwmPulse(){
  float power = this->power;
  HAL_GPIO_WritePin(this->pwmPort, this->pwmPin, GPIO_PIN_SET) ;
  HAL_Delay(this->frequency * power);
  
  HAL_GPIO_WritePin(this->pwmPort, this->pwmPin, GPIO_PIN_RESET) ;
  HAL_Delay(this->frequency * (1 - power));
  }

void DCMotorController::setDirection(MotorDirection direction){
  if (direction == DCMotorController::Clockwise){
      HAL_GPIO_WritePin(this->dir1Port, this->dir1Pin, GPIO_PIN_SET) ;
      HAL_GPIO_WritePin(this->dir2Port, this->dir2Pin, GPIO_PIN_RESET) ;
  }else if (direction == DCMotorController::CounterClockwise){

      HAL_GPIO_WritePin(this->dir1Port, this->dir1Pin,GPIO_PIN_RESET ) ;
      HAL_GPIO_WritePin(this->dir2Port, this->dir2Pin, GPIO_PIN_SET) ;
  }
}

void DCMotorController::stop(){
      HAL_GPIO_WritePin(this->dir1Port, this->dir1Pin, GPIO_PIN_RESET) ;
      HAL_GPIO_WritePin(this->dir2Port, this->dir2Pin, GPIO_PIN_RESET) ;
}
