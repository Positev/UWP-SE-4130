#include "PIDDriver.h"

PIDDriver::PIDDriver(MotorNumber number)
{
    if (number == Motor1)
    {
        this->motorController = new DCMotorController(M1_PWM_GPIO_Port, M1_PWM_Pin, M1_DIR_1_GPIO_Port, M1_DIR_1_Pin, M1_DIR_2_GPIO_Port, M1_DIR_2_Pin);
        this->encoder = new RoteryEncoderMonitor(M1_ENC_1_GPIO_Port, M1_ENC_1_Pin, M1_ENC_2_GPIO_Port, M1_ENC_2_Pin);
    }
    else if (number == Motor2)
    {
        this->motorController = new DCMotorController(M2_PWM_GPIO_Port, M2_PWM_Pin, M2_DIR_1_GPIO_Port, M2_DIR_1_Pin, M2_DIR_2_GPIO_Port, M2_DIR_2_Pin);
        this->encoder = new RoteryEncoderMonitor(M2_ENC_1_GPIO_Port, M2_ENC_1_Pin, M2_ENC_2_GPIO_Port, M2_ENC_2_Pin);
    }
    else
    {
        this->motorController = new DCMotorController(M3_PWM_GPIO_Port, M3_PWM_Pin, M3_DIR_1_GPIO_Port, M3_DIR_1_Pin, M3_DIR_2_GPIO_Port, M3_DIR_2_Pin);
        this->encoder = new RoteryEncoderMonitor(M3_ENC_1_GPIO_Port, M3_ENC_1_Pin, M3_ENC_2_GPIO_Port, M3_ENC_2_Pin);
    }
}

void PIDDriver::setDutyCycle(float dutyCycle)
{
    this->dutyCycle = dutyCycle;
    this->motorController->setPower(this->dutyCycle);
}

void PIDDriver::setDirectory(MotorDirection direction)
{
    this->direction = direction;
    this->motorController->setDirection(this->direction);
}

void PIDDriver::setTickTarget(int tick)
{
    this->tickTarget = tick;
}

void PIDDriver::setStop()
{
    this->motorController->stop();
}

PIDState PIDDriver::getState()
{
    return this->pidState;
}

int PIDDriver::checkState()
{

    if (encoder->getCount() > tickTarget)
        return 1;
    else
        return 0;
}