#ifndef SRC_CONTROLLERS_PIDDRIVER_H_
#define SRC_CONTROLLERS_PIDDRIVER_H_
#include "DCMotorController.h"
#include "RoteryEncoderMonitor.h"


enum PIDState
{
    PIDStop,
    PIDEnable
};

enum MotorNumber
{
    Motor1,
    Motor2,
    Motor3
};

/*
 * Track and report end stop state
 */
class PIDDriver
{
public:
    DCMotorController *motorController;
    RoteryEncoderMonitor *encoder;
    
    float compute(float error);

    void setDutyCycle(float);
    void setDirectory(MotorDirection);
    void setTickTarget(int);
    void setStop();

    PIDState getState();
    int checkState();
    PIDDriver(MotorNumber);
    //~PIDDriver();

private:
    float previousError;
    float totalError;

    MotorNumber number;

    float computeKP(float error);
    float computeKI(float error);
    float computeKD(float error);

    float dutyCycle;
    MotorDirection direction;
    int tickTarget;
    PIDState pidState = PIDEnable;
};
#endif /* SRC_CONTROLLERS_PIDDRIVER_H_ */