#ifndef SRC_CONTROLLERS_PIDDRIVER_H_
#define SRC_CONTROLLERS_PIDDRIVER_H_
#include "DCMotorController.h"
#include "RoteryEncoderMonitor.h"


    enum PIDState
    {
        Stop,
        Ensure
    };

enum MotorNumber{Motor1, Motor2, Motor3};
/*
 * Track and report end stop state
 */
class PIDDriver
{
public:
    DCMotorController* motorController;
    RoteryEncoderMonitor* encoder;


    float compute(float error);

    void setDutyCycle(float);
    void setDirectory(MotorDirection);
    void setAngularVelocity(int);
    void setStop();
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
    MotorDirection directory;
    int angularVelocity;
    PIDState pidState;

    


};
#endif /* SRC_CONTROLLERS_PIDDRIVER_H_ */