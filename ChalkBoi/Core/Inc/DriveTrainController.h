/*
 * DriveTrainController.h
 *
 *  Created on: Mar 31, 2021
 *      Author: trevorkeegan
 */

#ifndef SRC_CONTROLLERS_DRIVETRAINCONTROLLER_H_
#define SRC_CONTROLLERS_DRIVETRAINCONTROLLER_H_

#include "WheelDriver.h"
#include "PIDDriver.h"

class DriveTrainController
{

private:

    WheelDriver *wheelA;
    WheelDriver *wheelB;
    WheelDriver *wheelC;

    PIDDriver pid;


public:

	DriveTrainController(WheelDriver *wheelA, WheelDriver *wheelB, WheelDriver *wheelC);

    void handleTimerEvent();
    void move(float vx, float vy, float distance);
    void rotate(float degrees);
    void stop();








};



#endif /* SRC_CONTROLLERS_DRIVETRAINCONTROLLER_H_ */
