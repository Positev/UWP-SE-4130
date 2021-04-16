#ifndef SRC_CONTROLLERS_PIDDRIVER_H_
#define SRC_CONTROLLERS_PIDDRIVER_H_


/*
 * Track and report end stop state
 */
class PIDDRIVER
{

private:

    float previousError;
    float totalError;

    float computeKP(float error);
    float computeKI(float error);
    float computeKD(float error);

public:

	PIDDRIVER(float kp, float ki, float kd);

    float compute(float error);







};


#endif /* SRC_CONTROLLERS_PIDDRIVER_H_ */