#ifndef steeringPID_h
#define steeringPID_h
#include "Arduino.h" 
#include <Servo.h>

class steeringPID {
public:
	steeringPID(Servo steering_servo, float Kp, float Ki, float Kd);
	void set_error(float error);
    int update(int current_time, float left_distance, float right_distance);
private:
    Servo _steering_servo;

    int _u;

    int _Kp;
    int _Ki;
    int _Kd;

    int _last_time;
    float _delta_time;

    float _target_error;
    

    float _last_dist_error;
    float _dist_error;
    float _delta_error;
    float _integral_error;
};
#endif