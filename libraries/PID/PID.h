#ifndef PID_h
#define PID_h
#include "Arduino.h" 
#include <Servo.h>

class PID {
public:
    PID(Servo motor_servo, float Kp, float Ki, float Kd);
    void set_velocity(float velocity);
    int update(int current_time);
    void update_velocity(float velocity);
private:
    Servo _motor_servo;

    int _u;

    int _Kp;
    int _Ki;
    int _Kd;

    int _last_time;
    float _delta_time;

    float _velocity;
    float _target_velocity;

    float _last_velocity_error;
    float _velocity_error;
    float _delta_error;
    float _integral_error;
};
#endif