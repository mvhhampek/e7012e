#include "Arduino.h"
#include "PID.h"

PID::PID(Servo motor_servo, float Kp, float Ki, float Kd) {
  _motor_servo = motor_servo;

  _u = 0;
  
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;

	_velocity = 0;
	_target_velocity = 0;

  _last_time = millis();
  _delta_time = 0;

  _last_velocity_error = 0;
  _velocity_error = 0;
  _delta_error = 0;
  _integral_error = 0;
}

void PID::set_velocity(float velocity){
	_target_velocity = velocity;
}

int PID::update(int current_time) {
  if (current_time-_last_time > 100) { 
  _delta_time = float(current_time-_last_time);

  _velocity_error = _target_velocity-_velocity;
  _delta_error = -((_velocity_error-_last_velocity_error)*1000)/(_delta_time);
  _integral_error += _velocity_error * (_delta_time/1000);  
  
  _last_time = current_time;
  _last_velocity_error = _velocity_error;

  
  //if (current_time%100){
  //  Serial.print(_velocity_error);
  //  Serial.println(" _velocity_error");
  //  Serial.print(_delta_error);
  //  Serial.println(" _delta_error");
  //  Serial.print(_integral_error);
  //  Serial.println(" _integral_error");
  //  Serial.print(_target_velocity);
  //  Serial.println(" _target_velocity");
  //  Serial.print(_velocity);
  //  Serial.println(" _velocity");
  //  Serial.print(_delta_time);
  //  Serial.println(" _delta_time");
  //  Serial.println(" ");
  //}

  _u = ((_Kp*_velocity_error+_Ki*_integral_error+_Kd*_delta_error+1575)+_u)/2;
  if (_u<1575) {
    _u=1500;
  }
  if (_u>1700) {
    _u=1700;
  }
  }
  return _u;
}

void PID::update_velocity(float velocity){
  _velocity = velocity;
}