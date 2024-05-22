#include "Arduino.h"
#include "PID.h"

PID::PID(Servo motor_servo, float Kp, float Ki, float Kd, int buffer_size) {
    _motor_servo = motor_servo;

    _u = 0;
    
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;

    _buffer_size = buffer_size;
    _buffer_index = 0;
    
    integral_error_buffer = new float[_buffer_size];
    for (int i = 0; i < _buffer_size; i++) {
        integral_error_buffer[i] = 0.0;
    }

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

int PID::update(int current_time, float forward_distance) {
    if (current_time-_last_time > 100) { 
        _delta_time = float(current_time-_last_time);

        _velocity_error = _target_velocity-_velocity;
        _delta_error = -((_velocity_error-_last_velocity_error)*1000)/(_delta_time);


        //_integral_error += _velocity_error * (_delta_time/1000);  
        

        float new_error_contribution = _velocity_error * (_delta_time / 1000);
        integral_error_buffer[_buffer_index] = new_error_contribution; 
        _integral_error = 0;
        for (int i = 0; i < _buffer_size; i++) {
            _integral_error += integral_error_buffer[i];
        }
        
        
        //_integral_error += new_error_contribution; 


        _buffer_index = (_buffer_index + 1) % _buffer_size;


        _last_time = current_time;
        _last_velocity_error = _velocity_error;
        //Serial.println(_velocity_error);
        _u = (_Kp*_velocity_error+_Ki*_integral_error+_Kd*_delta_error+1575);
        if (_target_velocity == 0){
            _u = 1500;
            Serial.println("AAAAAAA: ");
        }
        /*
        Serial.print("P: ");
        Serial.print(_velocity_error);
        Serial.print("\t\tI: ");
        Serial.print(_integral_error);
        Serial.print("\t\tD: ");
        Serial.print(_delta_error);
        Serial.print("\t\tU: ");
        Serial.println(_u);
*/
        //_u *= forward_distance/200;
        //Serial.print("U: ");
        //Serial.println(_u);
        if (_u<1500) {
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


