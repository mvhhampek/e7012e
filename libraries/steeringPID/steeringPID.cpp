#include "Arduino.h"
#include "steeringPID.h"

steeringPID::steeringPID(Servo steering_servo, float Kp, float Ki, float Kd, int buffer_size) {
    _steering_servo = steering_servo;

    _u = 70; // output
    _buffer_size = buffer_size;
    _buffer_index = 0;
    
    integral_error_buffer = new float[_buffer_size];
    for (int i = 0; i < _buffer_size; i++) {
        integral_error_buffer[i] = 0.0;
    }
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;


    _last_time = millis();
    _delta_time = 0;

    _last_dist_error = 0;
    _dist_error = 0;
    _delta_error = 0;
    _integral_error = 0;

    

}

void steeringPID::set_error(float error){
	_target_error = error;
}

int steeringPID::update(int current_time, float left_distance, float right_distance) {
    if (current_time-_last_time > 100) { 
        _delta_time = float(current_time-_last_time);

    
        //Serial.print("left:");
        //Serial.print(left_distance);
        //Serial.print("\tright");
        //Serial.print(right_distance);

        //_dist_error = left_distance - right_distance;
        _dist_error = 70 - right_distance;
        // if (_dist_error < 50){
        //    _dist_error = 0;
        //}

        //_integral_error += _dist_error * (_delta_time/1000);  

        float new_error_contribution = _dist_error * (_delta_time / 1000);
        //_integral_error -= _integral_error_buffer[_buffer_index]; 
        integral_error_buffer[_buffer_index] = new_error_contribution; 
        _integral_error = 0;
        for (int i = 0; i < _buffer_size; i++) {
            _integral_error += integral_error_buffer[i];
        }
        //_integral_error += new_error_contribution; 


        _buffer_index = (_buffer_index + 1) % _buffer_size;


        _delta_error    = -((_dist_error-_last_dist_error)*1000)/(_delta_time);
        




        _last_time = current_time;
        _last_dist_error = _dist_error; 

        _u = 70 + _Kp*_dist_error + _Ki*_integral_error + _Kd*_delta_error; 
        
        //((_Kp*_dist_error+_Ki*_integral_error+_Kd*_delta_error+70)+_u)/2;
        // Constraints
        if (_u < 45) {
            _u = 45;
        }
        if (_u > 95) {
            _u = 95;
        }
        
        //Serial.print("p error: ");
        //Serial.print(_dist_error);
        //Serial.print("\ti error: ");
        //Serial.println(_integral_error);
        //Serial.print(_u);
        //Serial.println("\t");
        //Serial.print(_delta_error*_Kp);
        //Serial.print("\t");
        //Serial.print(_integral_error*_Ki);
        //Serial.print("\t");
        //Serial.println(_dist_error*_Kd);
        
    }
    return _u;
}

//void PID::update_velocity(float velocity){
//  _velocity = velocity;
//}


