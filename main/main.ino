#include <Servo.h>
#include <PID.h>

////// Constants //////
// Pins
const byte HALL_INTERRUPT_PIN = 12;
const byte MOTOR_PIN = 8;
const byte SERVO_PIN = 5;


const byte LEFT_PING_PIN = 26;
const byte LEFT_ECHO_PIN = 27;

const byte RIGHT_PING_PIN = 32;
const byte RIGHT_ECHO_PIN = 33;



// Wheel circumference in meters
const float WHEEL_CIRCUMFERENCE = 0.2042; 
const int  NUM_MAGNETS = 4;


volatile float left_distance = 0;
volatile float right_distance = 0; 

// Counter for hall interrupts
volatile int hall_counter = 0;     
volatile int last_hall_counter = 0;
volatile int delta_hall_counter = 0;

// Motor and steering servos 
Servo motor_servo;
Servo steering_servo;

// motor PID constants
volatile float Kp = 1.4;
volatile float Ki = 2.2;
volatile float Kd = 0;
PID pid(motor_servo, Kp, Ki, Kd);


// steerig PID constants
volatile float s_Kp = 1.4;
volatile float s_Ki = 2.2;
volatile float s_Kd = 0.1;
steeringPID steeringPID(steering_servo, s_Kp, s_Ki, s_Kd)




// Angle and "speed" to send to motor servo and steering servo
int motor_speed = 1500;  // 1500 stilla, 2000 max frammåt, 1000 max backåt
int steering_angle = 70; // mellan 40 och 100 typ

int speed_ref = 0;
int old_speed_ref = 0;
int old_steering_angle = 70;

// Forward speed computed by encoder
float current_vel = 0;
volatile float prev_left = 0;

// Time between pulses, used to calculate speed
volatile int last_time = 0;
volatile int current_time = 1;
volatile int delta_time = 0;

bool flag = true;
bool update_flag = false;

// user input...
String input = "";
int int_input = 0;

void setup() {

    Serial.begin(9600);
    
    pinMode(HALL_INTERRUPT_PIN, INPUT_PULLUP);
    pinMode(SERVO_PIN, OUTPUT);

    steering_servo.attach(SERVO_PIN);
    motor_servo.attach(MOTOR_PIN, 1000, 2000);

    attachInterrupt(digitalPinToInterrupt(HALL_INTERRUPT_PIN), hallISR, RISING);
    
    Serial.println("setup");
    steering_servo.write(70);
    motor_servo.writeMicroseconds(motor_speed);
  

    pinMode(LEFT_PING_PIN, OUTPUT);
    pinMode(LEFT_ECHO_PIN, INPUT);

    
    pinMode(RIGHT_PING_PIN, OUTPUT);
    pinMode(RIGHT_ECHO_PIN, INPUT);

    

    delay(1000);    
}

void loop() {
    current_time = millis();
    

    int u = pid.update(current_time);
    if (u<1585) {
      u = 1500;
    }
    motor_servo.writeMicroseconds(u);

    

    left_distance  = getDistance(LEFT_PING_PIN, LEFT_ECHO_PIN);
    right_distance = getDistance(RIGHT_PING_PIN, RIGHT_ECHO_PIN);
    int angle = steeringPID.update(current_time, left_distance, right_distance);
    steering_servo.write(angle);

    
    // uppdaterar delta_time
    if (update_flag) {
      updateDelta();
      update_flag = false;
    } else {
      // if not updated the deltas are reset
      delta_time = 0;
      delta_hall_counter = 0;
    }

    if (current_time-last_time > 50) {
      current_vel = 0;
      pid.update_velocity(current_vel);
    }


    //right_distance = getDistance(RIGHT_PING_PIN, RIGHT_ECHO_PING);
    //forward_distance = getDistance(FORWARD_PING_PIN, FORWARD_ECHO_PIN);
    
    //Serial.print("Right distance: ");
    //Serial.println(right_distance);
    //Serial.print("Forward distance: ");
    //Serial.println(forward_distance);
    

    String to_send = String(current_vel) + "-" + String(left_distance) + "-" + String(right_distance);
    
    Serial.println(to_send);


    if (Serial.available() > 0) { 
        setSpeedAndAngle();
    }    
}

void setSpeedAndAngle(){
    input = Serial.readStringUntil('\n');
    int split_index = input.indexOf('-');
    steering_angle = input.substring(0,split_index).toInt();
    speed_ref = input.substring(split_index+1).toInt();  

    if (old_speed_ref != speed_ref || old_steering_angle != steering_angle){
        pid.set_velocity(speed_ref);
        steering_servo.write(steering_angle);
        old_speed_ref = speed_ref;
        old_steering_angle = steering_angle;
    }

}


long getDistance(byte trigger_pin, byte echo_pin){
    long duration, cm;
    
    digitalWrite(trigger_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_pin, LOW);
    //pinMode(echo_pin, INPUT);
    duration = pulseIn(echo_pin, HIGH);
    cm = microsecondsToCentimeters(duration);

    //delay(100);
    return cm;

}



long microsecondsToCentimeters(long microseconds){
  return microseconds/29/2;
}

// Updaterar delta_time
void updateDelta(){
  // updates delta_time
  delta_time = current_time-last_time;
  last_time = current_time;

  // updates delta_hall_counter
  delta_hall_counter = hall_counter-last_hall_counter;
  last_hall_counter = hall_counter;

  if (delta_time>0) {
    current_vel = getVelocity();
  }
  pid.update_velocity(current_vel);
}

// Returnerar Velocity basarat på delta_time
float getVelocity(){
    float distance = (delta_hall_counter*WHEEL_CIRCUMFERENCE)/NUM_MAGNETS;
    current_vel = (((distance*1000)/(delta_time))+current_vel)/2;
    return current_vel;
}

void hallISR(){
    update_flag=true;
    hall_counter++;
}





        /*
        // change motor speed
        if (int_input > 1000 && int_input < 2000) {
            motor_speed = int_input;
            motor_servo.writeMicroseconds(motor_speed);
            Serial.print("Set motorspeed to ");
            Serial.println(motor_speed);
        } //change steering angle
        else if (int_input > 40 && int_input < 100) {
            steering_angle = int_input;
            steering_servo.write(steering_angle);
            Serial.print("Set steering angle to ");
            Serial.println(steering_angle);
        } //change ref
        else if (int_input > -5 && int_input < 5) {
            
            Serial.print("Set ref to ");
        }  */