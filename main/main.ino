
#include <Servo.h>
#include <PID.h>
#include <steeringPID.h>
// 130.240.152.251
////// Constants //////
// Pins
const byte HALL_INTERRUPT_PIN = 12;
const byte MOTOR_PIN = 7;
const byte SERVO_PIN = 5;


const byte LEFT_PING_PIN = 27;
const byte LEFT_ECHO_PIN = 26;

const byte RIGHT_PING_PIN = 52;
const byte RIGHT_ECHO_PIN = 53;

const byte FORWARD_PING_PIN = 22;
const byte FORWARD_ECHO_PIN = 23;

// Wheel circumference in meters
const float WHEEL_CIRCUMFERENCE = 0.2042; 
const int  NUM_MAGNETS = 4;


volatile float left_distance = 0;
volatile float right_distance = 0; 
volatile float forward_distance = 0;

// Counter for hall interrupts
volatile int hall_counter = 0;     
volatile int last_hall_counter = 0;
volatile int delta_hall_counter = 0;

// Motor and steering servos 
Servo motor_servo;
Servo steering_servo;

// motor PID constants
volatile float Kp = 12;
volatile float Ki = 1;
volatile float Kd = 0;
const int integral_buffer = 100;
PID pid(motor_servo, Kp, Ki, Kd, integral_buffer);


// steerig PID constants
volatile float s_Kp = 0.5;
volatile float s_Ki = 0.005;
volatile float s_Kd = -0.45;
const int s_integral_buffer = 20;
steeringPID s_pid(steering_servo, s_Kp, s_Ki, s_Kd, s_integral_buffer);


// N point running average for left, right distance
const int N_dist = 2;
float left_distances[N_dist] = {0};
float right_distances[N_dist] = {0}; 
float forward_distances[N_dist] = {0};
float left_avg = 0;
float right_avg = 0;
float forward_avg = 0;
int left_index = 0;
int right_index = 0;
int forward_index = 0;


const int N_spd = 1;
volatile float speeds[N_spd] = {0};
volatile int speed_index = 0;
volatile float speed_avg = 0;
// Angle and "speed" to send to motor servo and steering servo
int motor_speed = 1500;  // 1500 stilla, 2000 max frammåt, 1000 max backåt
int steering_angle = 70; // mellan 45 och 95 typ, 45 höger, 95 vänster


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
bool update_flag = false;
// user input...
String input = "";
int int_input = 0;
bool running = false;

void setup() {

    Serial.begin(9600);
    
    pinMode(HALL_INTERRUPT_PIN, INPUT_PULLUP);
    pinMode(SERVO_PIN, OUTPUT);
    pinMode(MOTOR_PIN, OUTPUT);

    steering_servo.attach(SERVO_PIN);
    motor_servo.attach(MOTOR_PIN, 1000, 2000);

    attachInterrupt(digitalPinToInterrupt(HALL_INTERRUPT_PIN), hallISR, RISING);
    
    Serial.println("setup");
    steering_servo.write(70);
    motor_servo.writeMicroseconds(motor_speed);
  

    //pinMode(LEFT_PING_PIN, OUTPUT);
    //pinMode(LEFT_ECHO_PIN, INPUT);

    
    pinMode(RIGHT_PING_PIN, OUTPUT);
    pinMode(RIGHT_ECHO_PIN, INPUT);

    pinMode(FORWARD_PING_PIN, OUTPUT);
    pinMode(FORWARD_ECHO_PIN, INPUT);

    

    delay(1000);    
}

void loop() {
    current_time = millis();
    
    int u = pid.update(current_time, forward_avg);
    Serial.print("U: ");
    Serial.print(u);

    //Serial.println(u);
    motor_servo.writeMicroseconds(u);
    

    //left_distance  = getDistance(LEFT_PING_PIN, LEFT_ECHO_PIN);
    //right_distance = getDistance(RIGHT_PING_PIN, RIGHT_ECHO_PIN);
    //left_avg = (getAverageDistance(LEFT_PING_PIN, LEFT_ECHO_PIN, left_distances, left_index, 2000));
    left_avg=0;
    right_avg = (getAverageDistance(RIGHT_PING_PIN, RIGHT_ECHO_PIN, right_distances, right_index, 2000));
    forward_avg = getAverageDistance(FORWARD_PING_PIN, FORWARD_ECHO_PIN, forward_distances, forward_index, 2000);
    right_avg = min(forward_avg, right_avg);

    pid.set_velocity(speed_ref);
    int angle = s_pid.update(current_time, left_avg, right_avg);
    
    if (running){
      steering_servo.write(angle);

    }

    //Serial.println(angle);

    
    
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
    }else{
        pid.update_velocity(current_vel);
    }


    //right_distance = getDistance(RIGHT_PING_PIN, RIGHT_ECHO_PING);
    //forward_distance = getDistance(FORWARD_PING_PIN, FORWARD_ECHO_PIN);
    
    Serial.print("\tR: ");
    Serial.print(right_avg);
    Serial.print("\tL: ");
    Serial.print(left_avg);
    //Serial.print("\tD: ");
    //Serial.print(right_avg-left_avg);
    Serial.print("\tF: ");
    Serial.print(forward_avg);
    Serial.print("\tU: ");
    Serial.println(angle);
   
    

    //String to_send = String(current_vel) + "-" + String(left_distance) + "-" + String(right_distance);
    
    //Serial.println(to_send);
    //Serial.println(current_vel);

    if (Serial.available() > 0) { 
        input = Serial.readStringUntil('\n');
        speed_ref=input.toFloat();
        pid.set_velocity(speed_ref);
        running = true;
    }   
    if (current_vel > 0.1 && !running){
        Serial.println("hej");
        speed_ref = 1;
        pid.set_velocity(speed_ref);
 

        running = true;
    }
}


void setSpeedAndAngle(){
    input = Serial.readStringUntil('\n');
    int split_index = input.indexOf('-');
    steering_angle = input.substring(0,split_index).toInt();
    speed_ref = input.substring(split_index+1).toInt();  

    if (old_speed_ref != speed_ref || old_steering_angle != steering_angle){
        pid.set_velocity(speed_ref);
        if (steering_angle >= 55 && steering_angle <= 95) {
            steering_servo.write(steering_angle);
        }
        old_speed_ref = speed_ref;
        old_steering_angle = steering_angle;
    }

}


long getDistance(byte trigger_pin, byte echo_pin){
    long duration, cm;
    // duration:
    digitalWrite(trigger_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_pin, LOW);
    duration = pulseIn(echo_pin, HIGH);
    
    
    // distance:
    return microsecondsToCentimeters(duration);
}

float getAverageDistance(byte trigger_pin, byte echo_pin, float *distances, int &index, int max){
    long cm = getDistance(trigger_pin, echo_pin);
    
    distances[index] = cm;
    index = (index + 1) % N_dist;

    float sum = 0;
    for (int i = 0; i < N_dist; i++) {
        sum += distances[i];
    }
    sum /= N_dist;
    if (sum > max){
        sum = max;
    }
    
    return sum;
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
    current_vel = getVelocity(speeds, speed_index);
  }
  pid.update_velocity(current_vel);
}

// Returnerar Velocity basarat på delta_time

float getVelocity(volatile float* speeds, volatile int &index){
    float distance = (delta_hall_counter*WHEEL_CIRCUMFERENCE)/NUM_MAGNETS;
    current_vel = (((distance*1000)/(delta_time))+current_vel)/2;

    speeds[index] = current_vel;
    index = (index + 1) % N_spd;

    float sum = 0;
    for (int i = 0; i < N_spd; i++) {
        sum += speeds[i];
    }
    
    return sum/N_spd;
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