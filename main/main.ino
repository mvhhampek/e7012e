#include <Servo.h>
#include <PID.h>
#include <steeringPID.h>
// 130.240.152.251
////// PINS //////
const byte HALL_INTERRUPT_PIN = 12;
const byte MOTOR_PIN = 7;
const byte SERVO_PIN = 5;

//const byte LEFT_PING_PIN = 27;
//const byte LEFT_ECHO_PIN = 26;

const byte RIGHT_PING_PIN = 52;
const byte RIGHT_ECHO_PIN = 53;

const byte FORWARD_PING_PIN = 22;
const byte FORWARD_ECHO_PIN = 23;



////// ENCODER //////
// Wheel circumference in meters
const float WHEEL_CIRCUMFERENCE = 0.2042; 
const int  NUM_MAGNETS = 4;
// Counter for hall interrupts
volatile int hall_counter = 0;     
volatile int last_hall_counter = 0;
volatile int delta_hall_counter = 0;
// Forward speed computed by encoder
float current_vel = 0;
volatile float prev_left = 0;

// Time between pulses, used to calculate speed
volatile int last_time = 0;
volatile int current_time = 1;
volatile int delta_time = 0;
bool hall_update_flag = false;


////// MOTOR //////
Servo motor_servo;
int motor_speed = 1500;  // 1500 stand still, 2000 max forward, 1000 max backwards

// Motor PID constants
volatile float Kp = 12;
volatile float Ki = 1;
volatile float Kd = 0;
const int integral_buffer = 100;

PID pid(motor_servo, Kp, Ki, Kd, integral_buffer);

// N point running average for measured speed
const int N_spd = 1;
volatile float speeds[N_spd] = {0};
volatile int speed_index = 0;
volatile float speed_avg = 0;

////// STEERING //////
Servo steering_servo;
int steering_angle = 70; // between 45 and 95, 45 max  right, 95 max left

// Steerig PID constants
volatile float s_Kp = 0.5;
volatile float s_Ki = 0.005;
volatile float s_Kd = -0.45;
const int s_integral_buffer = 20;

steeringPID s_pid(steering_servo, s_Kp, s_Ki, s_Kd, s_integral_buffer);



////// SONAR //////
// N point running average for distance
const int N_dist = 2;
//volatile float left_distance = 0;
volatile float right_distance = 0; 
volatile float forward_distance = 0;

//float left_distances[N_dist] = {0};
float right_distances[N_dist] = {0}; 
float forward_distances[N_dist] = {0};

//float left_avg = 0;
float right_avg = 0;
float forward_avg = 0;

//int left_index = 0;
int right_index = 0;
int forward_index = 0;



////// OTHER //////
int speed_ref = 0;
int old_speed_ref = 0;
int old_steering_angle = 70;

// user input
String input = "";
int int_input = 0;

//To stop steering while idle
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
    steering_servo.write(steering_angle);
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
    motor_servo.writeMicroseconds(u);

    right_avg = (getAverageDistance(RIGHT_PING_PIN, RIGHT_ECHO_PIN, right_distances, right_index, 2000));
    forward_avg = getAverageDistance(FORWARD_PING_PIN, FORWARD_ECHO_PIN, forward_distances, forward_index, 2000);
    right_avg = min(forward_avg, right_avg);

    pid.set_velocity(speed_ref);
    int angle = s_pid.update(current_time, left_avg, right_avg);
    if (running){
        steering_servo.write(angle);
    }

    // uppdaterar delta_time
    if (hall_update_flag) {
        updateDelta();
        hall_update_flag = false;
    } else {
        // if not updated the deltas are reset
        delta_time = 0;
        delta_hall_counter = 0;
    }

    if (current_time-last_time > 50) {
        current_vel = 0;
    }
    pid.update_velocity(current_vel)

    // written input
    if (Serial.available() > 0) { 
        input = Serial.readStringUntil('\n');
        speed_ref=input.toFloat();
        pid.set_velocity(speed_ref);
        running = true;
    }   
    // push, needs to be reset between to avoid false start
    if (current_vel > 0.1 && !running){
        speed_ref = 1;
        pid.set_velocity(speed_ref);
        running = true;
    }

    Serial.print("U_spd: ");
    Serial.print(u);
    Serial.print("\tR: ");
    Serial.print(right_avg);
    Serial.print("\tF: ");
    Serial.print(forward_avg);
    Serial.print("\tU_str: ");
    Serial.println(angle);
}


// write speed and angle, lab 4 
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

    if (delta_time > 0) {
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
    hall_update_flag=true;
    hall_counter++;
}