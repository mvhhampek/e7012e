#include <Servo.h>
#include <PID.h>

////// Constants //////
// Pins
const byte HALL_INTERRUPT_PIN = 12;
const byte MOTOR_PIN = 8;
const byte SERVO_PIN = 5;

// Wheel circumference in meters
const float WHEEL_CIRCUMFERENCE = 0.2042; 
const int  NUM_MAGNETS = 4;


// Counter for hall interrupts
volatile int hall_counter = 0;     
volatile int last_hall_counter = 0;
volatile int delta_hall_counter = 0;

// Motor and steering servos 
Servo motor_servo;
Servo steering_servo;

// PID constants
volatile float Kp = 1.4;
volatile float Ki = 2.2;
volatile float Kd = 0.7;

PID pid(motor_servo, Kp, Ki, Kd);


// Angle and "speed" to send to motor servo and steering servo
int motor_speed = 1500;  // 1500 stilla, 2000 max frammåt, 1000 max backåt
int steering_angle = 70; // mellan 40 och 100 typ

// Forward speed computed by encoder
float current_vel = 0;

// Time between pulses, used to calculate speed
volatile int last_time = 0;
volatile int current_time = 1;
volatile int delta_time = 0;

// NOT CURRENTLY IN USE
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
    steering_servo.write(50);
    motor_servo.writeMicroseconds(motor_speed);
    delay(1000);    
    motor_servo.writeMicroseconds(1500);
    delay(1000);
}

void loop() {
    delay(5);
    current_time = millis();

    int u = pid.update(current_time);
      //Serial.print(u);
      //Serial.println(" u");
    motor_servo.writeMicroseconds(u);

    if (current_time%1000==0) {
        Serial.print(current_vel);
        Serial.println(" m/s");
    }

    // uppdaterar delta_time
    if (update_flag) {
      updateDelta();
      update_flag = false;
    } else {
      // if not updated the deltas are reset
      delta_time = 0;
      delta_hall_counter = 0;
    }

    //Serial.print(current_time-last_time);
    //Serial.println(" delta");
    //Serial.print((WHEEL_CIRCUMFERENCE)/(NUM_MAGNETS*0.05));
    //Serial.println(" inte delta");

    if (current_time-last_time > 50) {
      current_vel = 0;
      pid.update_velocity(current_vel);
    }

    // printar velocity
    if (delta_time > 0) {
        Serial.print(current_vel);
        Serial.println(" m/s");
      }

    if (Serial.available() > 0) { 
        input = Serial.readStringUntil('\n');
        int_input = input.toInt();
        Serial.println(input);
        Serial.println(int_input);

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
            pid.set_velocity(int_input);
            Serial.print("Set ref to ");
        }  
    }    
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

    //Serial.print(delta_hall_counter);
    //Serial.println(" delta_hall_counter");
    //Serial.print(WHEEL_CIRCUMFERENCE);
    //Serial.println(" WHEEL_CIRCUMFERENCE");
    //Serial.print(NUM_MAGNETS);
    //Serial.println(" NUM_MAGNETS");
    //Serial.print(distance);
    //Serial.println(" distance");
    //Serial.print(current_vel);
    //Serial.println(" current_vel");
    //Serial.print(delta_time);
    //Serial.println(" delta_time");

    current_vel = (((distance*1000)/(delta_time))+current_vel)/2;
    return current_vel;
}

void hallISR(){
    update_flag=true;
    hall_counter++;
}