#include <Servo.h>

////// Constants //////
// Pins
const byte HALL_INTERRUPT_PIN = 12;
const byte MOTOR_PIN = 8;
const byte SERVO_PIN = 5;

// Wheel circumference in meters
const float wheel_circumference = 0.2042; 


// Counter for hall interrupts
volatile int hall_counter = 0;     

// Motor and steering servos 
Servo motor_servo;
Servo steering_servo;

// Angle and "speed" to send to motor servo and steering servo
int motor_speed = 1500;  // 1500 stilla, 2000 max frammåt, 1000 max backåt
int steering_angle = 70; // mellan 40 och 100 typ


// Forward speed computed by encoder
float current_vel = 0;

// Time between pulses, used to calculate speed
volatile float last_time = 0;
volatile float current_time = 0;
volatile float delta_time = 0;

// NOT CURRENTLY IN USE
bool flag = true;

// user input...
String input = "";
int int_input = 0;

void setup() {

    Serial.begin(9600);
    
    pinMode(HALL_INTERRUPT_PIN, INPUT_PULLUP);
    pinMode(MOTOR_PIN, OUTPUT);
    steering_servo.attach(SERVO_PIN);

    attachInterrupt(digitalPinToInterrupt(HALL_INTERRUPT_PIN), hallISR, RISING);
    

    motor_servo.attach(MOTOR_PIN, 1000, 2000);


    //Serial.println("setup");
    //motor_servo.writeMicroseconds(motor_speed);
    //delay(3000);
}

void loop() {
    if (Serial.available() > 0) { 
        input = Serial.readStringUntil('\n');
        int_input = input.toInt();
        Serial.println(input);
        Serial.println(int_input);
        //Serial.print(int_input);
        // change motor speed
        if (int_input > 1000 && int_input < 2000) {
            motor_speed = int_input;
            Serial.print("Set motorspeed to ");
            Serial.println(motor_speed);
        } //change steering angle
        else if (int_input > 40 && int_input < 100) {
            steering_angle = int_input;
            Serial.print("Set steering angle to ");
            Serial.println(steering_angle);
        }
      
    }
    
    
    //steering_servo.write(steering_angle);
    //motor_servo.writeMicroseconds(motor_speed);
    
    
    
    
    
    
    
  
}

// Borde flytta typ allt utom counter++ till loop(), millis + print är slow af
void hallISR(){
    current_time = millis(); // ajabaja
    delta_time = current_time-last_time;
    hall_counter++;
    last_time = current_time;
    flag = true;
    /*
    if (flag){
      Serial.println(hall_counter);
      flag = false;
    }*/
    delta_time /= 1000;
    current_vel = wheel_circumference/delta_time;
    
    if (delta_time > 0) {
        Serial.print(current_vel); //aja baja
        Serial.println(" m/s");
        delta_time = 0;
    }
}