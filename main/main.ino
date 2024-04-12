#include <Servo.h>

////// Constants //////
// Pins
const byte HALL_INTERRUPT_PIN = 12;
const byte MOTOR_PIN = 8;
const byte SERVO_PIN = 5;

// Wheel circumference in meters
const float WHEEL_CIRCUMFERENCE = 0.2042; 


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
bool update_flag = false;

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
    // uppdaterar delta_time
    if (update_flag) {
      updateTime()
      update_flag = false
    }

    // printar velocity
    if (delta_time > 0) {
        v = getVelocity()
        Serial.print(v);
        Serial.println(" m/s");
        delta_time = 0; // Tveksam på denna, tekniskt sett ska nog inte print funktionen återställa delta_time
    }

    if (Serial.available() > 0) { 
        input = Serial.readStringUntil('\n');
        int_input = input.toInt();
        Serial.println(input);
        Serial.println(int_input);

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

// Updaterar delta_time
void updateTime(){
  current_time = millis();
  delta_time = current_time-last_time;
  last_time = current_time;
}

// Returnerar Velocity basarat på delta_time
void getVelocity(){
    delta_time /= 1000;
    current_vel = WHEEL_CIRCUMFERENCE/delta_time;
    return current_vel
}

void hallISR(){
    update_flag=true
    hall_counter++;
    /*
    flag = true;
    if (flag){
      Serial.println(hall_counter);
      flag = false;
    }*/
    
    
}