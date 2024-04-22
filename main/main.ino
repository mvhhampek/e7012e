#include <Servo.h>

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
volatile float Kp = 10;
volatile float Ki = 8;
volatile float Kd = 3;

// Angle and "speed" to send to motor servo and steering servo
int motor_speed = 1500;  // 1500 stilla, 2000 max frammåt, 1000 max backåt
int steering_angle = 70; // mellan 40 och 100 typ

// Forward speed computed by encoder
float current_vel = 0;

// For PID
float last_error = 0;
float current_error = 0;
float integral_error = 0;
float delta_error = 0;
float ref = 1;
int u = 0;

// Time between pulses, used to calculate speed
volatile int last_time = 0;
volatile int current_time = 1;
volatile int delta_time = 0;
volatile int last_time_error = 0;
volatile int delta_time_error = 0;

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
    updateDelta();
}

void loop() {
    current_time = millis();
    delay(1);
    if (current_time%10==0) {
        updateDelta();
    }
    if (current_time%1000==0) {
        Serial.print(current_vel);
        Serial.println(" m/s");
        Serial.print(u);
        Serial.println(" u");
        Serial.print(current_error);
        Serial.println(" error");
        Serial.print(integral_error);
        Serial.println(" integral error");
        Serial.print(delta_error);
        Serial.println(" delta error");
        Serial.println(" ");
    }
    calcU();
    motor_servo.writeMicroseconds(u);
    // uppdaterar delta_time
    //if (update_flag) {
    //  update_flag = false;
    //} else {
    //  // if not updated the deltas are reset
    //  delta_time = 0;
    //  delta_hall_counter = 0;
    //}

    // printar velocity
    if (delta_time > 0) {
        //Serial.print(current_vel);
        //Serial.println(" m/s");
        //Serial.print(u);
        //Serial.println(" u");
        //Serial.print(current_error);
        //Serial.println(" error");
        //Serial.print(integral_error);
        //Serial.println(" integral error");
        //Serial.print(delta_error);
        //Serial.println(" delta error");
        //Serial.println(" ");
        
        //delta_time = 0; // Tveksam på denna, tekniskt sett ska nog inte print funktionen återställa delta_time // Flyttad upp till if(update_flag)
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
            motor_servo.writeMicroseconds(motor_speed);
            delay(50);
        } //change steering angle
        else if (int_input > 40 && int_input < 100) {
            steering_angle = int_input;
            Serial.print("Set steering angle to ");
            Serial.println(steering_angle);
            delay(50);
        } //change ref
        else if (int_input > -5 && int_input < 5) {
            ref = int_input;
            Serial.print("Set ref to ");
            Serial.println(ref);
            delay(50);
        }
        
        steering_servo.write(steering_angle);
    }

    delta_time_error = current_time - last_time_error;
    delta_error = -(current_error-last_error)/(delta_time_error/1000);
    integral_error += current_error * (delta_time_error/1000);  
    last_time_error = current_time; 
    
}


// Updaterar delta_time
void updateDelta(){
  // updates delta_time
  delta_time = current_time-last_time;
  last_time = current_time;

  // updates delta_hall_counter
  delta_hall_counter = hall_counter-last_hall_counter;
  last_hall_counter = hall_counter;

  current_vel = getVelocity();

  float last_error = current_error;
  current_error = ref-current_vel;
  

}


// Calculates U between -200 and 200
void calcU() {
  u = Kp*current_error+Ki*integral_error+Kd*delta_error+1500;
  if (u<1300) {
    u=1300;
  }
  if (u>1700) {
    u=1700;
  }
  motor_servo.writeMicroseconds(u);
}

// Returnerar Velocity basarat på delta_time
float getVelocity(){
    float distance = (delta_hall_counter*WHEEL_CIRCUMFERENCE)/NUM_MAGNETS;
    current_vel = ((distance/(delta_time/1000))+current_vel)/2;
    return current_vel;
}

void hallISR(){
    update_flag=true;
    hall_counter++;
    /*
    flag = true;
    if (flag){
      Serial.println(hall_counter);
      flag = false;
    }*/
}