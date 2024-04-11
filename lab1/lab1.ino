volatile int c = 0;
const byte interruptPin = 2;
const byte ledPin = 3;
// test

volatile float last_time = 0;
volatile float current_time = 0;


volatile float freq = 1;
volatile float delaytime = 500/freq;
volatile int brightness = 255;


void setup() {
  // put your setup code here, to run once:
    pinMode(ledPin, OUTPUT);
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), blink, RISING);
    Serial.begin(9600);
    Serial.println("setup");
}


void loop() {
    if (Serial.available() > 0) { 
        // skriv 5f för att sätta freq till 5Hz
        // 200b för att sätta brightness till 200
        String input = Serial.readStringUntil('\n');
        Serial.println(input);
        if (input.endsWith("f")){
            float newfreq = input.toFloat();
            if (newfreq < 0){
                Serial.println("Can't have negative frequency >:(");
            } else {
                freq = newfreq;
                delaytime = 500/freq;
                Serial.print("Frequency set to: ");
                Serial.print(freq);
                Serial.println(" Hz");
            }
        } else if (input.endsWith("b")){
            float newbrightness = input.toFloat();
            if (newbrightness < 0){
                Serial.println("Can't have negative brightness >:(");
            } else {
                brightness = newbrightness;
                Serial.print("Brightness set to: ");
                Serial.println(brightness);
            }
        }
      Serial.read();
    }
    
    delay(delaytime);
    analogWrite(ledPin, brightness);
    delay(delaytime);
    analogWrite(ledPin, 0);
}


void blink(){
    current_time = millis();
    float time_since_last_blink = current_time-last_time;
    if (time_since_last_blink < delaytime){
        return;
    }
    c++;
    Serial.print("Blink count: ");
    Serial.print(c);
    Serial.print("\t\Time since last blink: ");
    Serial.print(time_since_last_blink);
    Serial.println(" ms");
    last_time = current_time;
}