#include <Arduino.h>
#define ESTOP_PIN 27
#define LED_PIN 2
#define DEBOUNCE_TIME  40
#define EMAGNET_PIN 4
#define SERVO_PIN 26
#define LEDSTRIP_PIN 15
#define GREENLED_PIN 36
#define REDLED_PIN 39
#define STARTBUTTON_PIN 2
volatile bool intflag = false;
unsigned long lastDebounceTime = 0;

//EStop interrupt routine
void IRAM_ATTR EstopInterrupt() {
  //Restart
  esp_restart();

}

void setup() {
  //Initiate serial comms with Pi 5
  Serial.begin(115200);

  //Initiate mode of IO pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(STARTBUTTON_PIN, INPUT_PULLUP);
  pinMode(EMAGNET_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(LEDSTRIP_PIN, OUTPUT);
  pinMode(GREENLED_PIN, OUTPUT);
  pinMode(REDLED_PIN, OUTPUT);

  //Enable interrupt for EStop Button
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), EstopInterrupt, RISING);
  
}

void loop() {
  if (Serial.available() > 0) {                 //Check presence of data at serial port
    String data = Serial.readStringUntil('\n'); //Read command from Pi
    Serial.println("OK");                       //Feedback to Pi on receiving command
  switch(data){                                 //Take action depending on command received
    case "EMAGNET_ON":
      digitalWrite(EMAGNET_PIN, HIGH);          //Turn on electromagnet cluster
      break;
    case "EMAGNET_OFF":
      digitalWrite(EMAGNET_PIN, LOW);          //Turn off electromagnet cluster
      break;
    case "L":
      digitalWrite(LEDSTRIP_PIN, HIGH);
      break;
    case "S":
      //goto servo control
      Serial.println("Done");
      break;
    case "M":
      //goto stepper control
      Serial.println("Done");
      break;  
  }
  
}
}
