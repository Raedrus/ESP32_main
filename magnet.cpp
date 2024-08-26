#include <Arduino.h>
#include <ESP32Servo.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <esp_system.h>

#define EMAGNET_PIN 27

void setup()
{
  // Initiate serial comms with Pi 5
  Serial.begin(115200);
  pinMode(EMAGNET_PIN, OUTPUT);}

void loop()
{
  while (1)
  {
    /* code */
    Serial.println("Initiating MAGNET Test...");
    digitalWrite(EMAGNET_PIN, HIGH);
    delay(10000); // Delay for 10 sec
    Serial.println("Off MAGNET Test...");
    digitalWrite(EMAGNET_PIN, LOW);
    delay(10000); // Delay for 10 sec
    Serial.println("Check Done");
  }}