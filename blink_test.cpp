#include <Arduino.h>

#define LED 2
void blink();

void setup()
{
    pinMode(LED, OUTPUT);
}

void loop()
{
    blink();
}

void blink()
{
    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, LOW);
    delay(1000);
}