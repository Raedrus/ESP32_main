#include <Arduino.h>
#include <ESP32Servo.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <esp_system.h>

#define ESTOP_PIN 27
#define LED_PIN 2
#define EMAGNET_PIN 4
#define SERVO_PIN 26
#define LEDSTRIP_PIN 15
#define GREENLED_PIN 36
#define REDLED_PIN 39
#define STARTBUTTON_PIN 2

// TMC2209 stepper driver initiation
#define DIR_PIN 13           // Direction
#define STEP_PIN 12          // Step
#define STALL_PIN 11         // Connected to DIAG pin on the TMC2209
#define driver_ADDRESS 0b00 // Pins MS1 and MS2 connected to GND.
#define STALL_VALUE 100      // Stallguard values for each driver(0-255), higher number -> higher sensitivity.
#define RA_SENSE 0.11f       // Sense resistor value, match to your driverA
TMC2209Stepper driver(&Serial2, RA_SENSE, driver_ADDRESS);
constexpr uint32_t steps_per_round = 3000 * (60 / 16); // Calculation for steps per round required.
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);
bool startup = true; // set false after homing
bool stalled = false;

// Button debouncing variables
#define DEBOUNCE_TIME 40
volatile bool intflag = false;
unsigned long lastDebounceTime = 0;

// Function declarations
void IRAM_ATTR EstopInterrupt();
void IRAM_ATTR stallInterrupt();
void gripperOpen();
void gripperClose();

// EStop interrupt routine
void IRAM_ATTR EstopInterrupt()
{
  // Restart
  esp_restart();
}

// Stepper stall interrupt routine
void IRAM_ATTR stallInterrupt()
{
  stalled = true; // Stall flag set when stepper motor stalls
}

void gripperOpen()
{
  stepper.move(-100 * steps_per_round); // Move 100mm
  while (!stalled)
  {
    stepper.run();
  }
  stepper.setCurrentPosition(0); // Set current position as home
  stalled = false;
}

void gripperClose()
{
  stepper.move(100 * steps_per_round); // Move 100mm
  while (!stalled)
  {
    stepper.run();
  }
  stepper.setCurrentPosition(0); // Set current position as home
  stalled = false;
}

void setup()
{
  // Initiate serial comms with Pi 5
  Serial.begin(115200);

  // Initiate mode of IO pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(STARTBUTTON_PIN, INPUT_PULLUP);
  pinMode(EMAGNET_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(LEDSTRIP_PIN, OUTPUT);
  pinMode(GREENLED_PIN, OUTPUT);
  pinMode(REDLED_PIN, OUTPUT);

  // Enable interrupt for EStop Button
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), EstopInterrupt, RISING);

  // Initiate serial comms with TMC2209 stepper driver
  Serial2.begin(115200);

  // Enable interrupt for motor stall
  attachInterrupt(digitalPinToInterrupt(STALL_PIN), stallInterrupt, RISING);

  driver.begin();          // Initiate pins and registeries
  driver.rms_current(400); // Set stepperA current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver.pwm_autograd(1);  // Enable automatic gradient adaptation
  driver.pwm_autoscale(1);
  driver.microsteps(16);
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.SGTHRS(STALL_VALUE);

  stepper.setMaxSpeed(1.25 * steps_per_round);   // 100mm/s @ 80 steps/mm
  stepper.setAcceleration(10 * steps_per_round); // 2000mm/s^2
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();

  gripperOpen(); // Open the gripper
}

void loop()
{
  if (Serial.available() > 0)
  {                                             // Check presence of data at serial port
    String data = Serial.readStringUntil('\n'); // Read command from Pi
    Serial.println("OK");                       // Feedback to Pi on receiving command

    if (data == "EMAGNET_ON")
    {
      digitalWrite(EMAGNET_PIN, HIGH); // Turn on electromagnet cluster
    }
    else if (data == "EMAGNET_OFF")
    {
      digitalWrite(EMAGNET_PIN, LOW); // Turn off electromagnet cluster
    }
    else if (data == "L")
    {
      digitalWrite(LEDSTRIP_PIN, HIGH); // Turn on LED strip (example action)
    }
    else if (data == "S")
    {
      // Handle servo control (example action)
      Serial.println("Done");
    }
    else if (data == "G_OPEN")
    {
      gripperOpen(); // Open gripper
      Serial.println("Done");
    }
    else if (data == "G_CLOSE")
    {
      gripperClose(); // Close gripper
      Serial.println("Done");
    }
  }
}
