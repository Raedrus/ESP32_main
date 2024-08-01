#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <esp_system.h>

// TMC2209 stepper driver initiation
#define DIR_PIN 23          // Direction
#define STEP_PIN 22         // Step
#define STALL_PIN 21        // Connected to DIAG pin on the TMC2209
#define driver_ADDRESS 0b00 // Pins MS1 and MS2 connected to GND.
#define STALL_VALUE 3 // Stallguard values for each driver(0-255), higher number -> higher sensitivity.
#define RA_SENSE 0.11f      // Sense resistor value, match to your driverA
TMC2209Stepper driver(&Serial2, RA_SENSE, driver_ADDRESS);
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);
bool startup = true; // set false after homing
bool stalled = false;



void TMC2209settings();

// Stepper stall interrupt routine
void IRAM_ATTR stallInterrupt()
{
  stalled = true; // Stall flag set when stepper motor stalls
}

// TMC2209 Reset
void resetDriver()
{
  stalled = false;

  // Disable the driver
  driver.toff(0);
  delay(10); // Small delay to ensure the driver is properly disabled

  // Reinitialize the driver settings
  driver.toff(5); // Enable driver
  TMC2209settings();
}

void TMC2209settings()
{
  driver.begin();          // Initiate pins and registeries
  driver.rms_current(300); // Set stepperA current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver.pwm_autograd(1);  // Enable automatic gradient adaptation
  driver.pwm_autoscale(1);
  driver.microsteps(32);
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.SGTHRS(STALL_VALUE);

  stepper.setMaxSpeed(10000);
  stepper.setAcceleration(2000); // 2000mm/s^2
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();
}

// Function to open the gripper
void gripperOpen()
{
  resetDriver();
  stepper.setSpeed(-5000);
  while (!stalled)
  {
    stepper.runSpeed();
  }
  stepper.stop();
  stepper.setCurrentPosition(0); // Set current position as home
}

// Function to close the gripper
void gripperClose()
{
  resetDriver();
  stepper.setSpeed(5000);
  while (!stalled)
  {
    stepper.runSpeed();
  }
  stepper.stop();
  stepper.setCurrentPosition(0); // Set current position as home
}

void setup()
{
  Serial.begin(115200); // Initiate serial monitor
  while (!Serial) {
    ; // Wait for the serial port to connect. Needed for native USB port only
  }

  // Initiate serial comms with TMC2209 stepper driver
  Serial2.begin(115200);

  pinMode(DIR_PIN,OUTPUT);
  pinMode(STEP_PIN,OUTPUT);
  // Enable interrupt for motor stall
  attachInterrupt(digitalPinToInterrupt(STALL_PIN), stallInterrupt, RISING);

  driver.toff(0);
  delay(10); // Small delay to ensure the driver is properly disabled
  driver.toff(5); // Enable driver
  TMC2209settings(); // Initialize TMC2209 Stepper Driver

  Serial.println("Setup complete. Enter 'open' or 'close' to control the gripper.");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any leading/trailing whitespace
    Serial.print("Received command: ");
    Serial.println(input);
    if (input == "open") {
      Serial.println("Opening gripper...");
      Serial.println("Value of SG_RESULT");
      Serial.println(driver.SG_RESULT());
      gripperOpen();
      Serial.println("Gripper opened.");
    } else if (input == "close") {
      Serial.println("Closing gripper...");
      Serial.println("Value of SG_RESULT");
      Serial.println(driver.SG_RESULT());
      gripperClose();
      Serial.println("Gripper closed.");
    } else {
      Serial.println("Invalid command. Please enter 'open' or 'close'.");
    }
  }

}
