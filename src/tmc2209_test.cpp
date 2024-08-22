#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <esp_system.h>

// TMC2209 stepper driver initiation
#define DIR_PIN 2          // Direction
#define STEP_PIN 4        // Step
#define STALL_PIN 18        // Connected to DIAG pin on the TMC2209
#define EN_PIN 5
#define driver_ADDRESS 0b00 // Pins MS1 and MS2 connected to GND.
<<<<<<< HEAD:src/tmc2209_test.cpp
#define STALL_VALUE 58
// Stallguard values for each driver(0-255), higher number -> higher sensitivity.
=======
#define STALL_VALUE 3 // Stallguard values for each driver(0-255), higher number -> higher sensitivity.
>>>>>>> a74aea64580745815c883f54718ad9d983d47040:tmc2209_test.cpp
#define RA_SENSE 0.11f      // Sense resistor value, match to your driverA
TMC2209Stepper driver(&Serial2, RA_SENSE, driver_ADDRESS);
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

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
<<<<<<< HEAD:src/tmc2209_test.cpp
  driver.rms_current(1800); // Set stepperA current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver.pwm_autograd(1);  // Enable automatic gradient adaptation
  // driver.pwm_autoscale(1);
  driver.microsteps(4);
=======
  driver.rms_current(300); // Set stepperA current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver.pwm_autograd(1);  // Enable automatic gradient adaptation
  driver.pwm_autoscale(1);
  driver.microsteps(32);
>>>>>>> a74aea64580745815c883f54718ad9d983d47040:tmc2209_test.cpp
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
  digitalWrite(EN_PIN,LOW);
  resetDriver();
  stepper.enableOutputs();
  stepper.setSpeed(-500);
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
  digitalWrite(EN_PIN,LOW);
  resetDriver();
  stepper.enableOutputs();
  stepper.setSpeed(500);
  while (!stalled)
  {
    stepper.runSpeed();
  }
  
  int i = 0;
  while (i<15)
  {
  stepper.runSpeed();
  delay(5);
  i++;
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

<<<<<<< HEAD:src/tmc2209_test.cpp
  driver.pdn_disable(1);
  driver.ihold(5);

  pinMode(DIR_PIN,OUTPUT);
  pinMode(STEP_PIN,OUTPUT);
  pinMode(EN_PIN,OUTPUT);
=======
  pinMode(DIR_PIN,OUTPUT);
  pinMode(STEP_PIN,OUTPUT);
>>>>>>> a74aea64580745815c883f54718ad9d983d47040:tmc2209_test.cpp
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
      Serial.println("Value of pdn_uart");
      Serial.println(driver.pdn_uart());
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
