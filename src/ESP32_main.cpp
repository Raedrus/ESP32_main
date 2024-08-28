#include <Arduino.h>
#include <ESP32Servo.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <esp_system.h>

#define ESTOP_PIN 13
#define LED_PIN 2
#define EMAGNET_PIN 27
#define LEDSTRIP_PIN 15
#define GREENLED_PIN 25
#define REDLED_PIN 26
#define STARTBUTTON_PIN 2

// TMC2209 stepper driver initiation
#define DIR_PIN 2          // Direction
#define STEP_PIN 4         // Step
#define STALL_PIN 5        // Connected to DIAG pin on the TMC2209
#define driver_ADDRESS 0b00 // Pins MS1 and MS2 connected to GND.
#define STALL_VALUE 100     // Stallguard values for each driver(0-255), higher number -> higher sensitivity.
#define RA_SENSE 0.11f      // Sense resistor value, match to your driverA
TMC2209Stepper driver(&Serial2, RA_SENSE, driver_ADDRESS);
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);
bool startup = true; // set false after homing
bool stalled = false;

// Servo initiation
Servo lid_servo;
Servo gate_servo;
int lid_init_pos = 90;
int gate_init_pos = 100;
int gate_open_pos = 23;
#define LID_SERVO_PIN 12
#define GATE_SERVO_PIN 13

// Button debouncing variables
#define DEBOUNCE_TIME 40
volatile bool intflag = false;
unsigned long lastDebounceTime = 0;

// Function declarations
void TestLoop();
void TMC2209settings();
// void IRAM_ATTR EstopInterrupt();
// void IRAM_ATTR stallInterrupt();
// void gripperOpen();
// void gripperClose();

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

// Function to open the gripper
void gripperOpen()
{
  resetDriver();
  stepper.setSpeed(-1000);
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
  stepper.setSpeed(1000);
  while (!stalled)
  {
    stepper.runSpeed();
  }
  stepper.stop();
  stepper.setCurrentPosition(0); // Set current position as home
}

// Setup
void setup()
{
  // Initiate serial comms with Pi 5
  Serial.begin(115200);

  // Initiate mode of IO pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(STARTBUTTON_PIN, INPUT_PULLUP);
  pinMode(EMAGNET_PIN, OUTPUT);
  pinMode(LEDSTRIP_PIN, OUTPUT);
  pinMode(GREENLED_PIN, OUTPUT);
  pinMode(REDLED_PIN, OUTPUT);

  // Enable interrupt for EStop Button
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), EstopInterrupt, RISING);

  // Initiate serial comms with TMC2209 stepper driver
  Serial2.begin(115200);

  // Enable interrupt for motor stall
  attachInterrupt(digitalPinToInterrupt(STALL_PIN), stallInterrupt, RISING);

  TMC2209settings(); // Initialize TMC2209 Stepper Driver

  gripperOpen(); // Open the gripper

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  lid_servo.setPeriodHertz(50);               // Set period for servo
  lid_servo.attach(LID_SERVO_PIN, 500, 2400); // Attaches the servo at pin 26 to its servo object
  lid_servo.write(lid_init_pos);              // Moves the servo to desired home position

  gate_servo.setPeriodHertz(50);                // Set period for servo
  gate_servo.attach(GATE_SERVO_PIN, 500, 2400); // Attaches the servo pin 25 to its servo object
  gate_servo.write(gate_init_pos);              // Moves the servo to desired home position
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
}

void TMC2209settings()
{
  driver.begin();          // Initiate pins and registeries
  driver.rms_current(400); // Set stepperA current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver.pwm_autograd(1);  // Enable automatic gradient adaptation
  driver.pwm_autoscale(1);
  driver.microsteps(16);
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.SGTHRS(STALL_VALUE);

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500); // 2000mm/s^2
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();
}
void loop()
{

  if (Serial.available() > 0)
  {                                             // Check presence of data at serial port
    String data = Serial.readStringUntil('\n'); // Read command from Pi
    Serial.println("OK");                       // Feedback to Pi on receiving command

    if (data == "EMAGNET_ON") // Turn on electromagnet cluster
    {
      digitalWrite(EMAGNET_PIN, HIGH);
    }
    else if (data == "EMAGNET_OFF") // Turn off electromagnet cluster
    {
      digitalWrite(EMAGNET_PIN, LOW);
    }
    else if (data == "LIGHTS_ON") // Turn on LED strip
    {
      digitalWrite(LEDSTRIP_PIN, HIGH);
    }
    else if (data == "LIGHTS_OFF") // Turn off LED strip
    {
      digitalWrite(LEDSTRIP_PIN, LOW);
    }
    else if (data == "LID_OPEN") // Open the input lid
    {
      lid_servo.write(90); // Position can be adjusted as desired
      Serial.println("Done");
    }
    else if (data == "LID_CLOSE") // Close the input lid
    {
      lid_servo.write(90); // Position can be adjusted as desired
      Serial.println("Done");
    }
    else if (data == "GATE_OPEN") // Open the gate
    {
      gate_servo.write(90); // Position can be adjusted as desired
      Serial.println("Done");
    }
    else if (data == "GATE_CLOSE") // Close the gate
    {
      gate_servo.write(90); // Position can be adjusted as desired
      Serial.println("Done");
    }
    else if (data == "G_OPEN") // Open gripper
    {
      gripperOpen();
      Serial.println("Done");
    }
    else if (data == "G_CLOSE") // Close gripper
    {
      gripperClose();
      Serial.println("Done");
    }
    else if (data == "GLED_ON") // Turn on Green LED
    {
      digitalWrite(GREENLED_PIN, HIGH);
    }
    else if (data == "GLED_OFF") // Turn off Green LED
    {
      digitalWrite(GREENLED_PIN, LOW);
    }
    else if (data == "RLED_ON") // Turn on Red LED
    {
      digitalWrite(REDLED_PIN, HIGH);
    }
    else if (data == "RLED_OFF") // Turn off Red LED
    {
      digitalWrite(REDLED_PIN, LOW);
    }

    if (data=="Test")
    {
      Serial.print("Testing Mode. Input test command or exit: ");
      TestLoop();
    }
    else
    {
      Serial.println("Invalid input");
    }
  }
}

void TestLoop()
{
  int timeout_counter = 0;
  
  while (true)
  {
    
    delay(10); // Delay 10ms
    
    if (Serial.available() > 0)
    {
      String data = Serial.readStringUntil('\n'); // Read test command

      if (data == "EMAGNET")
      {
        // code for turning on Magnet for 5 sec
        Serial.println("Initiating MAGNET Test...");
        digitalWrite(EMAGNET_PIN, HIGH);
        delay(5000); // Delay for 5 sec
        digitalWrite(EMAGNET_PIN, LOW);
        Serial.println("Check Done");

        TestLoop(); // Go back to start of TestLoop
      }

      else if (data == "LED")
      {
        // code for turning on LED for 5 sec
        Serial.println("Initiating LED Strip Test...");
        digitalWrite(LEDSTRIP_PIN, HIGH);
        delay(5000); // Delay for 5 sec
        digitalWrite(LEDSTRIP_PIN, LOW);
        Serial.println("Check Done");

        TestLoop(); // Go back to start of TestLoop
      }

      else if (data == "GLED")
      {
        // code for turning on Green LED for 5 sec
        Serial.println("Initiating Green LED Test...");
        digitalWrite(GREENLED_PIN, HIGH);
        delay(5000); // Delay for 5 sec
        digitalWrite(GREENLED_PIN, LOW);
        Serial.println("Check Done");

        TestLoop(); // Go back to start of TestLoop
      }

      else if (data == "RLED")
      {
        // code for turning on Red LED for 5 sec
        Serial.println("Initiating Red LED Test...");
        digitalWrite(REDLED_PIN, HIGH);
        delay(5000); // Delay for 5 sec
        digitalWrite(REDLED_PIN, LOW);
        Serial.println("Check Done");

        TestLoop(); // Go back to start of TestLoop
      }

      else if (data == "LID")
      {
        // Wait for servo position input, max limit 100!!! should be adjusted based on practical use
        Serial.print("Initiating LID servo Test...");

        lid_servo.write(lid_init_pos); // Home Position
        int lid_servoPos;              // Current Position
        lid_servoPos = lid_init_pos;

        while (true)
        {
          delay(15);

          // code for rotating LID servo
          if (Serial.available() > 0)
          {
            String info_servo;
            info_servo = "";
            String return_text;
            info_servo = Serial.readStringUntil('\n'); // read input integer

            if (info_servo.toInt() < 0 || info_servo.toInt() > 100) // setting input limit
            {
              Serial.print("Out of limit (max 100)");
              continue;
            }


            // return the info variable for confirmation
            return_text = "info_servo variable string is: " + info_servo;
            Serial.println(return_text);

            // rotate servo to desired position
            if (info_servo.toInt() < lid_servoPos)
            {
              for (0; lid_servoPos > info_servo.toInt(); lid_servoPos -= 1)
              {
                lid_servo.write(lid_servoPos);
                delay(5);
              }
            }
            else if (info_servo.toInt() > lid_servoPos)
            {
              for (0; lid_servoPos < info_servo.toInt(); lid_servoPos += 1)
              {
                lid_servo.write(lid_servoPos);
                delay(5);
              }
            }

            else
            {
              delay(5);
              Serial.println("Invalid, try again");
              continue;
            }
            break;
          }
        }

        delay(2500); // stop for 2.5 sec

        // rotate back to Home Position
        if (lid_init_pos < lid_servoPos)
        {
          for (0; lid_servoPos > lid_init_pos; lid_servoPos -= 1)
          {
            lid_servo.write(lid_servoPos);
            delay(5);
          }
        }
        else if (lid_init_pos > lid_servoPos)
        {
          for (0; lid_servoPos < lid_init_pos; lid_servoPos += 1)
          {
            lid_servo.write(lid_servoPos);
            delay(5);
          }
        }

        Serial.println("Check Done");
      }

      else if (data == "GATE")
      {
        // Wait for servo position input, max limit 100!!! should be adjusted based on practical use
        Serial.print("Initiating GATE servo Test...");

        gate_servo.write(gate_init_pos); // Home Position
        int gate_servoPos;               // Current Position
        gate_servoPos = gate_init_pos;

        while (true)
        {
          delay(15);

          // code for rotating Gate servo
          if (Serial.available() > 0)
          {
            String info_servo;
            info_servo = "";
            String return_text;
            info_servo = Serial.readStringUntil('\n'); // read input integer

            if (info_servo.toInt() < 0 || info_servo.toInt() > 100) // setting input limit
            { 
              Serial.print("Out of limit (max 100)");
              continue;
            }
            

            // return the info variable for confirmation
            return_text = "info_servo variable string is: " + info_servo;
            Serial.println(return_text);

            // rotate servo to desired position
            if (info_servo.toInt() < gate_servoPos)
            {
              for (0; gate_servoPos > info_servo.toInt(); gate_servoPos -= 1)
              {
                gate_servo.write(gate_servoPos);
                delay(5);
              }
            }
            else if (info_servo.toInt() > gate_servoPos)
            {
              for (0; gate_servoPos < info_servo.toInt(); gate_servoPos += 1)
              {
                gate_servo.write(gate_servoPos);
                delay(5);
              }
            }
            else
            {
              delay(5);
            }
            break;
          }
        }

        delay(2500); // stop for 2.5 sec

        // rotate back to Home Position
        if (gate_init_pos < gate_servoPos)
        {
          for (0; gate_servoPos > gate_init_pos; gate_servoPos -= 1)
          {
            gate_servo.write(gate_servoPos);
            delay(5);
          }
        }
        else if (gate_init_pos > gate_servoPos)
        {
          for (0; gate_servoPos < gate_init_pos; gate_servoPos += 1)
          {
            gate_servo.write(gate_servoPos);
            delay(5);
          }
        }

        Serial.println("Check Done");
      }

      else if (data == "exit" || data == "Exit" || data == "EXIT")
      {
        Serial.println("Exit Testing Mode");
        break;
      }
      else
      {
        Serial.print("Invalid Test Command. Enter again: ");
        TestLoop(); // Restart TestLoop
      }
    }

  }
}
