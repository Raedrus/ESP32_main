#include <Arduino.h>
#include <FreeRTOSConfig.h>

TaskHandle_t Pi_Serial_Handler; // Manages incoming serial data
TaskHandle_t LED_Handler;       // Check Temp + Climate Control

#define ESTOP_PIN 32
#define EMAGNET_PIN 27
#define LEDSTRIP_PIN 14
#define GREENLED_PIN 25
#define REDLED_PIN 26
#define LID_LIMIT_PIN 15
#define VAC_MOTOR_PIN 33

//----------TMC2209 stepper driver initiation---------//
#define DIR_PIN 2    // Direction
#define STEP_PIN 4   // Step
#define STALL_PIN 18 // Connected to DIAG pin on the TMC2209
#define EN_PIN 5
#define driver_ADDRESS 0b00 // Pins MS1 and MS2 connected to GND.
#define STALL_VALUE 150     // Stallguard values for each driver(0-255), higher number -> higher sensitivity.
#define RA_SENSE 0.11f      // Sense resistor value, match to your driverA
TMC2209Stepper driver(&Serial2, RA_SENSE, driver_ADDRESS);
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);
bool startup = true; // set false after homing
bool stalled = false;

//-------------------Servo initiation--------------------//
Servo lid_servo;
Servo gate_servo;
Servo plat_servo;

//  Lid servo settings
int lid_init_pos = 90;
#define LID_SERVO_PIN 12

//  Gate servo settings
#define GATE_SERVO_PIN 13
int gate_init_pos = 100;
int gate_open_pos2 = 23;
int gate_open_pos1 = 45;

//  Platform servo settings
#define PLAT_SERVO_PIN 14
int plat_neutral_pos = 100;
int plat_left_pos =

// Button debouncing variables
#define DEBOUNCE_TIME 40
    volatile bool intflag = false;
unsigned long lastDebounceTime = 0;

int i = 0;
String data;

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
    vTaskDelay(10); // Small vTaskDelay to ensure the driver is properly disabled

    // Reinitialize the driver settings
    driver.toff(5); // Enable driver
    TMC2209settings();
}

// Function to open the gripper
void gripperOpen()
{
    //   resetDriver();
    //   i=0;
    //   stepper.setSpeed(1000000);
    //   while (i<2500)
    //   {
    //     stepper.runSpeed();
    //     i++;
    //     vTaskDelay(1);
    //   }
    //   stepper.stop();
    //   stepper.setCurrentPosition(0); // Set current position as home

    resetDriver();

    stepper.setSpeed(1000000);
    while (Serial.available() == 0)
    {
        stepper.runSpeed();
        vTaskDelay(1);
    }
    stepper.stop();
    stepper.setCurrentPosition(0); // Set current position as home

    data = Serial.readStringUntil('\n'); // Read command from Pi
    Serial.println("OK");
}

// Function to close the gripper
void gripperClose()
{
    resetDriver();
    stepper.setSpeed(-1000000);
    i = 0;
    // Serial.print("close");
    while (i < 6000)
    {
        stepper.runSpeed();
        i++;
        vTaskDelay(1);
    }
    stepper.stop();
    stepper.setCurrentPosition(0); // Set current position as home

    //   resetDriver();
    //   stepper.setSpeed(-500000);
    //   while (!stalled)
    //   {
    //     stepper.runSpeed();
    //     Serial.println(driver.SG_RESULT());
    //   }
    //   stepper.stop();
    //   stepper.setCurrentPosition(0); // Set current position as home
}

// Setup
void setup()
{
    // Initiate serial comms with Pi 5
    Serial.begin(115200);

    // Initiate mode of IO pins
    pinMode(ESTOP_PIN, INPUT_PULLUP);
    pinMode(LID_LIMIT_PIN, INPUT_PULLUP);
    pinMode(EMAGNET_PIN, OUTPUT);
    pinMode(LEDSTRIP_PIN, OUTPUT);
    pinMode(GREENLED_PIN, OUTPUT);
    pinMode(REDLED_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);

    // Enable interrupt for EStop Button
    attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), EstopInterrupt, FALLING);

    // Initiate serial comms with TMC2209 stepper driver
    Serial2.begin(115200);

    // Enable interrupt for motor stall
    attachInterrupt(digitalPinToInterrupt(STALL_PIN), stallInterrupt, RISING);

    TMC2209settings(); // Initialize TMC2209 Stepper Driver

    // Allow allocation of all timers
    // ESP32PWM::allocateTimer(0);
    // ESP32PWM::allocateTimer(1);
    // ESP32PWM::allocateTimer(2);
    // ESP32PWM::allocateTimer(3);
    lid_servo.setPeriodHertz(50);               // Set period for servo
    lid_servo.attach(LID_SERVO_PIN, 500, 2400); // Attaches the servo at pin 12 to its servo object
    lid_servo.write(lid_init_pos);              // Moves the servo to desired home position

    gate_servo.setPeriodHertz(50);                // Set period for servo
    gate_servo.attach(GATE_SERVO_PIN, 500, 2400); // Attaches the servo pin 13 to its servo object
    gate_servo.write(gate_init_pos);              // Moves the servo to desired home position

    plat_servo.setPeriodHertz(50);                // Set period for servo
    plat_servo.attach(PLAT_SERVO_PIN, 500, 2400); // Attaches the servo at pin 12 to its servo object
    plat_servo.write(plat_neutral_pos);           // Moves the servo to desired home position
                                                  // using default min/max of 1000us and 2000us
                                                  // different servos may require different min/max settings
                                                  // for an accurate 0 to 180 sweep
}

void TMC2209settings()
{
    digitalWrite(EN_PIN, LOW);
    driver.begin();           // Initiate pins and registeries
    driver.rms_current(1800); // Set stepperA current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    driver.pwm_autograd(1);   // Enable automatic gradient adaptation
    driver.pwm_autoscale(1);
    driver.microsteps(32);
    driver.TCOOLTHRS(0xFFFFF); // 20bit max
    driver.SGTHRS(STALL_VALUE);

    stepper.setMaxSpeed(1000000);
    stepper.setAcceleration(5000); // 2000mm/s^2
    stepper.setPinsInverted(false, false, true);
    stepper.enableOutputs();

    // digitalWrite(EN_PIN, HIGH); //TEMPORARY OFF??
}

void setup()
{
    Serial.begin(115200);

    // create a task that will be executed in the Serial_Com() function, with priority 1 and executed on core 1
    xTaskCreatePinnedToCore(
        Pi_SerialCom,        /* Task function. */
        "Pi Serial Manager", /* name of task. */
        10000,               /* Stack size of task */
        NULL,                // parameter of the task
        1,                   /* priority of the task */
        &Serial_Com_Handler, /* Task handle to keep track of created task */
        1);                  /* pin task to core 1 */
    delay(500);

    // create a task that will be executed in the Check_Temp() function, with priority 1 and executed on core 0
    xTaskCreatePinnedToCore(
        LED_Control,           /* Task function. */
        "LED Control Manager", /* name of task. */
        10000,                 /* Stack size of task */
        NULL,                  /* parameter of the task */
        2,                     /* priority of the task */
        &LED_Handler,          /* Task handle to keep track of created task */
        0);                    /* pin task to core 0 */
    delay(500);
}

// Handles communication and commands from Raspberry Pi
void Pi_SerialCOM(void *pvParameters)
{

    while (1)
    {
        if (Serial.available() > 0)
        { // Check presence of data at serial port

            data = Serial.readStringUntil('\n'); // Read command from Pi
            Serial.println("OK");                // Feedback to Pi on receiving command

            if (data == "EMAG_ON") // Turn on electromagnet cluster
            {
                digitalWrite(EMAGNET_PIN, HIGH);
                vTaskvTaskDelay(500);
                Serial.println("Done");
            }
            else if (data == "EMAG_OFF") // Turn off electromagnet cluster
            {
                digitalWrite(EMAGNET_PIN, LOW);
                vTaskvTaskDelay(500);
                Serial.println("Done");
            }
            else if (data == "LID_OPEN") // Open the input lid
            {
                lid_servo.write(110); // Position can be adjusted as desired
                while (digitalRead(LID_LIMIT_PIN) == LOW)
                {
                }
                lid_servo.write(90);
                Serial.println("Done");
            }
            else if (data == "LID_CLOSE") // Close the input lid
            {
                lid_servo.write(70); // Position can be adjusted as desired
                vTaskDelay(800);
                lid_servo.write(90);
                Serial.println("Done");
            }
            else if (data == "GATE_OPEN") // Open the gate
            {
                gate_servo.write(gate_open_pos1); // Position can be adjusted as desired
                vTaskDelay(500);
                gate_servo.write(gate_open_pos2); // Position can be adjusted as desired
                vTaskDelay(1000);
                Serial.println("Done");
            }
            else if (data == "GATE_CLOSE") // Close the gate
            {
                gate_servo.write(gate_init_pos); // Position can be adjusted as desired
                Serial.println("Done");
            }
            else if (data == "G_OPEN") // Open gripper
            {

                gripperOpen();
                vTaskDelay(300); // TRIED
                Serial.println("Done");
            }
            else if (data == "G_CLOSE") // Close gripper
            {
                gripperClose();
                Serial.println("Done");
            }
            else if (data == "PLAT_R")
            {
                plat_servo.write(plat_right_pos);
                Serial.println("Done");
            }
            else if (data == "PLAT_L")
            {

            }
            else if (data == "PLAT_N")
            {

            }
            else if (data == "GLED_ON") // Turn on Green LED
            {
                digitalWrite(GREENLED_PIN, HIGH);
                Serial.println("Done");
            }
            else if (data == "GLED_OFF") // Turn off Green LED
            {
                digitalWrite(GREENLED_PIN, LOW);
                Serial.println("Done");
            }
            else if (data == "RLED_ON") // Turn on Red LED
            {
                digitalWrite(REDLED_PIN, HIGH);
                Serial.println("Done");
            }
            else if (data == "RLED_OFF") // Turn off Red LED
            {
                digitalWrite(REDLED_PIN, LOW);
                Serial.println("Done");
            }

            if (data == "Test")
            {
                Serial.print("Testing Mode. Input test command or exit: ");
                TestLoop();
            }
            else
            {
                Serial.println("Invalid input");
            }
        }
        vTaskDelay(1);
    }

    void TestLoop()
    {
        int timeout_counter = 0;

        while (true)
        {

            vTaskDelay(10); // vTaskDelay 10ms

            if (Serial.available() > 0)
            {
                String data = Serial.readStringUntil('\n'); // Read test command

                if (data == "EMAGNET")
                {
                    // code for turning on Magnet for 5 sec
                    Serial.println("Initiating MAGNET Test...");
                    digitalWrite(EMAGNET_PIN, HIGH);
                    vTaskDelay(5000); // vTaskDelay for 5 sec
                    digitalWrite(EMAGNET_PIN, LOW);
                    Serial.println("Check Done");

                    TestLoop(); // Go back to start of TestLoop
                }

                else if (data == "LED")
                {
                    // code for turning on LED for 5 sec
                    Serial.println("Initiating LED Strip Test...");
                    digitalWrite(LEDSTRIP_PIN, HIGH);
                    vTaskDelay(5000); // vTaskDelay for 5 sec
                    digitalWrite(LEDSTRIP_PIN, LOW);
                    Serial.println("Check Done");

                    TestLoop(); // Go back to start of TestLoop
                }

                else if (data == "GLED")
                {
                    // code for turning on Green LED for 5 sec
                    Serial.println("Initiating Green LED Test...");
                    digitalWrite(GREENLED_PIN, HIGH);
                    vTaskDelay(5000); // vTaskDelay for 5 sec
                    digitalWrite(GREENLED_PIN, LOW);
                    Serial.println("Check Done");

                    TestLoop(); // Go back to start of TestLoop
                }

                else if (data == "RLED")
                {
                    // code for turning on Red LED for 5 sec
                    Serial.println("Initiating Red LED Test...");
                    digitalWrite(REDLED_PIN, HIGH);
                    vTaskDelay(5000); // vTaskDelay for 5 sec
                    digitalWrite(REDLED_PIN, LOW);
                    Serial.println("Check Done");

                    TestLoop(); // Go back to start of TestLoop
                }

                else if (data == "LID")
                {
                    Serial.print("Initiating LID servo Test...\nStarted");

                    lid_servo.write(70);
                    vTaskDelay(1500);
                    Serial.print("Reverse");
                    lid_servo.write(110);
                    vTaskDelay(1500);

                    lid_servo.write(90);

                    Serial.print("DONE");
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
                        vTaskDelay(15);

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
                                    vTaskDelay(5);
                                }
                            }
                            else if (info_servo.toInt() > gate_servoPos)
                            {
                                for (0; gate_servoPos < info_servo.toInt(); gate_servoPos += 1)
                                {
                                    gate_servo.write(gate_servoPos);
                                    vTaskDelay(5);
                                }
                            }
                            else
                            {
                                vTaskDelay(5);
                            }
                            break;
                        }
                    }

                    vTaskDelay(2500); // stop for 2.5 sec

                    // rotate back to Home Position
                    if (gate_init_pos < gate_servoPos)
                    {
                        for (0; gate_servoPos > gate_init_pos; gate_servoPos -= 1)
                        {
                            gate_servo.write(gate_servoPos);
                            vTaskDelay(5);
                        }
                    }
                    else if (gate_init_pos > gate_servoPos)
                    {
                        for (0; gate_servoPos < gate_init_pos; gate_servoPos += 1)
                        {
                            gate_servo.write(gate_servoPos);
                            vTaskDelay(5);
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
}

// Handles the indicator LED behavior
void LED_Handler(void *pvParameters)
{

    while (1)
    {
        
    }
}