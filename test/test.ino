#include <Arduino_FreeRTOS.h>
#include "MotorControl.h"
#include "Ultrasound.h"

//constants
const byte default_PWM = 60;
byte right_PWM = default_PWM;
byte left_PWM = default_PWM;
const int STOP_DISTANCE_CENTER = 25; // cm
const int STOP_DISTANCE_SIDE   = 20; // cm

// enum for Direction
enum Directions {forward, backward, left, right, halt};
// 0 - forward, 1 - backwards, 2 - left, 3 - right, 4 - halt

// enum for type of following/avoidance
enum Following {person, roam, command};
// 0 - person, 1 - roam


//globals for motor control
Directions currDir = forward;
Directions prevDir = halt;
Following followType = roam; // Default to roam for fun

//globals for crash avoidance
boolean dangerCenter = false;
boolean dangerLeft   = false;
boolean dangerRight  = false;
boolean dangerDetected = false;
long leftDistance = 0, centerDistance = 0, rightDistance = 0;
boolean lastCommandFromSerial = false;

//prototypes for RTOS tasks
void updateOrders  (void *pvParameters);
void updatePingData(void *pvParameters);
void driveACR      (void *pvParameters);

// Ping sensor pins
const int center_echo_pin = 8;
const int center_trig_pin = 9;
const int left_echo_pin   = A1;
const int left_trig_pin   = A0;
const int right_echo_pin  = 12;
const int right_trig_pin  = 11;

// Motor pins
const int LeftMotorA_pin    = A3;
const int RightMotorA_pin   = 6;
const int LeftMotorB_pin    = A4;
const int RightMotorB_pin   = 7;
const int LeftMotorPWM_pin  = 5;
const int RightMotorPWM_pin = 10;
const int LeftEncoder_pin = 2;
const int RightEncoder_pin = 3;

// OOP Initializations
MotorControl leftMotor  (LeftMotorA_pin,  LeftMotorB_pin,  LeftMotorPWM_pin);
MotorControl rightMotor (RightMotorA_pin, RightMotorB_pin, RightMotorPWM_pin);
Ultrasound leftUltrasound   (left_echo_pin, left_trig_pin);
Ultrasound centerUltrasound (center_echo_pin, center_trig_pin);
Ultrasound rightUltrasound  (right_echo_pin, right_trig_pin);
TickCounter leftEncoder (LeftEncoder_pin);
TickCounter rightEncoder (RightEncoder_pin);

int count = 0;
int error = 0;
int rightTicks = 0;
int leftTicks = 0;

void setup() {
  Serial.begin(9600); // Set baud-rate
//  xTaskCreate(driveACR,           (const portCHAR *) "Driving",         128, NULL, 2, NULL);      // Priority 2
//  xTaskCreate(updateEncoderTicks, (const portCHAR *) "Update Encoder Ticks", 128, NULL, 1, NULL); // Priority 1
//  xTaskCreate(updatePingData,     (const portCHAR *) "Updating Pings",  128, NULL, 3, NULL);      // Priority 3
//  xTaskCreate(updateOrders,       (const portCHAR *) "Updating Orders", 128, NULL, 4, NULL);      // Priority 4
//  xTaskCreate(adjustPower,        (const portCHAR *) "Adjusting Power", 128, NULL, 5, NULL);      // Priority 5
  attachInterrupt(digitalPinToInterrupt(RightEncoder_pin), tickRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LeftEncoder_pin), tickLeft, CHANGE);
  go_stop(); // Guarantee that both motors are not moving at start
  set_speed();
  go_forward();
}

// This is supposed to be empty (lets RTOS run uninterupted)
void loop() {
  count++;
  // Drive straight
  Serial.print("left ticks: "); Serial.println(leftTicks);
  Serial.print("right ticks: "); Serial.println(rightTicks);
  int kp = 80;
  if (count >= 20) {
    error = leftTicks - rightTicks;
    right_PWM += error / kp;
    set_speed();
    leftTicks = 0;
    rightTicks = 0;
    count = 0;
    error = 0;
    Serial.println("Hello");
  }
  centerDistance = centerUltrasound.getDistance();
  if (centerDistance <= STOP_DISTANCE_CENTER) {
    go_stop();
  } else {
    go_forward();
  }
}

void set_speed() {
  leftMotor.setPWM(left_PWM);
  rightMotor.setPWM(right_PWM);
}

void go_stop() {
  leftMotor.halt();
  rightMotor.halt();
}

void go_forward() {
  set_speed();
  leftMotor.forward();
  rightMotor.backward();
}

void go_backward() {
  set_speed();
  leftMotor.backward();
  rightMotor.forward();
}

void go_left() {
  set_speed();
  leftMotor.backward();
  rightMotor.backward();
}

void go_right() {
  set_speed();
  leftMotor.forward();
  rightMotor.forward();
}

void respondToCurrDir() {
  // TODO - if lastCommand was not from serial then don't check the currDir != prevDir

  // Only need to act on the currDir value if it's different from the prevDir
  if (currDir != prevDir) {
    if (currDir == forward)
      go_forward();
    else if (currDir == left)
      go_left();
    else if (currDir == right)
      go_right();
    else if (currDir == backward)
      go_backward();
    else if (currDir == halt)
      go_stop();
    else
      Serial.println("Error in respondToCurrDir() - currDir not found");
  }
}

//check the orders string for new commands
void updateOrders(void *pvParameters) {
  while (1) {
    if (Serial.available() > 0) {
      char incomingCharacter = Serial.read();
      Serial.print("I received: ");
      Serial.println(incomingCharacter);
      switch (incomingCharacter) {
        case '\n': // Disregard newlines
          break;
        case 'f':
          prevDir = currDir;
          currDir = forward;
          lastCommandFromSerial = true;
          break;
        case 'l':
          prevDir = currDir;
          currDir = left;
          lastCommandFromSerial = true;
          break;
        case 'r':
          prevDir = currDir;
          currDir = right;
          lastCommandFromSerial = true;
          break;
        case 'b':
          prevDir = currDir;
          currDir = backward;
          lastCommandFromSerial = true;
          break;
        case 'h':
          prevDir = currDir;
          currDir = halt;
          lastCommandFromSerial = true;
          break;
        case 'p':
          followType = person;
          break;
        case 'a':
          followType = roam;
          break;
        case 'c':
          followType = command;
          break;
        default:
          Serial.println("ERROR: Bad Message from Serial - character was not expected");
          break;
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS); //block task to allow other tasks to run
  }
}

//check sensors for new obstacles
void updatePingData(void *pvParameters) {
  while (1) {
    //Serial.println("Reading Ping Data");
    // Get distances
    leftDistance   = leftUltrasound.getDistance();
    centerDistance = centerUltrasound.getDistance();
    rightDistance  = rightUltrasound.getDistance();
    //    Serial.print("left cm: "); Serial.print(leftDistance);
    //    Serial.println();
    //    Serial.print("center cm: "); Serial.print(centerDistance);
    //    Serial.println();
    //    Serial.print("right cm: "); Serial.print(rightDistance);
    //    Serial.println();

    // Update danger booleans
    dangerLeft   = leftDistance   <= STOP_DISTANCE_SIDE;
    dangerCenter = centerDistance <= STOP_DISTANCE_CENTER;
    dangerRight  = rightDistance  <= STOP_DISTANCE_SIDE;
    dangerDetected = dangerCenter || dangerRight || dangerLeft;

    vTaskDelay(150 / portTICK_PERIOD_MS);
  }
}

//drives the robot accoring to the last received order
void driveACR(void *pvParameters) {
  while (1) {
    //Serial.println("Driving...");

    // Two variations of obstacle avoidance...
    //  1) Person following - just halt on obstacles
    //  2) Free Roam

    if (followType == person) {
      // Person following responds to serial commands, but ignores them and halts when we're in danger.
      // EXCEPT left, right, and halt commands (which we should let through)...

      if (currDir == left || currDir == right || currDir == halt) {
        // Current direction is left/right/halt, so lets let the robot move that way
        respondToCurrDir();
      } else {
        lastCommandFromSerial = false;
        go_stop();
      }
    } else if (followType == roam) {
      // Free Roam

      // Check where the danger is...
      if (currDir != halt || lastCommandFromSerial) {

        if (centerDistance <= STOP_DISTANCE_CENTER) { // If Danger at center...
          // Check whether we should go left or right
          if (leftDistance <= rightDistance) { // Right has more room than left, so go right.
            lastCommandFromSerial = false;
            prevDir = currDir;
            currDir = right;
            respondToCurrDir();
          }
          else { // Left has more room than right, so go left.
            lastCommandFromSerial = false;
            prevDir = currDir;
            currDir = left;
            respondToCurrDir();
          }
        } else if (leftDistance <= STOP_DISTANCE_SIDE) { // Issue @ left, so go right
          lastCommandFromSerial = false;
          prevDir = currDir;
          currDir = right;
          respondToCurrDir();
        } else if (rightDistance <= STOP_DISTANCE_SIDE) { // Issue @ right, so go left
          lastCommandFromSerial = false;
          prevDir = currDir;
          currDir = left;
          respondToCurrDir();
        } else { // respond to serial command. If obstacle is in the way, avoid the obstacle, then just free roam
          if (lastCommandFromSerial) {
            respondToCurrDir();
            lastCommandFromSerial = false;
          } else {
            prevDir = currDir;
            currDir = forward;
            respondToCurrDir();
          }
        }
      }
    } else if (followType = command) {
      respondToCurrDir();
    } else { // Neither followType matched... not possible
      Serial.println("ERROR: followType had no match");
    }

    vTaskDelay(60 / portTICK_PERIOD_MS);
  }
}


// Use Proportional Feedback and Encoder readings to match the right motor's speed to the left motor.
// In this case, left motor is the master and right motor is the slave.
void adjustPower(void *pvParameters) {
  while (1) {
    Serial.println("Adjusting Power");
    int error = 0;
    int kp = 2;
    leftEncoder.resetTicks();
    rightEncoder.resetTicks();

    vTaskDelay(150 / portTICK_PERIOD_MS); // block task for a long time to allow encoder to build up ticks

    error = leftEncoder.ticks - rightEncoder.ticks;
    right_PWM += error / kp;
    if (right_PWM >= default_PWM + 15) right_PWM = default_PWM + 15;
    if (right_PWM <= default_PWM - 15) right_PWM = default_PWM - 15;
    set_speed();
    Serial.println(right_PWM);
    Serial.println(left_PWM);
  }

}

// Updates the Encoder's tick count
void updateEncoderTicks(void *pvParameters) {
  int prev = 0;
  int curr = 0;
  while (1) {
    //Serial.println("Updating Ticks");
    if (currDir == forward || currDir == backward) {
      for (int i = 0; i < 10; i++) { //update ticks 10 times
        leftEncoder.updateTick();
        rightEncoder.updateTick();
      }
    }
    prev = curr;
    curr = leftEncoder.ticks;
    if (curr != prev) {
      Serial.println(leftEncoder.ticks);
      Serial.println(rightEncoder.ticks);
    }
    vTaskDelay(20 / portTICK_PERIOD_MS); //short delay period since accurate tick counts are desired
  }

}

void tickLeft() {
  leftTicks++;
}

void tickRight() {
  rightTicks++;
}
