#include "MotorControl.h"
#include "Ultrasound.h"
#include <cppQueue.h>

/* TODO
    To find things I still need to do, use ctrl-F to find all instances of "TODO" in the code
*/

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

//constants
const int default_PWM = 32;
int right_PWM = default_PWM - 9; //adjustment for initial PWM; proportional control makes this value irrelevant later on
int left_PWM = default_PWM;
const int STOP_DISTANCE_CENTER = 20; // cm
const int STOP_DISTANCE_SIDE   = 25; // cm

// enum for Direction
enum Directions {forward, backward, left, right, halt};
// 0 - forward, 1 - backwards, 2 - left, 3 - right, 4 - halt

enum Modes {calibration, obstacle, distance, following, waiting};
// calibration - calibrate motor power, obstacle - basic obstacle avoidance
// distance - specify distance to travel and turning angles
// following - no specified distance; turns while moving forward at different turning speeds
// waiting - robot is halted; awaiting commands

//globals for motor control
Directions currDir = forward;
Directions prevDir = halt;
Modes mode = waiting;
bool isExecuting = false;

long leftDistance = 0, centerDistance = 0, rightDistance = 0;

// Variables for the encoder
int calibration_count = 0;
int error = 0;
int kp = 50;
unsigned int rightTicks = 0;
unsigned int leftTicks = 0;
unsigned int distanceTicks = 0;
int precision = 5; // in centimeters; DO NOT CHANGE FOR NOW, Serial Messaging to include precision is not implemented
const int cm2ticks = 80;
const int units2deg = 3;
const int deg2ticks = 22; // TODO needs calibration

// Variables for Commands
int q_size = 31;
cppQueue q(sizeof(byte), q_size, FIFO);  // Queue to store commands from Nvidia
byte command;
bool isQueuePaused = false;

void setup() {
  Serial.begin(9600); // Set baud-rate
  while (!Serial) {
    ; // wait for serial to connect
  }
  Serial.write(B11111111);
  go_stop(); // Guarantee that both motors are not moving at start
  set_speed(right_PWM, left_PWM);
  delay(500);
}

bool once = true;
bool calibrate = true;

//void loop() {
//
//  if (once) {
//    once = false;
//    delay(1000);
//    go_distance(60, right);
//    delay(1000);
//    go_distance(60, left);
//
//  }
//}

void loop() {
  // Accept a Command from Serial
  while (Serial.available() > 0) {
    command = Serial.read();
    bool isSystemCommand = command >> 4 == 1;
    if (isSystemCommand) {
      processCommand(command);
      break;
    } else if (mode == distance) {
      q.push(&command);
    } else {
      isExecuting = true; // command does not go to Queue; processes immediately
      break;
    }
  }

  switch (mode) {
    case calibration:
      calibrate_motors();
      break;
    case distance:
      if (!q.isEmpty()) {
        q.pop(&command);  //store popped byte into variable value
        isExecuting = true;
        //Serial.write(command);
      } else {
        isExecuting = false;
      }
      break;
    case following:
      break;
    case waiting: // just here to make it clear that I did not forget about this mode. It just does nothing by default.
      break;
  }

  // Code to stop the robot if obstacle is detected goes here, since it will prevent any inputs from running until the obstacle is not detected
  if (isDanger()) {
    isExecuting = false;
  }

  if (isExecuting) {
    // Serial.write(command);
    processCommand(command);
    isExecuting = false;
  }
}

// Process command. This can be either from the Serial Port or the Command Queue.
// The behavior changes depending on what mode the robot is in. The serial can
// also be used to change modes. Commands are a single byte composed of
// Instruction Bits and Value Bits.
//
// Command codes are listed below:x
// Modes (0000):
//    Distance      0000 0001
//    Following     0000 0010
//    Calibration   0000 0011
//    Waiting       0000 0000
// System (0001): (These are not placed in the Queue)
//    E-Halt Robot  0001 0000 (and Clear Queue)
//    Clear Queue   0001 0001
// Directions:
//    Backward    001X XXXX
//    Left        01XX XXXX
//    Right       10XX XXXX
//    Forward     11XX XXXX
// When in Distance mode, the X'd values represent distances and in-place turn angles.
// When in following mode, the X'd values represent turning speeds (difference in
// power between motors). This only applies for left and right directions.
//
// Note: To change modes immediately, make sure to clear the Queue first
// TODO: update the serial codes in this comment
void processCommand(byte command) {
  int commandValue;
  bool isModeChange = command >> 4 == 0;
  bool isSystemCommand = command >> 4 == 1;
  if (isModeChange) {
    switch (command & B00001111) {
      case 0: // The robot only enters this mode after calibration
        mode = waiting;
        break;
      case 1:
        mode = distance;
        break;
      case 2:
        mode = following;
        q.clean();
        break;
      case 3:
        mode = calibration;
        break;
    }
  } else if (isSystemCommand) {
    switch (command & B00001111) {
      case 0: // Halt the robot immediately
        q.clean(); // clear Queue so that no other commands run
        isExecuting = false;
        updateDir(halt);
        respondToCurrDir();
        break;
      case 1: // Clear Queue (not sure why we would do this without halting, though)
        q.clean();
        isExecuting = false;
        break;
    }
  } else {
    // TODO Modify these according to the design in the Google Slide
    // Following Mode and Distance Mode make these do different things
    // Maybe have a go_distance(command) function and a follow(command) function
    if (mode == following) {
      switch (command >> 6) {
        case 0: //backwards/halt
          commandValue = command & B00011111;
          if (commandValue == 0) {
            updateDir(halt);
            respondToCurrDir();
          }
          break;
        case 1: //left
          commandValue = command & B00111111;
          scan(commandValue, left);

          break;
        case 2: //right
          commandValue = command & B00111111;
          scan(commandValue, right);

          break;
        case 3: //forward-right or forward-left
          commandValue = command & B00111111;
          switch (commandValue >> 5) {
            case 0: //right
              follow(commandValue & B00011111, right);
              break;
            case 1: //left
              follow(commandValue & B00011111, left);
              break;
          }

          break;

      }
    } else if (mode == distance) {
      switch (command >> 6) {
        case 0: //backwards
          commandValue = command & B00011111;
          // For the most part, I'd rather not go backwards

          break;
        case 1: //left
          commandValue = command & B00111111;
          go_distance(commandValue, left);



          break;
        case 2: //right
          commandValue = command & B00111111;
          go_distance(commandValue, right);


          break;
        case 3: //forward
          commandValue = command & B00111111;
          go_distance(commandValue, forward);

          break;

      }

    }
  }
}

// TODO: Complete this function
// TODO: Find the number of ticks per the angle of rotation
// TODO: Write to Serial: the distance traveled and if there was an obstacle
// Takes a centimeter input and outputs a distance tick count
// About 80 encoder ticks per centimeter
// For rotation, each unit = 3 degrees, with a maximum turn of 180 degrees for each direction.
int go_distance(byte unitDistance, Directions dir) {
  byte distance = (dir == forward) ? unitDistance * precision : unitDistance * units2deg;
  bool obstacle = false;
  unsigned int tick_target = (dir == forward) ? distance * cm2ticks : distance * deg2ticks;
  // Turn on the interrupt to detect encoder ticks
  attachInterrupt(digitalPinToInterrupt(LeftEncoder_pin), tickLeft, CHANGE);
  distanceTicks = 0;
  updateDir(dir);
  respondToCurrDir();

  while (distanceTicks < tick_target) {
    // Keeps going forward until distance is reached or ultrasonic sensor detects obstacle
    if (dir == forward && isDanger()) {
      obstacle = true;
      break;
    }

    // break if the distanceTicks stops updating (this means something is wrong)
  }

  updateDir(halt);
  respondToCurrDir();

  // Check for danger at the end of the maneuver when turning
  if (dir == left || dir == right) {
    if (isDanger()) {
      obstacle = true;
    }
  }

  // turn off the interrupts, so that they don't interfere with the Serial
  detachInterrupt(digitalPinToInterrupt(LeftEncoder_pin));

  // Send message back to Nvidia indicating distance traveled and if obstacle was encountered
  byte unitsTraveled = (dir == forward) ? distanceTicks / (cm2ticks * precision) : distanceTicks / (deg2ticks * units2deg);
  byte msg = generateDistSerial(obstacle, unitsTraveled);

  Serial.write(msg);

  return unitsTraveled;


}

// Used when the robot moves forward when following; makes robot veer left or right as desired
// TODO stop the robot if ultrasonic sensor detects something
void follow(byte turn_speed, Directions dir) {
  updateDir(forward);
  if (dir == right) {
    byte new_PWM = left_PWM > turn_speed ? left_PWM - turn_speed : 0;
    set_speed(right_PWM, new_PWM);
  } else {
    byte new_PWM = right_PWM > turn_speed ? right_PWM - turn_speed : 0;
    set_speed(new_PWM, left_PWM);
  }
  respondToCurrDir();
}

// Used in following Mode; when robot needs to find a target, you can set how fast the robot rotates in place
// Note that the larger the speed_reduction value, the slower the turn
void scan(byte speed_reduction, Directions dir) {
  if (dir == right) {
    updateDir(right);
  } else {
    updateDir(left);
  }
  if (right_PWM > speed_reduction && left_PWM > speed_reduction) {
    set_speed(right_PWM - speed_reduction, left_PWM - speed_reduction);
  } else {
    set_speed(0, 0);
  }

  respondToCurrDir();
}

// Call this function to tell robot to recalibrate motor powers to drive straight
// Modifies the right_PWM value
//
// TODO: Send back a Serial message if calibration succeeds/fails (if obstacle gets in way)
void calibrate_motors() {
  boolean halted = false;

  updateDir(halt); // stop the robot
  respondToCurrDir();
  delay(500);
  // Turn on the interrupt to detect encoder ticks
  attachInterrupt(digitalPinToInterrupt(RightEncoder_pin), tickRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LeftEncoder_pin), tickLeft, CHANGE);


  updateDir(forward); // robot starts moving
  respondToCurrDir();

  delay(2000); // Give robot chance to get up to speed

  calibration_count = 0;
  leftTicks = 0;
  rightTicks = 0;

  // Use Proportional Feedback and Encoder readings to match the right motor's speed to the left motor.
  // In this case, left motor is the master and right motor is the slave.
  while (1) {
    //    centerDistance = centerUltrasound.getDistance();
    //    if (centerDistance <= STOP_DISTANCE_CENTER) {
    //      updateDir(halt);
    //      respondToCurrDir();
    //      // turn off the interrupts, so that they don't interfere with the Serial
    //      detachInterrupt(digitalPinToInterrupt(RightEncoder_pin));
    //      detachInterrupt(digitalPinToInterrupt(LeftEncoder_pin));
    //      //TODO: Send Serial Message back to Nvidia stating a halt
    //      halted = true;
    //      break;
    //    }

    if (leftTicks > 2427) {
      error = leftTicks - rightTicks;
      int adjustment = error / kp;
      if (adjustment > 3) {
        right_PWM += 3;
      } else if (adjustment < -3) {
        right_PWM -= 3;
      } else {
        right_PWM += adjustment;
      }
      set_speed(right_PWM, left_PWM);

      leftTicks = 0;
      rightTicks = 0;
      calibration_count = 0;
      error = 0;
      if (adjustment >= -1 && adjustment <= 1) {
        break;
      }
    }
    calibration_count++;
  }

  updateDir(halt); // stop the robot!
  respondToCurrDir();
  mode = waiting;
  // turn off the interrupts, so that they don't interfere with the Serial
  detachInterrupt(digitalPinToInterrupt(RightEncoder_pin));
  detachInterrupt(digitalPinToInterrupt(LeftEncoder_pin));
  // TODO: Send a Serial Message to Nvidia stating successful calibration
  byte msg = generateDistSerial(halted, 0);
  Serial.write(msg);

}

// Generates the serial if the Nvidia asks for the robot's state
// bit 1    indicates that this is a state message (set to 1)
// bit 2-3  indicates the mode the robot is in
//          Distance Mode:    00
//          Following Mode:   01
//          Calibration Mode: 10
//          Waiting Mode:     11
// bit 4    Indicates if the Queue is paused (TODO: Not implemented yet)
// bits 5-8 Indicates the number of items in the Queue
byte generateStateSerial() {
  byte msg = B11111111;
  switch (mode) {
    case distance:
      msg = msg & B10011111;
      break;
    case following:
      msg = msg & B10111111;
      break;
    case calibration:
      msg = msg & B11011111;
      break;
    case waiting:
      break;
  }

  msg = msg & B11101111; //Queue is always NOT paused for now

  msg = msg & q.getCount(); //getCount() returns uint16, so... it should work?

  return msg;

}

// Generates the Serial Message if Robot just traveled a certain distance
// bit 1  ->  indicates it's a distance message (value 0)
// bit 2 ->  obstacle or not (0 is no obstacle; 1 is obstacle)
//      In calibration mode; no obstacle = success
// bit 3-8 ->  Distance (TODO: Not implemented yet, since it involves more coding)
byte generateDistSerial(bool detectedObstacle, byte unitsTraveled) {
  byte msg = B01111111;
  if (!detectedObstacle) {
    msg = msg & B10111111;
  }

  byte unitMsg = B11000000 | unitsTraveled;
  msg = msg & unitMsg;

  return msg;

}





bool isDanger() {
  centerDistance = centerUltrasound.getDistance();
  if (centerDistance <= STOP_DISTANCE_CENTER) {   // This means sometimes it will halt while turning in place, but... eh, I'll figure that out later
    updateDir(halt);
    respondToCurrDir();
    return true;
  }
  return false;
}

void updateDir(Directions newDir) {
  prevDir = currDir;
  currDir = newDir;
}

void respondToCurrDir() {
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

// We use the left wheel to measure distance
void tickLeft() {
  leftTicks++;
  distanceTicks++;
}

void tickRight() {
  rightTicks++;
}

// Sets motor PWM values
// If PWM is negative, this function auto sets PWM to 0
void set_speed(int new_right_PWM, int new_left_PWM) {
  int temp_right = new_right_PWM >= 0 ? new_right_PWM : 0;
  int temp_left = new_left_PWM >= 0 ? new_left_PWM : 0;
  leftMotor.setPWM(temp_left);
  rightMotor.setPWM(temp_right);
}

void go_stop() {
  leftMotor.halt();
  rightMotor.halt();
}

void go_forward() {
  leftMotor.forward();
  rightMotor.backward();
}

void go_backward() {
  leftMotor.backward();
  rightMotor.forward();
}

void go_left() {
  leftMotor.backward();
  rightMotor.backward();
}

void go_right() {
  leftMotor.forward();
  rightMotor.forward();
}
