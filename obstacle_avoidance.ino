
const int pwm_right = 5; //pwm in D5
const int inA_right = A4; //inA in A3
const int inB_right = A3; //inB in A4

const int pwm_left = 3; //pwm in D3
const int inA_left = 6; //inA in D6
const int inB_left = 7; //inB in D7

#define echoPin_right  A1 //echo in A1
#define trigPin_right  A2 //trig in A2

#define echoPin_left  12 //echo in D12
#define trigPin_left  11 //trig in D11

#define max_distance 250 //maximum distance the sensor is rated for in centimeters

void setup() {

pinMode(pwm_right, OUTPUT);
pinMode(inA_right,OUTPUT);
pinMode(inB_right,OUTPUT);

pinMode(pwm_left, OUTPUT);
pinMode(inA_left,OUTPUT);
pinMode(inB_left,OUTPUT);

pinMode(trigPin_right, OUTPUT); // Sets the trigPin as an OUTPUT
pinMode(echoPin_right, INPUT); // Sets the echoPin as an INPUT
pinMode(trigPin_left, OUTPUT); // Sets the trigPin as an OUTPUT
pinMode(echoPin_left, INPUT); // Sets the echoPin as an INPUT
Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed

}

void loop(){
int distance_right;
int distance_left;

distance_right = readPing_right();
distance_left = readPing_right();

  if (distance_right <= 20 || distance_left <= 20){
    moveStop();
    delay(300);
    moveBackwards();
    delay(400);
    moveStop();
    delay(300);
    distance_right = readPing_right();
    delay(300);
    distance_left = readPing_right();
    delay(300);
  
    if(distance_right >= distance_left){
      turnRight();
      delay(300);
      moveStop();
    }
    else{
      turnLeft();
      delay(300);
      moveStop();
    }
  
  }
  else{
    moveForward();
  }
  
}

int readPing_right(){
  long duration;
  int distance;
  
  // Clears trigPin
  digitalWrite(trigPin_right, LOW);
  delayMicroseconds(2);
  
  //Sets trigPin HIGH
  digitalWrite(trigPin_right, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_right, LOW);

  // reads the echopin and returns time
  duration = pulseIn(echoPin_right, HIGH);
  
  distance = duration *0.034/2;
  
  Serial.print("Distance_right: ");
  Serial.print(distance);

  return distance;
}

int readPing_left(){
  long duration;
  int distance;
  
  // Clears trigPin
  digitalWrite(trigPin_left, LOW);
  delayMicroseconds(2);
  
  //Sets trigPin HIGH
  digitalWrite(trigPin_left, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_left, LOW);

  // reads the echopin and returns time
  duration = pulseIn(echoPin_left, HIGH);
  
  distance = duration *0.034/2;
  
  Serial.print("Distance_left: ");
  Serial.print(distance);

  return distance;
}

void moveStop(){
  digitalWrite(inA_left,LOW);
  digitalWrite(inB_left,LOW);
  digitalWrite(inA_right,LOW);
  digitalWrite(inB_right,LOW);
}

void moveForward(){
  digitalWrite(inA_left, HIGH);
  digitalWrite(inB_left,LOW);
  digitalWrite(inA_right, LOW);
  digitalWrite(inB_right, HIGH);
  analogWrite(pwm_left,200);
  analogWrite(pwm_right,200);

}

void moveBackwards(){
  digitalWrite(inA_left, LOW);
  digitalWrite(inB_left,HIGH);
  digitalWrite(inA_right, HIGH);
  digitalWrite(inB_right, LOW);
  analogWrite(pwm_left,200);
  analogWrite(pwm_right,200);
}

void turnRight(){
  digitalWrite(inA_left, HIGH);
  digitalWrite(inB_left,LOW);
  digitalWrite(inA_right, HIGH);
  digitalWrite(inB_right, LOW);
  analogWrite(pwm_left,200);
  analogWrite(pwm_right,200);
}

void turnLeft(){
  digitalWrite(inA_left, LOW);
  digitalWrite(inB_left,HIGH);
  digitalWrite(inA_right, LOW);
  digitalWrite(inB_right, HIGH);
  analogWrite(pwm_left,200);
  analogWrite(pwm_right,200);
}
