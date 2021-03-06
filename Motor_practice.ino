const int pwm_right = 5; //pwm in D5
const int inA_right = A4; //inA in A3
const int inB_right = A3; //inB in A4
//
const int pwm_left = 3; //pwm in D3
const int inA_left = 6; //inA in D6
const int inB_left = 7; //inB in D7

void setup() {

pinMode(pwm_right, OUTPUT);
pinMode(inA_right,OUTPUT);
pinMode(inB_right,OUTPUT);

pinMode(pwm_left, OUTPUT);
pinMode(inA_left,OUTPUT);
pinMode(inB_left,OUTPUT);
}

void loop() {

//go forwards for 3 seconds
digitalWrite(inA_left, HIGH);
digitalWrite(inB_left,LOW);
analogWrite(pwm_left,200);

digitalWrite(inA_right, LOW);
digitalWrite(inB_right, HIGH);
analogWrite(pwm_right,200);

delay(3000);

//brake
digitalWrite(inA_left,LOW);
digitalWrite(inB_left,LOW);

digitalWrite(inA_right,LOW);
digitalWrite(inB_right,LOW);

delay(1000);

//go other direction for 3 seconds
digitalWrite(inA_left, LOW);
digitalWrite(inB_left,HIGH);

digitalWrite(inA_right, HIGH);
digitalWrite(inB_right, LOW);

delay(3000);

//brake
digitalWrite(inA_left,LOW);
digitalWrite(inB_left,LOW);

digitalWrite(inA_right,LOW);
digitalWrite(inB_right,LOW);

delay(1000);


}
