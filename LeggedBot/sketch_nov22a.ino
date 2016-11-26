/**************************************************************************/
/*!
    @file     read_simple_angle.ino
    @author   SOSAndroid (E. Ha.)
    @license  BSD (see license.txt)

  read a simple angle from AS5048B over I2C bus

    @section  sHISTORY

    v1.0 - First release
*/
/**************************************************************************/
#include <Wire.h>
#include <ams_as5048b.h>

//unit consts
#define U_RAW 1
#define U_TRN 2
#define U_DEG 3
#define U_RAD 4
#define U_GRAD 5
#define U_MOA 6
#define U_SOA 7
#define U_MILNATO 8
#define U_MILSE 9
#define U_MILRU 10

#include <PID_v1.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=1, Ki=0.5, Kd=0.001;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


float old_time;
float curr_time = 0;;
float old_degree;
float curr_degree = 0;

int enA = 10;
int in1 = 9;
int in2 = 8;

// motor two
//int enB = 5;
//int in3 = 7;
//int in4 = 6;
int speed = 255;

AMS_AS5048B mysensor(0b1000001);

void setup() {

  pinMode(enA, OUTPUT);
 // pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
//  pinMode(in3, OUTPUT);
//  pinMode(in4, OUTPUT);


  //Start serial
  Serial.begin(9600);
  pinMode(A3, OUTPUT);
  digitalWrite(A3, HIGH);
  while (!Serial) ; //wait until Serial ready

  //Start Wire object. Unneeded here as this is done (optionally) by the AMS_AS5048B object (see lib code - #define USE_WIREBEGIN_ENABLED)
  //Wire.begin();

  //init AMS_AS5048B object
  mysensor.begin();

  //consider the current position as zero
  mysensor.setZeroReg();

  millis();



  //initialize the variables we're linked to
  Input = 0;
  Setpoint = -350;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);

}

void loop() {
  old_time = curr_time;
  //print to serial the raw angle and degree angle every 2 seconds
  //print 2 times the exact same angle - only one measurement
  //Serial.print("Angle sensor raw : ");
  //Serial.println(mysensor.angleR(U_RAW, true), DEC);



  //Serial.print("Angle degree : ");
  //Serial.println(mysensor.angleR(U_DEG, false), DEC);
  old_degree = curr_degree;
  curr_degree = mysensor.angleR(U_DEG, true);
  curr_time = millis();

  float diff = (curr_degree - old_degree);
  
  //
  if ((curr_degree - old_degree) > 180) {
    diff = -360 + diff;
//    Serial.println("i hate brent");
  }
  else if ((curr_degree - old_degree) < -180) {
    diff = 360 + diff;
//     Serial.println("hello");
  }
  else {
    diff = curr_degree - old_degree;
//    Serial.println("mhey");
  }
  float a_velocity = 1000 * diff / (curr_time - old_time);

//  Serial.print("Angular velocity : ");

//  Serial.println(a_velocity);

        // set speed to 200 out of possible range 0~255
//   analogWrite(enA, speed);
        // turn on motor B
        //digitalWrite(in3, HIGH);
        //digitalWrite(in4, LOW);
        // set speed to 200 out of possible range 0~255
        //analogWrite(enB, speed);

//
    Input = a_velocity;//analogRead(in1);
//    Serial.println(Input);
    myPID.Compute();
    
    digitalWrite(in1, Output > 0);
    digitalWrite(in2, Output < 0);
    analogWrite(enA, abs(Output));
    //Serial.print(a_velocity);
    //Serial.print("vs");
    //Serial.print(Setpoint);
    //Serial.println(" ");
    Serial.println(Setpoint - a_velocity);
    delay(50);
}
