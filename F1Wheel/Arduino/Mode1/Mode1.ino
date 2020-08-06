// Mode 1. Single arduino for all buttons, 2 rotary and 5 analog.
// Buttons are done with a button matrix, see scematics: !TODO!


//#define GYRO_STEERING
#define ROTARY_ENCODER
#include <Joystick.h>

#include <Arduino.h>
#ifdef GYRO_STEERING
#include <TinyMPU6050.h>
#endif
#define REV_STEERING 1
#define UPSIDEDOWN 0


#ifdef ROTARY_ENCODER
#include "rotaryEncoder.h"
#include <MD_REncoder.h>

// set up encoder object
MD_REncoder R1 = MD_REncoder(0, 1);
MD_REncoder R2 = MD_REncoder(2, 3);

#define matrix1 18
#define matrix2 19
#define matrix3 20
#define matrix4 21

#define mbutt1 14
#define mbutt2 15
#define mbutt3 16

#define MATRIXLINES 4
#define MATRIXBUTTONS 3
#define MATRIXBUTTONSTOTAL MATRIXLINES*MATRIXBUTTONS

bool mbuttons[MATRIXBUTTONSTOTAL+1];
int mlines[MATRIXLINES];
int mbutts[MATRIXBUTTONS];


#endif


double totalTurnAngle = 360; // 360 is 180 to each side. typical road car have ~900

bool setupMode = false;
/*
 *  Constructing MPU-6050
 */
#ifdef GYRO_STEERING
MPU6050 mpu (Wire);
#endif
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_GAMEPAD,
  32, 0,                  // Button Count, Hat Switch Count
  true, true, true,     // X and Y, but no Z Axis
  true, true, false,   // No Rx, Ry, or Rz
  false, false,          // No rudder or throttle
  false, true, false);  // No accelerator, brake, or steering

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(0,INPUT_PULLUP);
  pinMode(1,INPUT_PULLUP);
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  pinMode(4,INPUT_PULLUP);
  pinMode(5,INPUT_PULLUP);
  pinMode(6,INPUT_PULLUP);
  pinMode(7,INPUT_PULLUP);
  pinMode(15,INPUT_PULLUP);
  pinMode(14,INPUT_PULLUP);
  pinMode(16,INPUT_PULLUP);
  setupMatrix();
  R1.begin();
  R2.begin();
  Serial.println("started..");
  delay(1000);
  Serial.print("1");
  //delay(1000); 
  //Serial.print("2");
  //delay(1000); 
  //Serial.print("3");
  //delay(1000); 
  //Serial.print("4");
  //delay(1000); 
  //Serial.print("5");
  // wait 5s so we can reprogram device incase of flooding on serial port
    // Initialize Joystick Library
  // Initialization

  #ifdef GYRO_STEERING
  mpu.Initialize();
  // Calibration
  Serial.println("=====================================");
  //Serial.println("Starting calibration...");
  //mpu.Calibrate();
  #endif

  Joystick.begin(false);
  //Joystick.setXAxisRange(-32767, 32767);
  
  Joystick.setXAxisRange(-512, 512);
  Joystick.setYAxisRange(-512, 512);
  Joystick.setZAxisRange(-512, 512);
  
  Joystick.setRxAxisRange(-512, 512);
  Joystick.setRyAxisRange(-512, 512);
  Joystick.setRzAxisRange(-512, 512);
}

void setupMatrix() {

  mlines[0] = matrix1;
  mlines[1] = matrix2;
  mlines[2] = matrix3;
  mlines[3] = matrix4;

  mbutts[0] = mbutt1;
  mbutts[1] = mbutt2;
  mbutts[2] = mbutt3;

  for (int i=0;i<MATRIXLINES;i++) {
    pinMode(mlines[i],INPUT_PULLUP);
  }
  for (int i=0;i<MATRIXBUTTONS;i++) {
    pinMode(mbutts[i],INPUT_PULLUP);
  }

}

long counter = 0;
long time1 = 0;
int button = 0; 
bool currentButtonState;
double prevang = 0.0;
int turns = 0;

void loop() {
  mode1();

}

void handleMatrix() {
  for (int i=0;i<MATRIXLINES;i++) {
    pinMode(mlines[i],OUTPUT);
    digitalWrite(mlines[i],LOW);
    for (int x=0;x<MATRIXBUTTONS;x++) {
      mbuttons[i*MATRIXBUTTONS+x+1] = !digitalRead(mbutts[x]);
    }
    pinMode(mlines[i],INPUT_PULLUP);
    digitalWrite(mlines[i],HIGH);
  }

  
}

void handleButton(int pin, int buttonNr, bool reverse) {
  if (pin <0) {
    bool state = mbuttons[-pin];
    Joystick.setButton(buttonNr, state);
  } else {
    bool state = false;
    state = digitalRead(pin);
    if (reverse) {
      state = !state;
    }
    Joystick.setButton(buttonNr, state);
  }
}

void handleMultiWheelButton(int aPort, int button1, int button2, int startButton, int sections) {
  int value = analogRead(aPort);
  sections = sections;
  if(sections == 0) {
    sections = 1;
  }
  int divider = 1024/sections;
  value = value/divider;

  handleButton(button1, value*2+startButton, true);
  handleButton(button2, value*2+startButton+1, true);
}
void handleEncoder(MD_REncoder R, int buttonNr) {
  uint8_t x = R.read();
  if (x) {
    Serial.print(x); 
        Serial.println("kaka"); 
  }
  if (x==16) {
    Joystick.setButton(buttonNr, true);
    Joystick.setButton(buttonNr+1, false);
  } else if (x==32) {
    Joystick.setButton(buttonNr, false);
    Joystick.setButton(buttonNr+1, true);
  } else {
    Joystick.setButton(buttonNr, false);
    Joystick.setButton(buttonNr+1, false);
  }
}

long rotarySticky = 0;
long stick = 10;

void mode1() {
  
  // read 2 gear shifters
  handleButton(4,30,false);
  handleButton(5,0,false);
  handleButton(6,31,false);
  handleButton(7,1,false);


  // Read 2 encoders

  //handleEncoder(R1,2);
  //handleEncoder(R2,4);
    uint8_t x = R1.read();
  if (x && setupMode) {
    Serial.print(x);
    Serial.println("banan"); 
  }
  if (x==16) {
    Joystick.setButton(2, true);
    Joystick.setButton(2+1, false);
    rotarySticky = millis()+stick;
    
  } else if (x==32) {
    Joystick.setButton(2, false);
    Joystick.setButton(2+1, true);
    rotarySticky = millis()+stick;
  } else {
    if (rotarySticky < millis()) {
      Joystick.setButton(2, false);
     Joystick.setButton(2+1, false);
  
    }
  }
   x = R2.read();
  if (x && setupMode) {
    Serial.print(x);
    Serial.println("banan"); 
  }
  if (x==16) {
    Joystick.setButton(4, true);
    Joystick.setButton(4+1, false);
    rotarySticky = millis()+stick;
    
  } else if (x==32) {
    Joystick.setButton(4, false);
    Joystick.setButton(4+1, true);
    rotarySticky = millis()+stick;
  } else {
    if (rotarySticky < millis()) {
      Joystick.setButton(4, false);
      Joystick.setButton(4+1, false);
  
    }
  }
  //getRotationType1(0, 1, 1)
  //getRotationType1(int pinIn1, int pinIn2, int nr)

  // read button matrix
  handleMatrix();
  Joystick.setButton(6, mbuttons[1]);
  Joystick.setButton(7, mbuttons[2]);
  Joystick.setButton(8, mbuttons[3]);
  
  Joystick.setButton(9, mbuttons[4]);
  Joystick.setButton(10, mbuttons[5]);
  Joystick.setButton(11, mbuttons[6]);
  
  Joystick.setButton(12, mbuttons[7]);
  Joystick.setButton(13, mbuttons[8]);
  //Joystick.setButton(14, mbuttons[9]);
  
  Joystick.setButton(14, mbuttons[10]);
  Joystick.setButton(15, mbuttons[11]);
  //Joystick.setButton(17, mbuttons[12]);
  
  
  // Read 4 analog inputs
  //int a0 = analogRead(A6);
  //a0 = (a0-512);
  //int a1 = analogRead(A7);
  //a1 = (a1-512);
  int a2 = analogRead(A9);
  a2 = (a2-512);
  int a3 = analogRead(A10);
  a3 = (a3-512);
  Joystick.setXAxis(0);
  //Joystick.setYAxis(a1);
  Joystick.setZAxis(a2);
  Joystick.setRxAxis(a3);
  //Joystick.setRyAxis(a0);
  Joystick.setRzAxis(0);
  // read multiwheel dial

  handleMultiWheelButton(A8, -9, -12, 16 , 5);

  // send state
  Joystick.sendState();
  counter++;

  if (time1<millis() && setupMode) {
    Serial.println(counter);
    time1 = millis()+200;
    counter = 0;

    Serial.print("matrixbuttons = ");
    for (int i=0;i<MATRIXBUTTONSTOTAL;i++) {
      Serial.print(" ");
      Serial.print(mbuttons[i+1]);
    }
    Serial.println("");
    
  }
}

void mode2() {
  #ifdef GYRO_STEERING
mpu.Execute();
#endif

  int a0 = analogRead(A0);
  
  int a1 = analogRead(A1);
  
  int a2 = analogRead(A2);
  
  int a3 = analogRead(A3);
  
  int a4 = analogRead(A6);
  
  int a5 = analogRead(A7);
  
  //int aMulti = analogRead(A8);
  
  int a7 = analogRead(A9);
  int a8 = analogRead(A10);
  double ang;
#ifdef GYRO_STEERING
  int x = mpu.GetRawAccX();
  int y = mpu.GetRawAccY();
  
  if(UPSIDEDOWN == 0) {
    ang = atan2(y,x);
  
  }else {
    ang = atan2(y,-x);

  }
  //double ang = atan2(y,x);

  if (prevang < 0 && ang > 0) {
    if (abs(ang) > 2.0) {
      turns = turns -1;
    }
    
  }
  if (prevang > 0 && ang < 0) {
    if (abs(ang) > 2.0) {
      turns = turns +1;
    }
  }
  prevang = ang;
  double turnMulti = (65534)/(totalTurnAngle*0.01745329252);
  ang = ang + (turns*6.28318530718); // pi*2
  ang = ang*turnMulti;
  //double tm= atan2(y,x)*10000;
  //int ang = tm/60;
  #else
  ang = (a0-512) * 64;
  #endif
  if( REV_STEERING == 1) {
    ang = -ang;
  }
  if (ang > 32766.0) {
    ang = 32766;
  }
  if (ang < -32766.0) {
    ang = -32766;
  }
  
  Joystick.setXAxis(0);
  Joystick.setYAxis(0);
  Joystick.setZAxis(0);
  //Joystick.setSteering(a3);
  //Joystick.setAccelerator(a4);
  Joystick.setBrake(0);


  handleButton(4,30,false);
  
  handleButton(5,0,false);
  
  handleButton(6,31,false);
  
  handleButton(7,1,false);


  handleMultiWheelButton(A8, 14, 15,10 , 4);

  
  //handleButton(14,2,true);
  
  //handleButton(15,3,true);
  
  Joystick.sendState();
  counter++;
  
  if (time1<millis() && setupMode) {
    Serial.println(counter);
    time1 = millis()+200;
    counter = 0;
    #ifdef GYRO_STEERING
      Serial.print("--- Raw data:");
    Serial.print("Raw AccX = ");
    Serial.print(mpu.GetRawAccX());
    Serial.print("Raw AccY = ");
    Serial.print(mpu.GetRawAccY());
    Serial.print("Raw AccZ = ");
    Serial.println(mpu.GetRawAccZ());
    Serial.print("angle = ");
    Serial.print(ang);
    Serial.print("angleRad = ");
    Serial.print(prevang);    
    Serial.print("turns = ");
    Serial.println(turns);
    #endif
    //currentButtonState = !currentButtonState;
  }
}
