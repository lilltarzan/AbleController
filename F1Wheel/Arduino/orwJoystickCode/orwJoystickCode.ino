#define GYRO_STEERING

#include <Joystick.h>

#include <Arduino.h>
#ifdef GYRO_STEERING
#include <TinyMPU6050.h>
#endif
#define REV_STEERING 0
#define UPSIDEDOWN 0

double totalTurnAngle = 720; // 360 is 180 to each side. typical road car have ~900

bool setupMode = true;
/*
 *  Constructing MPU-6050
 */
#ifdef GYRO_STEERING
MPU6050 mpu (Wire);
#endif
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_GAMEPAD,
  32, 0,                  // Button Count, Hat Switch Count
  true, true, true,     // X and Y, but no Z Axis
  false, false, false,   // No Rx, Ry, or Rz
  true, true,          // No rudder or throttle
  false, true, false);  // No accelerator, brake, or steering

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(0,INPUT_PULLUP);
  pinMode(1,INPUT_PULLUP);
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  pinMode(5,INPUT_PULLUP);
  pinMode(7,INPUT_PULLUP);
  pinMode(15,INPUT_PULLUP);
  pinMode(14,INPUT_PULLUP);
  pinMode(16,INPUT_PULLUP);
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
  Joystick.setXAxisRange(-32767, 32767);
  
  //Joystick.setXAxisRange(-512, 512);
  Joystick.setYAxisRange(-512, 512);
}
long counter = 0;
long time1 = 0;
int button = 0; 
bool currentButtonState;
double prevang = 0.0;
int turns = 0;

void loop() {
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
  int x = mpu.GetRawAccZ();
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
  
  Joystick.setXAxis(ang);
  Joystick.setYAxis(0);
  Joystick.setZAxis(0);
  //Joystick.setSteering(a3);
  //Joystick.setAccelerator(a4);
  Joystick.setBrake(0);

  
  handleButton(5,0,false);
  
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

void handleButton(int pin, int buttonNr, bool reverse) {

  bool state = false;
  state = digitalRead(pin);
  if (reverse) {
    state = !state;
  }
  Joystick.setButton(buttonNr, state);
}

void handleMultiWheelButton(int aPort, int button1, int button2, int startButton, int sections) {
  int value = analogRead(aPort);
  int divider = 1024/sections;
  value = value/divider;

  handleButton(button1, value*2+startButton, true);
  handleButton(button2, value*2+startButton+1, true);
}
