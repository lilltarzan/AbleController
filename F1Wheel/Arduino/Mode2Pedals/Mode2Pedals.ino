// ####### Configuration values
//#define GYRO_STEERING

#define THROTTLE_MAX 900 // Analog value at maximum position
#define THROTTLE_MIN 110 // Analog value at minimum position
#define THROTTLE_REV 1  // Set to 1 to reverse or 0 for normal
#define THROTTLE_PIN A8

#define BRAKE_MAX 800 // Analog value at maximum position
#define BRAKE_MIN 630 // Analog value at minimum position
#define BRAKE_REV 1  // Set to 1 to reverse or 0 for normal
#define BRAKE_PIN A9

#define CLUTCH_MAX 950 // Analog value at maximum position
#define CLUTCH_MIN 250 // Analog value at minimum position
#define CLUTCH_REV 1  // Set to 1 to reverse or 0 for normal
#define CLUTCH_PIN A10

// NOTE! These steering values is not used if gyro steering is in use
#define STEERING_MAX 1023 // Analog value at maximum position
#define STEERING_MIN 1 // Analog value at minimum position
#define STEERING_REV 0  // Set to 1 to reverse or 0 for normal
#define STEERING_PIN A7

// Flip gyro axis:
#define REV_STEERING 0
#define UPSIDEDOWN 0

double totalTurnAngle = 360; // 360 is 180 to each side. typical road car have ~900
#define ALLOW_MULTITURNS


// ####### End of Configuration

#include <Joystick.h>

#include <Arduino.h>
#ifdef GYRO_STEERING
#include <TinyMPU6050.h>
#endif



bool setupMode = false;
/*
 *  Constructing MPU-6050
 */
#ifdef GYRO_STEERING
MPU6050 mpu (Wire);
#endif
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  2, 0,                  // Button Count, Hat Switch Count
  false, false, true,     // X and Y, but no Z Axis
  false, false, false,   // No Rx, Ry, or Rz
  false, false,          // No rudder or throttle
  true, true, false);  // No accelerator, brake, or steering

long counter = 0;
long time1 = 0;
int button = 0; 
bool currentButtonState;
double ang;
double prevang = 0.0;
int turns = 0;

int th;
int thRaw;
int br;
int brRaw;
int cl;
int clRaw;


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
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(BRAKE_PIN, INPUT);
  pinMode(CLUTCH_PIN, INPUT);
  Serial.println("started..");
  delay(1000);
  Serial.print("1");
  
  // wait 1s so we can reprogram device incase of flooding on serial port
    // Initialize Joystick Library
  // Initialization

  #ifdef GYRO_STEERING
  mpu.Initialize();
  Serial.println("Gyro active");
  #endif

  Joystick.begin(false);
  Joystick.setXAxisRange(-32767, 32767);
  Joystick.setAcceleratorRange(0, 1023);
  Joystick.setBrakeRange(0, 1023);
  Joystick.setYAxisRange(-512, 512);
  Joystick.setZAxisRange(0, 1023);
}


void loop() {
#ifdef GYRO_STEERING
  mpu.Execute();
  gyroSteering();
#endif

  // Throttle
  thRaw = analogRead(THROTTLE_PIN);
  if(THROTTLE_REV == 1) {
    th = map(thRaw, THROTTLE_MAX, THROTTLE_MIN,0,1023);
  } else {
    th = map(thRaw, THROTTLE_MIN, THROTTLE_MAX,0,1023);
  }
  th = limit(th, 0, 1023);
  Joystick.setAccelerator(th);

  // Brake
  brRaw = analogRead(BRAKE_PIN);
  if(BRAKE_REV == 1) {
    br = map(brRaw, BRAKE_MAX, BRAKE_MIN,0,1023);
  } else {
    br = map(brRaw, BRAKE_MIN, BRAKE_MAX,0,1023);
  }
  br = limit(br, 0, 1023);
  Joystick.setBrake(br);


    // Clutch
  clRaw = analogRead(CLUTCH_PIN);
  if(CLUTCH_REV == 1) {
    cl = map(clRaw, CLUTCH_MAX, CLUTCH_MIN,0,1023);
  } else {
    cl = map(clRaw, CLUTCH_MIN, CLUTCH_MAX,0,1023);
  }
  cl = limit(cl, 0, 1023);
  Joystick.setZAxis(cl);

  
  Joystick.setYAxis(0);
  //Joystick.setZAxis(0);
  //Joystick.setSteering(a3);
  //Joystick.setAccelerator(a4);
  //Joystick.setBrake(0);

  
  handleButton(5,0,true);
  handleButton(7,1,true);

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
    Serial.print("Throttle, Raw: ");
    Serial.print(thRaw);
    Serial.print("map:");
    Serial.print(th);

    Serial.print("Brake, Raw: ");
    Serial.print(brRaw);
    Serial.print("map:");
    Serial.print(br);

    Serial.print("Clutch, Raw: ");
    Serial.print(clRaw);
    Serial.print("map:");
    Serial.println(cl);
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

int limit(int value, int m1, int m2) {
  if (value < m1) {
    value = m1;  
  }
  if (value > m2) {
    value = m2;
  }
  return value;
}
#ifdef GYRO_STEERING
void gyroSteering() {


  int x = mpu.GetRawAccZ();
  int y = mpu.GetRawAccY();
  int unusedaxis = mpu.GetRawAccX();
  
  if(UPSIDEDOWN == 0) {
    ang = atan2(y,x);
  
  }else {
    ang = atan2(y,-x);

  }
  //double ang = atan2(y,x);
  #ifdef ALLOW_MULTITURNS
  if (prevang < 0 && ang > 0) {
    if (abs(ang) > 2.0 && abs(prevang) > 2.0) {
      turns = turns -1;
    }
    
  }
  if (prevang > 0 && ang < 0) {
    if (abs(ang) > 2.0 && abs(prevang) > 2.0) {
      turns = turns +1;
    }
  }
  #endif
  prevang = ang;
  double turnMulti = (65534)/(totalTurnAngle*0.01745329252);
  ang = ang + (turns*6.28318530718); // pi*2
  ang = ang*turnMulti;
  //double tm= atan2(y,x)*10000;
  //int ang = tm/60;
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
}
#endif
  
