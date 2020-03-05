#include <Arduino.h>
#include"KittenBot.h"
#include <FlexiTimer2.h>
#include <VarSpeedServo.h>
#include "Adafruit_LEDBackpack.h"
#include "AccelStepper.h"
#include "MultiStepper.h"
#include "Adafruit_NeoPixel.h"
#include "PS2X_lib.h"
#include "TimerFreeTone.h"

int spdM[4];
bool enableM[4];
VarSpeedServo servo[10];
int counter = 0;
bool ps2enable = false;
byte vibrate = 0;
unsigned long ps2last = millis();
unsigned char servoPinMap[10]  ={4,7,8,11,12,13,A0,A1,A2,A3};

PS2X ps2x;
AccelStepper stpA(AccelStepper::FULL4WIRE, 5, 9, 6, 10);
AccelStepper stpB(AccelStepper::FULL4WIRE, 7, 12, 8, 13);
MultiStepper steppers;
Adafruit_NeoPixel rgbled(64);
Kittenbot_16x8matrix ledMat = Kittenbot_16x8matrix();

void motorCallback(){
    if(enableM[0] && counter == abs(spdM[0])){
      digitalWrite(M1_A, 0);
      digitalWrite(M1_B, 0);
    }
    if(enableM[1] && counter == abs(spdM[1])){
      digitalWrite(M2_A, 0);
      digitalWrite(M2_B, 0);
    }
    if(enableM[2] && counter == abs(spdM[2])){
      digitalWrite(M3_A, 0);
      digitalWrite(M3_B, 0);
    }
    if(enableM[3] && counter == abs(spdM[3])){
      digitalWrite(M4_A, 0);
      digitalWrite(M4_B, 0);
    }
    counter++;
    // reverse m1b and m2b by cc5
    if (counter >= 255) {
      if(enableM[0]){
        digitalWrite(M1_A, 0);
        digitalWrite(M1_B, 0);
      }
      if(enableM[1]){
        digitalWrite(M2_A, 0);
        digitalWrite(M2_B, 0);
      }
      if(enableM[2]){
        digitalWrite(M3_A, 0);
        digitalWrite(M3_B, 0);
      }
      if(enableM[3]){
        digitalWrite(M4_A, 0);
        digitalWrite(M4_B, 0);
      }
      counter = 0;
      if(enableM[0]){
        if (spdM[0] > 0) {
        digitalWrite(M1_A, 1);
        } else if (spdM[0] < 0) {
        digitalWrite(M1_B, 1);
        }
      }
      if(enableM[1]){
        if (spdM[1] > 0) {
        digitalWrite(M2_B, 1);
        } else if (spdM[1] < 0) {
        digitalWrite(M2_A, 1);
        }
      }
      if(enableM[2]){
        if (spdM[2] > 0) {
        digitalWrite(M3_A, 1);
        } else if (spdM[2] < 0) {
        digitalWrite(M3_B, 1);
        }
      }
      if(enableM[3]){
        if (spdM[3] > 0) {
        digitalWrite(M4_B, 1);
        } else if (spdM[3] < 0) {
        digitalWrite(M4_A, 1);
        }
      }
      
    }
}

void KittenBot::ps2loop(){
    // handle ps2x reading
	if(ps2enable && (millis()-ps2last) > 100){
	  ps2x.read_gamepad(false, vibrate);
	  ps2last = millis();
	}
}

KittenBot::KittenBot()
{

}

void KittenBot::init(){
  stpA.setMaxSpeed(600.0);
  stpA.setAcceleration(200.0);
  stpB.setMaxSpeed(600.0);
  stpB.setAcceleration(200.0);
  steppers.addStepper(stpA);
  steppers.addStepper(stpB);
  stepPos[0] = stepPos[1] = 0;
  enableM[0] = enableM[1] = 1;
  enableM[2] = enableM[3] = 0;
  // pulse per meter
  ppm = 14124;
  baseWidth = 0.118; // from master DI
  
  for(int i=0;i<8;i++){
    pinMode(MotorPin[i],OUTPUT);
    digitalWrite(MotorPin[i],0);
  }
  
  for(int i=0;i<10;i++){
    pinMode(servoPinMap[i],OUTPUT);
  }
  
  FlexiTimer2::set(1, 0.0001, motorCallback); // 100us, 255*100 = 25ms
  FlexiTimer2::start();
  
  rgbled.begin();
  rgbled.setBrightness(100);
}

void KittenBot::enableMotor(int m1, int m2, int m3, int m4){
  enableM[0] = m1;
  enableM[1] = m2;
  enableM[2] = m3;
  enableM[3] = m4;
}

void KittenBot::stepRun(long pos1, long pos2){
  stepRun(pos1,500,pos2,500);
}

void KittenBot::stepRun(long pos1, int spd1, long pos2, int spd2){
  enableMotor(0,0,0,0);
  stepPos[0] += pos1;
  stepPos[1] += pos2;
  stpA.setMaxSpeed(spd1);
  stpB.setMaxSpeed(spd2);
  steppers.moveTo(stepPos);
  steppers.runSpeedToPosition();
  stpA.disableOutputs();
  stpB.disableOutputs();
}

void KittenBot::stepMoveByIndex(int index, long pos, int speed){
  if(index == 1){
    stepRun(pos, speed, 0, 500);
  }else if(index == 2){
    stepRun(0, 500, pos, speed);
  }
}


void KittenBot::stepLine(float l){
    stepRun(-l*ppm/100, l*ppm/100);
}

void KittenBot::stepTurn(float d){
  ///180.0*3.141*KittenBot.BASE_WIDTH/2.0*KittenBot.PULSE_PER_METER
  float dis = d/180*3.14*ppm*baseWidth/2.0; // todo: the direction perform different to online mode
  stepRun(dis,dis);
}

void KittenBot::resetTimer2(){
    // servo[0].reset();
    
}


#define SPD_MAX 500.0f
void KittenBot::stepArc(float R, float degree){
  float L = baseWidth; // width of robot
  float V = 0.02; // linear speed cm/s
  float VL,VR; // linear speed
  float DL,DR; // distance
  float W; // angular speed
  float t; // time
  R = R/100; // cm -> m
  W = V/R;
  float spdRatio;
  float theta = -degree/180*PI; // change to rad
  VL = W*(1-L/2/R)*ppm; // change from m/s to pulse/s
  VR = W*(1+L/2/R)*ppm;
  //t = theta*R/V;
  DL = theta*(R-L/2)*ppm; // change from m to pulse
  DR = -theta*(R+L/2)*ppm;
  //Serial.print("#0 VL=");Serial.print(VL);
  //Serial.print(" ,VR=");Serial.print(VR);
  if(abs(VL)>abs(VR) && abs(VL)>SPD_MAX){
    spdRatio = SPD_MAX/VL;
    VL*=spdRatio;
    VR*=spdRatio;
  }else if(abs(VR)>SPD_MAX){
    spdRatio = SPD_MAX/VR;
    VL*=spdRatio;
    VR*=spdRatio;
  }
  //Serial.print(" ,VL=");Serial.print(VL);
  //Serial.print(" ,VR=");Serial.print(VR);
  //Serial.print(" ,DL=");Serial.print(DL);
  //Serial.print(" ,DR=");Serial.println(DR);

  stpA.move(DL);
  stpA.setSpeed(VL);
  stpB.move(DR);
  stpB.setSpeed(VR);

  while(stpA.distanceToGo()!=0 || stpB.distanceToGo()!=0){
    stpA.runSpeedToPosition();
    stpB.runSpeedToPosition();
  }
  // just ignore arc move 
  stpA.setCurrentPosition(stepPos[0]);
  stpB.setCurrentPosition(stepPos[1]);
}

void KittenBot::motorStop(){
  noInterrupts();
  for(int i=0;i<4;i++) spdM[i] = 0;
  //enableMotor(0,0,0,0);
  interrupts();
}

void KittenBot::motorRun2(int m1, int m2){
  noInterrupts();
  enableMotor(1,1,0,0);
  spdM[0] = m1;
  spdM[1] = m2;
  interrupts();
}

void KittenBot::motorRun2Delay(int m1, int m2, int delayms){
  motorRun2(m1, m2);
  delay(delayms);
  motorStop();
}

void KittenBot::motorRun4(int m1, int m2, int m3, int m4){
  noInterrupts();
  enableMotor(1,1,1,1);
  spdM[0] = m1;
  spdM[1] = m2;
  spdM[2] = m3;
  spdM[3] = m4;
  interrupts();
}

const float cos45 = cos(PI/4);
void KittenBot::omniWheel(int hspeed, int vspeed, int rspeed){
  int spdM1, spdM2, spdM3, spdM4;
  int tspd;
  // then map into 4 wheels
  tspd = vspeed/cos45;
  spdM1 = spdM4 = tspd;
  spdM2 = spdM3 = -tspd;
  tspd = hspeed/cos45;
  spdM1+=tspd;
  spdM2+=tspd;
  spdM3-=tspd;
  spdM4-=tspd;
  // no mapping for rotate
  spdM1+=rspeed;
  spdM2+=rspeed;
  spdM3+=rspeed;
  spdM4+=rspeed;
  // limit max and min value for each motor
  spdM1 = constrain(spdM1,-255,255);
  spdM2 = constrain(spdM2,-255,255);
  spdM3 = constrain(spdM3,-255,255);
  spdM4 = constrain(spdM4,-255,255);
  /*
  Serial.print("V=");Serial.print(vspeed);
  Serial.print(" ,H=");Serial.print(hspeed);
  Serial.print(" ,R=");Serial.print(rspeed);  
  Serial.print(" ,M1=");  Serial.print(spdM1);
  Serial.print(" ,M2=");  Serial.print(spdM2);
  Serial.print(" ,M3=");  Serial.print(spdM3);
  Serial.print(" ,M4=");  Serial.println(spdM4);
  */
  motorRun4(spdM1,spdM2,spdM3,spdM4);
}

void KittenBot::motorRun4Delay(int m1, int m2, int m3, int m4, int delayms){
  motorRun4(m1, m2, m3, m4);
  delay(delayms);
  motorStop();
}

void KittenBot::motorSetSpeed(int idx, int spd){
  noInterrupts();
  enableM[idx] = 1;
  spdM[idx] = spd;
  interrupts();
}

void KittenBot::rgbShow(int pin, int pix, int r, int g, int b){
  rgbled.setPin(pin);
  if(pix==0){
    for(int i=0;i<256;i++){
      rgbled.setPixelColor(i, r, g, b);
    }
  }else{
    rgbled.setPixelColor(pix-1, r, g, b);  
  }
  rgbled.show();  
}

void KittenBot::rgbBrightness(int brightness){
    rgbled.setBrightness(brightness);
}

float KittenBot::getBatteryVoltage()
{
  int a = analogRead(A7);
  float v = float(a) / 1024.0 * 5.2 * 2;
  return v;
}

int KittenBot::ultrasonic(int trigPin, int echoPin)
{
  float distance;
  unsigned int temp;
  pinMode(trigPin, OUTPUT); 
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT);
  temp = pulseIn(echoPin, HIGH, 300000);
  distance = (float)temp / 58.2;
  if(distance > 6){
      distance *= 1.28;
  }
  if(distance == 0){
      distance = 999;
  }
  return distance;
}

int KittenBot::ultrasonic(int pin)
{
  return ultrasonic(pin,pin);
}

#define PS2_DAT A3
#define PS2_CMD A2
#define PS2_SEL A1
#define PS2_CLK A0

int KittenBot::ps2Init(){
    int retry = 10;
    while(retry-->0){
        int error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);
        if(error == 0){
          ps2enable = true;
          ps2Vibrate(250);delay(500);
          ps2Vibrate(0);delay(200);
          ps2Vibrate(250);delay(500);
          ps2Vibrate(0);
          return 0;
        }
        
    }
    return -1;      

}

void KittenBot::ps2Vibrate(int value){
    noInterrupts();
    vibrate = value;
    interrupts();
}

int KittenBot::ps2ReadAxis(int axis){
    return ps2x.Analog(axis);
}

int KittenBot::ps2ReadButton(int btn){
    return ps2x.Analog(btn);
}

uint16_t mat[8];
char tmp[5] = "0000";
void KittenBot::matrixShow(const char * cmd){
  int index = 0;
  for(int i=0;i<32;i+=4){
    tmp[2] = cmd[i];
    tmp[3] = cmd[i+1];
    tmp[0] = cmd[i+2];
    tmp[1] = cmd[i+3];
    mat[index] = strtoul(tmp, NULL, 16);
    //Serial.print(String(mat[index], 16)+" ");
    index++;
  }
  //Serial.println("M21");
  ledMat.begin(0x70);
  ledMat.clear();
  ledMat.drawBitmap(0, 0, (uint8_t *)mat, 16, 8, LED_ON);
  ledMat.writeDisplay();
  
}

void KittenBot::matrixShowString(const char * str){
  int len = strlen(str);
  ledMat.begin(0x70);
  ledMat.clear();
  ledMat.setTextSize(0);
  ledMat.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
  ledMat.setTextColor(LED_ON);
  if (len <= 4){
    ledMat.setCursor(0,0);
    ledMat.print(str);
    ledMat.writeDisplay();
    return;
  }
  
  int offset = -(len+2)*5;
  for (int x=0; x>=offset; x--) {
    ledMat.clear();
    ledMat.setCursor(x,0);
    ledMat.print(str);
    ledMat.writeDisplay();
    delay(100);
  }
}

void KittenBot::matrixShowString(String str){
    matrixShowString(str.c_str());
}

void KittenBot::matrixFillRect(int x, int y, int w, int h, int color){
    ledMat.begin(0x70);
    ledMat.fillRect(x, y, w, h, color);
    ledMat.writeDisplay();
}


void KittenBot::servoArray(int idx, int degree, int speed){
    if(!servo[idx].attached()){
        servo[idx].attach(servoPinMap[idx]);
    }
    servo[idx].write(degree, speed);
}

void KittenBot::geekServoArray(int idx, int degree, int speed){
    servoArray(idx, (degree-90)*2/3+90, speed);
}

int extio1, extio2;
void KittenBot::extIoSet(int io1, int io2, int t){
    extio1 = io1;
    extio2 = io2;
    pinMode(extio1, OUTPUT);
    pinMode(extio2, OUTPUT);
    digitalWrite(extio1, 0);
    digitalWrite(extio2, 0);
    delay(t);
    digitalWrite(extio1, 1);
    digitalWrite(extio2, 1);
    
}
void KittenBot::extIo(int io1val, int io2val, int t){
    digitalWrite(extio1, io1val);
    digitalWrite(extio2, io2val);
    delay(t);
    digitalWrite(extio1, 1);
    digitalWrite(extio2, 1);
}

void KittenBot::buzzTone(int pin, int freq, int t){
    TimerFreeTone(pin, freq, t);
}



