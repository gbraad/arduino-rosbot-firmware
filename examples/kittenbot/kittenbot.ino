#include <Wire.h>
#include "KittenBot.h"
KittenBot kb;
#define FIRMWARE "Kittenbot V3.9\r\n"


// parse pin, 0~13 digital, 14.. analog pin
void parsePinVal(char * cmd, int * pin) {
  if (cmd[0] == 'A') {
    sscanf(cmd, "A%d\n", pin);
    *pin += A0;
  } else {
    sscanf(cmd, "%d\n", pin);
  }
}

void parsePinVal(char * cmd, int * pin, int * v0) {
  if (cmd[0] == 'A') {
    sscanf(cmd, "A%d %d\n", pin, v0);
    *pin += A0;
  } else {
    sscanf(cmd, "%d %d\n", pin, v0);
  }
}

void parsePinVal(char * cmd, int * pin, int * v0, int * v1) {
  if (cmd[0] == 'A') {
    sscanf(cmd, "A%d %d %d\n", pin, v0, v1);
    *pin += A0;
  } else {
    sscanf(cmd, "%d %d %d\n", pin, v0, v1);
  }
}

void parsePinVal(char * cmd, int * pin, int * v0, int * v1, int * v2, int * v3) {
  if (cmd[0] == 'A') {
    sscanf(cmd, "A%d %d %d %d %d\n", pin, v0, v1, v2, v3);
    *pin += A0;
  } else {
    sscanf(cmd, "%d %d %d %d %d\n", pin, v0, v1, v2, v3);
  }
}

void printPin(int pin) {
  if (pin >= 14) {
    Serial.print("A" + String(pin - 14));
  } else {
    Serial.print(String(pin));
  }
}

void echoPinValue(const char * code, int pin, int value) {
  Serial.print(code);Serial.print(" ");
  printPin(pin);Serial.print(" ");
  Serial.println(value);
}

void echoPinValue(const char * code, int pin, float value) {
  Serial.print(code);Serial.print(" ");
  printPin(pin);Serial.print(" ");
  Serial.println(value, 2);
}

void echoValue(const char * code, float value) {
  Serial.print(code);Serial.print(" ");
  Serial.println(value, 2);
}

void echoVersion() {
  Serial.print("M0 ");
  Serial.print(FIRMWARE);
}

void doPinMode(char * cmd) {
  int pin, mod;
  parsePinVal(cmd, &pin, &mod);
  pinMode(pin, mod);
}

void doDigitalWrite(char * cmd) {
  int pin, val;
  parsePinVal(cmd, &pin, &val);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, val);
}

void doDigitalRead(char * cmd) {
  int pin, val;
  parsePinVal(cmd, &pin);
  pinMode(pin, INPUT);
  val = digitalRead(pin);
  echoPinValue("M3", pin, val);
}

void doAnalogWrite(char * cmd) {
  int pin, val;
  parsePinVal(cmd, &pin, &val);
  if (pin == 3 || pin == 5 || pin == 6 || pin == 9 || pin == 10 || pin == 11) { // only work on 3,5,6,9,10,11
    analogWrite(pin, val);
  }
}

void doAnalogRead(char * cmd) {
  int pin, val;
  parsePinVal(cmd, &pin);
  pinMode(pin, INPUT);
  val = analogRead(pin);
  if (pin < 14) return;
  echoPinValue("M5", pin, val);
}

void doTone(char * cmd) {
  int pin, freq, t;
  parsePinVal(cmd, &pin, &freq, &t);
  kb.buzzTone(pin, freq, t);
}

void doEchoVin() {
  float v;
  v = kb.getBatteryVoltage();
  echoValue("M8", v);
}

void doRgb(char * cmd) {
  int pin, pix, r, g, b;
  parsePinVal(cmd, &pin, &pix, &r, &g, &b);
  kb.rgbShow(pin,pix,r,g,b);
}

void doRgbBrightness(char * cmd){
  int brightness;
  parsePinVal(cmd, &brightness);
  kb.rgbBrightness(brightness);
}

void doButton(char * cmd){
  int pin, val;
  parsePinVal(cmd, &pin);
  pinMode(pin, INPUT);
  val = !digitalRead(pin);
  echoPinValue("M10", pin, val);
}

void doOmniWheel(char * cmd){
  int vspeed = 0, hspeed = 0, rspeed=0;
  sscanf(cmd, "%d %d %d\n", &vspeed, &hspeed, &rspeed);
  kb.omniWheel(vspeed, hspeed, rspeed);
}

//--- M200 ----
void doDcSpeed(char *cmd)
{
  int index = 0, spd = 0;
  parsePinVal(cmd,&index,&spd);
  kb.motorSetSpeed(index, spd);
}

void doMotorDualDelay(char * cmd){
  int m1,m2,t;
  parsePinVal(cmd,&m1,&m2, &t);
  if(t == 0){
    kb.motorRun2((int)m1,(int)m2);  
  }else{
    kb.motorRun2Delay((int)m1,(int)m2, t);
    Serial.println("M204");
  } 
}

void doMotorFour(char * cmd){
  int m1,m2,m3,m4;
  sscanf(cmd, "%d %d %d %d\n", &m1, &m2, &m3, &m4);
  kb.motorRun4(m1, m2, m3, m4);
}

void doPing(char * cmd) {
  float distance;
  int trig, echo=-1;
  parsePinVal(cmd, &trig, &echo);
  if(echo>=0 && echo<A5){
    distance = kb.ultrasonic(trig,echo);  
  }else{
    distance = kb.ultrasonic(trig);
  }
  Serial.print("M222 ");
  Serial.println(distance);
}

// --- M100 ---
void doStepperSingle(char * cmd) {
  int idx, degree, speed;
  sscanf(cmd, "%d %d %d\n", &idx, &degree, &speed);
  long pos = (long)degree*2048/360;
  speed  = speed*2048/60;
  kb.stepMoveByIndex(idx, pos, speed);
  Serial.println("M100");
}

void doSetPPM(char * cmd) {
  float ppm = atof(cmd);
  kb.ppm = ppm;
}

void doSetWheelBase(char * cmd) {
  float baseWidth = atof(cmd);
  kb.baseWidth = baseWidth;    
}

void doStepperLine(char *cmd)
{
  int len;
  sscanf(cmd, "%d\n", &len);
  kb.stepLine(len);
  Serial.println("M101");
}

void doStepperTurn(char *cmd)
{
  int angle;
  sscanf(cmd, "%d\n", &angle);
  kb.stepTurn(angle);
  Serial.println("M102");
}

void doStepperArc(char *cmd)
{
  int diameter, angle;
  sscanf(cmd, "%d %d\n", &diameter, &angle);
  kb.stepArc(diameter,angle);
  Serial.println("M103");
}

void doServoArray(char * cmd){
  int idx, degree, speed;
  parsePinVal(cmd, &idx, &degree, &speed);
  kb.servoArray(idx, degree, speed);
}

void doGeekServoArray(char * cmd){
  int idx, degree, speed;
  parsePinVal(cmd, &idx, &degree, &speed);
  kb.geekServoArray(idx, degree, speed);
}

void doPS2Init(){
  int ret = kb.ps2Init();
  Serial.println("M220 "+String(ret));
}

void doPS2Axis(char * cmd){
  int axis;
  parsePinVal(cmd, &axis);
  int ret = kb.ps2ReadAxis(axis);
  Serial.println("M221 "+String(ret));
}

void doPS2Button(char * cmd){
  int button;
  parsePinVal(cmd, &button);
  int ret = kb.ps2ReadButton(button);
  Serial.println("M222 "+String(ret));
}

void doMatrixString(char * cmd){
  kb.matrixShowString(cmd);  
  Serial.println("M20");
}

void doMatrixShow(char * cmd){
  kb.matrixShow(cmd);
}

void doMatrixRect(char * cmd){
  int x,y,w,h,c;
  sscanf(cmd, "%d %d %d %d %d\n", &x, &y, &w, &h, &c);
  kb.matrixFillRect(x,y,w,h,c);
}

void doSoftReset() {
  asm volatile ("  jmp 0");
}

void doExtIO(char * cmd){
  int d12, d10, t;
  sscanf(cmd, "%d %d %d\n", &d12, &d10, &t);
  kb.extIo(d12, d10 ,t);
}

void doExtIOSet(char * cmd){
  int d12, d10, t;
  sscanf(cmd, "%d %d %d\n", &d12, &d10, &t);
  kb.extIoSet(d12, d10 ,t);
}


void parseMcode(char * cmd) {
  int code;
  char * tmp;
  code = atoi(cmd);
  cmd = strtok_r(cmd, " ", &tmp);

  switch (code) {
    case 0:
      echoVersion();
      break;
    case 1: // set pin mode: M1 pin mode
      doPinMode(tmp);
      break;
    case 2: // digital write: M2 pin level
      doDigitalWrite(tmp);
      break;
    case 3: // digital read: M3 pin
      doDigitalRead(tmp);
      break;
    case 4: // analog write: M4 pin pwm
      doAnalogWrite(tmp);
      break;
    case 5: // analog read: M5 pin
      doAnalogRead(tmp);
      break;
    case 6: // tone : M6 pin freq duration
      doTone(tmp);
      break;
    case 7: // servo : M7 pin degree
      // doServo(tmp);
      break;
    case 8: // read vin voltage
      doEchoVin();
      break;
    case 9: // rgb led
      doRgb(tmp);
      break;
    case 10: // button
      doButton(tmp);
      break;
    case 11: // rgb brightness
      doRgbBrightness(tmp);
      break;
    case 20:
      doMatrixString(tmp);
      break;
    case 21:
      doMatrixShow(tmp);
      break;
    case 22:
      doMatrixRect(tmp);
      break;      
    case 30:
      doExtIO(tmp);
      break;
    case 31:
      doExtIOSet(tmp);
      break;
    case 100: // single stepper movement
      doStepperSingle(tmp);
      break;
    case 101: // move in distance
      doStepperLine(tmp);
      break;
    case 102: // turn in degree
      doStepperTurn(tmp);
      break;
    case 103: // draw arc
      doStepperArc(tmp);
      break;
    case 104: // set ppm
      doSetPPM(tmp);
      break;
      case 105: // set wheel base
      doSetWheelBase(tmp);
      break;
    case 200:
      doDcSpeed(tmp);
      break;
    case 201:
      //doCarMove(tmp);
      break;
    case 203: // stop motors
      kb.motorStop();
      break;
    case 204:
      doMotorDualDelay(tmp);
      break;
    case 205:
      doMotorFour(tmp);
      break;
    case 209:
      doOmniWheel(tmp);
      break;
    case 212: // servo array
      doServoArray(tmp);
      break;
    case 213: // geek servo array
      doGeekServoArray(tmp);
      break;
    case 220:
      doPS2Init();
      break;   
    case 221:
      doPS2Axis(tmp);
      break;
    case 222:
      doPS2Button(tmp);
      break;
    case 250:
      doPing(tmp);
      break;
    case 999:
      doSoftReset();
      break;
  }
}


void parseCmd(char * cmd) {
  if (cmd[0] == 'M') { // mcode
    parseMcode(cmd + 1);
  }
}

void setup(){
  Serial.begin(115200);
  echoVersion();
  kb.init();
}

char buf[64];
int8_t bufindex;

void loop(){
  while (Serial.available()) {
    char c = Serial.read();
    buf[bufindex++] = c;
    if (c == '\n') {
      buf[bufindex] = '\0';
      parseCmd(buf);
      memset(buf, 0, 64);
      bufindex = 0;
    }
    if (bufindex >= 64) {
      bufindex = 0;
    }
  }
  kb.ps2loop();
}

