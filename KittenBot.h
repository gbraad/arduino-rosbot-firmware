#ifndef KittenBot_h
#define KittenBot_h

#define M1_A 5
#define M1_B 6
#define M2_A 9
#define M2_B 10
#define M3_A 7
#define M3_B 8
#define M4_A 12
#define M4_B 13

class KittenBot
{
  private:
    bool motorRunning;
    void enableMotor(int m1, int m2, int m3, int m4);
    long stepPos[2];
  public:
    float ppm;
    float baseWidth;

    KittenBot();

    void resetTimer2();
    
    void init();
    void motorSetSpeed(int index, int speed);
    void motorRun2(int spd1, int spd2);
    void motorRun2Delay(int spd1, int spd2, int delayms);

    void motorRun4(int spd1, int spd2, int spd3, int spd4); // drive 4 motor
    void motorRun4Delay(int spd1, int spd2, int spd3, int spd4, int delayms); // drive 4 motor
    void motorStop(void);
    void omniWheel(int x, int y, int roll);
    
    void stepRun(long pos1, long pos2);
    void stepRun(long pos1, int spd1, long pos2, int spd2);
    void stepLine(float length);
    void stepTurn(float degree);
    void stepArc(float R, float degree);
    void stepMoveByIndex(int index, long pos, int speed);
    void stepMoveMultiple(long pos1, int speed1, long pos2, int speed2);
    
    void servoArray(int index, int degree, int speed);
    void geekServoArray(int index, int degree, int speed);
    void servoArrayWait();
    
    int buttonRead(int pin);
    void ledRead(int pin);
    float getBatteryVoltage();
    void rgbShow(int pin, int pix, int r, int g, int b);
    void rgbBrightness(int brightness);
    void extIoSet(int io1, int io2, int t);
    void extIo(int io1val, int io2val, int t);
    void buzzTone(int pin, int freq, int duration);

    void ps2attach();
    
    int ultrasonic(int pin);
    int ultrasonic(int trigPin, int echoPin);
    // LED matrix
    void matrixShow(const char * data);
    void matrixShowString(const char * str);
    void matrixShowString(String str);
    void matrixFillRect(int x, int y, int w, int h, int color);
    
    int ps2Init();
    int ps2ReadButton(int button);
    int ps2ReadAxis(int axis);
    void ps2Vibrate(int value);
	void ps2loop();
  protected:
    unsigned char MotorPin[8] = { 5,6,9,10,7,8,12,13 };

};
#endif
