#include <tm1650.h>
#include "max6675.h"
#include "SCoop.h"
#include "Kalman.h"
#include <Wire.h>
#include <Math.h>
#include <SoftwareSerial.h> 
//Relay
#define relayPin 8
//Bluetooth
SoftwareSerial BT(10, 11);
char val;
//tone_pin
#define tone_pin 12
//max6675
#define thermoDO 4
#define thermoCS 5
#define thermoCLK 6
float temp_float=0;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
//EMG
#define analogInput A0
int value = 0, time = 0;
//tm1650
#define pin_SCK 3
#define pin_DIO 2
TM1650 tm1650(pin_SCK, pin_DIO);
#define ON DISPLAY_ON
#define OFF DISPLAY_OFF
#define SegmentMode _8_SEGMENT_MODE
char LightLevel = LV1;
unsigned char WorkMode = NORMAL_MODE;
const unsigned char Seg_test[8] = { 0x20, 0x01, 0x02, 0x04, 0x08, 0x10, 0x40, 0x80 };//define each segment
const unsigned char Number_arr[10] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f };//number 0-9 code
unsigned char i, j = 0, k = 0, l = 0, m = 0;
unsigned char temp[4] = { 0x3f, 0x06, 0x5b, 0x4f };
boolean err = 0;
unsigned char key_value = 0;
unsigned char show_buff[4];
unsigned char cel[4];
//mpu6050
float fRad2Deg = 57.295779513f; 
const int MPU = 0x68;
const int nValCnt = 7; 
const int nCalibTimes = 1000;
int calibData[nValCnt]; 
unsigned long nLastTime = 0; 
float fLastRoll = 0.0f; 
float fLastPitch = 0.0f; 
Kalman kalmanRoll; 
Kalman kalmanPitch; 
//scoop define
defineTask(temp_);
defineTask(EMG);
defineTask(angle);
/****************************************************************/
void temp_cov_dis(unsigned char* cel) {
  int cel_t = temp_float * 100;
  BT.print(cel_t);
  for (int i = 3; i >= 0; i--) {
    cel[i] = cel_t % 10;
    cel_t /= 10;
  }
  
  for (i = 0; i < 4; i++) {
    err = tm1650.DisplayOneDigi_TM1650(i + 1, Number_arr[cel[i]]);
    if (i == 1)
      err = tm1650.DisplayOneDigi_TM1650(i + 1, Number_arr[cel[i]] + Seg_test[7]);
  }
}

void EMG_detect() {
  value = analogRead(analogInput);
  Serial.println(value);
  sleep(500);
  while (value > 200 && time < 25) {
    sleep(200);
    time++;
  }
  while (value > 200) {
    BT.write("release");
    Serial.println("release");
    tone(tone_pin,2000,500);
    sleep(500);
  }
  time=0;
}

void WriteMPUReg(int nReg, unsigned char nVal) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.write(nVal);
  Wire.endTransmission(true);
}

unsigned char ReadMPUReg(int nReg) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.requestFrom(MPU, 1, true);
  Wire.endTransmission(true);
  return Wire.read();
}

void ReadAccGyr(int *pVals) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.requestFrom(MPU, nValCnt * 2, true);
  Wire.endTransmission(true);
  for (long i = 0; i < nValCnt; ++i) {
    pVals[i] = Wire.read() << 8 | Wire.read();
  }
}

void Calibration()
{
  float valSums[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0};
  for (int i = 0; i < nCalibTimes; ++i) {
    int mpuVals[nValCnt];
    ReadAccGyr(mpuVals);
    for (int j = 0; j < nValCnt; ++j) {
      valSums[j] += mpuVals[j];
    }
  }
  for (int i = 0; i < nValCnt; ++i) {
    calibData[i] = int(valSums[i] / nCalibTimes);
  }
  calibData[2] += 16384; 
}

float GetRoll(float *pRealVals, float fNorm) {
  float fNormXZ = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormXZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

float GetPitch(float *pRealVals, float fNorm) {
  float fNormYZ = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormYZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

void Rectify(int *pReadout, float *pRealVals) {
  for (int i = 0; i < 3; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 16384.0f;
  }
  pRealVals[3] = pReadout[3] / 340.0f + 36.53;
  for (int i = 4; i < 7; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 131.0f;
  }
}
/************************************************************/

//第一线程：temp
void temp_::setup() {
  err = tm1650.DisplayConfig_TM1650(LightLevel, SegmentMode, WorkMode, ON);
  sleep(200);
  pinMode(relayPin,OUTPUT);
}
void temp_::loop() {
  temp_float=thermocouple.readCelsius()-7.50;
  Serial.println(temp_float);
  if(temp_float<40)
  {
    digitalWrite(relayPin,LOW);
  }
  else
  {
    digitalWrite(relayPin,HIGH);
  }
  temp_cov_dis(cel);
  sleep(250);
}

//第二线程：EMG
void EMG::setup() {
  pinMode(analogInput, INPUT);
}
void EMG::loop() {
  EMG_detect();
}

//第三线程：mpu6050
void angle::setup(){
  Wire.begin(); 
  WriteMPUReg(0x6B, 0);
  Calibration(); 
  nLastTime = micros(); 
  pinMode(tone_pin,OUTPUT);
}
void angle::loop(){
int readouts[nValCnt];
  ReadAccGyr(readouts); 
  float realVals[7];
  Rectify(readouts, realVals); 
  float fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  float fRoll = GetRoll(realVals, fNorm); 
  if (realVals[1] > 0) {
    fRoll = -fRoll;
  }
  float fPitch = GetPitch(realVals, fNorm); 
  if (realVals[0] < 0) {
    fPitch = -fPitch;
  }
  unsigned long nCurTime = micros();
  float dt = (double)(nCurTime - nLastTime) / 1000000.0;
  float fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt);
  float fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);
  float fRollRate = (fNewRoll - fLastRoll) / dt;
  float fPitchRate = (fNewPitch - fLastPitch) / dt;
  fLastRoll = fNewRoll;
  fLastPitch = fNewPitch;
  nLastTime = nCurTime;
  Serial.print("Roll:");
  Serial.print(fNewRoll); 
  if(fNewRoll>30)
  {
    digitalWrite(tone_pin,HIGH);
    BT.println("danger");
  }
  else
  {
    digitalWrite(tone_pin,LOW);
  }
  sleep(100);
}
/************************************************************/

void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  mySCoop.start();
}

void loop() {
  yield();
}



















/*  
  while(1)
  {
    key_value = tm1650.Scan_Key_TM1650();  //获取按键值
    
  
    显示键值
    show_buff[0]=0;
    show_buff[1]=key_value/100;
    show_buff[2]=key_value/10%10;
    show_buff[3]=key_value%10;


    //转换按键值为1 2 3 4
    if(key_value==34){show_buff[3]=1;}
    else if(key_value==38){show_buff[3]=2;}
    else if(key_value==43){show_buff[3]=3;}
    else if(key_value==47){show_buff[3]=4;}
    else{show_buff[3]=0;}

    //显示键值1 2 3 4
    for(i = 0;i < 4; i++)
    {
      err = tm1650. DisplayOneDigi_TM1650(i+1,Number_arr[show_buff[i]]);
    } 
  }*/
