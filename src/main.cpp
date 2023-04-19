#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include<bits/stdc++.h>
#include <SPI.h> 
#include "nRF24L01.h"
#include "RF24.h" 
#include "wificalib.h"
#include <Adafruit_BMP280.h>
#include "bmp.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ESP32Servo.h>
#include <MPU9250.h>
#include <Wire.h>

/******************DEBUG****************/
#define DEBUG // for adding and removing debug code 
//#define calib
//#define test//with local variables
//#define testy//with local variables
//#define webtest
//#define webtestyaw
#define bmp_debug
//#define DEBUG_MPU
//#define DEBUG_GPS
//#define PID
//#define DEBUG_NRF
//#define DEBUG_MOTORS_SPEED
//#define DEBUG_PID_VALUES
//#define fixi
//#define fixir
//#define fixiy
//#define fixi_motors

/************data**********/
float data[5];



/****************radio mid points*****************/
//Yaw:1473 Roll:1465 Pitch :1454

/*****************nrf******************/
#define CE 12
#define CSN 14
#define SCK 26
#define MOSI 27
#define MISO 25
/****************LEDS***************/
#define Bled 15
#define Gled 4
#define Rled 2
/*****************bmp280************/
Adafruit_BMP280 bmp;
float temp,altitude,pressure;
/******************gps***************/
static const int TXPin = 33, RXPin = 32;
static const uint32_t GPSBaud = 9600;
double lng,lat,speed;
uint16_t month,day,year;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

/****************************/


/*************Defines_for_motor***********/
#define ESC_0 23 
#define ESC_1 19
#define ESC_2 5
#define ESC_3 18
// minimum and maximum PWM pulse width
#define MIN_THROTTLE 1000  
#define MAX_THROTTLE 1800 
//servo objects
Servo MOTOR_0;
Servo MOTOR_1;
Servo MOTOR_2;
Servo MOTOR_3;


/**********Defines_for_mpu***************/
MPU9250 mpu;
int16_t rollAvr = 0;
int16_t pitchAvr = 0;
int16_t yawAvr = 0;
int16_t Roll=0, Pitch=0, Yaw=0;
bool calibrate = true;
void inline MPU_Angles_Avr(int16_t &rollavr, int16_t &pitchavr, int16_t &yawavr);
void inline Measured_Roll_Pitch_Yaw(int16_t & Roll, int16_t & Pitch, int16_t & Yaw);
/***************PID coef************/
double PID_PITCH_kp =1;
double PID_PITCH_kd =0;
double PID_PITCH_ki =0;

double PID_ROLL_kp =PID_PITCH_kp;
double PID_ROLL_kd =PID_PITCH_kd;
double PID_ROLL_ki =PID_PITCH_ki;

double PID_YAW_kp =0;
double PID_YAW_kd =0;
double PID_YAW_ki =0;

float previous_pitch_error = .0;
float previous_roll_error = .0;
float previous_yaw_error = .0;
/************functions declaration***************/
void update_data(double lng,double lat,double speed,double temp,double pressure);
void Readytogo();
void accCalib();
void magnCalib();
int16_t inline ExecutePitchPID(const int16_t& pitch_set_point, const int16_t& measured_pitch,const float &dt);
int16_t inline ExecuteRollPID(const int16_t& roll_set_point, const int16_t& measured_roll,const float &dt);
int16_t inline ExecuteYawPID(const int16_t& yaw_set_point, const int16_t& measured_yaw,const float &dt);
void inline UpdateMotorsValues( const int16_t throttle, const int16_t pitch_pid_output, const int16_t roll_pid_output, const int16_t yaw_pid_output);
void InfoGPS();
void Task1code( void *parameter);
void bmptask( void *parameter);
void gpstask( void *parameter);
void sendingtask( void *parameter);
/****************nrf***************/
//nrf oject win pins
RF24 receiver(12, 14, 26, 25, 27);
//RF24 (CE, CSN, SCK, MISO, MOSI); 

const uint64_t p= 0x00011111;//IMPORTANT: The same as in the transmitter
const uint64_t r= 0x00011001;
int16_t values_received[4];



//time 
long current_time=0,last_time=0;
//task
TaskHandle_t Task1;

void setup(void){
  //wifi task
  pinMode(16,OUTPUT);
  
    

xTaskCreatePinnedToCore(
      bmptask, /* Function to implement the task */
      "Task1", /* Name of the task */
      8000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      NULL,  /* Task handle. */
      0); /* Core where the task should run */
xTaskCreatePinnedToCore(
      gpstask, /* Function to implement the task */
      "Task2", /* Name of the task */
      8000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      NULL,  /* Task handle. */
      0); /* Core where the task should run */


#ifdef DEBUG
Serial.begin(115200);
#endif
//LEDS setting
pinMode(Bled,OUTPUT);
pinMode(Gled,OUTPUT);
pinMode(Rled,OUTPUT);


// setting-up the motors //
MOTOR_0.attach(ESC_0, MIN_THROTTLE, MAX_THROTTLE);
MOTOR_1.attach(ESC_1, MIN_THROTTLE, MAX_THROTTLE);
MOTOR_2.attach(ESC_2, MIN_THROTTLE, MAX_THROTTLE);
MOTOR_3.attach(ESC_3, MIN_THROTTLE, MAX_THROTTLE);


// setting-up nRF module //


receiver.begin();// Begin operation of the chip.
 receiver.setChannel(30);// Which RF channel to communicate on, 0-127
 receiver.setPayloadSize(32);//size of data trame 
 receiver.setDataRate(RF24_250KBPS);
 receiver.openWritingPipe(r);
 receiver.openReadingPipe(1,p);
 receiver.startListening();


//setting MPU module
  MPU9250Setting setting;
  setting.accel_fs_sel        = ACCEL_FS_SEL::A4G;
  setting.gyro_fs_sel         = GYRO_FS_SEL::G500DPS;
  setting.mag_output_bits     = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate    = FIFO_SAMPLE_RATE::SMPL_500HZ;
  setting.gyro_fchoice        = 0x03;
  setting.gyro_dlpf_cfg       = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice       = 0x01;
  setting.accel_dlpf_cfg      = ACCEL_DLPF_CFG::DLPF_45HZ;
  mpu.setMagneticDeclination(3.11f); //tunisia 26 mars


Wire.begin();
if (!mpu.setup(0x68))
{
#ifdef DEBUG_MPU
Serial.println("Failed to initialize MPU9250.");
#endif

  while (1)
  {
  }
}


  /* Default settings from datasheet. */


#ifdef calib
    accCalib();
mpu.calibrateAccelGyro();
    digitalWrite(Bled,0);
     magnCalib();
 mpu.calibrateMag();
    digitalWrite(Gled,0);
    Readytogo();
#endif
 

}

void loop(){
current_time=millis();
Measured_Roll_Pitch_Yaw(Roll, Pitch, Yaw);


if(receiver.available()){
receiver.read(values_received, sizeof(values_received));
digitalWrite(Bled,1);
  digitalWrite(Rled,1);

  #ifdef DEBUG_NRF
      Serial.print("throtlle:");
  Serial.print(values_received[0]);
   Serial.print("   Yaw:");
    Serial.print(values_received[1]);
   Serial.print("   Roll:");
    Serial.print(values_received[2]);
   Serial.print("   Pitch:");
    Serial.print(values_received[3]);
   Serial.println("   ");

   #endif 
delayMicroseconds(1000);
   //UpdateMotorsValues(values_received[0],PitchPIDOutput,0,0);
   #ifdef fixi_motors
MOTOR_0.writeMicroseconds(values_received[0]);
MOTOR_1.writeMicroseconds(values_received[0]);
MOTOR_2.writeMicroseconds(values_received[0]);
MOTOR_3.writeMicroseconds(values_received[0]);
  #endif
if(values_received[0]>1050){
  current_time=micros();
  float dt=(current_time-last_time)/1000000;
int16_t RollPIDOutput   = ExecuteRollPID(values_received[2], Roll,dt);
int16_t PitchPIDOutput  = ExecutePitchPID(values_received[3], Pitch,dt);
int16_t YawPIDOutput  = ExecuteYawPID(values_received[1], Yaw,dt);
#ifdef PID
    Serial.print("RollPIDOutput:");
  Serial.print(RollPIDOutput);
   Serial.print("   PitchPIDOutput:");
    Serial.print(PitchPIDOutput);
   Serial.print("   YawPIDOutput :");
    Serial.println(YawPIDOutput );
#endif
UpdateMotorsValues(values_received[0],RollPIDOutput,PitchPIDOutput ,YawPIDOutput);

}
else{
  MOTOR_0.writeMicroseconds(MIN_THROTTLE);
MOTOR_1.writeMicroseconds(MIN_THROTTLE);
MOTOR_2.writeMicroseconds(MIN_THROTTLE);
MOTOR_3.writeMicroseconds(MIN_THROTTLE);
}
}

if(!receiver.available()){
  digitalWrite(Bled,0);
  digitalWrite(Rled,1);
  #ifdef DEBUG_NRF
    Serial.println(" No signal !  ");
  #endif 
MOTOR_0.writeMicroseconds(MIN_THROTTLE);
MOTOR_1.writeMicroseconds(MIN_THROTTLE);
MOTOR_2.writeMicroseconds(MIN_THROTTLE);
MOTOR_3.writeMicroseconds(MIN_THROTTLE);
}
if(current_time%1000<=30){
receiver.stopListening();

    receiver.write(data, sizeof(data));
   
    receiver.startListening();
    
}




update_data(lng,lat,speed,temp,pressure);

last_time=current_time;
//delay(10);

}

void inline MPU_Angles_Avr(int16_t & rollavr, int16_t & pitchavr, int16_t & yawavr){
#ifdef DEBUG_MPU
Serial.print("Calibration start");
#endif
for (int i = 0; i < 2000; i++)
{
mpu.update();
float x_angle = mpu.getRoll();
float y_angle = mpu.getPitch();
float z_angle = mpu.getYaw();
rollavr += (x_angle/2000);
pitchavr += (y_angle/2000);
yawavr += (z_angle/2000);
delay(1);
}
#ifdef DEBUG_MPU
Serial.println("Calibration done");
#endif
}
//this function measures the roll, pitch and yaw
void inline Measured_Roll_Pitch_Yaw(int16_t & Roll, int16_t & Pitch, int16_t & Yaw){
if (mpu.update()) {
Roll = mpu.getRoll() - rollAvr -2;
Pitch = mpu.getPitch() - pitchAvr +2;
Yaw = mpu.getYaw() - yawAvr;
#ifdef DEBUG_MPU
Serial.print("Roll: ");
Serial.print(Roll);
Serial.print(" Pitch: ");
Serial.print(Pitch);
Serial.print(" Yaw: ");
Serial.println(Yaw);
#endif
}

}


int16_t inline ExecutePitchPID(const int16_t& pitch_set_point, const int16_t& measured_pitch,const float&dt) {
float error;
float integral = .0;
float proportional;
float derivative;
long psp=map(pitch_set_point,1000,2000,-54,66);
psp=max((long)-50,min(psp,(long)50));
if(psp<=3&&psp>=-3)psp=0;
error = static_cast<float>(psp- measured_pitch);
// if(error<=3 && error>=-3)
// error = 0;
#ifdef test
proportional = PID_PITCH_kp * error;
derivative =PID_PITCH_kd* (error - (previous_roll_error))/dt;
integral += error*PID_PITCH_ki*dt;
previous_roll_error = error;
#endif
#ifdef webtest
proportional = Kp * error;
derivative =Kd* (error - (previous_roll_error))/dt;
integral += Ki*error*dt;
previous_roll_error = error;
#endif
#ifdef fixi
Serial.print("kp:");
    Serial.print(Kp);
    Serial.print(" kd:");
    Serial.print(Kd);
    Serial.print(" ki:");
    Serial.print(Ki);
Serial.print(" correction:");
Serial.print( Ki* integral  );
Serial.print(" previous_pitch_error:");
Serial.print(previous_pitch_error );
Serial.print(" mesured pitch:");
Serial.print(measured_pitch);
Serial.print(" pitch:");
Serial.println(psp);
#endif
int16_t a=proportional +  integral + derivative;
a=constrain(a,-400,400);
return a;
}

int16_t inline ExecuteRollPID(const int16_t& roll_set_point, const int16_t& measured_roll,const float&dt) {
float error;
float integral = .0;
float proportional;
float derivative;
long rsp=map(roll_set_point,1000,2000,-52,68);
rsp=max((long)-50,min(rsp,(long)50));
if(rsp<=3&&rsp>=-3)rsp=0;

error = static_cast<float>(rsp- measured_roll);

// if(error<=3 && error>=-3)
// error = 0;
#ifdef test
proportional = PID_ROLL_kp * error;
derivative =PID_ROLL_kd* (error - (previous_roll_error))/dt;
integral += error* PID_ROLL_ki*dt;
previous_roll_error = error;
#endif
#ifdef webtest
proportional = Kp * error;
derivative =Kd* (error - (previous_roll_error))/dt;
integral += Ki*error*dt;
previous_roll_error = error;
#endif
#ifdef fixir
Serial.print(" error:");
Serial.print(proportional + integral + derivative);
Serial.print(" mesured roll:");
Serial.print(measured_roll);
Serial.print(" roll:");
Serial.println(rsp);
#endif
int16_t a=proportional +  integral + derivative;
a=constrain(a,-400,400);
return a;
}
int16_t inline ExecuteYawPID(const int16_t& yaw_set_point, const int16_t& measured_yaw,const float&dt) {
  float error=0;
  static float integral = .0;
  float proportional;
  float derivative;
  long ysp=map(yaw_set_point,1000,2000,-52,68);
ysp=max((long)-50,min(ysp,(long)50));
if(ysp<=3&&ysp>=-3)ysp=0;

error = static_cast<float>(ysp - measured_yaw);

//if(error<=3 && error>=-3)error = 0;

#ifdef testy
  proportional = PID_YAW_kp * error;
  derivative = PID_YAW_kd * (error - (previous_yaw_error))/dt;
  integral +=  PID_YAW_ki *error*dt;
 #endif 
   #ifdef webtestyaw
  proportional = Kp * error;
  derivative = Kd* (error - (previous_yaw_error))/dt;
  integral +=  Ki*error*dt;
 #endif  
  previous_yaw_error = error;
#ifdef fixiy
Serial.print(" error:");
Serial.print(proportional + integral + derivative);
Serial.print(" mesured yaw:");
Serial.print(measured_yaw);
Serial.print(" yaw:");
Serial.println(ysp);

#endif
long long a=proportional + integral + derivative;
a=constrain(a,-400,400);
  return (a);
}
void inline UpdateMotorsValues( const int16_t throttle, const int16_t pitch_pid_output,
const int16_t roll_pid_output, const int16_t yaw_pid_output) {
long m0 = throttle + pitch_pid_output - roll_pid_output +yaw_pid_output;
long m1 = throttle + pitch_pid_output + roll_pid_output - yaw_pid_output;
long m2 = throttle - pitch_pid_output + roll_pid_output + yaw_pid_output;
long m3 = throttle - pitch_pid_output - roll_pid_output - yaw_pid_output;



m0=max((long)MIN_THROTTLE,min((long)MAX_THROTTLE,m0));
m1=max((long)MIN_THROTTLE,min((long)MAX_THROTTLE,m1));
m2=max((long)MIN_THROTTLE,min((long)MAX_THROTTLE,m2));
m3=max((long)MIN_THROTTLE,min((long)MAX_THROTTLE,m3));
#ifdef DEBUG_MOTORS_SPEED
Serial.print(" m0 : ");
Serial.print(m0);
Serial.print(" m1 : ");
Serial.print(m1);
Serial.print(" m2 : ");
Serial.print(m2);
Serial.print(" m3 : ");
Serial.print(m3);
Serial.print("\n");
#endif
 MOTOR_0.writeMicroseconds(m0);
MOTOR_1.writeMicroseconds(m1);
 MOTOR_2.writeMicroseconds(m2);
MOTOR_3.writeMicroseconds(m3);
}
void update_data(double alt,double lat,double speed,double temp,double pressure){
data[0]=lng;
data[1]=lat;
data[2]=speed;
data[3]=temp;
data[4]=pressure;
}
void Readytogo(){
   
  digitalWrite(Gled,1);
  digitalWrite(Bled,1);
  MPU_Angles_Avr(rollAvr, pitchAvr, yawAvr);
  digitalWrite(Gled,0);
  digitalWrite(Bled,0);
}
void accCalib(){
  for(int i=0;i<6;i++){
  if(i%2){
    digitalWrite(Bled,1);
    delay(300);
  }
  else{
    digitalWrite(Bled,0);
    delay(300);
      }

}
}
void magnCalib(){
  for(int i=0;i<6;i++){
  if(i%2){
    digitalWrite(Gled,1);
    delay(300);
  }
else{
    digitalWrite(Gled,0);
    delay(300);
}
}
}

void InfoGPS()
{
 
  if (gps.location.isValid())
  {
    lat=gps.location.lat();
    lng=gps.location.lng();
    speed=gps.speed.mph();
    #ifdef DEBUG_GPS
    Serial.print(F("Location: ")); 
    Serial.print(lat, 6);
    Serial.print(F(","));
    Serial.print(lng, 6);
    Serial.print(F(" Speed: "));
    Serial.print(lng, 6);
    #endif
  }
  else
  {
    #ifdef DEBUG_GPS
    Serial.print(F("INVALID"));
     #endif
  }

 
  if (gps.date.isValid())
  {
    month=gps.date.month();
    day=gps.date.day();
    year=gps.date.year();
     #ifdef DEBUG_GPS
      Serial.print(F("  Date/Time: "));
    Serial.print(month);
    Serial.print(F("/"));
    Serial.print(day);
    Serial.print(F("/"));
    Serial.print(year);
    Serial.print("    ");
     #endif
  }
  else
  {
    #ifdef DEBUG_GPS
    Serial.print(F("INVALID"));
    #endif
  }

  #ifdef DEBUG_GPS
  if (gps.time.isValid())
  {
    
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
    
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  #endif
  #ifdef DEBUG_GPS
  Serial.println();
  #endif
}
void Task1code( void *parameter) {
  /***********setup*************/
  //setupwificalib();

  //gps
    

  /**********loop**************/
  while(1) {
  while (ss.available() > 0)
  if (gps.encode(ss.read()))
      InfoGPS();
     
  //mainwificalib();
  
  }
  
}
void gpstask( void *parameter){
ss.begin(GPSBaud);
while(1) {
  while (ss.available() > 0)
  if (gps.encode(ss.read()))
      InfoGPS();

    vTaskDelay(pdMS_TO_TICKS(10));  
  }
  vTaskDelete(NULL); 
}
void bmptask( void *parameter){
  //bmp
   if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
   while (1) delay(10);
  }
//bmp
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    while(1){
    if (bmp.takeForcedMeasurement()){
  temp=bmp.readTemperature();
  altitude=bmp.readAltitude(1021);
  pressure=bmp.readPressure();
  #ifdef bmp_debug
    Serial.print(F("Temperature = "));
    Serial.print(temp);
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(pressure);
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(altitude); /* manouba 13/4 */
    Serial.println(" m");

    Serial.println();
    #endif
    
              }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);              
}
void sendingtask(void * x){
  while(1){
    receiver.stopListening();
    receiver.write(data, sizeof(data));
    receiver.startListening();
    //delay(10);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  vTaskDelete(NULL); 
}

// blue:4
// purpil:3
// green:2
// yellow:1