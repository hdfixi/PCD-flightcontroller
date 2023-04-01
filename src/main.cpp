#include <Arduino.h>
#include<bits/stdc++.h>
#include <SPI.h> 
#include "nRF24L01.h"
#include "RF24.h" 

//bluetooth


//DEBUG
#define calib
//#define DEBUG_MPU
#define PID
//#define DEBUG_NRF
//#define DEBUG_MOTORS_SPEED
//#define DEBUG_PID_VALUES
//#define fixi
//#define fixir
//#define fixiy
//#define fixi_motors
//radio mid points 
//Yaw:1473 Roll:1465 Pitch :1454


#define CE 12
#define CSN 14
#define SCK 26
#define MOSI 27
#define MISO 25
//LEDS
#define Bled 15
#define Gled 4
#define Rled 2

//RF24(uint16_t _cepin, uint16_t _cspin, uint16_t sck, uint16_t miso, uint16_t mosi)
//RF24 receiver(12, 14, 26, 25, 27);
#include <ESP32Servo.h>
// Defines
#define ESC_0 23 
#define ESC_1 19
#define ESC_2 5
#define ESC_3 18
// minimum and maximum PWM pulse width
#define MIN_THROTTLE 1000  
#define MAX_THROTTLE 1800 

#include <MPU9250.h>
#include <Wire.h>

// Defines
MPU9250 mpu;
int16_t rollAvr = 0;
int16_t pitchAvr = 0;
int16_t yawAvr = 0;
int16_t Roll=0, Pitch=0, Yaw=0;
bool calibrate = true;
void inline MPU_Angles_Avr(int16_t &rollavr, int16_t &pitchavr, int16_t &yawavr);
void inline Measured_Roll_Pitch_Yaw(int16_t & Roll, int16_t & Pitch, int16_t & Yaw);

#define PID_PITCH_kp 0
#define PID_PITCH_kd 0
#define PID_PITCH_ki 0

#define PID_ROLL_kp PID_PITCH_kp
#define PID_ROLL_kd PID_PITCH_kd
#define PID_ROLL_ki PID_PITCH_ki

#define PID_YAW_kp 1
#define PID_YAW_kd 2
#define PID_YAW_ki 0

float previous_pitch_error = .0;
float previous_roll_error = .0;
float previous_yaw_error = .0;
void Readytogo();
void accCalib();
void magnCalib();
int16_t inline ExecutePitchPID(const int16_t& pitch_set_point, const int16_t& measured_pitch);
int16_t inline ExecuteRollPID(const int16_t& roll_set_point, const int16_t& measured_roll);
int16_t inline ExecuteYawPID(const int16_t& yaw_set_point, const int16_t& measured_yaw);
void inline UpdateMotorsValues( const int16_t throttle, const int16_t pitch_pid_output, const int16_t roll_pid_output, const int16_t yaw_pid_output);

#define DEBUG // for adding and removing debug code 


Servo MOTOR_0;
Servo MOTOR_1;
Servo MOTOR_2;
Servo MOTOR_3;


RF24 receiver(12, 14, 26, 25, 27);
//RF24 (CE, CSN, SCK, MISO, MOSI); //check link for class methods:https://maniacbug.github.io/RF24/classRF24.html

const uint64_t p= 0xE8E8F0F0E2LL;//IMPORTANT: The same as in the transmitter

int16_t values_received[4];

void setup(void){
//wireless upload 
 //WiFi_setup(1); WebSerial_setup();  OTA_setup();
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
 receiver.setChannel(2);// Which RF channel to communicate on, 0-127
 receiver.setPayloadSize(8);//size of data trame 
 receiver.setDataRate(RF24_250KBPS);
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
if (mpu.update()) {
Measured_Roll_Pitch_Yaw(Roll, Pitch, Yaw);
    #ifdef DEBUG_MPU
       Serial.print(", Yaw:");Serial.print(Yaw);
      Serial.print(", Pitch:");Serial.print(Pitch); 
      Serial.print(", Roll:");Serial.println(Roll);
      //delay(50);
    #endif
    
  }

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
   Serial.print("   ");
   int16_t PitchPIDOutput  = ExecutePitchPID(values_received[3], Pitch);
   #endif 

   //UpdateMotorsValues(values_received[0],PitchPIDOutput,0,0);
   #ifdef fixi_motors
MOTOR_0.writeMicroseconds(values_received[0]);
MOTOR_1.writeMicroseconds(values_received[0]);
MOTOR_2.writeMicroseconds(values_received[0]);
MOTOR_3.writeMicroseconds(values_received[0]);
  #endif
if(values_received[0]>1050){
int16_t RollPIDOutput   = ExecuteRollPID(values_received[2], Roll);
int16_t PitchPIDOutput  = ExecutePitchPID(values_received[3], Pitch);
int16_t YawPIDOutput  = ExecuteYawPID(values_received[1], Yaw);
#ifdef PID
    Serial.print("RollPIDOutput:");
  Serial.print(RollPIDOutput);
   Serial.print("   PitchPIDOutput:");
    Serial.print(PitchPIDOutput);
   Serial.print("   YawPIDOutput :");
    Serial.println(YawPIDOutput );
#endif
UpdateMotorsValues(values_received[0],PitchPIDOutput,RollPIDOutput ,YawPIDOutput);
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
Serial.print("Calibration done");
#endif
}
//this function measures the roll, pitch and yaw
void inline Measured_Roll_Pitch_Yaw(int16_t & Roll, int16_t & Pitch, int16_t & Yaw){
if (mpu.update()) {
Roll = mpu.getRoll() - rollAvr -2;
Pitch = mpu.getPitch() - pitchAvr +2;
Yaw = mpu.getYaw() - yawAvr;
#ifdef DEBUG_MPUU
Serial.print("Roll: ");
Serial.print(Roll);
Serial.print(" Pitch: ");
Serial.print(Pitch);
Serial.print(" Yaw: ");
Serial.print(Yaw);
Serial.print("\n");
#endif
}
}


int16_t inline ExecutePitchPID(const int16_t& pitch_set_point, const int16_t& measured_pitch) {
float error;
float integral = .0;
float proportional;
float derivative;
long psp=map(pitch_set_point,1000,2000,-54,66);
psp=max((long)-50,min(psp,(long)50));
if(psp<=3&&psp>=-3)psp=0;
error = static_cast<float>(psp- measured_pitch);
if(error<=3 && error>=-3)
error = 0;
proportional = PID_PITCH_kp * error;
derivative = PID_PITCH_kd * (error - (previous_pitch_error));
integral += PID_PITCH_ki * (previous_pitch_error + error)/2;
previous_pitch_error = error;
#ifdef fixi
Serial.print(" error:");
Serial.print(error );
Serial.print(" previous_pitch_error:");
Serial.print(previous_pitch_error );
Serial.print(" mesured pitch:");
Serial.print(measured_pitch);
Serial.print(" pitch:");
Serial.println(psp);
#endif
return (proportional + integral + derivative);
}

int16_t inline ExecuteRollPID(const int16_t& roll_set_point, const int16_t& measured_roll) {
float error;
float integral = .0;
float proportional;
float derivative;
long rsp=map(roll_set_point,1000,2000,-52,68);
rsp=max((long)-50,min(rsp,(long)50));
if(rsp<=3&&rsp>=-3)rsp=0;

error = static_cast<float>(rsp - measured_roll);

if(error<=3 && error>=-3)
error = 0;
proportional = PID_ROLL_kp * error;
derivative =PID_ROLL_kd* (error - (previous_roll_error));
integral += PID_ROLL_ki* (previous_roll_error + error)/2;
previous_roll_error = error;
#ifdef fixir
Serial.print(" error:");
Serial.print(proportional + integral + derivative);
Serial.print(" mesured roll:");
Serial.print(measured_roll);
Serial.print(" roll:");
Serial.println(rsp);
#endif
return (proportional + integral + derivative);
}
int16_t inline ExecuteYawPID(const int16_t& yaw_set_point, const int16_t& measured_yaw) {
  float error=0;
  static float integral = .0;
  float proportional;
  float derivative;
  long ysp=map(yaw_set_point,1000,2000,-52,68);
ysp=max((long)-50,min(ysp,(long)50));
if(ysp<=3&&ysp>=-3)ysp=0;

error = static_cast<float>(ysp - measured_yaw);

if(error<=3 && error>=-3)error = 0;


  proportional = PID_YAW_kp * error;
  derivative = PID_YAW_kd * (error - (previous_yaw_error));
  integral += PID_YAW_ki * (previous_yaw_error + error)/2;
  
    
  previous_yaw_error = error;
#ifdef fixiy
Serial.print(" error:");
Serial.print(proportional + integral + derivative);
Serial.print(" mesured yaw:");
Serial.print(measured_yaw);
Serial.print(" yaw:");
Serial.println(ysp);
#endif
  return (proportional + integral + derivative);
}
void inline UpdateMotorsValues( const int16_t throttle, const int16_t pitch_pid_output,
const int16_t roll_pid_output, const int16_t yaw_pid_output) {
long m0 = throttle - pitch_pid_output - roll_pid_output -yaw_pid_output;
long m1 = throttle - pitch_pid_output + roll_pid_output + yaw_pid_output;
long m2 = throttle + pitch_pid_output + roll_pid_output - yaw_pid_output;
long m3 = throttle + pitch_pid_output - roll_pid_output + yaw_pid_output;



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

// blue:4
// purpil:3
// green:2
// yellow:1