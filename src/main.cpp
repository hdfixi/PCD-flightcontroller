#include <Arduino.h>
#include <SPI.h> 
#include "nRF24L01.h"
#include "RF24.h" 
//DEBUG
//#define DEBUG_MPU
#define DEBUG_NRF
//#define DEBUG_MOTORS_SPEED
//#define DEBUG_PID_VALUES



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

#define PID_PITCH_kp 2.3
#define PID_PITCH_ki 0
#define PID_PITCH_kd 0

#define PID_ROLL_kp PID_PITCH_kp
#define PID_ROLL_kd PID_PITCH_kd
#define PID_ROLL_ki PID_PITCH_ki

#define PID_YAW_kp 0
#define PID_YAW_kd 0
#define PID_YAW_ki 0

float previous_pitch_error = .0;
float previous_roll_error = .0;
void Readytogo();
void accCalib();
void magnCalib();
int16_t inline ExecutePitchPID(const int16_t& pitch_set_point, const int16_t& measured_pitch);
int16_t inline ExecuteRollPID(const int16_t& roll_set_point, const int16_t& measured_roll);

void inline UpdateMotorsValues( const int16_t throttle, const int16_t pitch_pid_output, const int16_t roll_pid_output, const int16_t yaw_pid_output);

#define DEBUG // for adding and removing debug code 


Servo MOTOR_0;
Servo MOTOR_1;
Servo MOTOR_2;
Servo MOTOR_3;


RF24 receiver(12, 14, 26, 25, 27);
//RF24 (CE, CSN, SCK, MISO, MOSI); //check link for class methods:https://maniacbug.github.io/RF24/classRF24.html

const uint64_t p= 0xE8E8F0F0E1LL;//IMPORTANT: The same as in the transmitter

int16_t values_received[4];

void setup(void){

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
  mpu.setMagneticDeclination(3.11); //tunisia 26 mars

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

    accCalib();
mpu.calibrateAccelGyro();
    digitalWrite(Bled,0);
    magnCalib();
mpu.calibrateMag();
    digitalWrite(Gled,0);
    Readytogo();


}


void loop(){
  int16_t RollPIDOutput   = ExecuteRollPID(values_received[2], Roll);
  int16_t PitchPIDOutput  = ExecutePitchPID(values_received[3], Pitch);
if(receiver.available()){
receiver.read(values_received, sizeof(int16_t)*4);
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
   UpdateMotorsValues(values_received[0],PitchPIDOutput,0,0);
   #ifdef DEBUG_MOTORS_SPEED
MOTOR_0.writeMicroseconds(values_received[0]);
MOTOR_1.writeMicroseconds(values_received[0]);
MOTOR_2.writeMicroseconds(values_received[0]);
MOTOR_3.writeMicroseconds(values_received[0]);
  #endif
}

else{
  #ifdef DEBUG_NRF
    Serial.println(" No signal !  ");
  #endif 
MOTOR_0.writeMicroseconds(MIN_THROTTLE);
MOTOR_1.writeMicroseconds(MIN_THROTTLE);
MOTOR_2.writeMicroseconds(MIN_THROTTLE);
MOTOR_3.writeMicroseconds(MIN_THROTTLE);
}
if (mpu.update()) {
    #ifdef DEBUG_MODE
      Serial.print(mpu.getYaw()); Serial.print(", ");
      Serial.print(mpu.getPitch()); Serial.print(", ");
      Serial.println(mpu.getRoll());
    #endif
    Measured_Roll_Pitch_Yaw(Roll, Pitch, Yaw);
  }




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
Roll = mpu.getRoll() - rollAvr - 2;
Pitch = mpu.getPitch() - pitchAvr +2;
Yaw = mpu.getYaw() - yawAvr;
#ifdef DEBUG_MPU
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
error = static_cast<float>(pitch_set_point - measured_pitch);
//if(error<=5 && error>=-5)
// error = 0;
proportional = PID_PITCH_kp * error;
derivative = PID_PITCH_kd * (error - (previous_roll_error));
integral += PID_PITCH_ki * (previous_roll_error + error)/2;
previous_pitch_error = error;
return static_cast<int16_t>(proportional + integral + derivative);
}

int16_t inline ExecuteRollPID(const int16_t& roll_set_point, const int16_t& measured_roll) {
float error;
float integral = .0;
float proportional;
float derivative;
error = roll_set_point - measured_roll;
//if(error<=10 && error>=-10)
//error = 0;
proportional = PID_ROLL_kp * error;
derivative =PID_ROLL_kd* (error - (previous_roll_error));
integral += PID_ROLL_ki* (previous_roll_error + error)/2;
previous_roll_error = error;
return static_cast<int16_t>(proportional + integral + derivative);
}
void inline UpdateMotorsValues( const int16_t throttle, const int16_t pitch_pid_output,
const int16_t roll_pid_output, const int16_t yaw_pid_output) {
int16_t m0 = throttle - pitch_pid_output - roll_pid_output + yaw_pid_output;
int16_t m1 = throttle - pitch_pid_output + roll_pid_output - yaw_pid_output;
int16_t m2 = throttle + pitch_pid_output - roll_pid_output - yaw_pid_output;
int16_t m3 = throttle + pitch_pid_output + roll_pid_output + yaw_pid_output;


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