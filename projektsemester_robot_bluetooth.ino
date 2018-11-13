#include<Wire.h>
#define LED_PIN 13 
bool blinkState = false;
///////////////////////////////////   CONFIGURATION HC-RS04/////////////////////////////
/*#include <NewPing.h>
#define TRIGGER_PIN 4
#define ECHO_PIN 3
#define MAX_DISTANCE 10
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
*////////////////////////////////////   CONFIGURATION Motor/////////////////////////////
#include <LMotorController.h>
double Speed;
int newspeed = 40;
int MIN_ABS_SPEED=40;
int forwardR=9;
int backwardR=8;
int tempocontrolR=10;
bool left =false , right=false;
int forwardL=6;
int backwardL=7;
int tempocontrolL=11;
LMotorController motorController(tempocontrolL, forwardL, backwardL, tempocontrolR, forwardR, backwardR, 1, 1);
///////////////////////////////////   CONFIGURATION MPU6050/////////////////////////////
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "I2Cdev.h"
int buffersize = 1000;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
const int MPU_addr=0x68;
double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //These will be the raw data from the MPU6050.
double compAngleX, compAngleY; //These are the angles in the complementary filter
#define degconvert 57.2957786 //there are like 57 degrees in a radian.
#define CONVERSIONG 3.9
MPU6050 accelgyro (0x68); // <-- use for AD0 high
int16_t ax, ay, az, gx, gy, gz;
double roll, pitch;
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
///////////////////////////////////   MAIN CONFIGURATION Arduino /////////////////////////////
#define CORRECT 0
long t = 0;
unsigned long lasttime = 0;
double deltat, lastangle = 0;
double setpoint = 0 , input = 0 , output = 0, error,lasterror=0;
double P = 0,
       I = 0,
       D = 0;
/*double set_kp = 54,
       set_ki = 284.5,
       set_kd =0.05;*/ 
double set_kp = 42,
       set_ki = 700,
       set_kd =0.63;
     // double set_kp = 48,
      //set_ki = 106,
      //set_kd =0.3;
float kp = 0,
      ki = 0,
      kd = 0;      
////errorofset -1.5 für Kabel, -0.35 für Batterie- negative heißt nach vorne
double erroroffset =-1.5;
double dta = 0;
MPU6050 mpu;
/////////////////////////////////////////////Definierung Bluetooth////////////////////////////
#include <SoftwareSerial.h>
int btstate = 0;
const int rxpin = 5; // pin used to receive 
const int txpin = 12; // pin used to send 
SoftwareSerial BTserial(rxpin, txpin);
////////void setup///////////////////////////////
void setup() {
  Wire.begin();
  #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up theMPU-6050)
  Wire.endTransmission(true);
  Serial.begin(115200);
  BTserial.begin(9600);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  ////MPU0605 calibration///////////////////
  //mpu_init();
  //getcalibration();
  pinMode(LED_PIN, OUTPUT);
  pinMode(tempocontrolR, OUTPUT);
  pinMode(tempocontrolL, OUTPUT);
  pinMode(forwardL, OUTPUT);
  pinMode(backwardL, OUTPUT);
  pinMode(forwardR, OUTPUT);
  pinMode(backwardR, OUTPUT);
}
/////////void loop //////////////
unsigned long actualtime;
void loop() { 
  mpu.setXGyroOffset(68);
  mpu.setYGyroOffset(-66);
  mpu.setZGyroOffset(91);
  mpu.setZAccelOffset(1061);
  actualtime = millis();
  deltat = (actualtime - lasttime); 
  blinkState = !blinkState;         ////LED blinkt/////////////////// 
  digitalWrite(LED_PIN, blinkState);
  anglempu();
  input = compAngleX;               ////set input = X-angle/////////
  setpoint = erroroffset;
  if (BTserial.available()) {BLUE();}
  PID();
  correctparam();
  //checkstatus
                                    ////show Parameter///////////
    Serial.print(" ");
    Serial.print(actualtime);
    Serial.print(" ");
    Serial.print(setpoint);
    Serial.print(" ");
    Serial.print(kp);
    Serial.print(" ");
    Serial.print(ki);
    Serial.print(" ");
    Serial.print(kd);
    Serial.print(" ");
    Serial.print(" input ");
    Serial.print(input);
    Serial.print(" ");
    Serial.print("  error  ");
    Serial.print(error);
    Serial.print(" ");
    Serial.print(" OUTPUT ");
    Serial.print(output);
    Serial.print(" ");
    Serial.print(" P ");
    Serial.print(P);
    Serial.print(" I ");
    Serial.print(I);
    Serial.print(" D ");
    Serial.println(D);
    kp = set_kp;
    ki = set_ki;
    kd = set_kd;
    lasttime = actualtime;
}
void BLUE(){   
    BTserial.write(Serial.read());
    btstate = BTserial.read(); // Reads the data from the serial port
    //Serial.print(btstate);
  // Controlling the LED
  if (btstate == '4') {
    //Serial.print("4");
    double compensspeed=-Speed;
    analogWrite(tempocontrolR, map(abs(Speed+compensspeed+newspeed),0,255,40,255));
    analogWrite(tempocontrolL, map(abs(Speed+compensspeed+newspeed),0,255,40,255));
    digitalWrite(forwardL, HIGH);digitalWrite(backwardL, LOW);
    digitalWrite(forwardR, LOW);digitalWrite(backwardR, HIGH);
    digitalWrite(4, HIGH); // LED ON
    delay(10);
    state = 0;    ///Reset state
  }
    else if (btstate == '3') {
      //Serial.print("3");
    double compensspeed=-Speed;
    analogWrite(tempocontrolR, map(abs(Speed+compensspeed+newspeed),0,255,40,255));
    analogWrite(tempocontrolL, map(abs(Speed+compensspeed+newspeed),0,255,40,255));
    digitalWrite(forwardL, LOW);digitalWrite(backwardL, HIGH);
    digitalWrite(forwardR, HIGH);digitalWrite(backwardR, LOW);
    digitalWrite(3, HIGH); // LED ON
    delay(10);
    state = 0;
  }
    else if (btstate == '1') {
    //Serial.print("1");
    erroroffset=erroroffset-0.75;
    delay(10);   
    digitalWrite(13, LOW); // LED OFF - checking only if this feature run
    state = 0;
  }
    else if (btstate == '7') {
    //Serial.print("reset state");
    erroroffset=-1.5;
    delay(10);   
    state = 0;
    PID();
  }
    else if (btstate == '2') {
    //Serial.print("2");
    erroroffset=erroroffset+0.75;
    delay(10);   
    digitalWrite(13, LOW); // LED ON
    state = 0;
  }
    else if (btstate == '8') {
    //Serial.print("reset");
    erroroffset=-1.5;
    delay(10);   
    state = 0;
  }
  else if (btstate == '5') {
    //Serial.print("5");
    digitalWrite(4, HIGH); // LED ON
    erroroffset=erroroffset+0.1;
    state = 0;
  }
    else if (btstate == '6') {
      //Serial.print("6");
    digitalWrite(4, HIGH); // LED ON
    erroroffset=erroroffset-0.1;
    state = 0;
  }
  ///////////////No need! because this code made delta t of PID run longer////////////
      //else if (btstate == '0') {
      //Serial.print("0");
    //digitalWrite(4, HIGH); // LED ON
    //state = 0;
  //}
}
//////////////////////////PID Calculation//////////////////////////////////////////
void PID(){     
    error = input - setpoint;
    P = kp * error;
    I += (ki * error) * (deltat/1000);
    I = constrain(I, -300, 300);
    dta = error - lasterror;
    D =  kd * dta / (deltat/1000);
    output = P + I + D;
    output = constrain(output, -255, 255);
    Speed=output;   
    motorController.move(constrain(Speed,-225,225), MIN_ABS_SPEED);
    lasterror=error; 
    lastangle = input;  
}
///////////////////////Start if kp,ki,kd Parameter conrection if any entry from  Serial Com-Port////////////////////// 
void correctparam(){
    #if CORRECT
        correct();
    #endif
  }
//////////////////////////////////////////Correct Kp , Ki,Kd,offsetangle////////////////////
void correct(){
     if (Serial.available()) {
      char p = Serial.read();
      switch (p) {
        case 'q':
          set_kp += 1;
          break;
        case 'w':
          set_kp -= 1;
          break;
        case 'e':
          set_kp += .1;
          break;
        case 'r':
          set_kp -= .1;
          break;
        case 'a':
          set_ki += 1;
          break;
        case 's':
          set_ki -= 1;
          break;
        case 'd':
          set_ki += .1;
          break;
        case 'f':
          set_ki -= .1;
          break;
        case 'y':
          set_kd += .01;
          break;
        case 'x':
          set_kd -= .01;
          break;
        case 'c':
          set_kd += .001;
          break;
        case 'v':
          set_kd -= .001;
          break;
      }}
     }
////////////////////////////////////////// mpu_init & calibration//////////////////////////////////////// 
void mpu_init(){
//   initialize device
  accelgyro.initialize();
   //start message
  //Serial.println("\nMPU6050 Calibration Sketch\n");
  delay(20);
  // verify connection
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  delay(10);
  // reset offsets
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);
}
///////////////////////////////////   Getcalibration   ////////////////////////////////////
void getcalibration(){
  if (state == 0) {
    meansensors();
    state++;
    delay(1000);
  }
  if (state == 1) {
    //Serial.println("\nCalculating offsets...");
    calibration();
    state++;
    delay(100);
  }
  if (state == 2) {
    meansensors();
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor readings with offsets:\t");
    Serial.print(mean_ax);
    Serial.print("\t");
    Serial.print(mean_ay);
    Serial.print("\t");
    Serial.print(mean_az);
    Serial.print("\t");
    Serial.print(mean_gx);
    Serial.print("\t");
    Serial.print(mean_gy);
    Serial.print("\t");
    Serial.println(mean_gz);
    Serial.print("Your offsets:\t");
    Serial.print(ax_offset);
    Serial.print("\t");
    Serial.print(ay_offset);
    Serial.print("\t");
    Serial.print(az_offset);
    Serial.print("\t");
    Serial.print(gx_offset);
    Serial.print("\t");
    Serial.print(gy_offset);
    Serial.print("\t");
    Serial.println(gz_offset);
  }
}
///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void meansensors(){
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
  while (i < (buffersize + 101)) {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}
void calibration(){
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  
  while (1){
    int ready = 0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);
    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);
    meansensors();

    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;
    if (abs(mean_ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset - mean_ay / acel_deadzone;
    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;
    if (abs(mean_gx) <= giro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);
    if (abs(mean_gy) <= giro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);
    if (abs(mean_gz) <= giro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);
    if (ready == 6) break;
  }
}
///////////////////////////////////////////////Check Angle/////////////////////////////////////////////////////////////
void anglempu(){ //Now begins the main loop. 
  //Collect raw data from the sensor.
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
double dt = deltat/1000;
  //the next two lines calculate the orientation of the accelerometer relative to the earth and convert the output of atan2 from radians to degrees
  //We will use this data to correct any cumulative errors in the orientation that the gyroscope develops.
  roll = atan2(AcY, AcZ)*degconvert;
  pitch = atan2(-AcX, AcZ)*degconvert;
  double accelerationZ = (int16_t)(AcZ * CONVERSIONG);
  double accelerationX = (int16_t)(AcX * CONVERSIONG);
  double accelerationY = (int16_t)(AcY * CONVERSIONG);
  //The gyroscope outputs angular velocities.  To convert these velocities from the raw data to deg/second, divide by 131.  
  //Notice, we're dividing by a double "131.0" instead of the int 131.
  double gyroXrate = GyX/131.0;
  double gyroYrate = GyY/131.0;
  double gyroZrate = GyZ/131.0;
  //THE COMPLEMENTARY FILTER
  //This filter calculates the angle based MOSTLY on integrating the angular velocity to an angular displacement.
  //dt, recall, is the time between gathering data from the MPU6050.  We'll pretend that the 
  //angular velocity has remained constant over the time dt, and multiply angular velocity by 
  //time to get displacement.
  //The filter then adds a small correcting factor from the accelerometer ("roll" or "pitch"), so the gyroscope knows which way is down. 
  compAngleX = 0.99 * (compAngleX + gyroXrate * dt) + 0.01 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.99 * (compAngleY + gyroYrate * dt) + 0.01 * pitch; 
}


