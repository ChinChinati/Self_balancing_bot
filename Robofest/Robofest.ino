/*
3,5 encoder right
19,4 encoder left
18 mpu interrupt
7 pwm2 
8 dir2
9 pwm1
10 dir1
motor driver pins
*/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//otor iver
int dir1 = 10,
    pwm1 = 9,
    dir2 = 8, 
    pwm2 = 7;
double angle=0;

//Encoders
long  int left = 0;
long  int right = 0;
int encoder_left_1 = 19;
int encoder_left_2 = 4;
int encoder_right_1 = 3;
int encoder_right_2 = 5;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////// Angular PID //////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//Angular PID parameters
double pwm = 0;
double setpoint = 0;
double new_setpoint = 0;
double Kp = 0; //proportional gain
double Ki = 0; //integral gain
double Kd = 0; //derivative gain
unsigned long last_time = 0;
double delta_error = 0;
double total_error = 0, last_error = 0;
int delta_time = 0;

void angular_pid(double value){

    unsigned long current_time = millis(); //returns the number of milliseconds passed since the Arduino started running the program
    delta_time = current_time - last_time; //delta time interval 
    //P
    double error = new_setpoint - value;

    //I
    total_error += error; 

    //D
    if(delta_time > 100){
      delta_error = (error - last_error); //difference of error for derivative term
      last_error = error;
      last_time = current_time;
    }

    //Output
    pwm = Kp*error + (Ki)*total_error + (Kd)*delta_error; //PID control compute

    Serial.print("Angular PID\t");
    Serial.print(error);
    Serial.print("\t");
    Serial.print(total_error);
    Serial.print("\t");
    Serial.println(delta_error);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////// Position PID //////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Angular PID parameters
double pos_setpoint = 0;
double pos_Kp = 0; //proportional gain
double pos_Ki = 0; //integral gain
double pos_Kd = 0; //derivative gain
double pos_delta_error = 0;
unsigned long pos_last_time = 0;
double pos_total_error=0, pos_last_error=0;
int pos_delta_time = 0;

void position_pid(double pos_value){

    unsigned long pos_current_time = millis(); //returns the number of milliseconds passed since the Arduino started running the program
    pos_delta_time = pos_current_time - pos_last_time; //delta time interval 
    //P
    double pos_error = pos_setpoint - pos_value;

    //I
    pos_total_error += pos_error; 

    //D
    if(pos_delta_time > 100){
      pos_delta_error = (pos_error - pos_last_error); //difference of error for derivative term
      pos_last_error = pos_error;
      pos_last_time = pos_current_time;
    }

    //Output
    new_setpoint = pos_Kp*pos_error + (pos_Ki)*pos_total_error + (pos_Kd)*pos_delta_error; //PID control compute

    Serial.print("Position PID\t");
    Serial.print(pos_error);
    Serial.print("\t");
    Serial.print(pos_total_error);
    Serial.print("\t");
    Serial.println(pos_delta_error);
}


MPU6050 mpu;


////////////////////////////////////////////////////////MPU///////////////////////////////////////////////////////////////////////////////////
//#define LED_PIN 3 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[640]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
float ax;
float ay;
float az;
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// ================================================================
// ===                      FUNCTIONS                           ===
// ================================================================

void encoder_left(){
  if(digitalRead(encoder_left_1)){
    if(!digitalRead(encoder_left_2)){
      left--;
    }
    if(digitalRead(encoder_left_2)){
      left++;
    }
  }

  if(!digitalRead(encoder_left_1)){
    if(digitalRead(encoder_left_2)){
      left--;
    }
    if(!digitalRead(encoder_left_2)){
      left++;
    }
  }
}

void encoder_right(){
  if(digitalRead(encoder_right_1)){
    if(!digitalRead(encoder_right_2)){
      right--;
    }
    if(digitalRead(encoder_right_2)){
      right++;
    }
  }

  if(!digitalRead(encoder_right_1)){
    if(digitalRead(encoder_right_2)){
      right--;
    }
    if(!digitalRead(encoder_right_2)){
      right++;
    }
  }
}

void MotorLeft(int val){

    if (val < 0){

        digitalWrite(dir1, LOW);
        analogWrite(pwm1, abs(pwm));
    }
    else if(val > 0){
        digitalWrite(dir1, HIGH);
        analogWrite(pwm1, abs(pwm));
    }
    else{
        digitalWrite(dir1, LOW);
        analogWrite(pwm1, 0);
    }
}

void MotorRight(int val){

    if (val < 0){

        digitalWrite(dir2, HIGH);
        analogWrite(pwm2,abs(pwm));
    }
    else if(val > 0){
        digitalWrite(dir2, LOW);
        analogWrite(pwm2, abs(pwm));
    }
    else{
        digitalWrite(dir2, LOW);
        analogWrite(pwm2, 0);
    }
}

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(115200);
    while (!Serial);
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    mpu.setXAccelOffset(-3051);
    mpu.setYAccelOffset(333);
    mpu.setZAccelOffset(1398);
    mpu.setXGyroOffset(-207);
    mpu.setYGyroOffset(-30);
    mpu.setZGyroOffset(48);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setDLPFMode(MPU6050_DLPF_BW_5);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(18), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

 
pinMode(encoder_left_1,INPUT);
pinMode(encoder_left_2,INPUT);
pinMode(encoder_right_1,INPUT);
pinMode(encoder_right_2,INPUT);
pinMode(pwm1,OUTPUT);
pinMode(pwm2,OUTPUT);
pinMode(dir1,OUTPUT);
pinMode(dir2,OUTPUT);
attachInterrupt(digitalPinToInterrupt(encoder_right_1), encoder_right, CHANGE);
attachInterrupt(digitalPinToInterrupt(encoder_left_1), encoder_left, CHANGE);
}




void loop() {
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {}
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////// Main Loop /////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            // display Euler angles in degrees
            /*Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);*/
            angle = ypr[1] * 180/M_PI;
            // Serial.print(angle);
            // Serial.print("\t");
            // Serial.print(left);
            // Serial.print("\t");
            // Serial.println(right);

            // position_pid(left);
            angular_pid(angle);
            
            if (abs(angle) < 15){
              // To be coded //


            } 
        blinkState = !blinkState;    
    }
}