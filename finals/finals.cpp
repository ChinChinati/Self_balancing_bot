#include<Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//IR ARRAY
bool ir[8]      = {0,0,0,0,0,0,0,0};
bool line[8]    = {0,0,0,0,0,0,0,0};
bool outside[2] = {0,0};

//Motor Driver
const unsigned short int dir1 = 10, pwm1 = 9, dir2 = 8, pwm2 = 7;

//Encoders
long int left = 0;
long int right = 0;
long int last_left = 0;
long int last_right = 0;
int delta_rotation = 0;

int delta_rotation_setpoint = 450;

const unsigned short int encoder_left_1 = 19;
const unsigned short int encoder_left_2 = 4;
const unsigned short int encoder_right_1 = 3;
const unsigned short int encoder_right_2 = 5;


const float motor_pwm_offset = 5;
float pwm = 0;
float delta = 0.0;
float angle = 0;
float setpoint = 8;
float velocity = -0.6;
float new_setpoint = 0;


//Angular PID parameters
float Kp = 2.8; //proportional gain
float Ki = 0.0; //integral gain
float Kd = 0; //derivative gain


// //Positional PID parameters
// float pos_Kp = 0.006; //proportional gain
// float pos_Ki = 0.001; //integral gain
// float pos_Kd = 1.5; //derivative gain
// float acc_Kp = 0.5;

//Positional PID parameters
float pos_Kp = 0.00; //proportional gain
float pos_Ki = 0.00; //integral gain
float pos_Kd = 0.9; //derivative gain
float acc_Kp = 0.5;

//Line PID parameters
double line_Kp = -0.15; // previous : -0.06;


//Line Follow
float line_position =0;
double line_position_error =0;

int callibrate = false;

void Print(){
    // // Serial.print(pos_current_time);
    // // Serial.print(",");
    // Serial.print(current_velocity);
    // Serial.print(",");
    // Serial.print(velocity_error);
    // Serial.print(",");
    // Serial.print(pos_Kd);
    // Serial.print(",");
    // Serial.print((pos_Kd)*velocity_error);
    // // Serial.print(",");
    // // Serial.print(pos_Kp);
    // Serial.print(",");
    // // Serial.print();
    // // Serial.print(",");
    // Serial.println(new_setpoint);
    // // Serial.print(",");
    // // Serial.print(setpoint);
    // Serial.print(",");
    // Serial.println(angle);
     Serial.print(ir[0]);
     Serial.print("\t");
     Serial.print(ir[1]);
     Serial.print("\t");
     Serial.print(ir[2]);
     Serial.print("\t");
     Serial.print(ir[3]);
     Serial.print("\t");
     Serial.print(ir[4]);
     Serial.print("\t");
     Serial.print(ir[5]);
     Serial.print("\t\t");
     Serial.print(line[0]);
     Serial.print("\t");
     Serial.print(line[1]);
     Serial.print("\t");
     Serial.print(line[2]);
     Serial.print("\t");
     Serial.print(line[3]);
     Serial.print("\t");
     Serial.print(line[4]);
     Serial.print("\t");
     Serial.print(line[5]);
     Serial.print("\t\t");
    //  Serial.print(outside[0]);
    //  Serial.print("\t");
    //  Serial.print(outside[1]);
    //  Serial.print("\t");
    //  Serial.print(outside[2]);
    //  Serial.print("\n");
    Serial.println(delta);
    
}

void callibration(){
      setpoint = angle*10;
      callibrate = true;
      Serial.println("################################################################################################");
      Serial.println(setpoint);
}

int sign(float val){
  return (val>=0)?1:-1;
}

// ================================================================
// ===                      ANGULAR PID                         ===
// ================================================================

//Angular PID compute parameters
unsigned long last_time = 0;
float delta_error = 0;
float total_error = 0, last_error = 0;
unsigned short int delta_time = 0;

void angular_pid(float value){

    unsigned long current_time = millis(); //returns the number of milliseconds passed since the Arduino started running the program
    delta_time = current_time - last_time; //delta time interval 
    //P
    double error = setpoint + new_setpoint - value;

    //I
    if(abs(error)<5)
    total_error += error; 
    else
    total_error = 0;
    // if (abs(total_error) >= 400)
    // total_error = 400*total_error/abs(total_error + 1e-3);


    //D
    if(delta_time > 100){
      delta_error = (error - last_error); //difference of error for derivative term
      last_error = error;
      last_time = current_time;
    }
    //Output
    pwm = Kp*error + (Ki)*total_error + (Kd)*delta_error; //PID control compute

    if (abs(pwm) > 100)
    pwm = 100*pwm/abs(pwm);


    // Serial.print("Angular PID\t");
    // Serial.print(error);
    // Serial.print("\t");
    // Serial.print(total_error);
    // Serial.print("\t");
    // Serial.println(delta_error);
    // Serial.println(error);
    // Serial.print("\t");
    // Serial.print(total_error);
    // Serial.print("\t");
    // Serial.println(pwm);
}

// ================================================================
// ===                      POSITION PID                        ===
// ================================================================

//Position PID parameters
unsigned long pos_last_time = 0;
double current_velocity = 0;
double current_acceleration = 0;
double previous_velocity = 0;
float velocity_error = 0;
float pos_total_error=0, pos_last_value=0;
unsigned short int pos_delta_time = 0;
long int velocity_sum_error = 0.0;
long target_position = 0.0;

void position_pid(double pos_value){

    unsigned long pos_current_time = millis(); //returns the number of milliseconds passed since the Arduino started running the program
    pos_delta_time = pos_current_time - pos_last_time; //delta time interval 

    //D
    if(pos_delta_time > 100){
      current_velocity = (pos_value - pos_last_value); //difference of error for derivative term
      current_acceleration = (current_velocity - previous_velocity);
      velocity_error = velocity - current_velocity;
      pos_last_value = pos_value;
      previous_velocity = current_velocity;
      pos_last_time = pos_current_time;
      velocity_sum_error += velocity_error*pos_Ki;
    }
    //double D

    // if (abs(current_velocity) > 7)
    // pos_Kd = 0.9;
    // else
    // pos_Kd = 1.15;

    //Output
    new_setpoint = (pos_Kp)*(target_position - pos_value) + (pos_Kd)*velocity_error + velocity_sum_error + current_acceleration*acc_Kp; //PID control compute
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
      right++;
    }
    if(digitalRead(encoder_right_2)){
      right--;
    }
  }

  if(!digitalRead(encoder_right_1)){
    if(digitalRead(encoder_right_2)){
      right++;
    }
    if(!digitalRead(encoder_right_2)){
      right--;
    }
  }
}

void MotorLeft(float val){

    if (val > 0){

        digitalWrite(dir1, LOW);
        analogWrite(pwm1, abs(val) + motor_pwm_offset);
        
    }
    else if(val < 0){
        digitalWrite(dir1, HIGH);
        analogWrite(pwm1, abs(val) + motor_pwm_offset);
        
    }
    else{
        digitalWrite(dir1, LOW);
        analogWrite(pwm1, 0);
    }
}

void MotorRight(float val){

    if (val > 0){

        digitalWrite(dir2, HIGH);
        analogWrite(pwm2,abs(val) + motor_pwm_offset);
    }
    else if(val < 0){
        digitalWrite(dir2, LOW);
        analogWrite(pwm2, abs(val) + motor_pwm_offset);
    }
    else{
        digitalWrite(dir2, LOW);
        analogWrite(pwm2, 0);
    }
}

void GetIrData(){
  ir[0] = digitalRead(A0);
  ir[1] = digitalRead(A1);
  ir[2] = digitalRead(A2);
  ir[3] = digitalRead(A3);
  ir[4] = digitalRead(A4);
  ir[5] = digitalRead(A5);
  ir[6] = digitalRead(A6);
  ir[7] = digitalRead(A7);
}

int ir_count = 0;
int flag = 0;
void FollowLine(){
    if(Serial.available()!=0){
      char input = Serial.read();
      if(input > 35 && input < 255){
      input = ((int)input - 48);
      if (input <= 10){
        if(input == 5)
         velocity = 0;
        else if(input == 8)
         velocity -= 0.2;
        else if(input == 2)
         velocity += 0.2;
        Serial.println("Input: ");
        Serial.print(velocity);
        if (abs(abs(velocity) - 0.0) <= 0.05){
          pos_Kp = 0.006;
          pos_Kd = 1.1;
          pos_Ki = 0.0;
          target_position = pos_last_value + sign(current_velocity)*1000;
        }
        else{
          pos_Kp = 0.0;
          pos_Kd = 0.9;
          pos_Ki = 0.00;
        }
        
      }
      else if (input == 65 - 48) // Turning
      delta = -10;
      else if (input == 68 - 48) //Turning
      delta = 10;
      else if (input == 99-48) //c
      callibration();
      else if (input == 110-48){ //n increase line kp
      line_Kp -= 0.02;
      Serial.println(line_Kp); 
      }
      else if (input == 109-48){ //m decrease line kp
      line_Kp += 0.02;
      Serial.println(line_Kp);
      }
      else if (input == 112-48){ //p Position hold toggle
      if(!pos_Kp)
        pos_Kp = 0.06;
      else
        pos_Kp = 0;
      target_position = pos_last_value;
      Serial.println(pos_Kp);
      }
      else
      delta = 0;
    }
    }
    if (abs(velocity) > 0.0){
      GetIrData();
      line_position = 3.5;
      ir_count = 0;
      for(int i=0;i<8;i++){
          if(ir[i]){
              line_position = i;
              ir_count++;
          }
      }
      if (ir_count != 0){
        line_position = line_position/ir_count;
        flag = 0;
      }
      line_position = line_position - 3.5;


      // if(outside[0])
      //     line_position_error = -9;
      // else if(outside[2])
      //     line_position_error = 9;

      // -VE DELTA LEFT
      if(ir_count)
      delta = line_Kp*line_position*(current_velocity);

      //Rotation init
      if(!ir_count && !flag){
        last_left = left;
        last_right = right;
        // delta = delta*1.8;
        delta = sign(delta)*15;
        flag = 1;
      }

      //Rotation check
      if(!ir_count){
        delta_rotation = abs((left - last_left) - (right - last_right));
        Serial.println(delta_rotation);
      }
      
      //Rotation Complete
      if(!ir_count && (delta_rotation_setpoint - delta_rotation) < 0){
        delta = 0;
      }
    }
  
}

void RemoteControl(){
    if(Serial.available()!=0){
      char input = Serial.read();
      if(input > 35 && input < 255){
      input = ((int)input - 48);
      if (input <= 10){
        if(input == 5)
         velocity = 0;
        else if(input == 8)
         velocity -= 1;
        else if(input == 2)
         velocity += 1;
        Serial.println("Input: ");
        Serial.print(velocity);
        if (abs(abs(velocity) - 0.0) <= 0.05){
          pos_Kp = 0.006;
          pos_Kd = 1.1;
          pos_Ki = 0.0;
          target_position = pos_last_value + sign(current_velocity)*1000;
        }
        else{
          pos_Kp = 0.0;
          pos_Kd = 0.9;
          pos_Ki = 0.00;
        }
        
      }
      else if (input == 65 - 48) // Turning
      delta = -10;
      else if (input == 68 - 48) //Turning
      delta = 10;
      else if (input == 99-48) //c
      callibration();
      else if (input == 110-48){ //n increase line kp
      line_Kp -= 0.02;
      Serial.println(line_Kp); 
      }
      else if (input == 109-48){ //m decrease line kp
      line_Kp += 0.02;
      Serial.println(line_Kp);
      }
      else if (input == 112-48){ //p Position hold toggle
      if(!pos_Kp)
        pos_Kp = 0.06;
      else
        pos_Kp = 0;
      target_position = pos_last_value;
      Serial.println(pos_Kp);
      }
      else
      delta = 0;
    }
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
    mpu.setXAccelOffset(554);
    mpu.setYAccelOffset(-1453);
    mpu.setZAccelOffset(1911);
    mpu.setXGyroOffset(23);
    mpu.setYGyroOffset(17);
    mpu.setZGyroOffset(-22);
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
pinMode(A0,INPUT);
pinMode(A1,INPUT);
pinMode(A2,INPUT);
pinMode(A3,INPUT);
pinMode(A4,INPUT);
pinMode(A5,INPUT);
pinMode(A6,INPUT);
pinMode(A7,INPUT);
attachInterrupt(digitalPinToInterrupt(encoder_right_1), encoder_right, CHANGE);
attachInterrupt(digitalPinToInterrupt(encoder_left_1), encoder_left, CHANGE);
}

void loop() {
    // RemoteControl();
    FollowLine();
    // Print();
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
            // ================================================================
            // ===                      MAIN LOOP                           ===
            // ================================================================
            angle = ypr[1] * 180/M_PI;
            position_pid((left+right)/2);
            angular_pid(angle*10);

            if (abs(angle) < 15){
              MotorLeft(pwm + delta);
              MotorRight(pwm - delta);
            }
            else{
              MotorLeft(0);
              MotorRight(0);
            }
        blinkState = !blinkState;    
    }

}



// working backup