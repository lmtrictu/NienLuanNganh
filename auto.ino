#include <AFMotor.h>
#include <Servo.h>
#include <PID_v1.h>
 #include "Wire.h"
 #include "I2Cdev.h"
 #include "MPU6050.h"
Servo radar_servo;
int servo_position=0;
int PIR = 16; 
int val = 0; 
int D=0;
int Delta=0;
AF_DCMotor motor1(1,MOTOR12_8KHZ);        // declare 1st motor name (say motor1)and set its frequency
AF_DCMotor motor2(2,MOTOR12_8KHZ);        // declare 
  const int MPU_addr=0x68;
 MPU6050 mpu;
 
 int16_t gyroX,gyroRate; 

 unsigned long currTime, prevTime=0, loopTime;
 int d=0;
 double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,5,3,3,DIRECT);
void setup(){
   myPID.SetMode(AUTOMATIC);
  pinMode(PIR,INPUT_PULLUP);
    pinMode(A1, OUTPUT);
  pinMode(A0, INPUT);
    Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
   mpu.initialize();
   Serial.begin(9600);
  radar_servo.attach(10);
    motor2.setSpeed(50);  // set motor2 speed as 250(max.) untill changed
 motor1.setSpeed(50);   // set motor1 speed as 250(max.) untill changed
  motor1.run(RELEASE);   // Release is an argument passed to function motor1.run which tells the motor 1 to stop.
  motor2.run(RELEASE);   // Release is an argument passed to function motor2.run which tells the motor 2 to stop.
}

void loop(){
  
  for(servo_position=30; servo_position<=130;servo_position+=1)
  {
  radar_servo.write(servo_position);
  
   float gyroAngle=0;
   float gyroAngle1=0;
  
   double time=0;  
digitalWrite(A1,HIGH);
delayMicroseconds(10);
digitalWrite(A1,LOW);
time=pulseIn(A0,HIGH,5000);
 Delta=abs(D-d);
 D=d;

d = (((time*330)/2000));
 val= digitalRead(PIR);
      if(((d>=80) && (d<=550) && (Delta>=9)) ||((d>=80) && (d<=550) && (val==1)) )
  {
     
    int rotA=servo_position-80;
    int rotAabs = 0.5*abs(rotA);
    Setpoint=abs(rotA);           // define setpoint
 
      
     if(rotA>0){
    
    currTime=millis();
   loopTime=currTime-prevTime;
   prevTime=currTime;
   gyroX=mpu.getRotationX();
   gyroRate=(map(gyroX, -32768, 32768, -250, 250));
      gyroAngle= gyroAngle+((float)gyroRate*loopTime/1000);
       gyroAngle1= abs(gyroAngle);
     Input=gyroAngle1;
   myPID.Compute();
   
       motor2.setSpeed(Output);             // set  motor speed to 250
 motor1.setSpeed(Output); 
        motor1.run(BACKWARD);              // use differential drive logic to turn robot
  motor2.run(BACKWARD); 
   }
          
         else if(rotA<0){
  
    currTime=millis();
   loopTime=currTime-prevTime;
   prevTime=currTime;
   gyroX=mpu.getRotationX();
   gyroRate=(map(gyroX, -32768, 32768, -250, 250));
      gyroAngle= gyroAngle+((float)gyroRate*loopTime/1000);
       gyroAngle1= abs(gyroAngle);
      Input=gyroAngle1;
   myPID.Compute();
         motor2.setSpeed(Output);             // set  motor speed to 250
 motor1.setSpeed(Output); 
        motor1.run(FORWARD);              // use differential drive logic to turn robot
  motor2.run(FORWARD); 
   
   }   
       
       else if (rotA==0){
        motor1.run(RELEASE);              // use differential drive logic to turn robot
  motor2.run(RELEASE); }
           delay(200);
        
     motor1.run(RELEASE);              // use differential drive logic to turn robot
  motor2.run(RELEASE); 
      radar_servo.write(80);
      
      while(d>=80 && d<=550){
      
      if ((d>250) && (d<=550) && ((val==1) || (Delta>=10))
      ){
              motor2.setSpeed(50);             // set  motor speed to 250
 motor1.setSpeed(50); 
        motor1.run(FORWARD);              // use differential drive logic to turn robot
  motor2.run(BACKWARD);
 delay(500); }


else if (d<=250 && d>=200){
  motor1.run(RELEASE);              // use differential drive logic to turn robot
  motor2.run(RELEASE); }
  
  else if (d<200 && d >= 80){
         motor2.setSpeed(50);             // set  motor speed to 250
 motor1.setSpeed(50); 
        motor1.run(BACKWARD);              // use differential drive logic to turn robot
  motor2.run(FORWARD);  }
  
  double time=0;  
digitalWrite(A1,HIGH);
delayMicroseconds(10);
digitalWrite(A1,LOW);
time=pulseIn(A0,HIGH,5000);
d = (((time*330)/2000)); }}
  
  
   motor1.run(RELEASE);              // use differential drive logic to turn robot
  motor2.run(RELEASE); 

delay(2);
}




  for(servo_position=130; servo_position>=30;servo_position-=1)
     {
  radar_servo.write(servo_position);
  
   float gyroAngle=0;
   float gyroAngle1=0;
  
   double time=0;  
digitalWrite(A1,HIGH);
delayMicroseconds(10);
digitalWrite(A1,LOW);
time=pulseIn(A0,HIGH,5000);
 Delta=abs(D-d);
 D=d;

d = (((time*330)/2000));
 val= digitalRead(PIR);
      if(((d>=80) && (d<=550) && (Delta>=9)) ||((d>=80) && (d<=550) && (val==1)) )
  {
     
    int rotA=servo_position-80;
    int rotAabs = 0.5*abs(rotA);
    Setpoint=abs(rotA);           // define setpoint
   
      
     if(rotA>0){
       
    currTime=millis();
   loopTime=currTime-prevTime;
   prevTime=currTime;
   gyroX=mpu.getRotationX();
   gyroRate=(map(gyroX, -32768, 32768, -250, 250));
      gyroAngle= gyroAngle+((float)gyroRate*loopTime/1000);
       gyroAngle1= abs(gyroAngle);
     Input=gyroAngle1;
   myPID.Compute();
   
     motor2.setSpeed(Output);             // set  motor speed to 250
 motor1.setSpeed(Output); 
        motor1.run(BACKWARD);              // use differential drive logic to turn robot
  motor2.run(BACKWARD);   
   }
          
         else if(rotA<0){
       
    currTime=millis();
   loopTime=currTime-prevTime;
   prevTime=currTime;
   gyroX=mpu.getRotationX();
   gyroRate=(map(gyroX, -32768, 32768, -250, 250));
      gyroAngle= gyroAngle+((float)gyroRate*loopTime/1000);
       gyroAngle1= abs(gyroAngle);
      Input=gyroAngle1;
   myPID.Compute();
   
   motor2.setSpeed(Output);             // set  motor speed to 250
 motor1.setSpeed(Output); 
        motor1.run(FORWARD);              // use differential drive logic to turn robot
  motor2.run(FORWARD);  
   }   
       
       else if (rotA==0){
        motor1.run(RELEASE);              // use differential drive logic to turn robot
  motor2.run(RELEASE); }
           delay(200);
   
    
     motor1.run(RELEASE);              // use differential drive logic to turn robot
  motor2.run(RELEASE); 
      radar_servo.write(80);
      
      while(d>=80 && d<=500){
      
      if ((d>250) && (d<=550) && ((val==1) && (Delta>=12))){
              motor2.setSpeed(50);             // set  motor speed to 250
 motor1.setSpeed(50); 
        motor1.run(FORWARD);              // use differential drive logic to turn robot
  motor2.run(BACKWARD);
 delay(500); }


else if (d<=250 && d>=200){
  motor1.run(RELEASE);              // use differential drive logic to turn robot
  motor2.run(RELEASE); }
  
  else if (d<200 && d >= 80){
         motor2.setSpeed(50);             // set  motor speed to 250
 motor1.setSpeed(50); 
        motor1.run(BACKWARD);              // use differential drive logic to turn robot
  motor2.run(FORWARD);  }
  
  double time=0;  
digitalWrite(A1,HIGH);
delayMicroseconds(10);
digitalWrite(A1,LOW);
time=pulseIn(A0,HIGH,5000);
d = (((time*330)/2000)); }}
  
  
   motor1.run(RELEASE);              // use differential drive logic to turn robot
  motor2.run(RELEASE); 

delay(2);
}



}
