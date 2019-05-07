#include <AFMotor.h>
byte t = 0; //bien nhan du lieu qua ttooth
AF_DCMotor motor1(1); //khai bao dong co 1
AF_DCMotor motor2(2);
void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600); //mo cong Serial
    motor1.setSpeed(255);   
    motor2.setSpeed(255);
}

void loop() 
{
  if ( Serial.available() > 0 )
  {
      t = Serial.read();
      Serial.println(t);
  }
  if ( t == 'a') //tien
  {     
      motor1.run(FORWARD);      // động cơ tiến 
      motor2.run(FORWARD);        
  }
  else if (t== 'e' ) //lui
  {
      motor1.run(BACKWARD);     // động cơ lùi  
      motor2.run(BACKWARD);       
  }
  else if (t== 'c') // dừng
  {
      motor1.run(RELEASE);
      motor2.run(RELEASE);
  }
 else if (t== 'd') //xoay phai
  {
      motor1.run(FORWARD);
      motor2.run(RELEASE);
  }
 else if (t== 'b')//XOAY TRAI
  {
      motor1.run(RELEASE);
      motor2.run(FORWARD);

  }
  delay(1);
}
