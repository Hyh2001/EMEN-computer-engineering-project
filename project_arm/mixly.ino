
#include <Wire.h>
#include "PS2X_lib.h"
#include "Adafruit_MS_PWMServoDriver.h"
#include "QGPMaker_MotorShield.h"
#include "QGPMaker_Encoder.h"

long ARM_MIN[]={0, 40, 45, 0};

long ARM_MAX[]={180, 105, 135, 180};

QGPMaker_MotorShield AFMS = QGPMaker_MotorShield();
PS2X ps2x;
QGPMaker_Servo *Servo0 = AFMS.getServo(0);
QGPMaker_Servo *Servo1 = AFMS.getServo(1);
QGPMaker_Servo *Servo2 = AFMS.getServo(2);
QGPMaker_Servo *Servo3 = AFMS.getServo(3);

void setup(){
  AFMS.begin(50);

  int error = 0;
  do{
    error = ps2x.config_gamepad(13,11,10,12, true, true);
    if(error == 0){
      break;
    }else{
      delay(100);
    }
  }while(1);
  for(int  i = 0; i < 50; i++)
  {
        ps2x.read_gamepad(false, 0);
        delay(10);
  }

  Servo0->writeServo(78);
  Servo1->writeServo(90);
  Servo2->writeServo(90);
  Servo3->writeServo(86);
}

void loop(){
  ps2x.read_gamepad(false, 0);
  delay(3);
  // 按下手柄X按钮，手柄震动一下
  if (ps2x.Button(PSB_CROSS)) {
    ps2x.read_gamepad(true, 200);
    delay(300);
    ps2x.read_gamepad(false, 0);

  }
  if (ps2x.Analog(PSS_LX) > 240) {
    if (Servo0->readDegrees() > ARM_MIN[(int)(0)]) {
      Servo0->writeServo((Servo0->readDegrees() - 1));
    }

  } else if (ps2x.Analog(PSS_LX) < 10) {
    if (Servo0->readDegrees() < ARM_MAX[(int)(0)]) {
      Servo0->writeServo((Servo0->readDegrees() + 1));
    }
  }
  if (ps2x.Analog(PSS_LY) > 240) {
    if (Servo1->readDegrees() > ARM_MIN[(int)(1)]) {
      Servo1->writeServo((Servo1->readDegrees() - 1));
    }

  } else if (ps2x.Analog(PSS_LY) < 10) {
    if (Servo1->readDegrees() < ARM_MAX[(int)(1)]) {
      Servo1->writeServo((Servo1->readDegrees() + 1));
    }
  }
  if (ps2x.Analog(PSS_RY) > 240) {
    if (Servo2->readDegrees() > ARM_MIN[(int)(2)]) {
      Servo2->writeServo((Servo2->readDegrees() - 1));
    }

  } else if (ps2x.Analog(PSS_RY) < 10) {
    if (Servo2->readDegrees() < ARM_MAX[(int)(2)]) {
      Servo2->writeServo((Servo2->readDegrees() + 1));
    }
  }
  if (ps2x.Analog(PSS_RX) > 240) {
    if (Servo3->readDegrees() > ARM_MIN[(int)(3)]) {
      Servo3->writeServo((Servo3->readDegrees() - 1));
    }

  } else if (ps2x.Analog(PSS_RX) < 10) {
    if (Servo3->readDegrees() < ARM_MAX[(int)(3)]) {
      Servo3->writeServo((Servo3->readDegrees() + 1));
    }
  }
  delay(10);

}