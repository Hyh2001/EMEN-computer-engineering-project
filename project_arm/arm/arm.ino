#include "PS2X_lib.h"                    //for connecting joypad
#include "BTJoystick.h"
#include "pid.h"
//#include "servo_control.h"
#include <SoftwareSerial.h> 
#include "Adafruit_MotorShield.h"         //for controlling step motor 
#include "Adafruit_MS_PWMServoDriver.h" 
#include <Wire.h>
#include "control_joy.h"
// servo parameters


//PID
PID_Controller* pid_base;
PID_Controller* pid_big_arm;
PID_Controller* pid_small_arm;
PID_Controller* pid_catch;

float count = 0;
void setup(){
  Serial.begin( 57600 );  //串口通讯波特率57600
  softSerial.begin(9600); //软串口通讯开启波特率9600，此项设置必须与蓝牙模块串口波特率一致
  softSerial.listen();    //开启软串口数据监听  
  AFMS.begin(50);      //舵机控制频率为50
  // connect jpypad
  search_controller();
  set_limits();
  back_to_original_angle();
  

}

void loop(){
  control_gamepad(ps2x);
  record_angle(ps2x);
  


}

