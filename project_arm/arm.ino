#include "pid.h"
#include <Servo.h>

Servo servo_base;
Servo servo_big_arm;
Servo servo_small_arm;
Servo servo_catch;
PID_Controller pid_base;
PID_Controller pid_big_arm;
PID_Controller pid_small_arm;
PID_Controller pid_catch;
void setup(){
  servo_base.attach(1);
  servo_big_arm.attach(2);
  servo_small_arm.attach(3);
  servo_catch.attach(4);


}

void loop(){



}

float PID_cal(PID_Controller* pid_x, int Now_value, int Aim_value, char flag){  // flag means using integral part
  float error = Now_value - Aim_value;
  pid_x->set_error(error);

  float output = pid_x->get_KD() * (pid_x->get_error_now() - pid_x->get_error_pre())+
  pid_x->get_KP()*pid_x->get_error_now();
  if(flag == 1){
    float integral = pid_x->start_integral();
    output = output + integral;
  }
  output = output > pid_x-> get_max() ? pid_x-> get_max() : output;
  output = output > -pid_x-> get_max() ? -pid_x-> get_max() : output;
  pid_x->set_error_pre(error);
  return output;
}