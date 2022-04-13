#include "PS2X_lib.h"                    //for connecting joypad
#include "BTJoystick.h"
//#include "pid.h"
//#include "servo_control.h"
#include <SoftwareSerial.h> 
#include "Adafruit_MotorShield.h"         //for controlling step motor 
#include "Adafruit_MS_PWMServoDriver.h" 
#include <Wire.h>
//#include "control_joy.h"
// servo parameters
int num_servos=4;  
int catch_pin =0 ;
int small_arm_pin = 1;
int big_arm_pin = 2;
int base_pin = 3;


Adafruit_MotorShield AFMS = Adafruit_MotorShield();
int MIN[4] = {0,45,40,0};
int MAX[4] = {180,105,135,180};
int angle_now[4] = {78,90,90,90};
int angle_recorded[4]; 
//PID
class PID_Controller{
private:
    float KI;
    float KP;
    float KD;
    float error_now;
    float error_pre;
    float integral;
    float integral_max;
    float output_max;

public:
    void set_error(float error){
        error_now = error;
    }
    void set_error_pre(float error){
        error_pre = error;
    }
    float get_error_now(){
        return error_now;
    }
    float get_error_pre(){
        return error_pre;
    }
    float get_KD(){
        return KD;
    }
    float get_KP(){
        return KP;
    }
    float get_max(){
        return output_max;
    }
    float start_integral(){
        integral += KI * error_now;
        integral = integral > integral_max ? integral_max : integral;
        integral = integral < -integral ? -integral_max : integral;
    }

};

float PID_cal(PID_Controller* pid_x, int Now_value, int Aim_value, char flag = 0){  // flag means using integral part
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
PID_Controller* pid_base;
PID_Controller* pid_big_arm;
PID_Controller* pid_small_arm;
PID_Controller* pid_catch;



void write_servo(int servo_num,int angle){
    
    double real_angle = angle*0.922222;
    //Serial.println(angle);
    if(real_angle > MAX[servo_num] || real_angle < MIN[servo_num]){
        Serial.println("angle out of range");
        //Serial.println(real_angle);
        write_servo(servo_num,78);
        angle_now[servo_num] = 78;
        return;
    }
    double pulse = real_angle/90.0 + 0.5;
    double pulselength;
    pulselength = 1000000;      
    pulselength /= 50;          
    pulselength /= 4096;       
    pulse *= 1000;              
    pulse /= pulselength;
    AFMS.setPWM( servo_num, pulse );
}
void back_to_original_angle(){
    write_servo(catch_pin,78);
    write_servo(small_arm_pin,90);
    write_servo(big_arm_pin,90);
    write_servo(base_pin,90);
}
int* speed_control(){
    int base_output = PID_cal(pid_base,angle_now[base_pin],angle_recorded[base_pin]);
    int catch_output = PID_cal(pid_catch,angle_now[catch_pin],angle_recorded[catch_pin]);
    int big_arm_output = PID_cal(pid_big_arm,angle_now[big_arm_pin],angle_recorded[big_arm_pin]);
    int small_arm_output = PID_cal(pid_small_arm,angle_now[small_arm_pin],angle_recorded[small_arm_pin]);
    int* output = new int[4];
    output[0] = base_output;
    output[1] = catch_output;
    output[2] = big_arm_output;
    output[3] = small_arm_output;
    return output;
}



// joypad
SoftwareSerial  softSerial( 2, 3 ); 
//define joypad
PS2X ps2x;
char vibrate = 0;
int error = 0;
// if flag = 0 means that it is in manual control 
// if flag = 1 means that it is in automatic control
char flag = 0;
// define pins for joypad
char PS2_SEL=10;  //定义PS2手柄引脚10
char PS2_CMD=11;  //定义PS2手柄引脚11
char PS2_DAT=12;  //定义PS2手柄引脚12
char PS2_CLK=13;  //定义PS2手柄引脚13

void search_controller(){
    while(1){
        error = ps2x.config_gamepad( PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true );  //此函数为配置无线手柄的初始化通信能力，如果ok，返回值0
        if( error == 0 )
        {
            Serial.println( "\n done" ); 
            break;  
        }
        else  {
            Serial.print( "wait" );  
            delay( 100 );
        }
    }
    ps2x.read_gamepad( true, 200 );
    delay(500);
}
void record_angle(){
    if(ps2x.Button(PSB_L1)){
        for(char i =0;i<4;i++){
            angle_recorded[i] = angle_now[i];
        }
    }
}
void manual_control(){
    if(error == 1){
        Serial.println("no gamepad");
        return ;
    }
    ps2x.read_gamepad(false,0);
    if(ps2x.Analog(PSS_LX)>240){
        //Serial.println(ps2x.Button(PSB_PAD_UP));
        Serial.println("plus");
        write_servo(catch_pin,angle_now[catch_pin]+1);
        angle_now[catch_pin] = angle_now[catch_pin] + 1;
        //rotate catch motor
    }
    if(ps2x.Analog(PSS_LX)<10){
        //Serial.println(ps2x.Button(PSB_PAD_UP));
        Serial.println("minus");
        write_servo(catch_pin,angle_now[catch_pin]-1);
        angle_now[catch_pin] = angle_now[catch_pin] - 1;
        //rotate catch motor
    }
    // }
    // if(ps2x.Analog(PSS_RY)){
    //     write_servo(base_pin,angle_now[base_pin]+1);
    //     angle_now[base_pin] ++;
    //     //rotate base motor 
    // }
    // if(ps2x.Analog(PSS_LY)){
    //     write_servo(big_arm_pin,angle_now[big_arm_pin]+1);
    //     angle_now[big_arm_pin] ++;
    //     //rotate big arm motor
    // }
    // if(ps2x.Analog(PSS_RX)){
    //     write_servo(small_arm_pin,angle_now[small_arm_pin]+1);
    //     angle_now[small_arm_pin] ++;
    //     //rotate small arm motor
    // }
}
void automatic_control(){
    if(error == 1){
        Serial.println("no gamepad");
        return ;
    }
    int* output = speed_control();
    write_servo(catch_pin,output[catch_pin]);
    write_servo(small_arm_pin,output[small_arm_pin]);
    write_servo(big_arm_pin,output[big_arm_pin]);
    write_servo(base_pin,output[base_pin]);
    //delay(5000);
    write_servo(catch_pin,angle_now[catch_pin]);
    write_servo(small_arm_pin,angle_now[small_arm_pin]);
    write_servo(big_arm_pin,angle_now[big_arm_pin]);
    write_servo(base_pin,angle_now[base_pin]);
}


void control_gamepad(){
    ps2x.read_gamepad(false,0);
    if(ps2x.Button(PSB_SELECT)){
        if(flag == 0){
            flag =1;
            Serial.println("change_to_automatic");
        }
        if(flag ==1 ){
            flag = 0;
            Serial.println("change_to_manual");
        }
    }
    if(flag == 0){
        manual_control();
    }
    if(flag == 1){
        automatic_control();
    }
}




float count = 0;
void setup(){
  Serial.begin( 57600 );  //串口通讯波特率57600
  softSerial.begin(9600); //软串口通讯开启波特率9600，此项设置必须与蓝牙模块串口波特率一致
  softSerial.listen();    //开启软串口数据监听  
  AFMS.begin(50);      //舵机控制频率为50
  delay(2000);
  // connect jpypad
  search_controller();
  back_to_original_angle();
  

}

void loop(){
  manual_control();
  //record_angle();
  


}
