#ifndef CONTROL_JOY_H
#define CONTROL_JOY_H
#include "PS2X_lib.h"                    //for connecting joypad
#include "BTJoystick.h"
#include <SoftwareSerial.h> 
#include "servo_control.h"
#include "pid.h"

SoftwareSerial  softSerial( 2, 3 ); 
//define joypad
PS2X ps2x;
char vibrate = 0;
int error = 1;
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
void record_angle(PS2X ps2x){
    if(ps2x.Button(PSB_L1)){
        for(char i =0;i<4;i++){
            angle_recorded[i] = angle_now[i];
        }
    }
}
void manual_control(PS2X ps2x){
    if(error == 1){
        Serial.println("no gamepad");
        return ;
    }
    ps2x.read_gamepad(false,vibrate);
    if(ps2x.Button(PSB_PAD_UP)){
        write_servo(catch_pin,angle_now[catch_pin]+1);
        angle_now[catch_pin] ++;
        //rotate catch motor
    }
    if(ps2x.Button(PSB_PAD_DOWN)){
        write_servo(base_pin,angle_now[base_pin]+1);
        angle_now[base_pin] ++;
        //rotate base motor 
    }
    if(ps2x.Button(PSB_PAD_RIGHT)){
        write_servo(big_arm_pin,angle_now[big_arm_pin]+1);
        angle_now[big_arm_pin] ++;
        //rotate big arm motor
    }
    if(ps2x.Button(PSB_PAD_LEFT)){
        write_servo(small_arm_pin,angle_now[catch_pin]+1);
        angle_now[small_arm_pin] ++;
        //rotate small arm motor
    }
}
void automatic_control(PS2X ps2x){
    if(error == 1){
        Serial.println("no gamepad");
        return ;
    }
    int* output = speed_control();
    write_servo(catch_pin,output[catch_pin]);
    write_servo(small_arm_pin,output[small_arm_pin]);
    write_servo(big_arm_pin,output[big_arm_pin]);
    write_servo(base_pin,output[base_pin]);
    delay(5000);
    write_servo(catch_pin,angle_now[catch_pin]);
    write_servo(small_arm_pin,angle_now[small_arm_pin]);
    write_servo(big_arm_pin,angle_now[big_arm_pin]);
    write_servo(base_pin,angle_now[base_pin]);
}
void control_gamepad(PS2X ps2x){
    if(ps2x.Button(PSB_SELECT)){
        if(flag == 0){
            flag =1;
        }
        if(flag ==1 ){
            flag = 0;
        }
    }
    if(flag == 0){
        manual_control(ps2x);
    }
    if(flag == 1){
        automatic_control(ps2x);
    }
}



#endif