#ifndef CONTROL_JOY_H
#define CONTROL_JOY_H
#include "PS2X_lib.h"                    //for connecting joypad
#include "BTJoystick.h"
#include <SoftwareSerial.h> 

SoftwareSerial  softSerial( 2, 3 ); 
//define joypad
PS2X ps2x;
char vibrate = 0;
int error = 1;
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

void control_gamepad(PS2X ps2x){
    if(error == 1){
        Serial.println("no gamepad");
        return ;
    }
    ps2x.read_gamepad(false,vibrate);
    if(ps2x.Button(PSB_PAD_UP)){
        //rotate catch motor
    }
    if(ps2x.Button(PSB_PAD_DOWN)){
        //rotate base motor 
    }
    if(ps2x.Button(PSB_PAD_RIGHT)){
        //rotate big arm motor
    }
    if(ps2x.Button(PSAB_PAD_LEFT)){
        //rotate small arm motor
    }
    


}



#endif