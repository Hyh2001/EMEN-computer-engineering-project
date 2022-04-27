#include <Wire.h>                         
#include <SoftwareSerial.h>               
#include "Adafruit_MotorShield.h"         
#include "Adafruit_MS_PWMServoDriver.h"   
#include "PS2X_lib.h"                     
#include "BTJoystick.h"      
#include "pid.h"             
#include <NewPing.h>

//define four pins for ps2
#define PS2_SEL 10  
#define PS2_CMD 11  
#define PS2_DAT 12  
#define PS2_CLK 13  

#define pressures   true    
#define rumble      true  
//for ultrasound
#define TRIGGER_PIN  A0
#define ECHO_PIN     A1
#define MAX_DISTANCE 20
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

SoftwareSerial  softSerial( 2, 3 );      

PS2X ps2x;                     //定义PS2手柄


int error = 0;      
byte type = 0;      
byte vibrate = 0;   

Adafruit_MotorShield AFMS = Adafruit_MotorShield();  //define AFMS control board

const int SERVOS = 4;       
const int ACC = 10;         
int PIN[SERVOS];            
int value[SERVOS];          
       
      
int currentAngle[SERVOS];   
int MIN[SERVOS];            
int MAX[SERVOS];            
int INITANGLE[SERVOS];     //limits of servos 
int previousAngle[SERVOS];  
int angle_recorded_1[SERVOS] = {78,90,90,90};
int angle_recorded_2[SERVOS] = {78,90,90,90};

// angle to pwm
void writeServo( uint8_t n, uint8_t angle )  
{
    double pulse;
    angle = (int)angle*0.922222;  
    pulse = 0.5 + angle / 90.0;  
    double pulselength;
    pulselength = 1000000;      
    pulselength /= 50;          //50 Hz
    pulselength /= 4096;        
    pulse *= 1000;             
    pulse /= pulselength;      
    AFMS.setPWM( n, pulse );    //pwm 
}
void control_sersor(){
    // Serial.print("Distance is:"  );
    // Serial.println(sonar.ping_cm());
    int dis = sonar.ping_cm();
    if(dis > 15 && dis <18){
        writeServo(PIN[0],90);
    }
    delay(1000);
    // if(sonar.ping_cm() > 5){
    //     Serial.print( "catch action" );
    // }
}
void setup()
{
    Serial.begin( 57600 );  
    delay( 2000 );          //延时2秒用于PS2手柄初始化
    Serial.print( "Search Controller.." );  
    do
    {
        error = ps2x.config_gamepad( PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble );  //此函数为配置无线手柄的初始化通信能力，如果ok，返回值0
        if( error == 0 )
        {
            Serial.println( "\nConfigured successful " );  //success
            break;  
        }
        else  
        {
            Serial.print( "wait \n" );  //wait 
            delay( 100 );
        }
    }while( 1 );  
    type = ps2x.readType();    
    switch( type )  
    {
        case 0:
            Serial.println( "Wireless DualShock Controller found " );  
            break;
        case 1:
            Serial.println( "Wireless DualShock Controller found " );  
            break;
        case 2:
            Serial.println( "Unknown Controller type found " );  
            break;
    }
    ps2x.read_gamepad( true, 200 );  //开机震动0.2秒用于提示手柄连接成功
    delay( 500 );
    
    AFMS.begin( 50 ); //50Hz motor rate
    init_Pins();
    for ( int i = 0; i < SERVOS; i ++ )  
    {
        value[i] = 0;
        previousAngle[i] = INITANGLE[i];
        currentAngle[i] = INITANGLE[i];
        writeServo( PIN[i], INITANGLE[i] );
    }
    
}

void loop()
{  
            //for ultrasound
        

        //ps2x.read_gamepad( false, vibrate );    //vibrate close
        //vibrate = ps2x.Analog( PSAB_CROSS );    //press x to vibrate
           
        Control();            //motor control    
        ReadRockerValue();    // joypad control
        record_angle();
        mode_control();
        delay( 10 );

}

void init_Pins(){  
    //catch
    PIN[0] = 0;  
    MIN[0] = 0;  
    MAX[0] = 180;  
    INITANGLE[0] = 78;  
    //small arm
    PIN[1] = 1;  
    MIN[1] = 45;  
    MAX[1] = 105;  
    INITANGLE[1] = 90;  
    //big arm
    PIN[2] = 2; 
    MIN[2] = 40;  
    MAX[2] = 135;  
    INITANGLE[2] = 90;  
    //base
    PIN[3] = 3;  
    MIN[3] = 0;    
    MAX[3] = 180;  
    INITANGLE[3] =90;  
}

void Home()   
{
  if (ps2x.Button(PSB_PAD_LEFT))  //back to original point
  {
    for ( int i = 0; i < SERVOS; i ++ )  
    {
      //determine wheter outof range
      while ( currentAngle[i] < INITANGLE[i] )  
      {
        currentAngle[i] += 1;  
        writeServo( PIN[i], currentAngle[i] );  
        delay(5);  
      }
     
     
      while ( currentAngle[i] > INITANGLE[i] )  
      {
        currentAngle[i] -= 1;  
        writeServo( PIN[i], currentAngle[i] );  
        delay(5);  
      }
    }
  }
}
void record_angle(){
    
    if(ps2x.Button(PSB_PAD_UP)){
        for(int i=0;i<SERVOS;i++){
            angle_recorded_1[i] = currentAngle[i];
            
        }
    }
    if(ps2x.Button(PSB_PAD_DOWN)){
        for(int i=0;i<SERVOS;i++){
            angle_recorded_2[i] = currentAngle[i];
            
        }
    }
}
void automatic_control(){
        int preserve_value[4];
        for(int i=0;i<SERVOS;i++){
            writeServo(i,angle_recorded_1[i]);
            preserve_value[i] = angle_recorded_1[i];
            angle_recorded_1[i] = angle_recorded_2[i];
            delay(50);
        }
        //delay(5000);
        for(int i=0;i<SERVOS;i++){
            writeServo(i,angle_recorded_2[i]);
            
            angle_recorded_2[i] = preserve_value[i];
            preserve_value[i] = 0;
            delay(50);
        }
}

void mode_control(){
    if(ps2x.Button(PSB_PAD_RIGHT)){
        automatic_control();
    }
    control_sersor();
    return;
}


void Control()  
{
    Home();  //back to original
    for ( int i = 0; i < SERVOS; i ++ )  //4 servos
    {   
      if ( value[i] > 150)  
      {
        if ( currentAngle[i] < MAX[i] )  
        currentAngle[i] += 1;  
        writeServo( PIN[i], currentAngle[i] );  
      }
        else if (value[i] < 100) 
      {
        if ( currentAngle[i] > MIN[i] )  
        currentAngle[i] -= 1;  
        writeServo( PIN[i], currentAngle[i] );  
      }        
     }
}

void ReadRockerValue()  //函数4：读取无线手柄摇杆值
{
value[0] = ps2x.Analog( PSS_LX );  //读取左摇杆横向模拟量值
value[1] = ps2x.Analog( PSS_RY );  //读取右摇杆纵向模拟量值
value[2] = ps2x.Analog( PSS_LY );  //读取左摇杆纵向模拟量值
value[3] = ps2x.Analog( PSS_RX );  //读取右摇杆横向模拟量值
value[3] = 255-value[3];
}
