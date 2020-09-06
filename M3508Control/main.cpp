#include "mbed.h"
#include <stdint.h>

/////User Define/////Herman T(HT)
#define UART_Debug
#define vloop
//#define ploop
//#define CANcommand

//#include <mcp_can.h>
//#include <SPI.h>
#ifdef UART_Debug
//Serial pc(USBTX, USBRX);
#endif

char display_buffer[8];

CAN can1(PA_11, PA_12, 1000000);//1Mbps for C620
CANMessage receive_msg;

//Receive CAN ID
#define Motor_1_RevID 0x201
#define Motor_2_RevID 0x202
#define Motor_3_RevID 0x203
#define Motor_4_RevID 0x204
#define Motor_5_RevID 0x205
#define Motor_6_RevID 0x206
#define Motor_7_RevID 0x207
#define Motor_8_RevID 0x208

//Those three variables will be directly referenced in loop
//Hence, modifying the values will change CAN output
uint16_t required_current = (int16_t){0};
float required_position[4]   = {0};
int16_t required_velocity[4] = {0};//The Sam Shum's code use uint//HT

//Some static Variable
static int16_t last_velocity[4] = {0};
static int velocity_p[4] = {0.3,0.3,0.3,0.3};

//////Utils Function copied from VESC Source Code/////HT
#define UTILS_LP_FAST(value, sample, filter_constant)   (value -= (filter_constant) * (value - (sample)))
int utils_truncate_number(float *number, float min, float max) {
    int did_trunc = 0;
    if (*number > max) {
        *number = max;
        did_trunc = 1;
    } else if (*number < min) {
        *number = min;
        did_trunc = 1;
    }
    return did_trunc;
}
int utils_truncate_number_abs(float *number, float max) {
    int did_trunc = 0;
    if (*number > max) {
        *number = max;
        did_trunc = 1;
    } else if (*number < -max) {
        *number = -max;
        did_trunc = 1;
    }
    return did_trunc;
}

/////CAN send/////
void CAN_Send(int16_t current1, int16_t current2,int16_t current3,int16_t current4 )    //CAN 发送 一标准帧数据
{
    CANMessage TxMessage;
    //  CanRxMsg RxMessage;
    /* transmit 1 message */
    TxMessage.id=0x200;          //SID;//0x00;       ID标示符
    //  TxMessage.ExtId=0;
    TxMessage.format=CANStandard;         //CAN_ID_EXT;//        //选择标准帧
    TxMessage.type=CANData;       //选择数据帧
    TxMessage.len=8;

//Motor Current Value Range from -16384 to 16384//HT
//Motor 1 Current Control
    TxMessage.data[0]=current1>>8;           //data1;
    TxMessage.data[1]=current1;           //data2;
//Motor 2 Current Control
    TxMessage.data[2]=current2>>8;            //data3;
    TxMessage.data[3]=current2;           //data4;
//Motor 3 Current Control
    TxMessage.data[4]=current3>>8;            //data5;
    TxMessage.data[5]=current3;          //data6;
//Motor 4 Current Control
    TxMessage.data[6]=current4>>8;            //data7;
    TxMessage.data[7]=current4;          //data8;
 
    can1.write(TxMessage);
}


/////CAN Send is Triggered when Receiving CAN Message/////
void RX()//CAN      中断接收程序  
{
   //receive_msg.id=0x001; //ignore if receive rubbish
   can1.read(receive_msg);
    uint16_t actual_local_position = (uint16_t)(receive_msg.data[0]<<8)|receive_msg.data[1];
    int16_t  actual_velocity       =  (int16_t)(receive_msg.data[2]<<8)|receive_msg.data[3];
    int16_t  actual_current        =  (int16_t)(receive_msg.data[4]<<8)|receive_msg.data[5]; 
    switch(receive_msg.id)
    {
        case Motor_1_RevID:
        {          
#ifdef vloop            
/////Velocity Loop/////          
            if ((required_velocity[0] <800)  && (required_velocity[0] > -800)) {velocity_p[0] = 10;} //ok
        else if ((required_velocity[0] <4000) && (required_velocity[0] >-4000)) {velocity_p[0] = 3;}
       else if ((required_velocity[0] <8000) && (required_velocity[0] >-8000)) {velocity_p[0] = 1.5;}
       else if ((required_velocity[0] <12000) && (required_velocity[0] >-12000)) {velocity_p[0] = 1;}
            
        last_velocity[0] = actual_velocity;
#endif                     
        }
        //Duplicate the function
        case Motor_2_RevID:
                {          
#ifdef vloop            
/////Velocity Loop/////           
            if ((required_velocity[1] <800)  && (required_velocity[1] > -800)) {velocity_p[1] = 1.3;} //ok
       else if ((required_velocity[1] <8000) && (required_velocity[1] >-8000)) {velocity_p[1] = 1;}
    
        last_velocity[1] = actual_velocity;
#endif    
        }
        case Motor_3_RevID:
        {
#ifdef vloop            
/////Velocity Loop/////           
            if ((required_velocity[2] <800)  && (required_velocity[2] > -800)) {velocity_p[2] = 1.3;} //ok
       else if ((required_velocity[2] <8000) && (required_velocity[2] >-8000)) {velocity_p[2] = 1;}
     
        last_velocity[2] = actual_velocity;
#endif
        }
        case Motor_4_RevID:
        {
#ifdef vloop            
/////Velocity Loop/////           
            if ((required_velocity[3] <800)  && (required_velocity[3] > -800)) {velocity_p[3] = 1.3;} //ok
       else if ((required_velocity[3] <8000) && (required_velocity[3] >-8000)) {velocity_p[3] = 1;}
                 
        last_velocity[3] = actual_velocity;
#endif
        }        
        case Motor_5_RevID:
        case Motor_6_RevID:
        case Motor_7_RevID:
        case Motor_8_RevID:                 
            break;
        default:
            break;
    }
    /////Data conversion from float to uint16_t/////HT
    uint16_t motor_out_1 = (uint16_t)((required_velocity[0]-last_velocity[0])* velocity_p[0]);
    uint16_t motor_out_2 = (uint16_t)((required_velocity[1]-last_velocity[1])* velocity_p[1]);
    uint16_t motor_out_3 = (uint16_t)((required_velocity[2]-last_velocity[2])* velocity_p[2]);
    uint16_t motor_out_4 = (uint16_t)((required_velocity[3]-last_velocity[3])* velocity_p[3]);
    /////Send First Set Motor Data/////HT
    CAN_Send(motor_out_1,motor_out_2,motor_out_3,motor_out_4);
}

int main() 
{
    //pc.baud(115200);
    can1.attach(&RX, CAN::RxIrq);

while(1)
 {
#ifdef vloop   
   for ( int i=0; i<=69; i++ ) 
   {
       required_velocity[0] = i*100;
       required_velocity[1] = -i*100;
       wait_ms(30);   
   }
   for ( int i=0; i<=69; i++ ) 
   {
       required_velocity[0] = 6900;
       required_velocity[1] = -6900+i*100;
       wait_ms(30);   
   }
#endif   
   
#ifdef ploop   
   /*
   for ( int i=0; i<=20; i++ ) 
   {
       required_position = i*36*19;
       wait_ms(600);       
   }   
   required_position = 0;
   wait_ms(1000);
   */
#endif   
   }
}
