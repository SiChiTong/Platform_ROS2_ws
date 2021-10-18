
#include "mbed.h"
#include "SerialStream.h"

#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include <cstring>
#include <iterator>
#include <sstream>
#include <string>
#include <type_traits>
#include <stdlib.h>
#include <stdio.h>
#include <vector>

#define TX_IMU PC_12 //e2box 의 RX
#define RX_IMU PD_2  //e2box 의 TX

#define TX_HC06 PC_10 //HC06의 RX
#define RX_HC06 PC_11 //HC06의 TX

#define SRT_LF PF_2
#define PWM_LF PB_8
#define DIR_LF PG_9
#define BRK_LF PC_7
#define PLS_LF PF_9

#define SRT_RF PF_8
#define PWM_RF PB_5
#define DIR_RF PG_14
#define BRK_RF PB_2 
#define PLS_RF PF_7

#define SRT_LB PE_3
#define PWM_LB PC_6
#define DIR_LB PF_15
#define BRK_LB PA_8
#define PLS_LB PE_6

#define SRT_RB PE_4
#define PWM_RB PA_15
#define DIR_RB PE_13
#define BRK_RB PB_1
#define PLS_RB PE_5

#define WATER PG_3

#define BRUSH PG_2

#define PPM_DECODE PG_1

#define Dt 0.1
#define Dt_uart 0.01
#define Dt_imu 0.01
#define Dt_main 0.01

#define WHEEL_RADIUS  0.0635
#define ROLLER_RADIUS 0.0267
#define LENGTH_X 0.29
#define LENGTH_Y 0.26

#define RPM_TO_RAD_P_SEC 0.10472
#define RAD_TO_RPM_P_SEC 9.5493

DigitalOut Dir_LF(DIR_LF);  //Dir_LF = 0;(Forward) Dir_LF = 1;(Backward)
DigitalOut Dir_RF(DIR_RF);  //Dir_RF = 1;(Forward) Dir_RF = 0;(Backward)
DigitalOut Dir_LB(DIR_LB);  //Dir_LB = 0;(Forward) Dir_LB = 1;(Backward)
DigitalOut Dir_RB(DIR_RB);  //Dir_RB = 1;(Forward) Dir_RB = 0;(Backward)

DigitalOut Brk_LF(BRK_LF);
DigitalOut Brk_RF(BRK_RF);
DigitalOut Brk_LB(BRK_LB);
DigitalOut Brk_RB(BRK_RB);

DigitalOut Srt_LF(SRT_LF);
DigitalOut Srt_RF(SRT_RF);
DigitalOut Srt_LB(SRT_LB);
DigitalOut Srt_RB(SRT_RB);

PwmOut Pwm_LF(PWM_LF);
PwmOut Pwm_RF(PWM_RF);
PwmOut Pwm_LB(PWM_LB);
PwmOut Pwm_RB(PWM_RB);

InterruptIn Pls_LF(PLS_LF);
InterruptIn Pls_RF(PLS_RF);
InterruptIn Pls_LB(PLS_LB);
InterruptIn Pls_RB(PLS_RB);

DigitalOut LED(LED1);

DigitalOut Brush(BRUSH);
DigitalOut Water(WATER);

InterruptIn PPM(PPM_DECODE);

UnbufferedSerial IMU(TX_IMU, RX_IMU, 115200);
UnbufferedSerial HC06(TX_HC06, RX_HC06, 115200);
UnbufferedSerial PC(USBTX, USBRX, 115200);

SerialStream<UnbufferedSerial> imu(IMU);
SerialStream<UnbufferedSerial> hc(HC06);
SerialStream<UnbufferedSerial> pc(PC);

Timer TMR_PPM;

Timer TMR_PLS_LF;
Timer TMR_PLS_RF;
Timer TMR_PLS_LB;
Timer TMR_PLS_RB;

Thread thread_uart;
Thread thread_imu;
Thread thread_main;

template <class T>
T map(T x, T in_min, T in_max, T out_min, T out_max);

template <class A>
A constrain(A x, A min, A max);

void Go_F(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel);
void Go_LF(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel);
void Go_RF(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel);
void Go_L(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel);
void Go_R(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel);
void Go_B(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel);
void Go_LB(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel);
void Go_RB(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel);
void Stop();
void Turn_L(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel);
void Turn_R(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel);
void Go_CMD(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel);

void Get_MotorControl_Protocol();

void Get_Robot_Status();

void CTRL();

void Get_CMD();
void Get_IMU_Protocol();

void Cal_AngVel();

void PID_AngleVel(string MotorName, float TargetRPM, float Kp, float Ki, float Kd);
void PID_CMD();
void PID();

void Check_PC_RX();
void Check_HC_RX();
void Check_IMU_RX();

void Check_RisingEdge_PPM();
void Check_FallingEdge_PPM();


void Check_RisingEdge_LF();
void Check_FallingEdge_LF();

void Check_RisingEdge_RF();
void Check_FallingEdge_RF();

void Check_RisingEdge_LB();
void Check_FallingEdge_LB();

void Check_RisingEdge_RB();
void Check_FallingEdge_RB();

void Check_Mode();

void Check_Br_W();

void Move(float T_vx, float T_vy, float T_yaw_vel);

void Thread_Uart();
void Thread_IMU();
void Thread_Main();

void ALL_RESET();

bool Do_PID = true;
bool Do_PID_LF = true, Do_PID_RF = true, Do_PID_LB = true, Do_PID_RB = true;

string MotorMode = "I";
string ControlMode = "PC";

uint32_t current_tick = 0;
uint32_t working_tick = 0;

uint32_t working_uart_tick = 0;
uint32_t current_uart_tick = 0;

uint32_t working_imu_tick = 0;
uint32_t current_imu_tick = 0;

uint32_t working_main_tick = 0;
uint32_t current_main_tick = 0;

char Buf_HC[50];
char HC_Char;
string inputString_HC;
char inputChar_HC = 0;
bool Flag_Start_Get_String = false;
bool HC_complete = false;
int HC_Count = 0;

char Buf_IMU[50];
char IMU_Char;
string inputString_IMU;
char inputChar_IMU = 0;
bool Flag_Start_Get_String_IMU = false;
bool IMU_complete = false;
int IMU_Count = 0;
char imubuf[64];
signed int imubuf_cnt = 0;
float RPY_imu[6];

float RPM_LF = 0, RPM_LF_LPF = 0;
float RPM_RF = 0, RPM_RF_LPF = 0;
float RPM_LB = 0, RPM_LB_LPF = 0;
float RPM_RB = 0, RPM_RB_LPF = 0;

float Error_LF, Error_LF_last, P_LF, I_LF, D_LF;
float Error_RF, Error_RF_last, P_RF, I_RF, D_RF;
float Error_LB, Error_LB_last, P_LB, I_LB, D_LB;
float Error_RB, Error_RB_last, P_RB, I_RB, D_RB;

uint8_t Input_RPM_LF = 0, Input_RPM_RF = 0, Input_RPM_LB = 0, Input_RPM_RB = 0;
float Target_RPM_LF = 0, Target_RPM_RF = 0, Target_RPM_LB = 0, Target_RPM_RB = 0;
float Target_RPM_CMD_LF = 0,Target_RPM_CMD_RF = 0,Target_RPM_CMD_LB = 0,Target_RPM_CMD_RB = 0;
float Target_vx = 0, Target_vy = 0, Target_Yaw_vel = 0;
float Yaw_Dot_LPF = 0;
//PPM
int i = 0, j = 0, k = 0, p = 0;
double ICValue_CH0 = 0., ICValue_CH1 = 0., ICValue_CH2 = 0., ICValue_CH3 = 0., ICValue_CH4 = 0., ICValue_CH5 = 0., ICValue_CH6 = 0., ICValue_CH7 = 0., ICValue_CH8 = 0.;
uint32_t ICValue_All_PPM[19], ICValue_All_PWM[19], ICValue_Divided_PWM[9];

//Encoder
double Period_LF, Period_RF, Period_LB, Period_RB;

double YAW = 0, PREV_YAW = 0, YAW_DOT = 0;

bool flag_all = false;
int32_t cnt_lf = 0, cnt_rf = 0, cnt_lb = 0, cnt_rb = 0;
int32_t cnt_lf_last = 0, cnt_rf_last = 0, cnt_lb_last = 0, cnt_rb_last = 0;
bool flag_cnt_lf = false, flag_cnt_rf = false, flag_cnt_lb = false, flag_cnt_rb = false;
bool flag_cnt = false;
bool flag_pc = false;
bool flag_change1 = false, flag_change2 = true;

bool Flag_Br = false;
bool Flag_W = false;

int main()
{
    Pwm_LF.period_us(100);
    Pwm_RF.period_us(100);
    Pwm_LB.period_us(100);
    Pwm_RB.period_us(100);

    Brk_LF = 0;
    Brk_RF = 0;
    Brk_LB = 0;
    Brk_RB = 0;
    LED = 0;
    Srt_LF = 1;
    Srt_RF = 1;
    Srt_LB = 1;
    Srt_RB = 1;

    Stop();

    PPM.enable_irq();
    PPM.rise(&Check_RisingEdge_PPM);
    PPM.fall(&Check_FallingEdge_PPM);
    TMR_PPM.start();
    Pls_LF.enable_irq();
    Pls_LF.rise(&Check_RisingEdge_LF);
    Pls_LF.fall(&Check_FallingEdge_LF);
    TMR_PLS_LF.start();

    Pls_RF.enable_irq();
    Pls_RF.rise(&Check_RisingEdge_RF);
    Pls_RF.fall(&Check_FallingEdge_RF);
    TMR_PLS_RF.start();

    Pls_LB.enable_irq();
    Pls_LB.rise(&Check_RisingEdge_LB);
    Pls_LB.fall(&Check_FallingEdge_LB);
    TMR_PLS_LB.start();

    Pls_RB.enable_irq();
    Pls_RB.rise(&Check_RisingEdge_RB);
    Pls_RB.fall(&Check_FallingEdge_RB);

    //HC06.attach(&Check_HC_RX, SerialBase::RxIrq);
    PC.attach(&Check_PC_RX, SerialBase::RxIrq);
    IMU.attach(&Check_IMU_RX, SerialBase::RxIrq);

    thread_uart.start(&Thread_Uart);
    thread_imu.start(&Thread_IMU);
    thread_main.start(&Thread_Main);

    while (true)
    {
        current_tick = Kernel::get_ms_count();

        working_tick = Kernel::get_ms_count();

        ThisThread::sleep_until(Kernel::get_ms_count() + (Dt * 1000 - (working_tick - current_tick)));
    }
}

void Check_Br_W()
{
    if(Flag_Br == true) Brush = 1;
    else if(Flag_Br == false) Brush = 0;

    if(Flag_W == true) Water = 1;
    else if(Flag_W == false) Water = 0;
}

void Go_F(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel) //OK
{
    Dir_LF = 0; // 정
    Dir_RF = 1; // 정
    Dir_LB = 0; // 정
    Dir_RB = 1; // 정

    Do_PID_LF = true;
    Do_PID_RF = true;
    Do_PID_LB = true;
    Do_PID_RB = true;

    Pwm_LF.pulsewidth_us(LF_Vel);
    Pwm_RF.pulsewidth_us(RF_Vel);
    Pwm_LB.pulsewidth_us(LB_Vel);
    Pwm_RB.pulsewidth_us(RB_Vel);
}
void Go_LF(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel) //OK
{
    Dir_RF = 1; // 정
    Dir_LB = 0; // 정

    Do_PID_LF = false;
    Do_PID_RF = true;
    Do_PID_LB = true;
    Do_PID_RB = false;

    Pwm_LF.pulsewidth_us(0);
    Pwm_RF.pulsewidth_us(RF_Vel);
    Pwm_LB.pulsewidth_us(LB_Vel);
    Pwm_RB.pulsewidth_us(0);
}
void Go_RF(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel) //OK
{
    Dir_LF = 0; // 정
    Dir_RB = 1; // 정

    Do_PID_LF = true;
    Do_PID_RF = false;
    Do_PID_LB = false;
    Do_PID_RB = true;

    Pwm_LF.pulsewidth_us(LF_Vel);
    Pwm_RF.pulsewidth_us(0);
    Pwm_LB.pulsewidth_us(0);
    Pwm_RB.pulsewidth_us(RB_Vel);
}
void Go_RB(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel) //OK
{
    Dir_RF = 0; // 역
    Dir_LB = 1; // 역

    Do_PID_LF = false;
    Do_PID_RF = true;
    Do_PID_LB = true;
    Do_PID_RB = false;

    Pwm_LF.pulsewidth_us(0);
    Pwm_RF.pulsewidth_us(RF_Vel);
    Pwm_LB.pulsewidth_us(LB_Vel);
    Pwm_RB.pulsewidth_us(0);
}
void Stop()
{
    ALL_RESET();
    
    Pwm_LF.pulsewidth_us(0);
    Pwm_RF.pulsewidth_us(0);
    Pwm_LB.pulsewidth_us(0);
    Pwm_RB.pulsewidth_us(0);
}
void Go_L(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel) //OK
{
    Dir_LF = 1; // 역
    Dir_RF = 1; // 정
    Dir_LB = 0; // 정
    Dir_RB = 0; // 역

    Do_PID_LF = true;
    Do_PID_RF = true;
    Do_PID_LB = true;
    Do_PID_RB = true;

    Pwm_LF.pulsewidth_us(LF_Vel);
    Pwm_RF.pulsewidth_us(RF_Vel);
    Pwm_LB.pulsewidth_us(LB_Vel);
    Pwm_RB.pulsewidth_us(RB_Vel);
}
void Go_R(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel) //OK
{
    Dir_LF = 0; // 정
    Dir_RF = 0; // 역
    Dir_LB = 1; // 역
    Dir_RB = 1; // 정

    Do_PID_LF = true;
    Do_PID_RF = true;
    Do_PID_LB = true;
    Do_PID_RB = true;

    Pwm_LF.pulsewidth_us(LF_Vel);
    Pwm_RF.pulsewidth_us(RF_Vel);
    Pwm_LB.pulsewidth_us(LB_Vel);
    Pwm_RB.pulsewidth_us(RB_Vel);
}
void Go_B(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel) //OK
{
    Dir_LF = 1; // 역
    Dir_RF = 0; // 역
    Dir_LB = 1; // 역
    Dir_RB = 0; // 역

    Do_PID_LF = true;
    Do_PID_RF = true;
    Do_PID_LB = true;
    Do_PID_RB = true;

    Pwm_LF.pulsewidth_us(LF_Vel);
    Pwm_RF.pulsewidth_us(RF_Vel);
    Pwm_LB.pulsewidth_us(LB_Vel);
    Pwm_RB.pulsewidth_us(RB_Vel);
}
void Go_LB(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel) //OK
{
    Dir_LF = 1; // 역
    Dir_RB = 0; // 역

    Do_PID_LF = true;
    Do_PID_RF = false;
    Do_PID_LB = false;
    Do_PID_RB = true;

    Pwm_LF.pulsewidth_us(LF_Vel);
    Pwm_RF.pulsewidth_us(0);
    Pwm_LB.pulsewidth_us(0);
    Pwm_RB.pulsewidth_us(RB_Vel);
}
void Turn_L(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel) //OK
{
    Dir_LF = 1; // 역
    Dir_RF = 1; // 정
    Dir_LB = 1; // 역
    Dir_RB = 1; // 정

    Do_PID_LF = true;
    Do_PID_RF = true;
    Do_PID_LB = true;
    Do_PID_RB = true;

    Pwm_LF.pulsewidth_us(LF_Vel);
    Pwm_RF.pulsewidth_us(RF_Vel);
    Pwm_LB.pulsewidth_us(LB_Vel);
    Pwm_RB.pulsewidth_us(RB_Vel);
}
void Turn_R(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel) //OK
{
    Dir_LF = 0; // 정
    Dir_RF = 0; // 역
    Dir_LB = 0; // 정
    Dir_RB = 0; // 역

    Do_PID_LF = true;
    Do_PID_RF = true;
    Do_PID_LB = true;
    Do_PID_RB = true;

    Pwm_LF.pulsewidth_us(LF_Vel);
    Pwm_RF.pulsewidth_us(RF_Vel);
    Pwm_LB.pulsewidth_us(LB_Vel);
    Pwm_RB.pulsewidth_us(RB_Vel);
}
void Go_CMD(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel) //OK
{
    Dir_LF = Target_RPM_LF >= 0? 0 : 1;
    Dir_RF = Target_RPM_RF >= 0? 1 : 0;
    Dir_LB = Target_RPM_LB >= 0? 0 : 1;
    Dir_RB = Target_RPM_RB >= 0? 1 : 0;

    Pwm_LF.pulsewidth_us(LF_Vel);
    Pwm_RF.pulsewidth_us(RF_Vel);
    Pwm_LB.pulsewidth_us(LB_Vel);
    Pwm_RB.pulsewidth_us(RB_Vel);
}

void PID_AngleVel(string MotorName, float TargetRPM, float Kp, float Ki, float Kd)
{
    if(MotorName == "LF")
    {
        Error_LF = TargetRPM - RPM_LF_LPF;
        P_LF = Error_LF * Kp;
        I_LF += (Error_LF * Ki) * Dt_main;
        D_LF = Kd * (Error_LF - Error_LF_last) / Dt_main;
        Error_LF_last = Error_LF;
        Input_RPM_LF = P_LF + I_LF + D_LF;
    }
    else if(MotorName == "RF")
    {
        Error_RF = TargetRPM - RPM_RF_LPF;
        P_RF = Error_RF * Kp;
        I_RF += (Error_RF * Ki) * Dt_main;
        D_RF = Kd * (Error_RF - Error_RF_last) / Dt_main;
        Error_RF_last = Error_RF;       
        Input_RPM_RF = P_RF + I_RF + D_RF;
    }
    else if(MotorName == "LB")
    {
        Error_LB = TargetRPM - RPM_LB_LPF;
        P_LB = Error_LB * Kp;
        I_LB += (Error_LB * Ki) * Dt_main;
        D_LB = Kd * (Error_LB - Error_LB_last) / Dt_main;
        Error_LB_last = Error_LB;       
        Input_RPM_LB = P_LB + I_LB + D_LB;
    }
    else if(MotorName == "RB")
    {
        Error_RB = TargetRPM - RPM_RB_LPF;
        P_RB = Error_RB * Kp;
        I_RB += (Error_RB * Ki) * Dt_main;
        D_RB = Kd * (Error_RB - Error_RB_last) / Dt_main;
        Error_RB_last = Error_RB;       
        Input_RPM_RB = P_RB + I_RB + D_RB;
    }
    I_LF = constrain<float>(I_LF, -100, 100);
    I_RF = constrain<float>(I_RF, -100, 100);
    I_LB = constrain<float>(I_LB, -100, 100);
    I_RB = constrain<float>(I_RB, -100, 100);
    Input_RPM_LF = constrain<float>(Input_RPM_LF, 0, 100);
    Input_RPM_RF = constrain<float>(Input_RPM_RF, 0, 100);
    Input_RPM_LB = constrain<float>(Input_RPM_LB, 0, 100);
    Input_RPM_RB = constrain<float>(Input_RPM_RB, 0, 100);
}
void Move(float T_vx, float T_vy, float T_yaw_vel)
{
    double desired_LF, desired_RF, desired_LB, desired_RB;
    uint8_t rpm_LF, rpm_RF, rpm_LB, rpm_RB;

    desired_LF = (1.0 / (WHEEL_RADIUS + ROLLER_RADIUS)) * (T_vx - T_vy - T_yaw_vel * (LENGTH_X + LENGTH_Y));
    desired_RF = (1.0 / (WHEEL_RADIUS + ROLLER_RADIUS)) * (T_vx + T_vy + T_yaw_vel * (LENGTH_X + LENGTH_Y));
    desired_LB = (1.0 / (WHEEL_RADIUS + ROLLER_RADIUS)) * (T_vx + T_vy - T_yaw_vel * (LENGTH_X + LENGTH_Y));
    desired_RB = (1.0 / (WHEEL_RADIUS + ROLLER_RADIUS)) * (T_vx - T_vy + T_yaw_vel * (LENGTH_X + LENGTH_Y));

    Target_RPM_CMD_LF = desired_LF * RAD_TO_RPM_P_SEC;
    Target_RPM_CMD_RF = desired_RF * RAD_TO_RPM_P_SEC;
    Target_RPM_CMD_LB = desired_LB * RAD_TO_RPM_P_SEC;
    Target_RPM_CMD_RB = desired_RB * RAD_TO_RPM_P_SEC;

    if(Target_RPM_CMD_LF > 0) Dir_LF = 0;
    else if(Target_RPM_CMD_LF < 0) Dir_LF = 1;

    if(Target_RPM_CMD_RF > 0) Dir_RF = 1;
    else if(Target_RPM_CMD_RF < 0) Dir_RF = 0;

    if(Target_RPM_CMD_LB > 0) Dir_LB = 0;
    else if(Target_RPM_CMD_LB < 0) Dir_LB = 1;

    if(Target_RPM_CMD_RB > 0) Dir_RB = 1;
    else if(Target_RPM_CMD_RB < 0) Dir_RB = 0;

    Pwm_LF.pulsewidth_us(Input_RPM_LF);
    Pwm_RF.pulsewidth_us(Input_RPM_RF);
    Pwm_LB.pulsewidth_us(Input_RPM_LB);
    Pwm_RB.pulsewidth_us(Input_RPM_RB);
}
void PID_CMD()
{
        if(Do_PID)
        {
            PID_AngleVel("LF", fabs(Target_RPM_CMD_LF), 0.2, 1.2, 0);

            PID_AngleVel("RF", fabs(Target_RPM_CMD_RF), 0.2, 1.2, 0);

            PID_AngleVel("LB", fabs(Target_RPM_CMD_LB), 0.2, 1.2, 0);

            PID_AngleVel("RB", fabs(Target_RPM_CMD_RB), 0.2, 1.2, 0);
        }
        else
        {
            Error_LF = 0;
            Error_RF = 0;
            Error_LB = 0;
            Error_RB = 0;
            RPM_LF_LPF = 0;
            RPM_RF_LPF = 0;
            RPM_LB_LPF = 0;
            RPM_RB_LPF = 0;
            Input_RPM_LF = 0;
            Input_RPM_RF = 0;
            Input_RPM_LB = 0;
            Input_RPM_RB = 0;
            cnt_lf = 0;
            cnt_rf = 0;
            cnt_lb = 0;
            cnt_rb = 0;
            cnt_lf_last = 0;
            cnt_rf_last = 0;
            cnt_lb_last = 0;
            cnt_rb_last = 0;
            I_LF = 0;
            I_RF = 0;
            I_LB = 0;
            I_RB = 0;
            Target_vx = 0;
            Target_vy = 0;
            Target_Yaw_vel = 0;
            Target_RPM_LF = 0;
            Target_RPM_RF = 0;
            Target_RPM_LB = 0;
            Target_RPM_RB = 0;
            Target_RPM_CMD_LF = 0;
            Target_RPM_CMD_RF = 0;
            Target_RPM_CMD_LB = 0;
            Target_RPM_CMD_RB = 0;
        }
}

void PID()
{
    if(flag_pc == false)
    {
        ALL_RESET();
        // ThisThread::sleep_for(1000);
        flag_pc = true;
    }
    else if(flag_pc == true)
    {
        if(Do_PID)
        {
            if(Do_PID_LF) PID_AngleVel("LF", fabs(Target_RPM_LF) , 0.2, 1.2, 0);
            else Input_RPM_LF = 0;

            if(Do_PID_RF) PID_AngleVel("RF", fabs(Target_RPM_RF), 0.2, 1.2, 0);
            else Input_RPM_RF = 0;

            if(Do_PID_LB) PID_AngleVel("LB", fabs(Target_RPM_LB), 0.2, 1.2, 0);
            else Input_RPM_LB = 0;

            if(Do_PID_RB) PID_AngleVel("RB", fabs(Target_RPM_RB), 0.2, 1.2, 0);
            else Input_RPM_RB = 0;
        }
        else
        {
            RPM_LF_LPF = 0;
            RPM_RF_LPF = 0;
            RPM_LB_LPF = 0;
            RPM_RB_LPF = 0;
            Input_RPM_LF = 0;
            Input_RPM_RF = 0;
            Input_RPM_LB = 0;
            Input_RPM_RB = 0;
            cnt_lf_last = 0;
            cnt_rf_last = 0;
            cnt_lb_last = 0;
            cnt_rb_last = 0;
            cnt_lf = 0;
            cnt_rf = 0;
            cnt_lb = 0;
            cnt_rb = 0;
            I_LF = 0;
            I_RF = 0;
            I_LB = 0;
            I_RB = 0;
            Target_vx = 0;
            Target_vy = 0;
            Target_Yaw_vel = 0;
            Target_RPM_LF = 0;
            Target_RPM_RF = 0;
            Target_RPM_LB = 0;
            Target_RPM_RB = 0;
            Target_RPM_CMD_LF = 0;
            Target_RPM_CMD_RF = 0;
            Target_RPM_CMD_LB = 0;
            Target_RPM_CMD_RB = 0;
        }        
    }
}
void Get_MotorControl_Protocol()
{
    if(HC_complete)
    {   
        inputString_HC = Buf_HC;
        if(inputString_HC.find("I") == -1)
        {
            flag_all = true;
        }
        if(inputString_HC.find("F:") != -1)
        {
            char For_Compare_char[20] = "";
            char* For_Compare_char_After;
            string Parsed_string = inputString_HC.substr(inputString_HC.find("F:") + 2, inputString_HC.find(';') - inputString_HC.find("F:") - 2);
            
            memcpy(For_Compare_char, Parsed_string.c_str(), 20);

            float For_Substitute = strtod(For_Compare_char, &For_Compare_char_After);
            if(strcmp(For_Compare_char, For_Compare_char_After))
            {
                MotorMode = "F";
                //Target_RPM = For_Substitute;
                Target_RPM_LF = For_Substitute;
                Target_RPM_RF = For_Substitute;
                Target_RPM_LB = For_Substitute;
                Target_RPM_RB = For_Substitute;
                Do_PID = true;
            }
        }
        if(inputString_HC.find("B:") != -1)
        {
            char For_Compare_char[20] = "";
            char* For_Compare_char_After;
            string Parsed_string = inputString_HC.substr(inputString_HC.find("B:") + 2, inputString_HC.find(';') - inputString_HC.find("B:") - 2);
            memcpy(For_Compare_char, Parsed_string.c_str(), 20);

            float For_Substitute = strtod(For_Compare_char, &For_Compare_char_After);
            if(strcmp(For_Compare_char, For_Compare_char_After))
            {
                MotorMode = "B";
                // Target_RPM = For_Substitute;
                Target_RPM_LF = For_Substitute;
                Target_RPM_RF = For_Substitute;
                Target_RPM_LB = For_Substitute;
                Target_RPM_RB = For_Substitute;
                Do_PID = true;
            }
        }
        if(inputString_HC.find("S:") != -1)
        {
            char For_Compare_char[20] = "";
            char* For_Compare_char_After;
            string Parsed_string = inputString_HC.substr(inputString_HC.find("S:") + 2, inputString_HC.find(';') - inputString_HC.find("S:") - 2);
            memcpy(For_Compare_char, Parsed_string.c_str(), 20);

            float For_Substitute = strtod(For_Compare_char, &For_Compare_char_After);
            if(strcmp(For_Compare_char, For_Compare_char_After))
            {
                MotorMode = "S";
                // Target_RPM = For_Substitute;
                Target_RPM_LF = 0;
                Target_RPM_RF = 0;
                Target_RPM_LB = 0;
                Target_RPM_RB = 0;
                Do_PID = false;
            }
        }
        if(inputString_HC.find("LF:") != -1)
        {
            char For_Compare_char[20] = "";
            char* For_Compare_char_After;
            string Parsed_string = inputString_HC.substr(inputString_HC.find("LF:") + 3, inputString_HC.find(';') - inputString_HC.find("LF:") - 3);
            memcpy(For_Compare_char, Parsed_string.c_str(), 20);

            float For_Substitute = strtod(For_Compare_char, &For_Compare_char_After);
            if(strcmp(For_Compare_char, For_Compare_char_After))
            {
                MotorMode = "LF";
                // Target_RPM = For_Substitute;
                Target_RPM_LF = For_Substitute;
                Target_RPM_RF = For_Substitute;
                Target_RPM_LB = For_Substitute;
                Target_RPM_RB = For_Substitute;
                Do_PID = true;
            }
        }
        if(inputString_HC.find("RF:") != -1)
        {
            char For_Compare_char[20] = "";
            char* For_Compare_char_After;
            string Parsed_string = inputString_HC.substr(inputString_HC.find("RF:") + 3, inputString_HC.find(';') - inputString_HC.find("RF:") - 3);
            memcpy(For_Compare_char, Parsed_string.c_str(), 20);

            float For_Substitute = strtod(For_Compare_char, &For_Compare_char_After);
            if(strcmp(For_Compare_char, For_Compare_char_After))
            {
                MotorMode = "RF";
                // Target_RPM = For_Substitute;
                Target_RPM_LF = For_Substitute;
                Target_RPM_RF = For_Substitute;
                Target_RPM_LB = For_Substitute;
                Target_RPM_RB = For_Substitute;
                Do_PID = true;
            }
        }
        if(inputString_HC.find("L:") != -1)
        {
            char For_Compare_char[20] = "";
            char* For_Compare_char_After;
            string Parsed_string = inputString_HC.substr(inputString_HC.find("L:") + 2, inputString_HC.find(';') - inputString_HC.find("L:") - 2);
            memcpy(For_Compare_char, Parsed_string.c_str(), 20);

            float For_Substitute = strtod(For_Compare_char, &For_Compare_char_After);
            if(strcmp(For_Compare_char, For_Compare_char_After))
            {
                MotorMode = "L";
                // Target_RPM = For_Substitute;
                Target_RPM_LF = For_Substitute;
                Target_RPM_RF = For_Substitute;
                Target_RPM_LB = For_Substitute;
                Target_RPM_RB = For_Substitute;
                Do_PID = true;
            }
        }
        if(inputString_HC.find("R:") != -1)
        {
            char For_Compare_char[20] = "";
            char* For_Compare_char_After;
            string Parsed_string = inputString_HC.substr(inputString_HC.find("R:") + 2, inputString_HC.find(';') - inputString_HC.find("R:") - 2);
            memcpy(For_Compare_char, Parsed_string.c_str(), 20);

            float For_Substitute = strtod(For_Compare_char, &For_Compare_char_After);
            if(strcmp(For_Compare_char, For_Compare_char_After))
            {
                MotorMode = "R";
                // Target_RPM = For_Substitute;
                Target_RPM_LF = For_Substitute;
                Target_RPM_RF = For_Substitute;
                Target_RPM_LB = For_Substitute;
                Target_RPM_RB = For_Substitute;
                Do_PID = true;
            }
        }
        if(inputString_HC.find("LB:") != -1)
        {
            char For_Compare_char[20] = "";
            char* For_Compare_char_After;
            string Parsed_string = inputString_HC.substr(inputString_HC.find("LB:") + 3, inputString_HC.find(';') - inputString_HC.find("LB:") - 3);
            memcpy(For_Compare_char, Parsed_string.c_str(), 20);

            float For_Substitute = strtod(For_Compare_char, &For_Compare_char_After);
            if(strcmp(For_Compare_char, For_Compare_char_After))
            {
                MotorMode = "LB";
                // Target_RPM = For_Substitute;
                Target_RPM_LF = For_Substitute;
                Target_RPM_RF = For_Substitute;
                Target_RPM_LB = For_Substitute;
                Target_RPM_RB = For_Substitute;
                Do_PID = true;
            }
        }
        if(inputString_HC.find("RB:") != -1)
        {
            char For_Compare_char[20] = "";
            char* For_Compare_char_After;
            string Parsed_string = inputString_HC.substr(inputString_HC.find("RB:") + 3, inputString_HC.find(';') - inputString_HC.find("RB:") - 3);
            memcpy(For_Compare_char, Parsed_string.c_str(), 20);

            float For_Substitute = strtod(For_Compare_char, &For_Compare_char_After);
            if(strcmp(For_Compare_char, For_Compare_char_After))
            {
                MotorMode = "RB";
                // Target_RPM = For_Substitute;
                Target_RPM_LF = For_Substitute;
                Target_RPM_RF = For_Substitute;
                Target_RPM_LB = For_Substitute;
                Target_RPM_RB = For_Substitute;
                Do_PID = true;
            }
        }
        if(inputString_HC.find("TL:") != -1)
        {
            char For_Compare_char[20] = "";
            char* For_Compare_char_After;
            string Parsed_string = inputString_HC.substr(inputString_HC.find("TL:") + 3, inputString_HC.find(';') - inputString_HC.find("TL:") - 3);
            memcpy(For_Compare_char, Parsed_string.c_str(), 20);

            float For_Substitute = strtod(For_Compare_char, &For_Compare_char_After);
            if(strcmp(For_Compare_char, For_Compare_char_After))
            {
                MotorMode = "TL";
                // Target_RPM = For_Substitute;
                Target_RPM_LF = For_Substitute;
                Target_RPM_RF = For_Substitute;
                Target_RPM_LB = For_Substitute;
                Target_RPM_RB = For_Substitute;
                Do_PID = true;
            }
        }
        if(inputString_HC.find("TR:") != -1)
        {
            char For_Compare_char[20] = "";
            char* For_Compare_char_After;
            string Parsed_string = inputString_HC.substr(inputString_HC.find("TR:") + 3, inputString_HC.find(';') - inputString_HC.find("TR:") - 3);
            memcpy(For_Compare_char, Parsed_string.c_str(), 20);

            float For_Substitute = strtod(For_Compare_char, &For_Compare_char_After);
            if(strcmp(For_Compare_char, For_Compare_char_After))
            {
                MotorMode = "TR";
                // Target_RPM = For_Substitute;
                Target_RPM_LF = For_Substitute;
                Target_RPM_RF = For_Substitute;
                Target_RPM_LB = For_Substitute;
                Target_RPM_RB = For_Substitute;
                Do_PID = true;
            }
        }
        if(inputString_HC.find("Br:") != -1) // 브러쉬
        {
            char For_Compare_char[20] = "";
            char* For_Compare_char_After;
            string Parsed_string = inputString_HC.substr(inputString_HC.find("Br:") + 3, inputString_HC.find(';') - inputString_HC.find("Br:") - 3);
            memcpy(For_Compare_char, Parsed_string.c_str(), 20);

            float For_Substitute = strtod(For_Compare_char, &For_Compare_char_After);
            if(strcmp(For_Compare_char, For_Compare_char_After))
            {
                if(For_Substitute == 1) Flag_Br = true;
                else if(For_Substitute == 0) Flag_Br = false;
            }
        }
        if(inputString_HC.find("W:") != -1) // 물 펌프
        {
            char For_Compare_char[20] = "";
            char* For_Compare_char_After;
            string Parsed_string = inputString_HC.substr(inputString_HC.find("W:") + 2, inputString_HC.find(';') - inputString_HC.find("W:") - 2);
            memcpy(For_Compare_char, Parsed_string.c_str(), 20);

            float For_Substitute = strtod(For_Compare_char, &For_Compare_char_After);
            if(strcmp(For_Compare_char, For_Compare_char_After))
            {
                if(For_Substitute == 1) Flag_W = true;
                else if(For_Substitute == 0) Flag_W = false;
            }
        }
        if(inputString_HC.find("CMD:") != -1)
        {
            char For_Compare_char[20] = "";
            char* For_Compare_char_After;
            
            string Parsed_string_1 = inputString_HC.substr(inputString_HC.find("CMD:") + 4, inputString_HC.find(';') - inputString_HC.find("CMD:") - 4);

            bool CMD_complete = false;

            vector<unsigned long> pos_comma(4);

            pos_comma[0] = Parsed_string_1.find(",");
            pos_comma[1] = Parsed_string_1.find(",",pos_comma[0] + 1);
            pos_comma[2] = Parsed_string_1.find(",",pos_comma[1] + 1);
            pos_comma[3] = Parsed_string_1.length();

            char str_cmd_RF[20] = "",
                 str_cmd_LF[20] = "",
                 str_cmd_RB[20] = "",
                 str_cmd_LB[20] = "";

            if((pos_comma[0] != string::npos) &&
            (pos_comma[1] != string::npos) &&
            (pos_comma[2] != string::npos))
            {
                CMD_complete = true;
            }

            if(CMD_complete)
            {
                memcpy(str_cmd_LF, Parsed_string_1.c_str(), pos_comma[0]);
                memcpy(str_cmd_RF, &Parsed_string_1[pos_comma[0] + 1], pos_comma[1] - pos_comma[0] - 1);
                memcpy(str_cmd_LB, &Parsed_string_1[pos_comma[1] + 1], pos_comma[2] - pos_comma[1] - 1);
                memcpy(str_cmd_RB, &Parsed_string_1[pos_comma[2] + 1], pos_comma[3] - pos_comma[2] - 1);

                char *str_cmd_LF_after, *str_cmd_RF_after, *str_cmd_LB_after, *str_cmd_RB_after;

                double cmd_LF = strtod(str_cmd_LF,&str_cmd_LF_after);
                double cmd_RF = strtod(str_cmd_RF,&str_cmd_RF_after);
                double cmd_LB = strtod(str_cmd_LB,&str_cmd_LB_after);
                double cmd_RB = strtod(str_cmd_RB,&str_cmd_RB_after);

                if(strcmp(str_cmd_LF,str_cmd_LF_after) && strcmp(str_cmd_RF,str_cmd_RF_after) &&
                   strcmp(str_cmd_LB,str_cmd_LB_after) && strcmp(str_cmd_RB,str_cmd_RB_after))
                {                    
                    MotorMode = "CMD";                    
                    Target_RPM_LF = cmd_LF;
                    Target_RPM_RF = cmd_RF;
                    Target_RPM_LB = cmd_LB;
                    Target_RPM_RB = cmd_RB;
                    Do_PID = true;
                }
            }
        }

        HC_Count = 0;
        memset(Buf_HC, 0, sizeof(Buf_HC));
        HC_complete = false;
    }
}

void Check_PC_RX()
{    
    if(PC.read(&HC_Char,1))
    {   
        inputChar_HC = HC_Char;
        if(inputChar_HC == '<' && !HC_complete) Flag_Start_Get_String = true;
        if(Flag_Start_Get_String)
        {
            Buf_HC[HC_Count++] = inputChar_HC;
            if(inputChar_HC == '>')
            {
                Flag_Start_Get_String = false;
                HC_complete = true;
            }
        }

    }
} 

void Check_HC_RX()
{
    if(HC06.read(&HC_Char,1))
    //if(PC.read(&HC_Char,1))
    {   
        inputChar_HC = HC_Char;
        // printf("%c\n", HC_Char);
        if(inputChar_HC == '<' && !HC_complete) Flag_Start_Get_String = true;
        if(Flag_Start_Get_String)
        {
            Buf_HC[HC_Count++] = inputChar_HC;
            if(inputChar_HC == '>')
            {
                Flag_Start_Get_String = false;
                HC_complete = true;
            }
        }

    }
}

void Check_IMU_RX()
{
    if(IMU.read(&IMU_Char,1))
    {   
        inputChar_IMU = IMU_Char;
        if(inputChar_IMU == '*' && !IMU_complete) Flag_Start_Get_String_IMU = true;
        if(Flag_Start_Get_String_IMU)
        {
            Buf_IMU[IMU_Count++] = inputChar_IMU;
            if(inputChar_IMU == '\n')
            {
                Flag_Start_Get_String_IMU = false;
                IMU_complete = true;
            }
        }
    }
}

void Check_RisingEdge_PPM()
{
    TMR_PPM.reset();
}
void Check_FallingEdge_PPM()
{
    ICValue_All_PPM[i] = TMR_PPM.read_us();
    i++;
    if(i == 19)
    {
        for(j = 0; j < 19; j++)
        {
            ICValue_All_PWM[j] = ICValue_All_PPM[j];
        }
        i = 0;
        for(k = 18; k > -1;  k--)
        {
            if(ICValue_All_PWM[k] >= 10000)
            {
                j = k;
            }
        }
        for(p = 0; p <= 8; p++)
        {
            ICValue_Divided_PWM[p] = ICValue_All_PWM[p+j];
        }
        ICValue_CH0 = ICValue_Divided_PWM[0];
        ICValue_CH1 = ICValue_Divided_PWM[1];
        ICValue_CH2 = ICValue_Divided_PWM[2];
        ICValue_CH3 = ICValue_Divided_PWM[3];
        ICValue_CH4 = ICValue_Divided_PWM[4];
        ICValue_CH5 = ICValue_Divided_PWM[5];
        ICValue_CH6 = ICValue_Divided_PWM[6];
        ICValue_CH7 = ICValue_Divided_PWM[7];
        ICValue_CH8 = ICValue_Divided_PWM[8];
    }
}

double vx = 0, vy = 0, yaw_dot = 0;
float prev_yaw = 0;
float yaw = 0;
void Get_Robot_Status()
{
    // yaw = RPY_imu[2];

    double pi_dot_1 = Dir_LF == 1 ? - RPM_LF_LPF * RPM_TO_RAD_P_SEC : RPM_LF_LPF * RPM_TO_RAD_P_SEC;
    double pi_dot_2 = Dir_RF == 0 ? - RPM_RF_LPF * RPM_TO_RAD_P_SEC : RPM_RF_LPF * RPM_TO_RAD_P_SEC;
    double pi_dot_3 = Dir_LB == 1 ? - RPM_LB_LPF * RPM_TO_RAD_P_SEC : RPM_LB_LPF * RPM_TO_RAD_P_SEC;
    double pi_dot_4 = Dir_RB == 0 ? - RPM_RB_LPF * RPM_TO_RAD_P_SEC : RPM_RB_LPF * RPM_TO_RAD_P_SEC;

    vx = ((WHEEL_RADIUS + ROLLER_RADIUS) / 4) * (pi_dot_1 + pi_dot_2 + pi_dot_3 + pi_dot_4);
    vy = ((WHEEL_RADIUS + ROLLER_RADIUS) / 4) * ((-pi_dot_1) + pi_dot_2 + pi_dot_3 + (-pi_dot_4));
    vx *= 0.75;
    vy *= 0.75;
    // yaw_dot = (yaw - prev_yaw) / Dt;
    
    // prev_yaw = yaw;
}

void Get_IMU_Protocol()
{
    int n,i = 0;
    char* addr;
    int result;
    if(IMU_complete)
    {   
        addr = strtok(Buf_IMU,"*,\n");
        while(addr != NULL)
        {
            RPY_imu[i] = atof(addr);
            addr = strtok(NULL,"*,\n");
            i++;
        }
        RPY_imu[2] *= -1;
        i = 0;

        IMU_Count = 0;
        memset(Buf_IMU, 0, sizeof(Buf_IMU));
        IMU_complete = false;
    }
}

void Cal_AngVel()
{
    RPM_LF = (cnt_lf - cnt_lf_last) / Dt_main * 60 * 0.8 / 360.0;
    cnt_lf_last = cnt_lf;
    if(RPM_LF <= 0) RPM_LF *= -1;
    RPM_LF_LPF = RPM_LF_LPF * 0.95 + RPM_LF * 0.05;

    RPM_RF = (cnt_rf - cnt_rf_last) / Dt_main * 60 * 0.8 / 360.0;
    cnt_rf_last = cnt_rf;
    if(RPM_RF <= 0) RPM_RF *= -1;
    RPM_RF_LPF = RPM_RF_LPF * 0.95 + RPM_RF * 0.05;

    RPM_LB = (cnt_lb - cnt_lb_last) / Dt_main * 60 * 0.8 / 360.0;
    cnt_lb_last = cnt_lb;
    if(RPM_LB <= 0) RPM_LB *= -1;
    RPM_LB_LPF = RPM_LB_LPF * 0.95 + RPM_LB * 0.05;

    RPM_RB = (cnt_rb - cnt_rb_last) / Dt_main * 60 * 0.8 / 360.0;
    cnt_rb_last = cnt_rb;
    if(RPM_RB <= 0) RPM_RB *= -1;
    RPM_RB_LPF = RPM_RB_LPF * 0.95 + RPM_RB * 0.05;

}

void Check_RisingEdge_LF()
{
    flag_cnt_lf = true;
}

void Check_FallingEdge_LF()
{ 
    if(flag_cnt_lf == true && Dir_LF == 0)
    {
        cnt_lf++;
        flag_cnt_lf = false;
    }
    else if(flag_cnt_lf == true && Dir_LF == 1)
    {
        cnt_lf--;
        flag_cnt_lf = false;
    }
}
void Check_RisingEdge_RF()
{
    flag_cnt_rf = true;
}
void Check_FallingEdge_RF()
{
    if(flag_cnt_rf == true && Dir_RF == 0)
    {
        cnt_rf++;
        flag_cnt_rf = false;
    }
    else if(flag_cnt_rf == true && Dir_RF == 1)
    {
        cnt_rf--;
        flag_cnt_rf = false;
    }
}
void Check_RisingEdge_LB()
{
    flag_cnt_lb = true;
}
void Check_FallingEdge_LB()
{
    if(flag_cnt_lb == true && Dir_LB == 0)
    {
        cnt_lb++;
        flag_cnt_lb = false;
    }
    else if(flag_cnt_lb == true && Dir_LB == 1)
    {
        cnt_lb--;
        flag_cnt_lb = true;
    }
}
void Check_RisingEdge_RB()
{
    flag_cnt_rb = true;
}
void Check_FallingEdge_RB()
{
    if(flag_cnt_rb == true && Dir_RB == 0)
    {
        cnt_rb++;
        flag_cnt_rb = false;
    }
    else if(flag_cnt_rb == true && Dir_RB == 1)
    {
        cnt_rb--;
        flag_cnt_rb = true;
    }
}

void Check_Mode()
{
    //비상정지
    if(ICValue_CH6 >= 1400)  // 출발
    {
        if((ICValue_CH7 < 1200 && ICValue_CH7 > 1000))          //모드 변경
        {
            if(flag_change1 == false) // PC에서 조종기
            {
                ALL_RESET();
                Do_PID = false;
                flag_pc = false;
                flag_all = false;
                MotorMode = "S";
                Stop();
                ThisThread::sleep_for(1000);
                flag_change1 = true;
            }
            else if(flag_change1 == true)
            {
                Do_PID = true;
                flag_all = true;
                MotorMode = "A";
                ControlMode = "Controller";
                flag_change2 = false;
            }
        }
        else 
        {   
            if(flag_change2 == false) // 조종기에서 PC
            {
                ALL_RESET();
                Do_PID = false;
                flag_all = false;
                flag_pc = false;
                MotorMode = "S";
                Stop();
                ThisThread::sleep_for(1000);
                flag_change2 = true;
            }   
            else if(flag_change2 == true)
            {
                Do_PID = true;
                flag_all = true;
                ControlMode = "PC";  
                flag_change1 = false;
            }
        }   
        if(flag_cnt == false)
        {
            flag_all = false;
            Target_vx = 0;
            Target_vy = 0;
            Target_Yaw_vel = 0;
            Srt_LF = 0;
            Srt_RF = 0;
            Srt_LB = 0;
            Srt_RB = 0;
            Do_PID = false;
            Input_RPM_LF = 0;
            Input_RPM_RF = 0;
            Input_RPM_LB = 0;
            Input_RPM_RB = 0;
            I_LF = 0;
            I_RF = 0;
            I_LB = 0;
            I_RB = 0;
            Target_RPM_LF = 0;
            Target_RPM_RF = 0;
            Target_RPM_LB = 0;
            Target_RPM_RB = 0;
            Target_RPM_CMD_LF = 0;
            Target_RPM_CMD_RF = 0;
            Target_RPM_CMD_LB = 0;
            Target_RPM_CMD_RB = 0;
            cnt_lf = 0;
            cnt_rf = 0;
            cnt_lb = 0;
            cnt_rb = 0;
            cnt_lf_last = 0;
            cnt_rf_last = 0;
            cnt_lb_last = 0;
            cnt_rb_last = 0;
            ThisThread::sleep_for(1000);
            flag_cnt = true;
        }
        else if(flag_cnt == true)
        {
            Srt_LF = 1;
            Srt_RF = 1;
            Srt_LB = 1;
            Srt_RB = 1;
            Do_PID = true;
            flag_all = true;
            // MotorMode = "I";
        }
    }
    else if(ICValue_CH6 < 800) // 정지
    {
        flag_cnt = false;
        flag_all = false;
        Srt_LF = 0;
        Srt_RF = 0;
        Srt_LB = 0;
        Srt_RB = 0;
        Do_PID = false;
        Input_RPM_LF = 0;
        Input_RPM_RF = 0;
        Input_RPM_LB = 0;
        Input_RPM_RB = 0;
        I_LF = 0;
        I_RF = 0;
        I_LB = 0;
        I_RB = 0;
        Target_RPM_LF = 0;
        Target_RPM_RF = 0;
        Target_RPM_LB = 0;
        Target_RPM_RB = 0;
        Target_RPM_CMD_LF = 0;
        Target_RPM_CMD_RF = 0;
        Target_RPM_CMD_LB = 0;
        Target_RPM_CMD_RB = 0;
        cnt_lf = 0;
        cnt_rf = 0;
        cnt_lb = 0;
        cnt_rb = 0;
        cnt_lf_last = 0;
        cnt_rf_last = 0;
        cnt_lb_last = 0;
        cnt_rb_last = 0;

        Stop();
        MotorMode = "S";
    }
}

void CTRL()
{
    Target_vx = map<float>(ICValue_CH2, 700.0, 1500.0, -0.5, 0.5);
    if(Target_vx >= -0.1 && Target_vx <= 0.1) Target_vx = 0;

    Target_vy = map<float>(ICValue_CH3, 700.0, 1500.0, -0.5, 0.5);
    if(Target_vy >= -0.1 && Target_vy <= 0.1) Target_vy = 0;

    Target_Yaw_vel = map<float>(ICValue_CH5, 700.0, 1500.0, -1.5, 1.5);
    if(Target_Yaw_vel >= -0.3 && Target_Yaw_vel <= 0.3) Target_Yaw_vel = 0;
}

void Thread_IMU()
{
    while(true)
    {
        current_imu_tick = Kernel::get_ms_count();

        int n,i = 0;
        char* addr;
        int result;
        if(IMU_complete)
        {   
            addr = strtok(Buf_IMU,"*,\n");
            while(addr != NULL)
            {
                RPY_imu[i] = atof(addr);
                addr = strtok(NULL,"*,\n");
                i++;
            }
            RPY_imu[2] *= -1;
            i = 0;

            IMU_Count = 0;
            memset(Buf_IMU, 0, sizeof(Buf_IMU));
            IMU_complete = false;
        }
        YAW = RPY_imu[2];
        YAW_DOT = (YAW - PREV_YAW) / Dt_imu;
        PREV_YAW = YAW;

        working_imu_tick = Kernel::get_ms_count();

        ThisThread::sleep_until(Kernel::get_ms_count() + (Dt_imu * 1000 - (working_imu_tick - current_imu_tick)));
    }
}

void Thread_Uart()
{
    while(true)
    {
        current_uart_tick = Kernel::get_ms_count();

        // printf("%d, %d, %d, %d\n",(int)Input_RPM_LF,(int)Input_RPM_RF,(int)Input_RPM_LB,(int)Input_RPM_RB);
        // hc.printf("%.2f, %.2f, %.2f\n",vx,vy,yaw_dot);
        // hc.printf("%.2f, %.2f, %.2f, %.5f, %.2f, %.2f, %.2f\n",RPY_imu[0], RPY_imu[1], RPY_imu[2],YAW_DOT, PREV_YAW, yaw_dot, prev_yaw);
        // hc.printf("%.2f\n",yaw_dot);
        // hc.printf("%.2f,%.2f\n",yaw_dot,prev_yaw);
        // hc.printf("%.2f, %.2f\n",YAW, YAW_DOT);
        // hc.printf("%.2f, %.2f, %.2f\n",Target_vx, Target_vy, Target_Yaw_vel);
        // hc.printf("%.2f, %.2f, %.2f\n",vx, vy)
        char asdf[2];
        char asd[2];
        strcpy(asdf,ControlMode.c_str());
        strcpy(asd,MotorMode.c_str());
        // hc.printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %.2f, %.2f, %.2f, %s, %s\n",(int)ICValue_CH0,(int)ICValue_CH1,(int)ICValue_CH2,(int)ICValue_CH3
                                                                    // ,(int)ICValue_CH4,(int)ICValue_CH5,(int)ICValue_CH6,(int)ICValue_CH7,(int)ICValue_CH8
                                                                    // ,Target_vx, Target_vy, Target_Yaw_vel, asdf, asd);
        // hc.printf("%d, %d, %d, %d, %s, %s, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",Input_RPM_LF,Input_RPM_RF, Input_RPM_LB, Input_RPM_RB,asdf, asd, Target_vx, Target_vy, Target_Yaw_vel
                                                                                    // ,Target_RPM_CMD_LF,Target_RPM_CMD_RF,Target_RPM_CMD_LB,Target_RPM_CMD_RB);
        // hc.printf("%d, %d, %d, %d\n",(int)I_LF, Input_RPM_LF, (int)Target_RPM_LF, (int)Target_RPM_CMD_LF);
        // hc.printf("%s, %s, %d, %d, %d, %.2f, %d, %d, %d, %d\n",asdf, asd, (int)I_LF, (int)Input_RPM_LF, (int)Target_RPM_CMD_LF, Target_vx, (int)Error_LF,flag_change1, flag_change2, flag_all);
        printf("<VX:%.3f;VY:%.3f;DY:%.3f;Y:%.3f;Br:%d;W:%d;>\n",
                    vx,vy,YAW_DOT,RPY_imu[2],Flag_Br,Flag_W);
        // hc.printf("%d, %d, %d, %d\n",flag_pc,flag_all,flag_change1,flag_change2);
        
        working_uart_tick = Kernel::get_ms_count();

        ThisThread::sleep_until(Kernel::get_ms_count() + (Dt_uart * 1000 - (working_uart_tick - current_uart_tick)));
    }
}

void Thread_Main()
{
    while(true)
    {
        current_main_tick = Kernel::get_ms_count();

        Get_MotorControl_Protocol();
        Check_Mode();

        if(flag_all == false)
        {
            Srt_LF = 0;
            Srt_RF = 0;
            Srt_LB = 0;
            Srt_RB = 0;
            ALL_RESET();
            continue;
        }
        // Get_IMU_Protocol();
        Srt_LF = 1;
        Srt_RF = 1;
        Srt_LB = 1;
        Srt_RB = 1;
        
        if(ControlMode == "PC")
        {
            PID();
            if(MotorMode == "F") Go_F(Input_RPM_LF, Input_RPM_RF, Input_RPM_LB, Input_RPM_RB);
            else if(MotorMode == "B") Go_B(Input_RPM_LF, Input_RPM_RF, Input_RPM_LB, Input_RPM_RB);
            else if(MotorMode == "LF") Go_LF(Input_RPM_LF, Input_RPM_RF, Input_RPM_LB, Input_RPM_RB);
            else if(MotorMode == "RF") Go_RF(Input_RPM_LF, Input_RPM_RF, Input_RPM_LB, Input_RPM_RB);
            else if(MotorMode == "L") Go_L(Input_RPM_LF, Input_RPM_RF, Input_RPM_LB, Input_RPM_RB);
            else if(MotorMode == "R") Go_R(Input_RPM_LF, Input_RPM_RF, Input_RPM_LB, Input_RPM_RB);
            else if(MotorMode == "LB") Go_LB(Input_RPM_LF, Input_RPM_RF, Input_RPM_LB, Input_RPM_RB);
            else if(MotorMode == "RB") Go_RB(Input_RPM_LF, Input_RPM_RF, Input_RPM_LB, Input_RPM_RB);
            else if(MotorMode == "TL") Turn_L(Input_RPM_LF, Input_RPM_RF, Input_RPM_LB, Input_RPM_RB);
            else if(MotorMode == "TR") Turn_R(Input_RPM_LF, Input_RPM_RF, Input_RPM_LB, Input_RPM_RB);
            else if(MotorMode == "CMD") Go_CMD(Input_RPM_LF, Input_RPM_RF, Input_RPM_LB, Input_RPM_RB); // PC로부터 명령 받을 때
            else if(MotorMode == "S" || MotorMode == "I")
            {
                Stop();
                ALL_RESET();
            }
        }
        else if(ControlMode == "Controller") // 조종기로 조종할 때
        {
            if(MotorMode == "S")
            {
                Stop();
                ALL_RESET();
            }
            else if(MotorMode == "A")
            {
                CTRL();
                PID_CMD();
                Move(Target_vx, Target_vy, Target_Yaw_vel);
            }
        }
        if(ControlMode == "Controller") LED = 1;
        else if(ControlMode == "PC") LED = 0;
        if(MotorMode == "S")
        {
            Stop();
            ALL_RESET();
        }

        Cal_AngVel();
        Get_Robot_Status();
        Check_Br_W();

        working_main_tick = Kernel::get_ms_count();

        ThisThread::sleep_until(Kernel::get_ms_count() + (Dt_main * 1000 - (working_main_tick - current_main_tick)));
    }
}

template <class T>
T map(T x, T in_min, T in_max, T out_min, T out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


template <class A>
A constrain(A x, A min, A max)
{
    if(x >= max) x = max;
    else if(x < min) x = min;
    return x;
}

void ALL_RESET()
{
    Error_LF = 0;
    Error_LF = 0;
    Error_LF = 0;
    Error_LF = 0;
    cnt_lf = 0;
    cnt_rf = 0;
    cnt_lb = 0;
    cnt_rb = 0;
    cnt_lf_last = 0;
    cnt_rf_last = 0;
    cnt_lb_last = 0;
    cnt_rb_last = 0;
    Input_RPM_LF = 0;
    Input_RPM_RF = 0;
    Input_RPM_LB = 0;
    Input_RPM_RB = 0;
    Target_RPM_CMD_LF = 0;
    Target_RPM_CMD_RF = 0;
    Target_RPM_CMD_LB = 0;
    Target_RPM_CMD_RB = 0;
    Target_RPM_LF = 0;
    Target_RPM_RF = 0;
    Target_RPM_LB = 0;
    Target_RPM_RB = 0;
    Target_vx = 0;
    Target_vy = 0;
    Target_Yaw_vel = 0;
    I_LF = 0;
    I_RF = 0;
    I_LB = 0;
    I_RB = 0;        
    RPM_LF_LPF = 0;
    RPM_RF_LPF = 0;
    RPM_LB_LPF = 0;
    RPM_RB_LPF = 0;
    Do_PID = false;
}