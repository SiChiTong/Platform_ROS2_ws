#include "Kernel.h"
#include "ThisThread.h"
#include "Thread.h"
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

#include "string.h"
#include "QEI.h"

#define TX_IMU PC_12
#define RX_IMU PD_2

#define TX_HC06 PC_10
#define RX_HC06 PC_11

#define PWM_LF PB_3
#define DIR_LF PG_9
#define BRK_LF PC_7

#define PWM_RF PB_5
#define DIR_RF PG_14
#define BRK_RF PB_2

#define PWM_LB PC_6
#define DIR_LB PF_15
#define BRK_LB PA_8

#define PWM_RB PA_15
#define DIR_RB PE_13
#define BRK_RB PB_1

#define ENC_A_LF PF_9
#define ENC_B_LF PG_1

#define ENC_A_RF PF_7
#define ENC_B_RF PF_8

#define ENC_A_LB PE_6
#define ENC_B_LB PE_3

#define ENC_A_RB PE_5
#define ENC_B_RB PE_4

#define ENCODER_RESOLUTION_LF 1274
#define ENCODER_RESOLUTION_RF 1274
#define ENCODER_RESOLUTION_LB 1274
#define ENCODER_RESOLUTION_RB 1274

#define Dt 0.001f

#define WHEEL_RADIUS  0.0300
#define ROLLER_RADIUS 0.00575

#define RPM_TO_RAD_P_SEC 0.10472

#define I_max 20.0f

#include <chrono>

UnbufferedSerial IMU(TX_IMU, RX_IMU, 115200);
UnbufferedSerial HC06(TX_HC06, RX_HC06, 115200);
UnbufferedSerial PC(USBTX, USBRX, 115200);

SerialStream<UnbufferedSerial> imu(IMU);
SerialStream<UnbufferedSerial> hc(HC06);
SerialStream<UnbufferedSerial> pc(PC);

DigitalOut Dir_LF(DIR_LF);  //Dir_LF = 0;(Forward) Dir_LF = 1;(Backward)
DigitalOut Dir_RF(DIR_RF);  //Dir_RF = 1;(Forward) Dir_RF = 0;(Backward)
DigitalOut Dir_LB(DIR_LB);  //Dir_LB = 0;(Forward) Dir_LB = 1;(Backward)
DigitalOut Dir_RB(DIR_RB);  //Dir_RB = 0;(Forward) Dir_RB = 1;(Backward)

DigitalOut Brk_LF(BRK_LF);
DigitalOut Brk_RF(BRK_RF);
DigitalOut Brk_LB(BRK_LB);
DigitalOut Brk_RB(BRK_RB);

PwmOut Pwm_LF(PWM_LF);
PwmOut Pwm_RF(PWM_RF);
PwmOut Pwm_LB(PWM_LB);
PwmOut Pwm_RB(PWM_RB);

QEI Encoder_LF(ENC_A_LF, ENC_B_LF, NC, ENCODER_RESOLUTION_LF, QEI::X2_ENCODING);
QEI Encoder_RF(ENC_A_RF, ENC_B_RF, NC, ENCODER_RESOLUTION_RF, QEI::X2_ENCODING);
QEI Encoder_LB(ENC_A_LB, ENC_B_LB, NC, ENCODER_RESOLUTION_LB, QEI::X2_ENCODING);
QEI Encoder_RB(ENC_A_RB, ENC_B_RB, NC, ENCODER_RESOLUTION_RB, QEI::X2_ENCODING);

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

void Check_Encoder();
void Encoder_Reset();

void PID_AngleVel(string MotorName, float TargetRPM, float Kp, float Ki, float Kd);
void PID();

void Check_HC_RX();
void Get_MotorControl_Protocol();

void Check_IMU_RX();
void Get_IMU_Protocol();
int EBimuAsciiParser(float *item, int niumber_of_item);

int cnt = 0;

uint32_t current_tick = 0;
uint32_t working_tick = 0;

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

string MotorMode = "I";
int MotorSpeed = 0;
int32_t Enc_Val_LF = 0;
int32_t Enc_Val_RF = 0;
int32_t Enc_Val_LB = 0;
int32_t Enc_Val_RB = 0;
int32_t Enc_Val_LF_last = 0;
int32_t Enc_Val_RF_last = 0;
int32_t Enc_Val_LB_last = 0;
int32_t Enc_Val_RB_last = 0;

float RPM_LF = 0, RPM_LF_LPF = 0;
float RPM_RF = 0, RPM_RF_LPF = 0;
float RPM_LB = 0, RPM_LB_LPF = 0;
float RPM_RB = 0, RPM_RB_LPF = 0;

float Error_LF, Error_LF_last, P_LF, I_LF, D_LF;
float Error_RF, Error_RF_last, P_RF, I_RF, D_RF;
float Error_LB, Error_LB_last, P_LB, I_LB, D_LB;
float Error_RB, Error_RB_last, P_RB, I_RB, D_RB;

int Input_RPM_LF = 0, Input_RPM_RF = 0, Input_RPM_LB = 0, Input_RPM_RB = 0;


// float Target_RPM = 0
float Target_RPM_LF = 0, Target_RPM_RF = 0, Target_RPM_LB = 0, Target_RPM_RB = 0;

bool Do_PID = true;
bool Do_PID_LF = true, Do_PID_RF = true, Do_PID_LB = true, Do_PID_RB = true;

void Go_F(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel) //OK
{
    Dir_LF = 0;
    Dir_RF = 1;
    Dir_LB = 0;
    Dir_RB = 0;

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
    Dir_RF = 1;
    Dir_LB = 0;

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
    Dir_LF = 0;
    Dir_RB = 0;

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
    Dir_RF = 0;
    Dir_LB = 1;

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
    Pwm_LF.pulsewidth_us(0);
    Pwm_RF.pulsewidth_us(0);
    Pwm_LB.pulsewidth_us(0);
    Pwm_RB.pulsewidth_us(0);    
}

void Go_L(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel) //OK
{
    Dir_LF = 1;
    Dir_RF = 1;
    Dir_LB = 0;
    Dir_RB = 1;

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
    Dir_LF = 0;
    Dir_RF = 0;
    Dir_LB = 1;
    Dir_RB = 0;

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
    Dir_LF = 1;
    Dir_RF = 0;
    Dir_LB = 1;
    Dir_RB = 1;

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
    Dir_LF = 1;
    Dir_RB = 1;

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
    Dir_LF = 1;
    Dir_RF = 1;
    Dir_LB = 1;
    Dir_RB = 0;

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
    Do_PID_LF = true;
    Do_PID_RF = true;
    Do_PID_LB = true;
    Do_PID_RB = true;

    Dir_LF = 0;
    Dir_RF = 0;
    Dir_LB = 0;
    Dir_RB = 1;

    Pwm_LF.pulsewidth_us(LF_Vel);
    Pwm_RF.pulsewidth_us(RF_Vel);
    Pwm_LB.pulsewidth_us(LB_Vel);
    Pwm_RB.pulsewidth_us(RB_Vel);
}

void Go_CMD(float LF_Vel, float RF_Vel, float LB_Vel, float RB_Vel) //OK
{
    Do_PID_LF = true;
    Do_PID_RF = true;
    Do_PID_LB = true;
    Do_PID_RB = true;

    Dir_LF = LF_Vel >= 0? 0 : 1;
    Dir_RF = RF_Vel >= 0? 1 : 0;
    Dir_LB = LB_Vel >= 0? 0 : 1;
    Dir_RB = RB_Vel >= 0? 0 : 1;

    Pwm_LF.pulsewidth_us(fabs(LF_Vel));
    Pwm_RF.pulsewidth_us(fabs(RF_Vel));
    Pwm_LB.pulsewidth_us(fabs(LB_Vel));
    Pwm_RB.pulsewidth_us(fabs(RB_Vel));
}

void init_I_gain()
{
    I_LB = 0.0;
    I_RB = 0.0;
    I_LF = 0.0;    
    I_RF = 0.0;
}

void Check_Encoder()
{
    Enc_Val_LF = Encoder_LF.getPulses();
    RPM_LF = (Enc_Val_LF - Enc_Val_LF_last) / Dt * 60 * 0.282 / 360.0;
    Enc_Val_LF_last = Enc_Val_LF;
    RPM_LF *= -1;
    RPM_LF_LPF = RPM_LF_LPF * 0.95 + RPM_LF * 0.05;

    Enc_Val_RF = Encoder_RF.getPulses();
    RPM_RF = (Enc_Val_RF - Enc_Val_RF_last) / Dt * 60 * 0.282 / 360.0;
    Enc_Val_RF_last = Enc_Val_RF;
    RPM_RF_LPF = RPM_RF_LPF * 0.95 + RPM_RF * 0.05;

    Enc_Val_LB = Encoder_LB.getPulses();
    RPM_LB = (Enc_Val_LB - Enc_Val_LB_last) / Dt * 60 * 0.282 / 360.0;
    Enc_Val_LB_last = Enc_Val_LB;
    RPM_LB *= -1;
    RPM_LB_LPF = RPM_LB_LPF * 0.95 + RPM_LB * 0.05;

    Enc_Val_RB = Encoder_RB.getPulses();
    RPM_RB = (Enc_Val_RB - Enc_Val_RB_last) / Dt * 60 * 0.282 / 360.0;
    Enc_Val_RB_last = Enc_Val_RB;    
    RPM_RB_LPF = RPM_RB_LPF * 0.95 + RPM_RB * 0.05;
}

void Encoder_Reset()
{
    Encoder_LF.reset();
    Encoder_RF.reset();
    Encoder_LB.reset();
    Encoder_RB.reset();
}

void PID_AngleVel(string MotorName, float TargetRPM, float Kp, float Ki, float Kd)
{
    if(MotorName == "LF")
    {
        Error_LF = TargetRPM - RPM_LF_LPF;
        P_LF = Error_LF * Kp;
        I_LF += (Error_LF * Ki) * Dt;
        I_LF = constrain<float>(I_LF, -I_max, I_max);
        D_LF = Kd * (Error_LF - Error_LF_last) / Dt;
        Error_LF_last = Error_LF;
        Input_RPM_LF = P_LF + I_LF + D_LF;
    }
    else if(MotorName == "RF")
    {
        Error_RF = TargetRPM - RPM_RF_LPF;
        P_RF = Error_RF * Kp;
        I_RF += (Error_RF * Ki) * Dt;
        I_RF = constrain<float>(I_RF, -I_max, I_max);
        D_RF = Kd * (Error_RF - Error_RF_last) / Dt;
        Error_RF_last = Error_RF;       
        Input_RPM_RF = P_RF + I_RF + D_RF;
    }
    else if(MotorName == "LB")
    {
        Error_LB = TargetRPM - RPM_LB_LPF;
        P_LB = Error_LB * Kp;
        I_LB += (Error_LB * Ki) * Dt;
        I_LB = constrain<float>(I_LB, -I_max, I_max);
        D_LB = Kd * (Error_LB - Error_LB_last) / Dt;
        Error_LB_last = Error_LB;       
        Input_RPM_LB = P_LB + I_LB + D_LB;
    }
    else if(MotorName == "RB")
    {
        Error_RB = TargetRPM - RPM_RB_LPF;
        P_RB = Error_RB * Kp;
        I_RB += (Error_RB * Ki) * Dt;
        I_RB = constrain<float>(I_RB, -I_max, I_max);
        D_RB = Kd * (Error_RB - Error_RB_last) / Dt;
        Error_RB_last = Error_RB;       
        Input_RPM_RB = P_RB + I_RB + D_RB;
    }
    
    Input_RPM_LF = constrain<float>(Input_RPM_LF, -100, 100);
    Input_RPM_RF = constrain<float>(Input_RPM_RF, -100, 100);
    Input_RPM_LB = constrain<float>(Input_RPM_LB, -100, 100);
    Input_RPM_RB = constrain<float>(Input_RPM_RB, -100, 100);
}

void PID()
{
    PID_AngleVel("LF", (Target_RPM_LF), 1.5, 7.0, 0);
    PID_AngleVel("RF", (Target_RPM_RF), 1.5, 7.0, 0);
    PID_AngleVel("LB", (Target_RPM_LB), 1.5, 7.0, 0);
    PID_AngleVel("RB", (Target_RPM_RB), 1.5, 7.0, 0);
}

void Get_MotorControl_Protocol()
{
    if(HC_complete)
    {   
        inputString_HC = Buf_HC;
        if(inputString_HC.find("F:") != -1)
        {
            char For_Compare_char[20] = "";
            char* For_Compare_char_After;
            string Parsed_string = inputString_HC.substr(inputString_HC.find("F:") + 2, inputString_HC.find(';') - inputString_HC.find("F:") - 2);
            
            memcpy(For_Compare_char, Parsed_string.c_str(), 20);

            float For_Substitute = strtod(For_Compare_char, &For_Compare_char_After);
            if(strcmp(For_Compare_char, For_Compare_char_After))
            {
                hc.printf("%d\n",(int)For_Substitute);
                MotorMode = "F";                
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
                hc.printf("%d\n",(int)For_Substitute);
                MotorMode = "B";
                Target_RPM_LF = -For_Substitute;
                Target_RPM_RF = -For_Substitute;
                Target_RPM_LB = -For_Substitute;
                Target_RPM_RB = -For_Substitute;
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
                hc.printf("%d\n",(int)For_Substitute);
                MotorMode = "S";                
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
                hc.printf("%d\n",(int)For_Substitute);
                MotorMode = "LF";
                Target_RPM_LF = 0;
                Target_RPM_RF = For_Substitute;
                Target_RPM_LB = For_Substitute;
                Target_RPM_RB = 0;
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
                hc.printf("%d\n",(int)For_Substitute);
                MotorMode = "RF";
                Target_RPM_LF = For_Substitute;
                Target_RPM_RF = 0;
                Target_RPM_LB = 0;
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
                hc.printf("%d\n",(int)For_Substitute);
                MotorMode = "L";
                Target_RPM_LF = -For_Substitute;
                Target_RPM_RF = For_Substitute;
                Target_RPM_LB = For_Substitute;
                Target_RPM_RB = -For_Substitute;
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
                hc.printf("%d\n",(int)For_Substitute);
                MotorMode = "R";
                Target_RPM_LF = For_Substitute;
                Target_RPM_RF = -For_Substitute;
                Target_RPM_LB = -For_Substitute;
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
                hc.printf("%d\n",(int)For_Substitute);
                MotorMode = "LB";
                Target_RPM_LF = -For_Substitute;
                Target_RPM_RF = 0;
                Target_RPM_LB = 0;
                Target_RPM_RB = -For_Substitute;
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
                hc.printf("%d\n",(int)For_Substitute);
                MotorMode = "RB";
                Target_RPM_LF = 0;
                Target_RPM_RF = -For_Substitute;
                Target_RPM_LB = -For_Substitute;
                Target_RPM_RB = 0;
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
                hc.printf("%d\n",(int)For_Substitute);
                MotorMode = "TL";
                Target_RPM_LF = -For_Substitute;
                Target_RPM_RF = For_Substitute;
                Target_RPM_LB = -For_Substitute;
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
                hc.printf("%d\n",(int)For_Substitute);
                MotorMode = "TR";
                Target_RPM_LF = For_Substitute;
                Target_RPM_RF = -For_Substitute;
                Target_RPM_LB = For_Substitute;
                Target_RPM_RB = -For_Substitute;
                Do_PID = true;
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

double vx = 0, vy = 0, yaw_dot = 0, yaw_dot_LPF = 0;
float prev_yaw = 0;

void Get_Robot_Status()
{
    float yaw = RPY_imu[2];

    // double pi_dot_1 = Dir_LF == 1 ? - RPM_LF_LPF * RPM_TO_RAD_P_SEC : RPM_LF_LPF * RPM_TO_RAD_P_SEC;
    // double pi_dot_2 = Dir_RF == 0 ? - RPM_RF_LPF * RPM_TO_RAD_P_SEC : RPM_RF_LPF * RPM_TO_RAD_P_SEC;
    // double pi_dot_3 = Dir_LB == 1 ? - RPM_LB_LPF * RPM_TO_RAD_P_SEC : RPM_LB_LPF * RPM_TO_RAD_P_SEC;
    // double pi_dot_4 = Dir_RB == 1 ? - RPM_RB_LPF * RPM_TO_RAD_P_SEC : RPM_RB_LPF * RPM_TO_RAD_P_SEC;

    double pi_dot_1 = RPM_LF_LPF * RPM_TO_RAD_P_SEC;
    double pi_dot_2 = RPM_RF_LPF * RPM_TO_RAD_P_SEC;
    double pi_dot_3 = RPM_LB_LPF * RPM_TO_RAD_P_SEC;
    double pi_dot_4 = RPM_RB_LPF * RPM_TO_RAD_P_SEC;

    vx = ((WHEEL_RADIUS + ROLLER_RADIUS) / 4) * (pi_dot_1 + pi_dot_2 + pi_dot_3 + pi_dot_4);
    vy = ((WHEEL_RADIUS + ROLLER_RADIUS) / 4) * ((-pi_dot_1) + pi_dot_2 + pi_dot_3 + (-pi_dot_4));

    // WHY ?????
    // vx *= 0.6;
    // vy *= 0.6;
    vx *= 0.75;
    vy *= 0.75;

    yaw_dot = (yaw - prev_yaw) / Dt;

    if(prev_yaw != 0) yaw_dot_LPF = yaw_dot_LPF * 0.95 + yaw_dot * 0.05;
    
    prev_yaw = yaw;
}

template <class A>
A constrain(A x, A min, A max)
{
    if(x >= max) x = max;
    else if(x < min) x = min;
    return x;
}

void init_serial_thread(Thread& th, int period_ms)
{    
    auto func_ = [period_ms]()
    {
        while (true) {
            auto cur_time = Kernel::Clock::now();

            printf("<VX:%.3f;VY:%.3f;DY:%.3f;Y:%.3f;>\n",
                     vx,vy,yaw_dot_LPF,RPY_imu[2]);

            //printf("<LF:%.3f;RF:%.3f;LB:%.3f;RB:%.3f;>\n",
                     //Error_LF,Error_RF,Error_LB,Error_RB);
                     //Target_RPM_LF,Target_RPM_RF,Target_RPM_LB,Target_RPM_RB);
                     //RPM_LF_LPF,RPM_RF_LPF,RPM_LB_LPF,RPM_RB_LPF);
                     //Input_RPM_LF,Input_RPM_RF,Input_RPM_LB,Input_RPM_RB);
            

            //printf("<LF:%d;RF:%d;LB:%d;RB:%d;>\n",
                    //Dir_LF.read(),Dir_RF.read(),Dir_LB.read(),Dir_RB.read());
                    //Input_RPM_LF,Input_RPM_RF,Input_RPM_LB,Input_RPM_RB);

            ThisThread::sleep_until(cur_time + chrono::milliseconds(static_cast<chrono::milliseconds::rep>(period_ms)));
        }
    };

    th.start(func_);
}