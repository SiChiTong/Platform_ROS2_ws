#include "mbed.h"
#include "RobotControl.hpp"
 
Thread thread_SendTo_serial;

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

    int cnt_send = 0;

    Stop();
    Encoder_Reset();

    init_serial_thread(thread_SendTo_serial, 20);

    // HC06.attach(&Check_HC_RX, SerialBase::RxIrq);
    PC.attach(&Check_PC_RX, SerialBase::RxIrq);
    IMU.attach(&Check_IMU_RX, SerialBase::RxIrq);

    while (true) 
    {        
        current_tick = Kernel::get_ms_count();

        Get_IMU_Protocol();
        Get_MotorControl_Protocol();
        Check_Encoder();

        PID();

        if(MotorMode != "S")
        {
            Go_CMD(Input_RPM_LF, Input_RPM_RF, Input_RPM_LB, Input_RPM_RB);
        }
        else {
            Stop();
        }

        Get_Robot_Status();

        auto cur_time = Kernel::Clock::now();
        ThisThread::sleep_until(cur_time + chrono::milliseconds(static_cast<chrono::milliseconds::rep>(Dt*1000)));
    }
}
