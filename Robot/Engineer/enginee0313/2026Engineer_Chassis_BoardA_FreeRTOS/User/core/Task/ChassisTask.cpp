#include "ChassisTask.hpp"

uint8_t send_str2[sizeof(float) * 8]; // 分配8个float空间（32字节）

ALG::PowerControl::PowerControl<4> power3508;
Alg::PowerControlTestVersion::PowerControlTestVersion pcl3508;
Alg::CalculationBase::Macanum_IK macanum_ik(0.274, 0.075);
ALG::PID::PID wheels_pid[4] = {
    ALG::PID::PID(15.0f, 0.07f, 0.0f, 16385.0f, 2500.0f, 200.0f),
    ALG::PID::PID(15.0f, 0.07f, 0.0f, 16385.0f, 2500.0f, 200.0f),
    ALG::PID::PID(15.0f, 0.07f, 0.0f, 16385.0f, 2500.0f, 200.0f),
    ALG::PID::PID(15.0f, 0.07f, 0.0f, 16385.0f, 2500.0f, 200.0f)
};

ALG::PID::PID pid_test(40.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f);

// float coefficients3508[6] = { 1.151590, 0.002333, 0.000237,
//                               0.013946,  0.217840, 0.000041  }; //3508多项式系数(自己拟合)

// float coefficients3508[6] = { 2.144951, -0.002828, 0.000025,
//                               0.016525,  0.115369, 0.000015  }; //3508多项式系数

// float coefficients3508[6] = { 1.553948, -0.016163, 0.000113,
//                                  0.008966, 0.237189, 0.000024      }; //3508多项式系数（混合滤波拟合）

// float coefficients3508[6] = { 1.911668, 0.010517, 0.000014,
//                               0.012374, 0.205045, 0.000023      }; //3508多项式系数（pid正确拟合）

// float coefficients3508[6] = { 1.600001, -0.013492, 0.000068,
//                               0.006555, 0.240404, 0.000023      }; //3508多项式系数（sin步长20，梯度步长10，2s）

// float coefficients3508[6] = { 1.156272, 0.002326, 0.000237,
//                               0.013947, 0.217382, 0.000041  }; //3508多项式系数（3次拟合）

// float coefficients3508[6] = { 1.999532, -0.015631, -0.000060,
//                               0.017183, 0.118311, 0.000017  };//3508多项式系数（R2=0.94）

float coefficients3508[6] = { 1.63099240, 0.15842237,  0.00863915,
                              0.02763158, 0.16835670, 0.00000218  };//3508多项式系数（R2=0.958）

float Power_Max = 115.0f; //设定裁判系统限制功率120W
float Power_predict = 0.0f;
float I3508[4],V3508[4],I_other[4];

Output_chassis chassis_output;

void vofa_init()
{
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    HAL::UART::Data send_data{send_str2, sizeof(float) * 8};
}

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6, float x7, float x8, float x9, float x10,float x11)
{
    const uint8_t sendSize = sizeof(float); // 单浮点数占4字节

    // 将6个浮点数据写入缓冲区（小端模式）
    *((float*)&send_str2[sendSize * 0]) = x1;
    *((float*)&send_str2[sendSize * 1]) = x2;
    *((float*)&send_str2[sendSize * 2]) = x3;
    *((float*)&send_str2[sendSize * 3]) = x4;
    *((float*)&send_str2[sendSize * 4]) = x5;
    *((float*)&send_str2[sendSize * 5]) = x6;
    *((float*)&send_str2[sendSize * 6]) = x7;
    *((float*)&send_str2[sendSize * 7]) = x8;
    *((float*)&send_str2[sendSize * 8]) = x9;
    *((float*)&send_str2[sendSize * 9]) = x10;
    *((float*)&send_str2[sendSize * 10]) = x11;

    // 写入帧尾（协议要求 0x00 0x00 0x80 0x7F）
    *((uint32_t*)&send_str2[sizeof(float) * 11]) = 0x7F800000; // 小端存储为 00 00 80 7F

    // 通过DMA发送完整帧（6数据 + 1帧尾 = 7个float，共28字节）
    auto &uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    HAL::UART::Data send_data{send_str2, sizeof(float) * 13};
    uart8.transmit_dma(send_data);
}

void Chassis_Control()
{
    macanum_ik.MacanumInvKinematics(DT7.get_left_y(), DT7.get_left_x(), -DT7.get_right_x(), 0.0f, 49.79f);
    macanum_ik.SetRotationalGain(-129.0f);

    for(int i = 0; i < 4; i++)
    {
        wheels_pid[i].UpDate(macanum_ik.GetMotor(i), Motor3508.getVelocityRpm(i+1));
        chassis_output.output_wheel[i] = wheels_pid[i].getOutput();
    }

    // float target_v = pcl3508.SinExpected(0.001, 20.0, 469.0, 4.0);
    // float target_v = pcl3508.SteadyStateExpectation(2000.0, 10.0, 469.0);


    // pid_test.UpDate(target_v,Motor3508.getVelocityRpm(2));
    // pid_test.UpDate(469.0, Motor3508.getVelocityRpm(2));
    // vofa_send(target_v, Motor3508.getVelocityRpm(2), 0.0, 0.0, 0.0, 0.0);
    for(int i = 0;i < 4;i++)
    {
        I3508[i] = wheels_pid[i].getOutput() * 20.0f / 16384.0f;
        // I3508[i] = Motor3508.getCurrent(i + 1) ;  
        V3508[i] = Motor3508.getVelocityRads(i + 1) * 14.0;
        I_other[i] = 0.0f;
    }

    float Pmax3508 = Power_Max;

    power3508.AttenuatedPower(I3508, V3508, coefficients3508, 0.0, Pmax3508); //功率衰减法
    // power3508.DecayingCurrent(I3508,V3508,coefficients3508,I_other,0.0,Pmax3508); //电流衰减法

    for(int i = 0;i < 4; i++)
    {
        chassis_output.output_wheel[i] = power3508.getCurrentCalculate(i) * 16384.0f / 20.0f;
    }
}

extern "C" void ChassisTask(void *argument)
{
    //初始化蜂鸣器管理器
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    // vofa_init();
    for(;;)
    {
        //更新蜂鸣器管理器状态
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        Chassis_Control();
        osDelay(1);
    }
}