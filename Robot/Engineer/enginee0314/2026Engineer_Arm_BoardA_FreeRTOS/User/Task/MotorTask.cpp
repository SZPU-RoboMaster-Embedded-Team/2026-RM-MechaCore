#include "MotorTask.hpp"

uint8_t send_str2[sizeof(float) * 8];
bool init_flag = false;

void vofa_init()
{
    auto &uart7 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart7);
    HAL::UART::Data send_data{send_str2, sizeof(float) * 8};
}

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6)
{
    const uint8_t sendSize = sizeof(float);

    *((float*)&send_str2[sendSize * 0]) = x1;
    *((float*)&send_str2[sendSize * 1]) = x2;
    *((float*)&send_str2[sendSize * 2]) = x3;
    *((float*)&send_str2[sendSize * 3]) = x4;
    *((float*)&send_str2[sendSize * 4]) = x5;
    *((float*)&send_str2[sendSize * 5]) = x6;

    *((uint32_t*)&send_str2[sizeof(float) * 6]) = 0x7F800000;

    auto &uart7 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart7);
    HAL::UART::Data send_data{send_str2, sizeof(float) * 8};
    uart7.transmit_dma(send_data);
}

void Motor_Init()
{
    static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);

    can1.register_rx_callback([](const HAL::CAN::Frame &frame)
    {
        Motor4340PJ4.Parse(frame);
        Motor8009P.Parse(frame);
    });

    can2.register_rx_callback([](const HAL::CAN::Frame &frame)
    {
        Motor4340PJ5.Parse(frame);
        Motor4310P.Parse(frame);
        Motor4310.Parse(frame);
    });
}

template<typename MotorType>
static void handle_dm_disable_sequence(MotorType& motor, uint8_t id, uint8_t& step, CAN_HandleTypeDef *hcan)
{
    if (!motor.getIsenable(id))
    {
        step = 0;
        motor.ctrl_Mit(hcan, id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    switch (step)
    {
        case 0:
            motor.Off(hcan, id, BSP::Motor::DM::MIT);
            step = 1;
            break;

        case 1:
            motor.ClearErr(hcan, id, BSP::Motor::DM::MIT);
            step = 2;
            break;

        case 2:
            if (motor.getError(id) == 0)
            {
                motor.setIsenable(id, false);
            }
            step = 0;
            break;

        default:
            step = 0;
            break;
    }
}

template<typename MotorType>
static void handle_dm_enable_sequence(MotorType& motor, uint8_t id, uint8_t& step, CAN_HandleTypeDef *hcan)
{
    if (!motor.getIsenable(id))
    {
        motor.On(hcan, id, BSP::Motor::DM::MIT);
        if(motor.getError(id) == 1)
        {
            motor.setIsenable(id, true);
        }
    }
    else
    {
        motor.ctrl_Mit(hcan, id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
}

void Motor_Control_loop()
{
    static uint8_t enable_seq = 0;
    static uint8_t disable_seq = 0;
    static uint8_t off_step[8] = {0};
    static bool last_control_mode = false;

    // const bool force_disable = TASK::ARM::offline_disable_request;
    // const uint8_t current_s1 = DT7.get_s1();
    // const uint8_t current_s2 = DT7.get_s2();
    // const bool enable_mode = (current_s1 == 1 && current_s2 == 1);
    // const bool control_mode = (current_s1 == 1 && (current_s2 == 2 || current_s2 == 3));

    // if(!force_disable && (enable_mode || control_mode))
    // {
    //     if (enable_mode)
    //     {
    const bool force_disable = TASK::ARM::offline_disable_request;
    const bool remote_connected = DT7.isConnected();
    const uint8_t current_s1 = DT7.get_s1();
    const uint8_t current_s2 = DT7.get_s2();

    const bool enable_mode = (current_s1 == 1 && current_s2 == 1);
    const bool control_mode = (current_s1 == 1 && (current_s2 == 2 || current_s2 == 3));

    const bool allow_enable_mode = remote_connected && enable_mode;
    const bool allow_control_mode = !force_disable && control_mode;

    if (allow_enable_mode || allow_control_mode)
    {
        if (enable_mode)
        {
            enable_seq++;

            if(enable_seq % 4 == 1)
            {
                handle_dm_enable_sequence(Motor8009P, 1, off_step[0], &hcan1);
                handle_dm_enable_sequence(Motor4340PJ5, 1, off_step[4], &hcan2);
            }
            else if(enable_seq % 4 == 2)
            {
                handle_dm_enable_sequence(Motor8009P, 2, off_step[1], &hcan1);
                handle_dm_enable_sequence(Motor4310P, 1, off_step[5], &hcan2);
            }
            else if(enable_seq % 4 == 3)
            {
                handle_dm_enable_sequence(Motor8009P, 3, off_step[2], &hcan1);
                handle_dm_enable_sequence(Motor4310P, 2, off_step[6], &hcan2);
            }
            else
            {
                handle_dm_enable_sequence(Motor4340PJ4, 1, off_step[3], &hcan1);
                handle_dm_enable_sequence(Motor4310, 1, off_step[7], &hcan2);
            }

            last_control_mode = false;
            // vofa_send(0.0f, Motor8009P.getAngleDeg(2), 0.0f, Motor8009P.getAngleDeg(3), 0.0f, 0.0f);
            return;
        }

        if (!last_control_mode)
        {
            TASK::ARM::arm.resetSoftStart();
        }

        last_control_mode = true;
        TASK::ARM::arm.JointControl();
        // vofa_send(0.0f, Motor8009P.getAngleDeg(2), 0.0f, Motor8009P.getAngleDeg(3), 0.0f, 0.0f);
        return;
    }

    last_control_mode = false;
    disable_seq++;

    if(disable_seq % 4 == 1)
    {
        handle_dm_disable_sequence(Motor8009P, 1, off_step[0], &hcan1);
        handle_dm_disable_sequence(Motor4340PJ5, 1, off_step[4], &hcan2);
    }
    else if(disable_seq % 4 == 2)
    {
        handle_dm_disable_sequence(Motor8009P, 2, off_step[1], &hcan1);
        handle_dm_disable_sequence(Motor4310P, 1, off_step[5], &hcan2);
    }
    else if(disable_seq % 4 == 3)
    {
        handle_dm_disable_sequence(Motor8009P, 3, off_step[2], &hcan1);
        handle_dm_disable_sequence(Motor4310P, 2, off_step[6], &hcan2);
    }
    else
    {
        handle_dm_disable_sequence(Motor4340PJ4, 1, off_step[3], &hcan1);
        handle_dm_disable_sequence(Motor4310, 1, off_step[7], &hcan2);
    }

    // vofa_send(0.0f, Motor8009P.getAngleDeg(2), 0.0f, Motor8009P.getAngleDeg(3), 0.0f, 0.0f);
}

extern "C" void MotorTask(void *argument)
{
    TickType_t Lasttick = xTaskGetTickCount();

    vofa_init();
    Motor_Init();
    for(;;)
    {
        Motor_Control_loop();
        //vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));
        osDelay(1);
    }
}
