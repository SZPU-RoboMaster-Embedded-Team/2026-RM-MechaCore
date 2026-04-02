#include "InHpp.hpp"
#include "RMRefereeSystemCRC.hpp"

/*  =========================== 全局变量的初始化 ===========================  */ 
RMRefereeSystemData_t            RMRefereeSystemData            = { 0 };  // 裁判系统当前帧数据
unsigned char                    MyRefereeSys8Data              = 0;      // 裁判系统串口数据接收缓冲区

game_status_t                    game_status_0x0001             = { 0 };  // 0x0001 比赛状态数据
dart_info_t                      dart_info_0x0105               = { 0 };  // 0x0105 飞镖发射口倒计时
robot_status_t                   robot_status_0x0201            = { 0 };  // 0x0201 机器人性能体系数据
power_heat_data_t                power_heat_data_0x0202         = { 0 };  // 0x0202 实时底盘缓冲能量和射击热量数据
shoot_data_t                     shoot_data_0x0207              = { 0 };  // 0x0207 实时射击数据
dart_client_cmd_t                dart_client_cmd_0x020A         = { 0 };  // 0x020A 飞镖机器人客户端指令数据

/*  =========================== 函数的声明 ===========================  */ 
static void RMRefereeSystemParseData(uint8_t *MypDatas, uint16_t size);
static void RMRefereeSystemGetData(uint8_t MypData);
void RMRefereeSystemParse(void);

/************************************************************************
* @brief:       解析裁判系统数据(一帧数据)
*************************************************************************/
static void RMRefereeSystemParseData(uint8_t *MypDatas, uint16_t size)
{
    if (size < 9)
    {
        return;
    }

    RMRefereeSystemData.SOF         = MypDatas[0];
    RMRefereeSystemData.data_length = (uint16_t)(MypDatas[1] | (MypDatas[2] << 8));
    RMRefereeSystemData.seq         = MypDatas[3];
    RMRefereeSystemData.CRC8        = MypDatas[4];
    RMRefereeSystemData.cmd_id      = (uint16_t)(MypDatas[5] | (MypDatas[6] << 8));

    if (RMRefereeSystemData.data_length > sizeof(RMRefereeSystemData.data))
    {
        return;
    }

    for (uint16_t i = 0; i < RMRefereeSystemData.data_length; i++)
    {
        RMRefereeSystemData.data[i] = MypDatas[7 + i];
    }

    uint16_t crc_index = (uint16_t)(7 + RMRefereeSystemData.data_length);
    if (crc_index + 1 >= size)
    {
        return;
    }
    RMRefereeSystemData.frame_tail = (uint16_t)(MypDatas[crc_index] | (MypDatas[crc_index + 1] << 8));

    switch (RMRefereeSystemData.cmd_id)
    {
    case 0x0001:
        memcpy(&game_status_0x0001, (void *)RMRefereeSystemData.data, sizeof(game_status_0x0001));
        break;
    case 0x0105:
        memcpy(&dart_info_0x0105, (void *)RMRefereeSystemData.data, sizeof(dart_info_0x0105));
        break;
    case 0x0201:
        memcpy(&robot_status_0x0201, (void *)RMRefereeSystemData.data, sizeof(robot_status_0x0201));
        break;
    case 0x0202:
        memcpy(&power_heat_data_0x0202, (void *)RMRefereeSystemData.data, sizeof(power_heat_data_0x0202));
        break;
    case 0x0207:
        memcpy(&shoot_data_0x0207, (void *)RMRefereeSystemData.data, sizeof(shoot_data_0x0207));
        break;
    case 0x020A:
        memcpy(&dart_client_cmd_0x020A, (void *)RMRefereeSystemData.data, sizeof(dart_client_cmd_0x020A));
        break;
    default:
        break;
    }
}

/************************************************************************
* @brief:       裁判系统串口数据接收状态机（按字节喂入）
*************************************************************************/
static void RMRefereeSystemGetData(uint8_t MypData)
{
    /* 按照协议：frame_header(5) + cmd_id(2) + data(n) + frame_tail(2) */
    static uint8_t  rx_buf[128] = {0};
    static uint16_t rx_index    = 0;
    static uint16_t frame_len   = 0;

    if (rx_index == 0)
    {
        if (MypData == 0xA5)
        {
            rx_buf[rx_index++] = MypData;
        }
        return;
    }

    rx_buf[rx_index++] = MypData;

    if (rx_index == 5)
    {
        if (!Verify_CRC8_Check_Sum(rx_buf, 5))
        {
            rx_index  = 0;
            frame_len = 0;
            return;
        }

        uint16_t data_length = (uint16_t)(rx_buf[1] | (rx_buf[2] << 8));
        frame_len            = (uint16_t)(5 + 2 + data_length + 2);

        if (frame_len > sizeof(rx_buf))
        {
            rx_index  = 0;
            frame_len = 0;
            return;
        }
    }

    if (frame_len > 0 && rx_index >= frame_len)
    {
        if (Verify_CRC16_Check_Sum(rx_buf, frame_len))
        {
            RMRefereeSystemParseData(rx_buf, frame_len);
        }

        rx_index  = 0;
        frame_len = 0;
    }
}

/************************************************************************
* @brief:       裁判系统解析入口（在串口中断回调中调用）
*************************************************************************/
void RMRefereeSystemParse(void)
{
    RMRefereeSystemGetData(MyRefereeSys8Data);
    HAL_UART_Receive_IT(&HuartHandle_RMRefereeSystem, &MyRefereeSys8Data, sizeof(MyRefereeSys8Data));
}
