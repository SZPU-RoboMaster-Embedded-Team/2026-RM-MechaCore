#ifndef __VisionCommunication_Hpp
#define __VisionCommunication_Hpp

#define VisionSendDelay         27 // 发送频率,单位为赫兹(Hz)

#define VisionUartHandle        huart8
#define VisionUartInstance      UART8
#define VisionUartReceiveLength 11

enum VisionMode
{
    Single = 0,
    First  = 1,
    Second = 2,
    Third  = 3,
    Fourth = 4
};

// 发送类
typedef class VisionUartSendClass
{
public:
    unsigned char Head;
    // int Yaw_Temp; // 视觉发送临时变量
    int16_t YawAngle;              // 镖架当前yaw角度
    int16_t PullingForce;          // 拉簧拉力
    uint8_t AimingFinishCount;      // 瞄准完成计数
    uint8_t DartIndex;             // 当前飞镖序号
    uint8_t Reserved;              // 保留位
} VisionUartSend_t;

// 接收类
typedef class VisionUartReceiveClass
{
public:
    unsigned char ReceiveArr[VisionUartReceiveLength];  // DMA 原始接收缓冲区（不要修改）
    unsigned char ParsedArr[VisionUartReceiveLength];   // 帧修正后的解析数组
    int16_t YawAngle;              // yaw视觉角
    // int16_t YawAngleComp;          // yaw视觉角（补偿后）
    int16_t YawOffset;             // yaw补偿值
    int16_t MotorValue;            // 电机值
    uint8_t DartID;                // 飞镖序号
    uint8_t DetectFlag;            // 检测标志
    uint8_t Reserved;              // 保留位

    int16_t Target_Yaw;            // 目标yaw角度（未补偿）
    // int16_t TargetYawComp;         // 目标yaw角度（补偿后）
    bool isVisionMisaligned;       // 表示视觉接收的数据帧错位
    
    uint32_t LastRxTick;           // 最后接收到有效数据的时间戳
    bool isConnected;              // 连接状态标志
    
    void GetData();                // 获取数据函数
    void FixFrameError();          // 修复数据帧错位函数
    void CheckConnection();        // 检查连接状态并触发蜂鸣器
} VisionUartReceive_t;

extern VisionUartSend_t VisionUartSend;
extern VisionUartReceive_t VisionUartReceive;

#endif
