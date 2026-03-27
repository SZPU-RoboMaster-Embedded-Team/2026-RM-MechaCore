#ifndef __MotorControl_Hpp
#define __MotorControl_Hpp

#include "RmMotor.hpp"
#include "PID.hpp"
#include "Filter.hpp"

/**
 * @brief 电机对象类 (统一化版本)
 * @details 将电机的反馈数据、PID算子、TD滤波器和输出缓冲区封装在一起
 *          直接持有成员（非指针），实现真正的"一个对象管一个电机"
 */
class MotorObject {
public:
    RmMotorMeasure_t  Feedback;   // 电机反馈数据
    PID_Speed_Temp    SpeedPID;   // 电机速度环PID
    TD_t              Filter;     // TD跟踪微分滤波器
    int               Output;     // 输出缓冲区（发送给CAN）

    /**
     * @brief 更新电机控制
     * @details 读取反馈 RPM -> 计算 PID -> 写入输出缓冲区
     */
    void Update() {
        SpeedPID.Current = Feedback.RPM;
        SpeedPID.Compute();
        Output = SpeedPID.Final_Output;
    }
};

// 电机索引枚举：与 CAN ID 一一对应 (INDEX = CAN_ID - 0x201)
// 改电机配置只需改这里和 MotorsInit()
enum MotorIndex_e {
    INDEX_RESERVED = 0,    // 0x201: 保留位（暂未使用）
    INDEX_LEFT_LIFT,       // 0x202: 左边上下升降 M2006
    INDEX_RIGHT_LOAD,      // 0x203: 右边上膛 M3508
    INDEX_LEFT_LOAD,       // 0x204: 左边上膛 M3508
    INDEX_YAW,             // 0x205: Yaw轴 M6020
    INDEX_RIGHT_LIFT,      // 0x206: 右边上下升降 M2006
    INDEX_SPRING,          // 0x207: 拉簧调节 M3508
    TOTAL_CONTROL_MOTORS   // 控制电机总数 (=7)
    // 0x208: 角度传感器 M3508 — 独立于 Motors[] 数组
};

// 统一对象数组
extern MotorObject Motors[TOTAL_CONTROL_MOTORS];

#endif // __MotorControl_Hpp
