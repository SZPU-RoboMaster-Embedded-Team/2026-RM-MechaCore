#pragma once

#include "../APP/Heat_Detector/Heat_Detector.hpp"
#include "../Algorithm/FSM/alg_fsm.hpp"
#include "../Algorithm/LADRC/Adrc.hpp"
#include "../BSP/SimpleKey/SimpleKey.hpp"
#include "../User/Algorithm/PID.hpp"

#include "../User/BSP/DWT/DWT.hpp"

namespace TASK::Shoot
{
using namespace Alg::LADRC;

enum Booster_Status
{
    // 失能
    DISABLE,
    // 停止
    STOP,
    // 单发模式
    ONLY,
    // 连发模式
    AUTO
};

/**
 * @brief 防卡弹状态类型
 *
 */
enum Jamming_Status
{
    NORMAL = 0, // 正常模式
    SUSPECT,    // 可疑堵转
    PROCESSING, // 处理堵转
};

class Class_ShootFSM;

class Class_JammingFSM : public Class_FSM
{
  private:
    // 堵转力矩
    float stall_torque = 0.2f;
    // 堵转时间阈值，超过则为堵转
    static constexpr uint32_t stall_time = 200;
    // 从堵转停止时间阈值，超过则停止堵转
    static constexpr uint32_t stall_stop = 300;

    Class_ShootFSM *Booster = nullptr; // 任务指针

  public:
    void UpState(void);

    // 添加设置Booster指针的方法
    void setBooster(Class_ShootFSM *booster)
    {
        Booster = booster;
    }
};

/**
 * @brief 使用中科大的有限状态机实现发射机构相关逻辑
 *
 */
class Class_ShootFSM : public Class_FSM
{

  public:
    // 显式声明构造函数
    Class_ShootFSM();

    // ADRC控制器
    void Control(void);

    void UpState(void);

    void setTargetDailTorque(float torque)
    {
        target_Dail_torque = torque;
    }

    void setNowStatus(Booster_Status state)
    {
        Set_Status(state);
    }

    // 设置开火标志位
    void setFireFlag(bool flag)
    {
        fire_flag = flag;
    }

  protected:
    // 初始化相关常量

    // 热量控制

    // 检测摩擦轮力矩变化

    // 拨盘控制

    // CAN发送
    void CAN_Send(void);
    void HeatLimit();

    // 将期望发射频率转化为rpm(转轴)
    float rpm_to_hz(float tar_hz);

    //  将期望频率转化为角度
    float hz_to_angle(float fire_hz);
    void Jamming(float angle, float err);

  private:
    float target_Dail_torque = 0;
    float Dail_target_pos;
    float target_friction_L_torque = 0;
    float target_friction_R_torque = 0;

    float target_friction_omega = 5900;
    float target_fire_hz;
    float Max_dail_angle = 20.0f; // 拨盘最快频率
    Class_JammingFSM JammingFMS;

    // 开火标志位
    uint8_t fire_flag = 0;

    APP::Heat_Detector::Class_FSM_Heat_Limit Heat_Limit;
    // 发射机构控制模式
    Adrc adrc_friction_L_vel;
    Adrc adrc_friction_R_vel;
    Adrc adrc_Dail_vel;

    Kpid_t Kpid_Dail_pos;
    Kpid_t Kpid_Dail_vel;

    PID pid_Dail_pos;
    PID pid_Dail_vel;

    // 用于单发检测，获取上升沿判断是否击发子弹
    // BSP::Key::SimpleKey key_fire;
};
inline Class_ShootFSM shoot_fsm;
} // namespace TASK::Shoot

// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif

    void ShootTask(void *argument);

#ifdef __cplusplus
}
#endif