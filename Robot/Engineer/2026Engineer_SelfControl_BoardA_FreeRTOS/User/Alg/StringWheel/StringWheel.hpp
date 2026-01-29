// #pragma once

// #include <math.h>
// #include <cstdint>
// #include <cstring> // 添加头文件
// #include "../APP/Tools.hpp"
// #include "../HAL/HAL.hpp"
// #include "../BSP/Motor/Dji/DjiMotor.hpp"
// #include "../BSP/Motor/Lk/Lk_motor.hpp"
// #include "../../Alg/ChassisCalculation/CalculationBase.hpp"
// #define M_PI 3.14159265358979323846

// const float ANGLE_45_RAD = M_PI / 4;    // 45度弧度值
// const float COS_45 = cosf(ANGLE_45_RAD); // 45度余弦值
// const float SIN_45 = sinf(ANGLE_45_RAD); // 45度正弦值

// /**
//  * @brief 底盘控制基类，定义通用底盘接口和参数
//  */
// namespace Alg::Chassis
// {
//     class ChassisBase
//     {
//     public:
//         struct Parameters
//         {
//             float Vx = 0.0f;
//             float Vy = 0.0f;
//             float Vw = 0.0f;
//             float Speed[4] = {0};
//             float MaxSpeed = 0.0f;
//             float Angle[4] = {0};
//             float a = 0.0f; //底盘中心 X 轴距离的一半
//             float b = 0.0f; //底盘中心 Y 轴距离的一半
    
//             //动力学计算中间变量
//             float tmp_force[4] = {0};
//             float Target_Wheel_Current[4] = {0};
//             float Target_Wheel_Omega[4] = {0};
//         };
//         ChassisBase() = default;
//         virtual ~ChassisBase() = default;
//         virtual void UpDate(float vx, float vy, float vw, float maxSpeed) = 0;
        
//         const Parameters& GetParams() const { return params_; } //当对象是const或通过const引用访问时，仍可以获取参数
//         Parameters& GetParams() { return params_; }

//         /**
//          * @brief 设置底盘几何尺寸
//          * @param a 底盘中心X轴距离的一半
//          * @param b 底盘中心Y轴距离的一半
//          */
//         void SetChassisDimensions(float a, float b) // 添加设置底盘尺寸的公共接口
//         { 
//             params_.a = a;
//             params_.b = b;
//             InitializeWheelPositions();
//         }
        
//         inline void SetMaxSpeed(float maxSpeed) 
//         {
//             params_.MaxSpeed = maxSpeed;
//         }
//         inline float GetX() const      // 获取x方向线速度
//         { 
//             return params_.Vx; 
//         }
//         inline float GetY() const       // 获取y方向线速度
//         {
//             return params_.Vy; 
//         }
//         inline float GetZ() const       // 获取绕z轴角速度
//         {
//             return params_.Vw; 
//         }
//         inline void SetX(float vx)      // 设置x方向线速度
//         { 
//             params_.Vx = vx; 
//         }
//         inline void SetY(float vy)      // 设置y方向线速度
//         { 
//             params_.Vy = vy; 
//         }
//         inline void SetZ(float vw)      // 设置绕z轴角速度
//         { 
//             params_.Vw = vw; 
//         }
//     protected:
//         Parameters params_;
//          float wheelPos[4][2] = {{0}};
//         /**
//          * @brief 初始化轮子位置坐标
//          */
//         void InitializeWheelPositions()
//         {
//             wheelPos[0][0] = -params_.a; wheelPos[0][1] = params_.b;   // 左前
//             wheelPos[1][0] = params_.a;  wheelPos[1][1] = params_.b;   // 右前
//             wheelPos[2][0] = params_.a;  wheelPos[2][1] = -params_.b;  // 右后
//             wheelPos[3][0] = -params_.a; wheelPos[3][1] = -params_.b;  // 左后
//         }

//         /**
//          * @brief 限制速度在安全范围内
//          * @param speed 待限制的速度值
//          * @param maxSpeed 最大速度限制
//          */
//         void LimitSpeed(float& speed, float maxSpeed) 
//         {
//             speed = Tools_t::clamp(speed, -maxSpeed, maxSpeed);
//         }

//         /**
//          * @brief 角度归一化到指定范围内
//          * @param angle 待归一化的角度
//          * @param tar_angle 目标范围
//          * @return 归一化后的角度
//          */

//         float NormalizeAngle(float angle, float tar_angle)
//         {
//             while (angle > tar_angle) 
//                 angle -= tar_angle;
//             while (angle < -tar_angle) 
//                 angle += tar_angle;  
//             return angle;
//         }
//             // 轮组方位角
//         const float Wheel_Azimuth[4] = {M_PI / 4.0f,
//                                     3.0f * M_PI / 4.0f,
//                                     5.0f * M_PI / 4.0f,
//                                     7.0f * M_PI / 4.0f,};

//     };
// }
// /**
//  * @brief 舵轮控制类，实现全向移动底盘控制
//  * 
//  * 继承自：
//  * - ChassisBase: 底盘基础功能
//  * - InverseKinematicsBase: 逆运动学接口
//  * - ForwardKinematicsBase: 正运动学接口  
//  * - InverseDynamicsBase: 逆动力学接口
//  */
// class SteeringWheel : public Alg::Chassis::ChassisBase
//                       public Alg::CalculationBase::InverseKinematicsBase,
//                       public Alg::CalculationBase::ForwardKinematicsBase,
//                       public Alg::CalculationBase::InverseDynamicsBase
// {
//     private:
//         // 舵向电机（LK电机）和轮向电机（DJI电机）的引用
//         BSP::Motor::LK::LK4005<4>& steer_motors;
//         BSP::Motor::Dji::GM3508<4>& wheel_motors;
//         // 底盘物理参数
//         const float Wheel_Radius = 0.05f;                  // 轮半径（米）
//         const float Wheel_To_Core_Distance[4] = {0.3f, 0.3f, 0.3f, 0.3f}; // 轮到中心距离
//         const float Wheel_Azimuth[4] = {0, M_PI/2, M_PI, 3*M_PI/2}; // 轮安装方位角（弧度）
//         float current_steer_angles[4] = {0};
//         // 控制参数
//         const float VW_THRESHOLD = 2.0f;
//         // 防单轮超速系数
//         const float WHEEL_SPEED_LIMIT_FACTOR = 0.5f;
//     public:
//         /**
//          * @brief 构造函数
//          * @param steer 转向电机实例
//          * @param wheel 驱动电机实例
//          */
//         SteeringWheel(BSP::Motor::LK::LK4005<4>& steer, BSP::Motor::Dji::GM3508<4>& wheel)
//             : steer_motors(steer), wheel_motors(wheel) 
//             {
//                 // 设置底盘尺寸（根据Wheel_To_Core_Distance计算）
//                 float a = Wheel_To_Core_Distance[0] * cosf(M_PI/4.0f);  // X方向半长
//                 float b = Wheel_To_Core_Distance[0] * sinf(M_PI/4.0f);  // Y方向半宽
//                 SetChassisDimensions(a, b);
//             }
//         void UpDate(float vx, float vy, float vw, float maxSpeed)
//         {
//             this->params_.Vx = vx;
//             this->params_.Vy = vy;
//             this->params_.Vw = vw;
//             for(int i = 0; i < 4; i++) 
//             {
//                 current_steer_angles[i] = steer_motors.getAngleRad(i+1);
//             }
//             Kinematics_Inverse_Resolution(vx, vy, vw, maxSpeed);
//         }
//         /**
//          * @brief 转向电机就近转位处理
//          * 
//          * 确保转向电机以最短路径转动到目标角度，避免不必要的整圈转动
//          */
//         void _Steer_Motor_Kinematics_Nearest_Transposition()
//         {
//             for (int i = 0; i < 4; i++)
//             {
//                 float tmp_delta_angle = NormalizeAngle(params_.Angle[i] - current_steer_angles[i], 2.0f * M_PI);

//                 // 根据转动角度范围决定是否需要就近转位
//                 if (-M_PI / 2.0f <= tmp_delta_angle && tmp_delta_angle <= M_PI / 2.0f)
//                 {
//                     // ±PI / 2之间无需反向就近转位
//                     params_.Angle[i] = tmp_delta_angle + current_steer_angles[i];
//                 }
//                 else
//                 {
//                     // 需要反转扣圈情况
//                     params_.Angle[i] = NormalizeAngle(tmp_delta_angle + M_PI, 2.0f * M_PI) + current_steer_angles[i];
//                     params_.Target_Wheel_Omega[i] *= -1.0f;
//                     params_.Speed[i] *= -1.0f;
//                 }
//             }
//         }
//         /**
//          * @brief 逆运动学解算
//          * @param vx X方向线速度
//          * @param vy Y方向线速度
//          * @param vw 绕Z轴角速度  
//          * @param maxSpeed 最大速度限制
//          */
//         void Kinematics_Inverse_Resolution(float vx, float vy, float vw, float maxSpeed)
//         {  
//             // 同时更新基类中的信号值
//             SetSignal_xyw(vx, vy, vw);

//             for (int i = 0; i < 4; i++)
//             {
//                 float tmp_velocity_x, tmp_velocity_y, tmp_velocity_modulus;

//                 // 使用Class_Chassis的算法：解算到每个轮组的具体线速度
//                 tmp_velocity_x = vx - vw * Wheel_To_Core_Distance[i] * sinf(Wheel_Azimuth[i]);
//                 tmp_velocity_y = vy + vw * Wheel_To_Core_Distance[i] * cosf(Wheel_Azimuth[i]);
                
//                 tmp_velocity_modulus = sqrtf(tmp_velocity_x * tmp_velocity_x + tmp_velocity_y * tmp_velocity_y);

//                 // 根据线速度决定轮向电机角速度（转换为RPM）
//                 params_.Target_Wheel_Omega[i] = tmp_velocity_modulus / Wheel_Radius;
//                 params_.Speed[i] = params_.Target_Wheel_Omega[i] * 60.0f / (2.0f * M_PI); // rad/s转RPM
//                 LimitSpeed(params_.Speed[i], maxSpeed);

//                 // 根据速度的xy分量分别决定舵向电机角度
//                 if (tmp_velocity_modulus == 0.0f)
//                 {
//                     // 排除除零问题，保持当前角度
//                     params_.Angle[i] = current_steer_angles[i];
//                 }
//                 else
//                 {
//                     // 没有除零问题
//                     params_.Angle[i] = atan2f(tmp_velocity_y, tmp_velocity_x);
//                 }
//             }
//             // 执行就近转位
//             _Steer_Motor_Kinematics_Nearest_Transposition();
//         }
//         /**
//          * @brief 正运动学解算
//          * 
//          * 根据轮子实际速度计算底盘整体运动状态
//          */
//         void Kinematics_Forward_Resolution()
//         {
//             float sumVx = 0.0f, sumVy = 0.0f, sumVw = 0.0f;
//             int validWheels = 0;

//             for (int i = 0; i < 4; i++)
//             {
//                 float wheel_omega = wheel_motors.getVelocityRads(i+1);
//                 float steer_angle = steer_motors.getMultiAngle(i+1) * M_PI / 180.0f;

//                 // 计算轮子在底盘坐标系中的速度分量
//                 float wheel_vx = wheel_omega * Wheel_Radius * cosf(steer_angle);
//                 float wheel_vy = wheel_omega * Wheel_Radius * sinf(steer_angle);

//                 sumVx += wheel_vx;
//                 sumVy += wheel_vy;
//                 validWheels++;

//                 // 计算角速度贡献 (基于轮子安装位置)
//                 float delta_angle = steer_angle - Wheel_Azimuth[i];
//                 sumVw += (wheel_omega * Wheel_Radius * sinf(delta_angle)) / Wheel_To_Core_Distance[i];
                
//                 // 更新基类中的角速度数组
//                 this->w[i] = wheel_omega;
//             }

//             // 计算平均值得到底盘整体运动状态
//             if (validWheels > 0) {
//                 this->params_.Vx = sumVx / validWheels;
//                 this->params_.Vy = sumVy / validWheels;
//                 this->params_.Vw = sumVw / validWheels;
//             } else {
//                 this->params_.Vx = 0;
//                 this->params_.Vy = 0;
//                 this->params_.Vw = 0;
//             }
//         }
//         /**
//          * @brief 逆动力学计算
//          * 
//          * 根据目标力/力矩计算电机电流指令
//          */
//         void DynamicsInverseCalculation()
//         {
//             float force_x, force_y, torque_omega;   

//             force_x = 0.0f;//来自PID X的输出，后续添加;
//             force_y = 0.0f;//来自PID Y的输出，后续添加;
//             torque_omega = 0.0f;//来自PID Omega的输出，后续添加;

//             // 每个轮的扭力
//             float tmp_force[4];
//             for (int i = 0; i < 4; i++)
//             {
//                 // 解算到每个轮组的具体摩擦力
//                 tmp_force[i] = force_x * cosf(steer_motors.getAngleDeg(i)) + force_y * sinf(steer_motors.getAngleDeg(i)) - 
//                             torque_omega / Wheel_To_Core_Distance[i] * sinf(Wheel_Azimuth[i] - steer_motors.getAngleDeg(i));
                
//                 // 更新力和扭矩
//                 Set_FxFyTor(force_x, force_y, torque_omega);
//             }
//             for (int i = 0; i < 4; i++)
//             {
//                 // 摩擦力转换至扭矩
//                 params_.Target_Wheel_Current[i] = tmp_force[i] * Wheel_Radius + WHEEL_SPEED_LIMIT_FACTOR *
//                                         (params_.Target_Wheel_Omega[i] - wheel_motors.getVelocityRads(i));
//             }
//         }

// };

