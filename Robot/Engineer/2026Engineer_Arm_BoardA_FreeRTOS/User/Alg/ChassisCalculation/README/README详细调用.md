# ChassisCalculation 底盘运动解算模块

## 简介
本模块提供了机器人底盘运动学和动力学的通用解算框架，目前支持 **全向轮 (Omni)** 和 **舵轮 (String/Steering Wheel)** 两种底盘构型。

## 模块结构

本模块由三个核心文件组成：

### 1. 基础类 (`CalculationBase.hpp`)
定义了所有底盘解算的通用接口，包含三个基类：
- **`ForwardKinematicsBase`**: 正向运动学基类（已知轮子速度 -> 求底盘速度）
- **`InverseKinematicsBase`**: 逆向运动学基类（已知底盘目标速度 -> 求轮子速度）
- **`InverseDynamicsBase`**: 逆向动力学基类（已知底盘受力 -> 求轮子扭矩）

### 2. 全向轮解算 (`OmniCalculation.hpp`)
针对三轮或四轮全向轮底盘的实现。
- **`Omni_FK`**: 正向运动学。计算 `ChassisVx`, `ChassisVy`, `ChassisVw`。
- **`Omni_IK`**: 逆向运动学。输入目标 $v_x, v_y, \omega$，输出各轮转速。
- **`Omni_ID`**: 逆向动力学。输入目标力 $F_x, F_y$ 和力矩 $T$，输出各轮扭矩。

### 3. 舵轮解算 (`StringWheel.hpp`)
针对舵轮（转向轮）底盘的实现，包含特殊的舵向逻辑。
- **`String_FK`**: 正向运动学。需结合当前舵角和轮速计算底盘状态。
- **`String_IK`**: 逆向运动学。
    - 核心功能：计算目标轮速及舵向角度。
    - **就近转位 (`Nearest Transposition`)**：自动计算最优转向路径（如转动 30° 还是反向转动 150° 并反转轮子），防止电线缠绕并提高响应速度。
- **`String_ID`**: 逆向动力学。计算轮子所需的驱动扭矩。

---

## 使用示例

### 全向轮 (Omni) 初始化与调用
# 待验证！！！

### 舵轮 (String Wheel) 初始化与调用
### 初始化
```c++
float wheel_azimuth[4] = {7*PI_/4, PI_/4, 3*PI_/4, 5*PI_/4};    // 舵向安装角度
float phase_offset[4] = {-1.29999f, 3.43989f, 2.84999f, 1.32980f};    // 舵向相位补偿（初始角度）

//  初始化解算模块，第一个参数为轮子半径（m），第二个参数为轮子到中心点距离（m），第三个参数为舵向安装角度，第四个参数为舵向相位补偿
Alg::CalculationBase::String_IK string_ik(0.17f, 0.055f, wheel_azimuth, phase_offset);  // 逆向运动学
Alg::CalculationBase::String_FK string_fk(0.17f, 0.055f, wheel_azimuth, phase_offset);  // 正向运动学
Alg::CalculationBase::String_ID string_id(0.17f, 0.055f, wheel_azimuth, phase_offset);  // 逆向动力学

//  双环舵向PID，逆运动学后舵轮驱动用
ALG::PID::PID stringAngle_pid[4] = {                            // 舵向角度PID
    ALG::PID::PID(8.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 5.0f),   
    ALG::PID::PID(9.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 10.0f),
    ALG::PID::PID(9.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 10.0f),
    ALG::PID::PID(8.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 20.0f)
};
ALG::PID::PID stringVelocity_pid[4] = {                          // 舵向速度PID
    ALG::PID::PID(80.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(80.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(80.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(80.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f)
};

//  单环轮速PID，逆运动学后轮子驱动用
ALG::PID::PID wheel_pid[4] = {
    ALG::PID::PID(0.9f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(0.9f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(0.9f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(0.9f, 0.0f, 0.0f, 16384.0f, 2500.0f, 150.0f)
};

//  底盘平动PID，正运动学后作为转化成底盘整体牵引力
ALG::PID::PID translational_pid[2] = {
    ALG::PID::PID(300.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),   // X方向
    ALG::PID::PID(300.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f)    // Y方向
};
//  底盘旋转PID，正运动学后作为转化成底盘整体旋转力矩
ALG::PID::PID rotational_pid(200.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f);   // Z方向
```
### 调用(任务函数中)
```c++
    for(int i = 0; i < 4; i++)
    {
        string_ik.Set_current_steer_angles(Motor6020.getAngleRad(i+1), i);  //设置舵向电机当前角度 逆运动学
        string_fk.Set_current_steer_angles(Motor6020.getAngleRad(i+1), i);  //设置舵向电机当前角度 正运动学
        string_id.Set_current_steer_angles(Motor6020.getAngleRad(i+1), i);  //设置舵向电机当前角度 逆动力学
    }
    string_fk.StringForKinematics(Motor3508.getVelocityRads(1), Motor3508.getVelocityRads(2), Motor3508.getVelocityRads(3), Motor3508.getVelocityRads(4));  //正运动学，传四个电机的当前转速，求底盘整体速度
    string_ik.StringInvKinematics(chassis_target.target_translation_x, chassis_target.target_translation_y, chassis_target.target_rotation, 0.0f, 2.7f, 15.88f);  //逆运动学 传底盘期望速度，以及舵向相位补偿（包括小陀螺直线和旋转矩阵），还有平动增益和旋转增益
    
    for(int i = 0; i < 4; i++)
    {    
        // 舵向双环pid，计算逆运动学的控制量
        stringAngle_pid[i].UpDate(string_ik.GetMotor_direction(i)*57.3f, Motor6020.getAngleDeg(i+1)); // 舵向角度PID
        stringVelocity_pid[i].UpDate(stringAngle_pid[i].getOutput(), Motor6020.getVelocityRpm(i+1)); // 舵向速度PID

        // 轮子单环pid，计算逆运动学的控制量
        wheel_pid[i].UpDate(string_ik.GetMotor_wheel(i), Motor3508.getVelocityRpm(i+1));

        // 逆运动学输出
        chassis_output.out_string[i] = stringVelocity_pid[i].getOutput(); // 舵向输出
        //chassis_output.out_wheel[i] = wheel_pid[i].getOutput(); 轮向输出，如果是非力控则到这里就是完整的底盘控制。这一条在普通速控底盘不用注释，力控则需要注释掉。

        // 正运动学输出，计算底盘平动和旋转的牵引力和力矩
        translational_pid[0].UpDate(chassis_target.target_translation_x, string_fk.GetChassisVx()); // X方向牵引力
        translational_pid[1].UpDate(chassis_target.target_translation_y, string_fk.GetChassisVy()); // Y方向牵引力
        rotational_pid.UpDate(chassis_target.target_rotation, string_fk.GetChassisVw()); // Z方向旋转力矩
        
        // 逆动力学解算，计算轮向电机的旋转力矩
        string_id.StringInvDynamics(translational_pid[0].getOutput(), translational_pid[1].getOutput(), rotational_pid.getOutput()); // 逆动力学，计算轮子所需的驱动扭矩

        // 将力矩转化为控制电流输出
        chassis_output.out_wheel[i] = string_id.GetMotorTorque(i) / 15.76f / 0.7f / 0.3f * 819.2f + wheel_pid[i].getOutput(); // 轮向输出
    }
```
### 注意事项
1. **坐标系方向**：使用前请确认 X/Y 轴定义以及角度正方向（通常为逆时针）。
2. **单位**：推荐统一使用国际单位制（米、弧度、秒）。
3. **就近转位**：舵轮的 `String_IK` 会自动处理反转逻辑，因此获取的 `GetMotor_wheel` 速度可能会变号（负数），这是正常的，代表轮子反转以配合舵向。
