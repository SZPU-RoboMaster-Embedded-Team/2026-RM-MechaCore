// 在ArmTask.cpp中添加的力控PID代码
// 假设你已经有了重力补偿力矩值

// 1. 在文件顶部添加力控PID相关的全局变量

// 使用现有的ALG::PID::PID类创建力矩控制器
ALG::PID::PID pid_torque_joint1(8.0f, 0.2f, 0.1f, 16384.0f, 1000.0f, 0.0f);  // 关节1力矩PID
ALG::PID::PID pid_torque_joint2(6.0f, 0.15f, 0.08f, 16384.0f, 800.0f, 0.0f);  // 关节2力矩PID
ALG::PID::PID pid_torque_joint3(5.0f, 0.1f, 0.06f, 16384.0f, 600.0f, 0.0f);   // 关节3力矩PID
ALG::PID::PID pid_torque_joint4(4.0f, 0.08f, 0.05f, 16384.0f, 400.0f, 0.0f);  // 关节4力矩PID

// 重力补偿力矩值 (由外部计算或查表得到)
float gravity_compensation_torque[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // N·m

// 目标力矩值 (重力补偿 + 控制力矩)
float target_torque[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // N·m

// 当前力矩反馈值
float current_torque[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // N·m

// 2. 力矩反馈计算函数 (从电机电流估算力矩)
float calculateTorqueFromCurrent(int16_t motor_current, float torque_constant = 0.3f) {
    // 电机电流转力矩
    // motor_current: 电机电流值 (-16384 ~ 16384)
    // torque_constant: 力矩常数 (N·m/A)
    
    float current_ampere = (float)motor_current * 20.0f / 16384.0f; // 假设满量程20A
    return current_ampere * torque_constant;
}

// 3. 力矩转电机控制指令函数
int16_t torqueToMotorCommand(float target_torque, float torque_constant = 0.3f) {
    // 力矩转电机控制指令
    float required_current = target_torque / torque_constant; // 所需电流 (A)
    int16_t motor_cmd = (int16_t)(required_current * 16384.0f / 20.0f); // 转换为电机指令
    
    // 限幅保护
    if (motor_cmd > 16384) motor_cmd = 16384;
    if (motor_cmd < -16384) motor_cmd = -16384;
    
    return motor_cmd;
}

// 4. 设置重力补偿力矩的接口函数
void setGravityCompensationTorque(int joint_index, float torque_nm) {
    if (joint_index >= 0 && joint_index < 4) {
        gravity_compensation_torque[joint_index] = torque_nm;
    }
}

// 批量设置重力补偿力矩
void setAllGravityCompensationTorques(float torque1, float torque2, float torque3, float torque4) {
    gravity_compensation_torque[0] = torque1;
    gravity_compensation_torque[1] = torque2;
    gravity_compensation_torque[2] = torque3;
    gravity_compensation_torque[3] = torque4;
}

// 5. 设置额外控制力矩的接口函数
void setAdditionalTorque(int joint_index, float additional_torque) {
    if (joint_index >= 0 && joint_index < 4) {
        target_torque[joint_index] = gravity_compensation_torque[joint_index] + additional_torque;
    }
}

// 6. 力控PID主控制函数
void TorqueControlLoop() {
    // 更新目标力矩 (重力补偿 + 额外控制力矩)
    for (int i = 0; i < 4; i++) {
        target_torque[i] = gravity_compensation_torque[i]; // 默认只有重力补偿
    }
    
    // 获取当前电机电流并转换为力矩反馈
    current_torque[0] = calculateTorqueFromCurrent(Motor6020.getCurrent(1), 0.3f);
    current_torque[1] = calculateTorqueFromCurrent(Motor6020.getCurrent(2), 0.3f);
    current_torque[2] = calculateTorqueFromCurrent(Motor6020.getCurrent(3), 0.3f);
    current_torque[3] = calculateTorqueFromCurrent(Motor6020.getCurrent(4), 0.3f);
    
    // PID控制计算
    float pid_output[4];
    pid_output[0] = pid_torque_joint1.UpDate(target_torque[0], current_torque[0]);
    pid_output[1] = pid_torque_joint2.UpDate(target_torque[1], current_torque[1]);
    pid_output[2] = pid_torque_joint3.UpDate(target_torque[2], current_torque[2]);
    pid_output[3] = pid_torque_joint4.UpDate(target_torque[3], current_torque[3]);
    
    // 将PID输出设置给电机
    Motor6020.setMotor(1, (int16_t)pid_output[0]);
    Motor6020.setMotor(2, (int16_t)pid_output[1]);
    Motor6020.setMotor(3, (int16_t)pid_output[2]);
    Motor6020.setMotor(4, (int16_t)pid_output[3]);
}

// 7. 简化版本 - 直接前馈控制 (如果PID效果不理想)
void DirectTorqueControl() {
    // 直接将重力补偿力矩转换为电机指令 (前馈控制)
    int16_t motor_cmd[4];
    
    motor_cmd[0] = torqueToMotorCommand(gravity_compensation_torque[0], 0.3f);
    motor_cmd[1] = torqueToMotorCommand(gravity_compensation_torque[1], 0.3f);
    motor_cmd[2] = torqueToMotorCommand(gravity_compensation_torque[2], 0.3f);
    motor_cmd[3] = torqueToMotorCommand(gravity_compensation_torque[3], 0.3f);
    
    // 设置电机指令
    Motor6020.setMotor(1, motor_cmd[0]);
    Motor6020.setMotor(2, motor_cmd[1]);
    Motor6020.setMotor(3, motor_cmd[2]);
    Motor6020.setMotor(4, motor_cmd[3]);
}

// 8. 在原有ArmTask函数中的使用示例
/*
void ArmTask(void *argument)
{
    TASK::ROBOTIC_ARM::robotic_arm.Motor_Init();
    
    // 初始化力控PID
    pid_torque_joint1.Clear();
    pid_torque_joint2.Clear();
    pid_torque_joint3.Clear();
    pid_torque_joint4.Clear();
    
    for(;;)
    {
        // 方法1: 外部设置重力补偿力矩值 (例如从查表或实时计算获得)
        // 这些值需要根据当前关节角度实时更新
        setAllGravityCompensationTorques(1.2f, 2.5f, 1.8f, 0.5f); // 示例值
        
        // 方法2: 如果需要额外的控制力矩 (例如轨迹跟踪)
        // setAdditionalTorque(1, 0.3f); // 给关节2添加0.3N·m的额外力矩
        
        // 执行力控PID
        TorqueControlLoop();
        
        // 或者使用直接前馈控制 (更简单但可能精度较低)
        // DirectTorqueControl();
        
        // 发送CAN消息
        Motor6020.canTxMsg();
        
        osDelay(1); // 1ms控制周期
    }
}
*/

// 9. 调试和监控函数
void printTorqueStatus() {
    // 用于调试的状态打印函数
    printf("Target Torques: [%.2f, %.2f, %.2f, %.2f] N·m\n", 
           target_torque[0], target_torque[1], target_torque[2], target_torque[3]);
    printf("Current Torques: [%.2f, %.2f, %.2f, %.2f] N·m\n", 
           current_torque[0], current_torque[1], current_torque[2], current_torque[3]);
    printf("Gravity Compensation: [%.2f, %.2f, %.2f, %.2f] N·m\n", 
           gravity_compensation_torque[0], gravity_compensation_torque[1], 
           gravity_compensation_torque[2], gravity_compensation_torque[3]);
}

// 10. PID参数在线调节函数
void updateTorquePIDParams(int joint_index, float kp, float ki, float kd) {
    switch(joint_index) {
        case 0:
            // 注意：如果你的PID类支持参数更新，使用相应的方法
            // pid_torque_joint1.setParams(kp, ki, kd);
            break;
        case 1:
            // pid_torque_joint2.setParams(kp, ki, kd);
            break;
        case 2:
            // pid_torque_joint3.setParams(kp, ki, kd);
            break;
        case 3:
            // pid_torque_joint4.setParams(kp, ki, kd);
            break;
    }
}

// 使用说明：
/*
1. 将上述代码添加到你的ArmTask.cpp文件中
2. 根据你的实际电机参数调整力矩常数 (torque_constant)
3. 根据你的系统调整PID参数
4. 在主循环中调用 setAllGravityCompensationTorques() 设置重力补偿力矩
5. 调用 TorqueControlLoop() 执行力控PID
6. 如果需要更简单的控制，可以使用 DirectTorqueControl()
*/