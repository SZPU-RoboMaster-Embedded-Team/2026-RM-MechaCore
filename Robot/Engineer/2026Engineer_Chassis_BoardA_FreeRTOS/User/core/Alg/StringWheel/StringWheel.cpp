// #include "../Algorithm/StringWheel.hpp"
// void SteeringWheel::_Steer_Motor_Kinematics_Nearest_Transposition()
// {
//     for (int i = 0; i < 4; i++)
//     {
//         float tmp_delta_angle = NormalizeAngle(params_.Angle[i] - current_steer_angles[i], 2.0f * M_PI);

//         // 根据转动角度范围决定是否需要就近转位
//         if (-M_PI / 2.0f <= tmp_delta_angle && tmp_delta_angle <= M_PI / 2.0f)
//         {
//             // ±PI / 2之间无需反向就近转位
//             params_.Angle[i] = tmp_delta_angle + current_steer_angles[i];
//         }
//         else
//         {
//             // 需要反转扣圈情况
//             params_.Angle[i] = NormalizeAngle(tmp_delta_angle + M_PI, 2.0f * M_PI) + current_steer_angles[i];
//             params_.Target_Wheel_Omega[i] *= -1.0f;
//             params_.Speed[i] *= -1.0f;
//         }
//     }
// }

// void SteeringWheel::Kinematics_Inverse_Resolution(float vx, float vy, float vw, float maxSpeed)
// {  

//     for (int i = 0; i < 4; i++)
//     {
//         float tmp_velocity_x, tmp_velocity_y, tmp_velocity_modulus;

//         // 使用Class_Chassis的算法：解算到每个轮组的具体线速度
//         tmp_velocity_x = vx - vw * Wheel_To_Core_Distance[i] * arm_sin_f32(Wheel_Azimuth[i]);
//         tmp_velocity_y = vy + vw * Wheel_To_Core_Distance[i] * arm_cos_f32(Wheel_Azimuth[i]);
//         arm_sqrt_f32(tmp_velocity_x * tmp_velocity_x + tmp_velocity_y * tmp_velocity_y, &tmp_velocity_modulus);

//         // 根据线速度决定轮向电机角速度（转换为RPM）
//         params_.Target_Wheel_Omega[i] = tmp_velocity_modulus / Wheel_Radius;
//         params_.Speed[i] = params_.Target_Wheel_Omega[i] * 60.0f / (2.0f * M_PI); // rad/s转RPM
//         LimitSpeed(params_.Speed[i], maxSpeed);

//         // 根据速度的xy分量分别决定舵向电机角度
//         if (tmp_velocity_modulus == 0.0f)
//         {
//             // 排除除零问题，保持当前角度
//             params_.Angle[i] = current_steer_angles[i];
//         }
//         else
//         {
//             // 没有除零问题
//             params_.Angle[i] = atan2f(tmp_velocity_y, tmp_velocity_x);
//         }
//     }
//     // 执行就近转位
//     _Steer_Motor_Kinematics_Nearest_Transposition();
// }
// void SteeringWheel::Kinematics_Forward_Resolution()
// {
//     float sumVx = 0.0f, sumVy = 0.0f, sumVw = 0.0f;
//     int validWheels = 0;

//     for (int i = 0; i < 4; i++)
//     {
//         float wheel_omega = wheel_motors.getVelocityRads(i+1);
//         float steer_angle = steer_motors.getMultiAngle(i+1) * M_PI / 180.0f;

//         // 计算轮子在底盘坐标系中的速度分量
//         float wheel_vx = wheel_omega * Wheel_Radius * cosf(steer_angle);
//         float wheel_vy = wheel_omega * Wheel_Radius * sinf(steer_angle);

//         sumVx += wheel_vx;
//         sumVy += wheel_vy;
//         validWheels++;

//         // 计算角速度贡献 (基于轮子安装位置)
//         float delta_angle = steer_angle - Wheel_Azimuth[i];
//         sumVw += (wheel_omega * Wheel_Radius * sinf(delta_angle)) / Wheel_To_Core_Distance[i];
//     }

//     // 计算平均值得到底盘整体运动状态
//     if (validWheels > 0) {
//         this->params_.Vx = sumVx / validWheels;
//         this->params_.Vy = sumVy / validWheels;
//         this->params_.Vw = sumVw / validWheels;
//     } else {
//         this->params_.Vx = 0;
//         this->params_.Vy = 0;
//         this->params_.Vw = 0;
//     }
// }
// void SteeringWheel::DynamicsInverseCalculation()
// {
//     float force_x, force_y, torque_omega;   

//     force_x = 0.0f;//来自PID X的输出，后续添加;
//     force_y = 0.0f;//来自PID Y的输出，后续添加;
//     torque_omega = 0.0f;//来自PID Omega的输出，后续添加;

//     // 每个轮的扭力
//     float tmp_force[4];
//     for (int i = 0; i < 4; i++)
//     {
//         // 解算到每个轮组的具体摩擦力
//         tmp_force[i] = force_x * arm_cos_f32(steer_motors.getAngleDeg(i)) + force_y * arm_sin_f32(steer_motors.getAngleDeg(i)) - 
//                     torque_omega / Wheel_To_Core_Distance[i] * arm_sin_f32(Wheel_Azimuth[i] - steer_motors.getAngleDeg(i));
//     }
//     for (int i = 0; i < 4; i++)
//     {
//         // 摩擦力转换至扭矩
//         params_.Target_Wheel_Current[i] = tmp_force[i] * Wheel_Radius + WHEEL_SPEED_LIMIT_FACTOR *
//                                 (params_.Target_Wheel_Omega[i] - wheel_motors.getVelocityRads(i));

//     }

// }