// #include "../APP/Heat_Control.hpp"
// #include "../../User/BSP/DWT/DWT.hpp" // 用于时间计算

// namespace HeatControl {
// void HeatController::UpDate() {
//     // 更新当前状态计时（FSM内置计时）
//     Status[Get_Now_Status_Serial()].Count_Time++;
    
//     // 计算摩擦轮速度差
//     float velDiff = frictionLeftVel - frictionRightVel;
    
//     // 状态机处理逻辑
//     switch (Get_Now_Status_Serial()) {
//         case DISABLE: {
//             // 速度超过阈值80%且持续300个周期，切换到使能状态
//             if (velDiff > CUR_VEL_THRESHOLD * 0.8f) {
//                 if (Status[DISABLE].Count_Time > 300) {
//                     Set_Status(ENABLE);
//                 }
//             }
//             break;
//         }
        
//         case ENABLE: {
//             // 计算电流差并加入滑动窗口检测
//             float currentDiff = frictionLeftCurrent - frictionRightCurrent;
//             bool isBeyond = currentDetector.addValue(currentDiff);
            
//             // 检测到击发，累加热量和计数
//             if (isBeyond) {
//                 currentDetector.reset();
//                 currentHeat += 10.0f;
//                 fireCount++;
//             }
            
//             // 计算时间差（使用DWT定时器）
//             auto& timer = BSP::DWTimer::GetInstance(168);
//             deltaTime = timer.GetDeltaT(&boosterTime);
            
//             // 热量冷却计算
//             currentHeat -= static_cast<float>(boosterHeatCd) * deltaTime;
//             if (currentHeat < 0.0f) {
//                 currentHeat = 0.0f;
//             }
            
//             // 速度低于阈值，切换到失能状态
//             if (velDiff < CUR_VEL_THRESHOLD) {
//                 Set_Status(DISABLE);
//             }
            
//             // 热量限制逻辑
//             float heatRemain = heatLimit - currentHeat;
//             if (heatRemain > heatLimitSnubber) {
//                 // 热量充足，不限制
//                 currentFire = targetFire;
//             } 
//             else if (heatRemain >= heatLimitStop && heatRemain < heatLimitSnubber) {
//                 // 缓冲区域，线性限制
//                 float heatDiff = heatLimitStop - heatLimitSnubber;
//                 float weightTarget = (heatLimitStop - heatRemain) / heatDiff;
//                 float weightHeatCd = (heatRemain - heatLimitSnubber) / heatDiff;
//                 currentFire = targetFire * weightTarget + 
//                              (static_cast<float>(boosterHeatCd) / 10.0f) * weightHeatCd;
//             } 
//             else if (heatRemain < heatLimitStop) {
//                 // 热量不足，停止发射
//                 currentFire = 0.0f;
//             }
//             break;
//         }
        
//         default:
//             break;
//     }
// }

// } // namespace HeatControl