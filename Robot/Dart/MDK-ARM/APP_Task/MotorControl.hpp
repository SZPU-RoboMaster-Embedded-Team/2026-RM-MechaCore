#ifndef __MotorControl_Hpp
#define __MotorControl_Hpp

#include "RmMotor.hpp"
#include "PID.hpp"
#include "Filter.hpp"

/**
 * @brief 电机对象类 (移植后版本)
 */
class MotorObject {
public:
    RmMotorMeasure_t  Feedback;
    ALG::PID::PID     SpeedPID;
    TD_t              Filter;
    float             Target;
    int               Output;

    MotorObject() : Target(0), Output(0) {
        memset(&Feedback, 0, sizeof(Feedback));
    }

    void Update() {
        Output = (int)SpeedPID.UpDate(Target, (float)Feedback.RPM);
    }
};

// 电机索引枚举
enum MotorIndex_e {
    INDEX_RESERVED = 0,
    INDEX_LEFT_LIFT,
    INDEX_RIGHT_LOAD,
    INDEX_LEFT_LOAD,
    INDEX_YAW,
    INDEX_RIGHT_LIFT,
    INDEX_SPRING,
    TOTAL_CONTROL_MOTORS
};

extern MotorObject Motors[TOTAL_CONTROL_MOTORS];

#endif
