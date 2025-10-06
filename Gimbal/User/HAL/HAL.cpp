// #include "HAL.hpp"
// #include "arm_math.h"

// HAL *HAL::hal = nullptr;

// HAL *HAL::get()
// {
//     return hal;
// }

// bool HAL::check()
// {
//     return hal != nullptr;
// }

// bool HAL::inject(HAL *_hal)
// {
//     if (_hal == nullptr)
//     {
//         return false;
//     }

//     _hal->init();
//     hal = _hal;
//     return true;
// }

// void HAL::destroy()
// {
//     if (hal == nullptr)
//         return;

//     delete hal;
//     hal = nullptr;
// }

// float HAL::_sinf(float x)
// {
//     return arm_sin_f32(x);
// }

// float HAL::_cosf(float x)
// {
//     return arm_cos_f32(x);
// }

// float HAL::_sqrt(float in, float pOut)
// {
//     return arm_sqrt_f32(in, &pOut);
// }

// float HAL::_atan2(float x, float y)
// {
//     return atan2f(x, y);
// }

// // 默认的取符号函数
// int HAL::_sgn(float x)
// {
//     if (x > 0)
//         return 1;

//     else if (x < 0)
//         return -1;

//     return 0;
// }

// float HAL::_floatEqual(float a, float b)
// {
//     return fabs(a - b) < 1e-6f;
// }


