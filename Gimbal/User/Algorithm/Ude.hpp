#include "math.h"

#define toque_const_6020 1.420745050719895e-5f
static float rpm2av(float rpm)
{
    return rpm * 3.1415926 / 30.0f;
}

class Ude
{
  public:
    Ude(float k, float B, float u_max, float break_I) : k_(k), B_(B), u_max(u_max), break_I(break_I) {};
    ~Ude() = default;

    float UdeCalc(float rpm, float u, float err)
    {
        vel_ = rpm2av(rpm);

        // 积分隔离
        if (abs(err) < break_I)
            u0_ += u * B_ * 0.001;
        else
            u0_ = 0;

        // 积分限幅
        if (u0_ > u_max)
            u0_ = u_max;
        if (u0_ < -u_max)
            u0_ = -u_max;

        cout_ = (k_ * (vel_ - u0_)) / B_;

        return cout_;
    }

    float getCout()
    {
        return cout_;
    }

    float getU0()
    {
        return u0_;
    }

    inline void clear()
    {
        u0_ = 0;
        cout_ = 0;
    }

    float vel_;

  private:
    float xxx;
    float B_;
    float cout_;
    float u_max;
    float break_I;
    float u0_;
    float u_;
    float k_;
};