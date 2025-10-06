

#pragma once

#include "../User/APP/Tools.hpp"

namespace APP::Data
{

class GimbalData
{
  private:
    float tar_yaw;
    float tar_pitch;

    constexpr static float DM_kp = 50.0f;
    constexpr static float DM_kd = 2.0f;

  public:
    void setTarYaw(float yaw)
    {
        tar_yaw = yaw;
    }

    void setTarPitch(float pitch)
    {
        tar_pitch = pitch;
    }

    float getTarYaw()
    {
        return tar_yaw;
    }

    float getTarPitch()
    {

        return tar_pitch;
    }

    float getDmKp()
    {
        return DM_kp;
    }

    float getDmKd()
    {
        return DM_kd;
    }
};

inline GimbalData gimbal_data;
} // namespace APP::Data