#ifndef HI12_IMU_HPP
#define HI12_IMU_HPP

#include "HI12Base.hpp"

namespace BSP::IMU
{
    class HI12_float : public HI12Base
    {
      public:
        HI12_float()
            : offset(6), acc{0.0f}, gyro{0.0f}, angle{0.0f}, quaternion{0.0f}, last_angle(0.0f), add_angle(0.0f)
        {
        }

        void DataUpdate(uint8_t *pData, uint16_t size = 82)
        {
            if (pData == nullptr || size < 82)
            {
                return;
            }

            this->updateTimestamp();

            int frame_start = -1;
            for (int i = 0; i <= static_cast<int>(size) - 82; ++i)
            {
                if (pData[i] != 0x5A || pData[i + 1] != 0xA5)
                {
                    continue;
                }

                const uint8_t len_low = pData[i + 2];
                const uint8_t len_high = pData[i + 3];
                const int possible_len = len_low + (len_high << 8);
                if (possible_len == 76)
                {
                    frame_start = i;
                    break;
                }
            }

            if (frame_start < 0)
            {
                return;
            }

            uint8_t *frame = pData + frame_start;
            Verify(frame);
            if (!GetVerify())
            {
                return;
            }

            acc[0] = this->R4(frame + offset + 12);
            acc[1] = this->R4(frame + offset + 16);
            acc[2] = this->R4(frame + offset + 20);

            gyro[0] = this->R4(frame + offset + 24);
            gyro[1] = this->R4(frame + offset + 28);
            gyro[2] = this->R4(frame + offset + 32);

            angle[0] = this->R4(frame + offset + 48);
            angle[1] = this->R4(frame + offset + 52);
            angle[2] = this->R4(frame + offset + 56);

            quaternion[0] = this->R4(frame + offset + 60);
            quaternion[1] = this->R4(frame + offset + 64);
            quaternion[2] = this->R4(frame + offset + 68);
            quaternion[3] = this->R4(frame + offset + 72);
        }

        float GetAcc(int index)
        {
            return acc[index];
        }

        float GetGyro(int index)
        {
            return gyro[index];
        }

        float GetGyroRPM(int index)
        {
            return gyro[index] / 6.0f;
        }

        float GetAngle(int index)
        {
            return angle[index];
        }

        float GetQuaternion(int index)
        {
            return quaternion[index];
        }

        float GetPitch_180()
        {
            return angle[1] + 90.0f;
        }

        float GetYaw_360()
        {
            return angle[2] + 180.0f;
        }

        float GetAddYaw()
        {
            const float data = angle[2] + 180.0f;

            if (data - last_angle < -180.0f)
            {
                add_angle += 360.0f - last_angle + data;
            }
            else if (data - last_angle > 180.0f)
            {
                add_angle += -(360.0f - data + last_angle);
            }
            else
            {
                add_angle += data - last_angle;
            }

            last_angle = data;
            return add_angle;
        }

      private:
        int offset;
        float acc[3];
        float gyro[3];
        float angle[3];
        float quaternion[4];
        float last_angle;
        float add_angle;
    };
}

#endif
