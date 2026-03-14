#ifndef HI12_IMU_HPP
#define HI12_IMU_HPP

#include "HI12Base.hpp"

namespace BSP::IMU
{
    class HI12_float : public HI12Base
    { 
        public: 
            HI12_float(UART_HandleTypeDef *huart) 
                : offset(6), acc{0}, gyro{0}, angle{0}, quaternion{0}
            {
                SetUart(huart);
            }

            void DataUpdate(uint8_t *pData)
            {
                this->updateTimestamp();
                Verify(pData);
                if(GetVerify())
                {
                    acc[0] = this->R4(pData+offset+12);
                    acc[1] = this->R4(pData+offset+16);
                    acc[2] = this->R4(pData+offset+20);
                    
                    gyro[0] = this->R4(pData+offset+24);
                    gyro[1] = this->R4(pData+offset+28);
                    gyro[2] = this->R4(pData+offset+32);
                    
                    angle[0] = this->R4(pData+offset+48);
                    angle[1] = this->R4(pData+offset+52);
                    angle[2] = this->R4(pData+offset+56);
                    
                    quaternion[0] = this->R4(pData+offset+60);
                    quaternion[1] = this->R4(pData+offset+64);
                    quaternion[2] = this->R4(pData+offset+68);
                    quaternion[3] = this->R4(pData+offset+72);
                }
            }

            float GetAcc(int index)
            {
                return acc[index];
            }

            float GetGyro(int index)
            {
                return gyro[index];
            }

            float GetAngle(int index)
            {
                return angle[index];
            }

            float GetQuaternion(int index)
            {
                return quaternion[index];
            }

        private:
            int offset;
            float acc[3];
            float gyro[3];
            float angle[3];
            float quaternion[4];
    };

    class HI12_int : public HI12Base
    { 
        public: 
            HI12_int(UART_HandleTypeDef *huart) 
                : offset(6), acc{0}, gyro{0}, angle{0}, quaternion{0}
            {
                SetUart(huart);
            }
     
            void DataUpdate(uint8_t *pData)
            {
                this->updateTimestamp();
                Verify(pData);
                if(GetVerify())
                {
                    acc[0] = this->Init16(pData+offset+0) * 0.00048828f;
                    acc[1] = this->Init16(pData+offset+2) * 0.00048828f;
                    acc[2] = this->Init16(pData+offset+4) * 0.00048828f;
                    
                    gyro[0] = this->Init16(pData+offset+6)  * 0.061035f;  
                    gyro[1] = this->Init16(pData+offset+8)  * 0.061035f;  
                    gyro[2] = this->Init16(pData+offset+10) * 0.061035f;  
                    
                    angle[0] = this->Init32(pData+offset+18) * 0.001f;  // 使用Init32直接解析32位数据
                    angle[1] = this->Init32(pData+offset+22) * 0.001f;
                    angle[2] = this->Init32(pData+offset+26) * 0.001f;
                    
                    quaternion[0] = this->Uint16(pData+offset+32) * 0.0001f;
                    quaternion[1] = this->Uint16(pData+offset+34) * 0.0001f;
                    quaternion[2] = this->Uint16(pData+offset+36) * 0.0001f;
                    quaternion[3] = this->Uint16(pData+offset+38) * 0.0001f;
                }
            }

            float GetAcc(int index)
            {
                return acc[index];
            }

            float GetGyro(int index)
            {
                return gyro[index];
            }

            float GetAngle(int index)
            {
                return angle[index];
            }

            float GetQuaternion(int index)
            {
                return quaternion[index];
            }

        private:
            int offset;
            float acc[3];
            float gyro[3];
            float angle[3];
            float quaternion[4];
    };
}

#endif
