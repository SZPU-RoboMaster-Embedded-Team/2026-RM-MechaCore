#ifndef HI12BASE_HPP
#define HI12BASE_HPP 

#include "../user/core/BSP/Common/StateWatch/state_watch.hpp"
#include "../user/core/BSP/Common/StateWatch/buzzer_manager.hpp"
#include <string.h>

namespace BSP::IMU
{
    class HI12Base
    {
        public:
            HI12Base(int timeThreshold = 100) : statewatch_(timeThreshold) 
            {
            }
            virtual ~HI12Base() = default;
            
            float R4(uint8_t *p) 
            {
                float r; 
                memcpy(&r,p,4); 
                return r;
            };

            int16_t Init16(uint8_t *p) 
            {
                int16_t r; 
                memcpy(&r,p,2); 
                return __builtin_bswap16(r);
            };

            uint16_t Uint16(uint8_t *p)
            {
                uint16_t r; 
                memcpy(&r,p,2); 
                return __builtin_bswap16(r);
            }

            int32_t Init32(uint8_t *p) 
            {
                int32_t r; 
                memcpy(&r,p,4); 
                return __builtin_bswap32(r);
            };

            void crc16_update(uint16_t *currentCrc, const uint8_t *src, uint32_t lengthInBytes)
            {
                uint16_t crc = *currentCrc;  // 使用16位，不是32位！
                for (uint32_t j = 0; j < lengthInBytes; ++j)
                {
                    crc ^= (src[j] << 8);  // 将字节左移8位后异或
                    for (uint32_t i = 0; i < 8; ++i)
                    {
                        if (crc & 0x8000)  // 检查最高位
                        {
                            crc = (crc << 1) ^ 0x1021;
                        }
                        else
                        {
                            crc <<= 1;
                        }
                    }
                }
                *currentCrc = crc;
            }

            void header()
            {
                if(Header1 == 0x5A && Header2 == 0xA5)
                {
                    Header_flag = true;
                }
                else
                {
                    Header_flag = false;
                }
            }

            void crc(uint8_t *pData)
            {
                payload_len = Length1 + (Length2 << 8);
                crc_calculated = 0;
                crc16_update(&crc_calculated, pData, 4);
                crc16_update(&crc_calculated, pData + 6, payload_len);
                crc_received = CRC1 + (CRC2 << 8);
                if(crc_calculated == crc_received)
                {
                    CRC_flag = true;
                }
                else 
                {
                    CRC_flag = false;
                }
            }
            
            void Verify(uint8_t *pData)
            {
                SetHeader(pData);
                SetCrc(pData);
                SetLength(pData);
                header();
                crc(pData);
            }

            
            void updateTimestamp()
            {
                statewatch_.UpdateLastTime();
            }

            bool isConnected()
            {
                statewatch_.UpdateTime();
                statewatch_.CheckStatus();
                if(statewatch_.GetStatus() == BSP::WATCH_STATE::Status::OFFLINE)
                {
                    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestIMURing();
                }
                return statewatch_.GetStatus() == BSP::WATCH_STATE::Status::ONLINE;
            }

            void SetUart(UART_HandleTypeDef *huart)
            {
                huart_ = huart;
            }

            void SetHeader(uint8_t *pData)
            {
                Header1 = pData[0];
                Header2 = pData[1];
            }

            void SetLength(uint8_t *pData)
            {
                Length1 = pData[2];
                Length2 = pData[3];
            }

            void SetCrc(uint8_t *pData)
            {
                CRC1 = pData[4];
                CRC2 = pData[5];
            }

            bool GetVerify()
            {
                if(Header_flag && CRC_flag)
                {
                    return Verify_flag = true;
                }
                else
                {
                    return Verify_flag = false;
                }
            }

            void ClearORE(UART_HandleTypeDef *huart, uint8_t *pData, int Size)
            {
                if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
                {
                    __HAL_UART_CLEAR_OREFLAG(huart);
                    HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size);
                }
            }

        private:
            BSP::WATCH_STATE::StateWatch statewatch_;
            UART_HandleTypeDef *huart_;
            uint8_t Header1;
            uint8_t Header2;
            uint8_t Length1;
            uint8_t Length2;
            uint8_t CRC1;
            uint8_t CRC2;
            int16_t payload_len;
            uint16_t crc_received;
            uint16_t crc_calculated;
            bool Header_flag = false;
            bool CRC_flag = false;
            bool Verify_flag = false;
    };
}

#endif
