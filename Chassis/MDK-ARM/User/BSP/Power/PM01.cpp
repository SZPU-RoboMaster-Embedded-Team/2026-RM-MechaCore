// #include "PM01.hpp"

// void PM01::PM01SendRemote(uint32_t StdId)
// {
//     RM_FDorCAN_Send(&hcan2, StdId, 0, CAN_TX_MAILBOX2);
// }

// void PM01::PM01SendData(uint32_t StdId, uint16_t Data)
// {
//     uint8_t SendData[2];

//     SendData[0] = Data >> 8;
//     SendData[1] = Data;

//     RM_FDorCAN_Send(&hcan2, StdId, SendData, CAN_TX_MAILBOX2);
// }

// void PM01::PM01Parse(CAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[])
// {
//     switch (RxHeader.StdId)
//     {
//     case 0x610:
//         PM10_State.Mod_state = (RxHeaderData[0] << 8 | RxHeaderData[1]);
//         PM10_State.Err_state = (RxHeaderData[2] << 8 | RxHeaderData[3]);
//         break;
//     case 0x611:
//         PM01_Data.In_Data[Power]    = (float)(RxHeaderData[0] << 8 | RxHeaderData[1]) * 0.01f;
//         PM01_Data.In_Data[Voltage]  = (float)(RxHeaderData[2] << 8 | RxHeaderData[3]) * 0.01f;
//         PM01_Data.In_Data[Ampere]   = (float)(RxHeaderData[4] << 8 | RxHeaderData[5]) * 0.01f;
//         break;
//     case 0x612:
//         PM01_Data.Out_Data[Power]   = (float)(RxHeaderData[0] << 8 | RxHeaderData[1]) * 0.01f;
//         PM01_Data.Out_Data[Voltage] = (float)(RxHeaderData[2] << 8 | RxHeaderData[3]) * 0.01f;
//         PM01_Data.Out_Data[Ampere]  = (float)(RxHeaderData[4] << 8 | RxHeaderData[5]) * 0.01f;
//         break;
//     case 0x613:
//         Temperature         = (RxHeaderData[0] << 8 | RxHeaderData[1]) * 0.1f;
//         PM01_Time.Add_Time  = (RxHeaderData[2] << 8 | RxHeaderData[3]);
//         PM01_Time.Cur_Time  = (RxHeaderData[4] << 8 | RxHeaderData[5]);
//         break;
//     default:
//         break;
//     }
//     time.UpLastTime();
// }