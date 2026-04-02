#include "InHpp.hpp"

VofaDebugger VofaCallBack;
std::vector<std::string> VofaName;

// // 判断字符串是否为有效数字
// bool isNumeric(const std::string &str)
// {
//     if (str.empty()) return false;
//     for (size_t i = 0; i < str.size(); ++i) {
//         char c = str[i];
//         if (!std::isdigit(static_cast<unsigned char>(c)) && c != '.' && c != '-' && c != '+') {
//             return false;
//         }
//     }
//     return true;
// }

// void VofaDebugger::GetData(const std::vector<uint8_t> &data)
// {
//     // TODO: 实现数据获取逻辑
//     VofaName.push_back("上膛电机转速");
//     VofaName.push_back("Yaw轴(默认3830)");
//     VofaName.push_back("升降电机转速");
//     VofaName.push_back("拉簧/滑块位置");
//     VofaName.push_back("启动/急停");

//     // 检查帧尾是否为 0x0A
//     if (data.empty() || data.back() != 0x0A) {
//         // 数据不完整，缺少帧尾 0x0A
//         return;
//     }

//     // 查找分隔符 0x3A 的位置
//     std::vector<uint8_t>::const_iterator splitIt = std::find(data.begin(), data.end(), 0x3A);
//     // 如果找到分隔符
//     if (splitIt != data.end()) {
//         // 计算分隔符的位置
//         size_t splitIndex = std::distance(data.begin(), splitIt);

//         // 提取分隔符前的数据（不包括分隔符）
//         std::vector<uint8_t> beforeSplit(data.begin(), splitIt);

//         // 提取分隔符后的数据（不包括分隔符和帧尾）
//         std::vector<uint8_t> afterSplit(splitIt + 1, data.end() - 1);

//         // 打印结果
//         // 转换前检查 beforeSplit 是否为合法字符串数据
//         std::string beforeSplitStr(beforeSplit.begin(), beforeSplit.end());
//         std::string utf8Result1 = hexStringToUtf8String(beforeSplitStr);

//         std::string afterSplitStr(afterSplit.begin(), afterSplit.end());
//         std::string value = hexStringToUtf8String(afterSplitStr);

//         // 尝试转换值为整数
//         if (isNumeric(value)) {
//             int intValue = std::atoi(value.c_str());
//             for (size_t i = 0; i < VofaName.size(); ++i) {
//                 if (utf8Result1 == VofaName[i]) {
//                     ExcuteResult(static_cast<int>(i + 1), intValue);
//                     break;
//                 }
//             }
//         }

//     } else {
//         // 未找到分隔符 0x3A
//     }
// }

// void VofaDebugger::ExcuteResult(int index, int value)
// {
//     switch (index) {
//         case 1:
//             // 处理“上膛电机转速”
//             Dart.Set_Motor_Target(&Motors[INDEX_RIGHT_LOAD].SpeedPID.Target, value);
//             break;
//         case 2:
//             // 处理 Yaw轴
//             break;
//         case 3:
//             // 左右丝杆
//             break;
//         case 4:
//             // 上下推进
//             break;
//         case 5:
//             // 启动/急停
//             break;
//         default:
//             break;
//     }
// }

// void VofaDebugger::ProcessReceivedData()
// {
//     // 获取当前 DMA 传输剩余的数据量
//     uint32_t remaining = __HAL_DMA_GET_COUNTER(VofaUartHandle.hdmarx);

//     // 计算已接收的数据长度
//     uint16_t receivedLength = VofaReceiveLength - remaining;

//     if (receivedLength == 0 || receivedLength > VofaReceiveLength) return;

//     // 创建一个临时数组用于存储有效数据
//     std::vector<uint8_t> valid_data(receivedLength);

//     // 清空接收缓存 test
//     memset(VofaCallBack.ReceiveArr, 0, VofaReceiveLength);

//     // 将有效数据拷贝到 valid_data 中
//     memcpy(&valid_data[0], VofaCallBack.ReceiveArr, receivedLength);

//     // 调用 GetData 处理接收到的数据
//     //VofaCallBack.GetData(valid_data);

//     // 重新启动 DMA 接收
//     HAL_UART_Receive_DMA(&VofaUartHandle, VofaCallBack.ReceiveArr, VofaReceiveLength);
// }

void VofaDebugger::ProcessReceivedData()
{
    __HAL_UART_CLEAR_IDLEFLAG(&VofaUartHandle); // 清除空闲中断标志位
    
    // 获取当前 DMA 传输剩余的数据量 , 计算已接收的数据长度
    ReceivedLength = VofaReceiveLength - __HAL_DMA_GET_COUNTER(VofaUartHandle.hdmarx);

    HAL_UART_Transmit_DMA(&VofaUartHandle, VofaCallBack.ReceiveArr, VofaReceiveLength);

    //memset(VofaCallBack.ReceiveArr, '\0', VofaReceiveLength); // 清空接收缓存
    ReceivedLength = 0;                                       // 重置已接收长度

    HAL_UART_DMAStop(&VofaUartHandle);          // 停止DMA传输，防止干扰
    // 重新启动 DMA 接收
    HAL_UART_Receive_DMA(&VofaUartHandle, VofaCallBack.ReceiveArr, VofaReceiveLength);
}

// void test()
// {
//     // 输入的十六进制字符串
//     std::string hex_str = "E6 91 A9 E6 93 A6 E8 BD AE E8 BD AC E9 80 9F";
//     char* cstr = new char[hex_str.size() + 1];
//     std::strcpy(cstr, hex_str.c_str());

//     std::vector<unsigned char> bytes;
//     char* token = std::strtok(cstr, " ");

//     while (token != nullptr) {
//         unsigned int byte = static_cast<unsigned int>(std::strtol(token, nullptr, 16));
//         bytes.push_back(static_cast<unsigned char>(byte));
//         token = std::strtok(nullptr, " ");
//     }

//     // 将字节数组转换为字符串
//     std::string result(bytes.begin(), bytes.end());

//     delete[] cstr;
// }
