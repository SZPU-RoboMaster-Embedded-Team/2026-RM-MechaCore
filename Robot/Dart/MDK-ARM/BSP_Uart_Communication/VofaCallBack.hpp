#ifndef __VofaCallBack_Hpp
#define __VofaCallBack_Hpp

#include <string>
#include <vector>
#include <cstdlib>
#include <cctype>

#define VofaReceiveLength 128 // Vofa接收数据长度

class VofaDebugger
{
public:
    uint32_t ReceivedLength; // 已接收数据长度
    bool isConnected;
    unsigned char ReceiveArr[VofaReceiveLength];
    unsigned char ReceiveData[21];

    void ProcessReceivedData();
    void GetData(const std::vector<uint8_t> &data);
    void ExcuteResult(int index,int value);
};
extern VofaDebugger VofaCallBack;


#endif
