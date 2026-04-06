#pragma once

class RM_Key
{
public:
    //当前状态，上一刻状态，上升沿状态，下降沿状态
    bool NowKey,lastKey,RisingEdge,FallingEdge;
    void UpKey(bool key);//更新信号
    bool GetRisingKey();//获取上升沿信号
    bool GetFallingKey();//获取下降沿信号
};
inline void RM_Key::UpKey(bool key)
{
    this->lastKey = this->NowKey;//更新上一次信号
    this->NowKey = key;//获取当前信号
    this->RisingEdge = this->FallingEdge = false;//情况状态
    if(this->NowKey - this->lastKey == 1)this->FallingEdge = true;//设置上升沿信号
    if(this->lastKey - this->NowKey == 1)this->RisingEdge = true;//设置下降沿信号
}

inline bool RM_Key::GetRisingKey()
{
    return this->RisingEdge;//上升沿信号
}

inline bool RM_Key::GetFallingKey()
{
    return this->FallingEdge;//下降沿信号
}
