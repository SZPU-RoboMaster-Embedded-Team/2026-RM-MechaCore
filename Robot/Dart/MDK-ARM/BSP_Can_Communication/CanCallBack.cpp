#include "InHpp.hpp"
/*  =========================== 全局变量的初始化 =========================== */ 
Can_RX_T Can_RX = {0};
CAN_HandleTypeDef *Can1_HandleTypedef = &hcan1;

/**
**********************************************************************
* @brief:      	HAL_CAN_RxFifo0MsgPendingCallback: Hal库的Can的通道0接收中断函数
* @param[in]: 	void
* @retval:      void
* @details:    	该函数用于接收机甲大师电机的Can数据,主要接收Yaw轴、升降、上膛和拉簧电机的数据
***********************************************************************
**/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Can_RX.Header, Can_RX.Data);
	
	if(hcan->Instance == Can1_HandleTypedef->Instance)
	RmMotorGetCanData();
}


// /**
// **********************************************************************
// * @brief:      	HAL_CAN_RxFifo0MsgPendingCallback: Hal库的Can的通道1接收中断函数
// * @param[in]: 	void
// * @retval:      void
// * @details:    	该函数用于接收机甲大师电机的Can数据,主要接收6个FrM3508的电机数据
// ***********************************************************************
// **/
// void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
// 	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &Can_RX.Header, Can_RX.Data);
// 	CAN_HandleTypeDef *Can1_HandleTypedef = &hcan2;

// 	if(hcan->Instance == Can1_HandleTypedef->Instance)
// 	RmMotorGetCanData();
// }

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE; //使能过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);//配置过滤器
    HAL_CAN_Start(&hcan1);//启动CAN
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//启动接收中断


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
//    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
//    HAL_CAN_Start(&hcan2);
//    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}
