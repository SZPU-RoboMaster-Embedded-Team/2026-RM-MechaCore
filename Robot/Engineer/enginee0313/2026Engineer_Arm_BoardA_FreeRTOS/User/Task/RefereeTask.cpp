#include "RefereeTask.hpp"

extern "C" void RefereeTask(void *argument)
{
    TickType_t Lasttick = xTaskGetTickCount();

    for(;;)
    {
        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));
        osDelay(1);
    }
}