#include "RefereeTask.hpp"

extern "C" void RefereeTask(void *argument)
{
    for(;;)
    {
        osDelay(1);
    }
}