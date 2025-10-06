#include "RefeeTask.hpp"
#include "../APP/UI/UI_Queue.hpp"
#include "../APP/UI/Static/darw_static.hpp"
#include "../APP/UI/Dynamic/darw_dynamic.hpp"
#include "cmsis_os2.h"

void RefeeTask(void *argument)
{
   UI::Static::UI_static.Init();
    for (;;)
    {
       UI::Dynamic::UI_dynamic.darw_UI();
       UI::UI_send_queue.send();

        osDelay(1);
    }
}
