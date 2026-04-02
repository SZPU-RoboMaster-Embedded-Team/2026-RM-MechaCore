#include "RefeeTask.hpp"
#include "../APP/UI/UI_Queue.hpp"
#include "../APP/UI/Static/darw_static.hpp"
#include "../APP/UI/Dynamic/darw_dynamic.hpp"
#include "CommunicationTask.hpp"
#include "cmsis_os2.h"
extern "C" uint32_t HAL_GetTick(void);

void RefeeTask(void *argument)
{
    UI::Static::UI_static.Init();
    bool last_ref_ready = false;
    bool last_ctrl = false;
    uint32_t last_periodic_rebuild_ms = HAL_GetTick();

    for (;;)
    {
        const uint32_t now_ms = HAL_GetTick();
        const bool ref_ready = UI::UI_send_queue.referee_id_ready();
        const bool ctrl_now = Gimbal_to_Chassis_Data.getCtrl();

        bool request_hard_rebuild = false;
        bool request_soft_rebuild = false;
        // 裁判链路由未就绪变为就绪时，触发一次硬重建。
        if (ref_ready && !last_ref_ready) {
            request_hard_rebuild = true;
        }
        // 全局 Ctrl 上升沿触发硬重建（不依赖底盘模式）。
        if (ctrl_now && !last_ctrl) {
            request_hard_rebuild = true;
        }
        // 周期性自愈：客户端重启会清空 UI，目标端没有显式回调。
        if (ref_ready && (now_ms - last_periodic_rebuild_ms >= 5000U)) {
            request_soft_rebuild = true;
            last_periodic_rebuild_ms = now_ms;
        }

        if (request_hard_rebuild &&
            !UI::UI_send_queue.is_Delete_all &&
            UI::UI_send_queue.size == 0 &&
            UI::UI_send_queue.wz_size == 0) {
            UI::UI_send_queue.setDeleteAll();
            UI::Static::UI_static.Init();
        } else if (request_soft_rebuild &&
                   !UI::UI_send_queue.is_Delete_all &&
                   UI::UI_send_queue.size == 0 &&
                   UI::UI_send_queue.wz_size == 0) {
            // 软重建：仅重发静态 Add，不强制清屏。
            UI::Static::UI_static.Init();
        }

        last_ref_ready = ref_ready;
        last_ctrl = ctrl_now;

        UI::Dynamic::UI_dynamic.darw_UI();
        UI::UI_send_queue.send_wz();
        UI::UI_send_queue.send();

        // 静态图元和文本发送完毕后，解锁动态层更新。
        if (!UI::UI_send_queue.is_up_ui &&
            UI::UI_send_queue.size == 0 &&
            UI::UI_send_queue.wz_size == 0) {
            UI::UI_send_queue.is_up_ui = true;
        }

        osDelay(10);
    }
}
