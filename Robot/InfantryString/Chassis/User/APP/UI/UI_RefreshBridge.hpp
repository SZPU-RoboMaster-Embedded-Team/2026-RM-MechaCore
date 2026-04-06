#pragma once

#include <stdint.h>

extern "C" uint32_t HAL_GetTick(void);

namespace UI::Refresh
{
    struct Probe
    {
        volatile uint32_t uart6_rx_isr_count;
        volatile uint16_t uart6_last_size;

        volatile uint32_t can_ui_frame_count;

        volatile uint32_t refee_loop_count;
        volatile uint32_t draw_run_count;
        volatile uint32_t draw_skip_count;

        volatile uint32_t last_draw_tick_ms;
        volatile uint32_t last_dirty_tick_ms;

        volatile uint8_t dirty;
    };

    extern Probe g_probe;

    inline void NotifyUart6RxFromISR(uint16_t size)
    {
        g_probe.uart6_rx_isr_count++;
        g_probe.uart6_last_size = size;
        g_probe.last_dirty_tick_ms = ::HAL_GetTick();
        g_probe.dirty = 1;
    }

    inline void NotifyUiDataUpdated()
    {
        g_probe.can_ui_frame_count++;
        g_probe.last_dirty_tick_ms = ::HAL_GetTick();
        g_probe.dirty = 1;
    }

    inline bool ShouldDraw(uint32_t now_ms, uint32_t heartbeat_ms = 50)
    {
        if (g_probe.dirty) {
            return true;
        }
        return (now_ms - g_probe.last_draw_tick_ms) >= heartbeat_ms;
    }

    inline void MarkRefeeLoop()
    {
        g_probe.refee_loop_count++;
    }

    inline void MarkDrawRun(uint32_t now_ms)
    {
        g_probe.draw_run_count++;
        g_probe.last_draw_tick_ms = now_ms;
        g_probe.dirty = 0;
    }

    inline void MarkDrawSkip()
    {
        g_probe.draw_skip_count++;
    }
}
