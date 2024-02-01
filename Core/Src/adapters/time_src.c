#include "time_src.h"

static TIM_TypeDef * _sys_timer = NULL;
static uint32_t _time = 0;

void init_system_timer(TIM_TypeDef * timer)
{
    _sys_timer = timer;
}

void sys_timer_irq(TIM_TypeDef * timer)
{
    if (timer == _sys_timer)
    {
        ++_time;
    }
}

uint32_t get_current_rest_time()
{
    if (_sys_timer)
        return _sys_timer->CNT;
    return 0;
}

struct val_time_t get_current_time()
{
    struct val_time_t tm = {
        .ms = _time,
        .rest_us = get_current_rest_time()
    };
    return tm;
}