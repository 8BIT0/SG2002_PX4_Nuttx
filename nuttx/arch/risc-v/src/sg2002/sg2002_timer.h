#ifndef __SG2002_TIMER_H
#define __SG2002_TIMER_H

#include <nuttx/config.h>
#include <nuttx/timers/timer.h>

#include "chip.h"
#include "hardware/sg2002_timer.h"

#ifdef CONFIG_TIMER

typedef enum {
    SG2002_Timer_Set_Freq,
    SG2002_Timer_Get_Current_CountVal,
    SG2002_Timer_Get_Current_LoadCount,
    SG2002_Timer_Check_IntStatus
} SG2002_TimerCMDList_TypeDef;

struct timer_lowerhalf_s *sg2002_timer_initialize(int port);

int sg2002_timer_uninitialize(struct timer_lowerhalf_s *dev);

#endif

#endif

