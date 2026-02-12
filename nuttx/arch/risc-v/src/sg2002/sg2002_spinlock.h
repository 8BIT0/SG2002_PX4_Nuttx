#ifndef __SG2002_SPINLOCK_H
#define __SG2002_SPINLOCK_H

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sg2002_spinlock.h"

#define __CVI_ARCH_SPIN_LOCK_UNLOCKED { 0 }

#define __CVI_RAW_SPIN_LOCK_INITIALIZER(spinlock_hw_field)   \
	{                   \
		.locks = __CVI_ARCH_SPIN_LOCK_UNLOCKED,  \
		.hw_field = spinlock_hw_field, \
	}

#define DEFINE_CVI_SPINLOCK(x, y) SG2002_RawSpinLock_TypeDef x = __CVI_RAW_SPIN_LOCK_INITIALIZER(y)

int sg2002_raw_spin_lock_irqsave(SG2002_RawSpinLock_TypeDef *lock);
void sg2002_raw_spin_unlock_irqrestore(SG2002_RawSpinLock_TypeDef *lock, int flag);

#define drv_spin_lock_irqsave(lock, flags)              \
	{	flags = sg2002_raw_spin_lock_irqsave(lock);  }

#define drv_spin_unlock_irqrestore(lock, flags)     \
	sg2002_raw_spin_unlock_irqrestore(lock, flags)


void sg2002_spinlock_init(void);
void sg2002_spinlock_deinit(void);



#endif
