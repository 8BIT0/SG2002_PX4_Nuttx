#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>

#include <arch/board/board.h>

#include "hardware/sg2002_spinlock.h"
#include "hardware/sg2002_cache.h"
#include "hardware/sg2002_mbox.h"
#include "hardware/sg2002_mmio.h"

typedef struct {
    sem_t sem;
    uint32_t reg_base;
    uint8_t lockCount[SG2002_Spin_MAX + 1];
} SG2002_Spinlock_Priv_TypeDef;

SG2002_Spinlock_Priv_TypeDef Spinlock_priv = {
    .reg_base = SG2002_SPINLOCK_REG_ADDR,
};

void sg2002_spinlock_init(void) {
    memset(&Spinlock_priv.sem, 0, sizeof(sem_t));

    for (uint8_t i = 0; i < (SG2002_Spin_MAX + 1); i ++)
        Spinlock_priv.lockCount[i] = 0;

    nxsem_init(&Spinlock_priv.sem, 0, 1);
}

void sg2002_spinlock_deinit(void) {
    nxsem_destroy(&Spinlock_priv.sem);

    for (uint8_t i = 0; i < (SG2002_Spin_MAX + 1); i ++)
        Spinlock_priv.lockCount[i] = 0;
}

static inline int hw_spin_trylock(SG2002_RawSpinLock_TypeDef *lock) {
    writew(lock->locks, Spinlock_priv.reg_base + sizeof(int) * lock->hw_field);
    
    if (readw(Spinlock_priv.reg_base + sizeof(int) * lock->hw_field) == lock->locks)
        return MAILBOX_LOCK_SUCCESS;
    
    return MAILBOX_LOCK_FAILED;
}

int sg2002_spin_lock(SG2002_RawSpinLock_TypeDef *lock) {
    uint64_t i = 0;
    uint64_t loops = 1000000;
    uint32_t systime = 0;
    struct timespec time;

    SG2002_RawSpinLock_TypeDef _lock = {.hw_field = lock->hw_field, .locks = lock->locks};

    if (lock->hw_field >= SG2002_Spin_LINUX_RTOS) {
        nxsem_wait(&Spinlock_priv.sem);

        if (Spinlock_priv.lockCount[lock->hw_field] == 0)
            Spinlock_priv.lockCount[lock->hw_field]++;
        
        _lock.locks = (Spinlock_priv.lockCount[lock->hw_field] << 8);
        Spinlock_priv.lockCount[lock->hw_field]++;
        
        nxsem_post(&Spinlock_priv.sem);
    } else {
        memset(&time, 0, sizeof(time));

        if (clock_gettime(CLOCK_REALTIME, &time) < 0)
            return MAILBOX_LOCK_FAILED;

        systime = (time.tv_sec * 1000) + (time.tv_nsec / 1000000);

        /* lock ID can not be 0, so set it to 1 at least */
        if ((systime & 0xFFFF) == 0)
            systime = 1;

        lock->locks = (unsigned short) (systime & 0xFFFF);
    }

    for (i = 0; i < loops; i++) {
        if (hw_spin_trylock(&_lock) == MAILBOX_LOCK_SUCCESS) {
            lock->locks = _lock.locks;
            return MAILBOX_LOCK_SUCCESS;
        }
        
        up_udelay(1);
    }
    
    return MAILBOX_LOCK_FAILED;
}

int sg2002_raw_spin_lock_irqsave(SG2002_RawSpinLock_TypeDef *lock) {
    // save and disable irq
    int32_t flag = (__get_MSTATUS() & 8);
    __disable_irq();

    // lock
    if (sg2002_spin_lock(lock) == MAILBOX_LOCK_FAILED) {
        // if spinlock failed , restore irq
        if (flag)
            __enable_irq();
    
        return MAILBOX_LOCK_FAILED;
    }

    return flag;
}

void sg2002_raw_spin_unlock_irqrestore(SG2002_RawSpinLock_TypeDef *lock, int flag) {
    // unlock
    if (readw(Spinlock_priv.reg_base + sizeof(int) * lock->hw_field) != lock->locks)
        return;

    writew(lock->locks, Spinlock_priv.reg_base + sizeof(int) * lock->hw_field);

    // restore irq
    if (flag)
        __enable_irq();	
}

