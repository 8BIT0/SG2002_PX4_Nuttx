#ifndef __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_TIMER_H
#define __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_TIMER_H

#define SG2002_TIMER_FREQ               25000000UL

#define SG2002_TIMER0_BASE              0x030A0000ul
#define SG2002_TIMER0_SIZE              0x14u

#define SG2002_TIMER1_BASE              (SG2002_TIMER0_BASE + SG2002_TIMER0_SIZE)
#define SG2002_TIMER1_SIZE              SG2002_TIMER0_SIZE

#define SG2002_TIMER2_BASE              (SG2002_TIMER1_BASE + SG2002_TIMER1_SIZE)
#define SG2002_TIMER2_SIZE              SG2002_TIMER1_SIZE

#define SG2002_TIMER3_BASE              (SG2002_TIMER2_BASE + SG2002_TIMER2_SIZE)
#define SG2002_TIMER3_SIZE              SG2002_TIMER2_SIZE

#define SG2002_TIMER4_BASE              (SG2002_TIMER3_BASE + SG2002_TIMER3_SIZE)
#define SG2002_TIMER4_SIZE              SG2002_TIMER3_SIZE

#define SG2002_TIMER5_BASE              (SG2002_TIMER4_BASE + SG2002_TIMER4_SIZE)
#define SG2002_TIMER5_SIZE              SG2002_TIMER4_SIZE

#define SG2002_TIMER6_BASE              (SG2002_TIMER5_BASE + SG2002_TIMER5_SIZE)
#define SG2002_TIMER6_SIZE              SG2002_TIMER5_SIZE

#define SG2002_TIMER7_BASE              (SG2002_TIMER6_BASE + SG2002_TIMER6_SIZE)
#define SG2002_TIMER7_SIZE              SG2002_TIMER6_SIZE

typedef enum {
    SG2002_Timer_4 = 4,
    SG2002_Timer_5,
    SG2002_Timer_6,
    SG2002_Timer_7,
} SG2002_TimerList_TypeDef;

typedef union {
    struct {
        uint32_t en         : 1;        /* bit0: enable bit         0-disable / 1-enable */
        uint32_t mode       : 1;        /* bit1: mode bit           0-free running mode / 1-user defined count mode */
        uint32_t int_mask   : 1;        /* bit2: interrupt mask     0-not masked / 1- masked*/
        uint32_t res        : 29;
    } field;

    uint32_t val;
} SG2002_TimerControlReg_TypeDef;

typedef union {
    struct {
        uint32_t eoi        : 1;        /* bit0: contain correspond timer irq status */
        uint32_t res        : 31; 
    } field;

    uint32_t val;
} SG2002_TimerEOI_Reg_TypeDef;

typedef union {
    struct {
        uint32_t int_status : 1;
        uint32_t res        : 31;
    } field;

    uint32_t val;
} SG2002_TimerIntStatus_TypeDef;        /* read only */

#endif
