#ifndef __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_GPIO_H
#define __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_GPIO_H


#define SG2002_GPIOA_BASE 0x03020000ul
#define SG2002_GPIOB_BASE 0x03021000ul
#define SG2002_GPIOC_BASE 0x03022000ul
#define SG2002_GPIOD_BASE 0x03023000ul

typedef union {
    struct {
        uint32_t : 1;
        uint32_t reserved : 31;
    } field;

    uint32_t val;
} SG2002_GPIO_SWPortA_DR_Reg_TyeDef;

typedef union {
    struct {
        uint32_t : 1;
        uint32_t reserved : 31;
    } field;

    uint32_t val;
} SG2002_GPIO_SWPortA_DDR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t : 1;
        uint32_t reserved : 31;
    } field;

    uint32_t val;
} SG2002_GPIO_IntEN_Reg_TypeDef;

typedef union {
    struct {
        uint32_t : 1;
        uint32_t reserved : 31;
    } field;

    uint32_t val;
} SG2002_GPIO_IntMask_Reg_TypeDef;

typedef union {
    struct {
        uint32_t : 1;
        uint32_t reserved : 31;
    } field;

    uint32_t val;
} SG2002_GPIO_IntType_Level_Reg_TypeDef;

typedef union {
    struct {
        uint32_t : 1;
        uint32_t reserved : 31;
    } field;
    
    uint32_t val;
} SG2002_GPIO_Int_Priority_Reg_TypeDef;

typedef union {
    struct {
        uint32_t : 1;
        uint32_t reserved : 31;
    } field;

    uint32_t val;
} SG2002_GPIO_IntStatus_Reg_TypeDef;

#endif
