#ifndef __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_GPIO_H
#define __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_GPIO_H

#define SG2002_GPIO0_BASE                   0x03020000ul    /* GPIO A */
#define SG2002_GPIO1_BASE                   0x03021000ul    /* GPIO B */
#define SG2002_GPIO2_BASE                   0x03022000ul    /* GPIO C */
#define SG2002_GPIO3_BASE                   0x03023000ul    /* GPIO D */

/* register offset definition */
#define SG2002_GPIO_SWPORTA_DR_OFFSET       0x00
#define SG2002_GPIO_SWPORTA_DDR_OFFSET      0x04
#define SG2002_GPIO_INTEN_OFFSET            0x30
#define SG2002_GPIO_INTMASK_OFFSET          0x34
#define SG2002_GPIO_INTTYPE_LEVEL_OFFSET    0x38
#define SG2002_GPIO_INT_POLARITY_OFFSET     0x3C
#define SG2002_GPIO_INTSTATUS_OFFSET        0x40
#define SG2002_GPIO_RAW_INTSTATUS_OFFSET    0x44
#define SG2002_GPIO_DEBOUNCE_OFFSET         0x48
#define SG2002_GPIO_PORTA_EOI_OFFSET        0x4C
#define SG2002_GPIO_EXT_PORTA_OFFSET        0x50
#define SG2002_GPIO_LS_SYNC_OFFSET          0x60

typedef enum {
    SG2002_GPIO_Input = 0,
    SG2002_GPIO_Output,
    SG2002_GPIO_Invalid,
} SG2002_GPIO_DirType_List;

typedef enum {
    SG2002_INT_LowLevel = 0,
    SG2002_INT_HighLevel,
    SG2002_INT_FallingEdge,
    SG2002_INT_RisingEdge,
} SG2002_GPIO_InterruptType_List;

typedef struct{
    uint32_t val;
} SG2002_GPIO_SWPortA_DR_Reg_TyeDef;

typedef struct{
    uint32_t val;
} SG2002_GPIO_SWPortA_DDR_Reg_TypeDef;

typedef struct{
    uint32_t val;
} SG2002_GPIO_IntEN_Reg_TypeDef;

typedef struct{
    uint32_t val;
} SG2002_GPIO_IntMask_Reg_TypeDef;

typedef struct{
    uint32_t val;
} SG2002_GPIO_IntType_Level_Reg_TypeDef;

typedef struct{
    uint32_t val;
} SG2002_GPIO_Int_Polarity_Reg_TypeDef;

typedef struct{
    uint32_t val;
} SG2002_GPIO_IntStatus_Reg_TypeDef;    /* read only */

typedef struct{
    uint32_t val;
} SG2002_GPIO_RawIntStatus_Reg_TypeDef; /* read only */

typedef struct{
    uint32_t val;
} SG2002_GPIO_Debounce_Reg_TypeDef;

typedef struct{
    uint32_t val;
} SG2002_GPIO_PortA_EOI_Reg_TypeDef;    /* write only */

typedef struct{
    uint32_t val;
} SG2002_GPIO_EXT_PortA_Reg_TypeDef;    /* read only */

typedef union {
    struct {
        uint32_t sync : 1;
        uint32_t reserved : 31;
    } field;

    uint32_t val;
} SG2002_GPIO_LS_Sync_Reg_TypeDef;

#endif
