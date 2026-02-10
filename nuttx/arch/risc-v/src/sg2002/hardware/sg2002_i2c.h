#ifndef __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_IIC_H
#define __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_IIC_H

#define SG2002_I2C_BUS_NUM                  2

#define SG2002_I2C1_BASE                    0x04010000
#define SG2002_I2C3_BASE                    0x04030000

#define SG2002_IC_CLK			            166666666

/* High and low times in different speed modes (in ns) */
#define SG2002_MIN_SS_SCL_HIGHTIME          4000
#define SG2002_MIN_SS_SCL_LOWTIME           4700
#define SG2002_MIN_FS_SCL_HIGHTIME          600
#define SG2002_MIN_FS_SCL_LOWTIME           1300

/* int reg bit definition */
#define SG2002_BIT_I2C_INT_RX_UNDER         (1 << 0)
#define SG2002_BIT_I2C_INT_RX_OVER          (1 << 1)
#define SG2002_BIT_I2C_INT_RX_FULL          (1 << 2)
#define SG2002_BIT_I2C_INT_TX_OVER          (1 << 3)
#define SG2002_BIT_I2C_INT_TX_EMPTY         (1 << 4)
#define SG2002_BIT_I2C_INT_RD_REQ           (1 << 5)
#define SG2002_BIT_I2C_INT_TX_ABRT          (1 << 6)
#define SG2002_BIT_I2C_INT_RX_DONE          (1 << 7)
#define SG2002_BIT_I2C_INT_ACTIVITY         (1 << 8)
#define SG2002_BIT_I2C_INT_STOP_DET         (1 << 9)
#define SG2002_BIT_I2C_INT_START_DET        (1 << 10)
#define SG2002_BIT_I2C_INT_GEN_ALL          (1 << 11)
#define SG2002_I2C_INTR_MASTER_MASK         (SG2002_BIT_I2C_INT_TX_ABRT  | \
                                             SG2002_BIT_I2C_INT_STOP_DET | \
                                             SG2002_BIT_I2C_INT_RX_FULL  | \
                                             SG2002_BIT_I2C_INT_TX_EMPTY)
                                              
/* status reg definitions */
#define SG2002_IC_STATUS_SA                 0x40
#define SG2002_IC_STATUS_MA                 0x20
#define SG2002_IC_STATUS_RFF                0x10
#define SG2002_IC_STATUS_RFNE               0x08
#define SG2002_IC_STATUS_TFE                0x04
#define SG2002_IC_STATUS_TFNF               0x02
#define SG2002_IC_STATUS_ACT                0x01

#define SG2002_IC_CON_OFFSET                0x000   /* I2C control */
#define SG2002_IC_TAR_OFFSET                0x004   /* I2C Target Addres */
#define SG2002_IC_SAR_OFFSET                0x008   /* I2C Slave Address */
#define SG2002_IC_DATA_CMD_OFFSET           0x010   /* I2C Rx/Tx Data Buffer and Command */
#define SG2002_IC_SS_SCL_HCNT_OFFSET        0x014   /* Standard speed I2C Clock SCL High Count */
#define SG2002_IC_SS_SCL_LCNT_OFFSET        0x018   /* Standard speed I2C Clock SCL Low Count */
#define SG2002_IC_FS_SCL_HCN_OFFSET         0x01c   /* Fast speed I2C Clock SCL High Count */
#define SG2002_IC_FS_SCL_LCNT_OFFSET        0x020   /* Fast speed I2C Clock SCL Low Count */
#define SG2002_IC_INTR_STAT_OFFSET          0x02c   /* I2C Interrupt Status */
#define SG2002_IC_INTR_MASK_OFFSET          0x030   /* I2C Interrupt Mask */
#define SG2002_IC_RAW_INTR_STAT_OFFSET      0x034   /* I2C Raw Interrupt Stat */
#define SG2002_IC_RX_TL_OFFSET              0x038   /* I2C Receive FIFO Threshold */
#define SG2002_IC_TX_TL_OFFSET              0x03c   /* I2C Transmit FIFO Threshold */
#define SG2002_IC_CLR_INTR_OFFSET           0x040   /* Clear Combined and Individual Interrupts */
#define SG2002_IC_CLR_RX_UNDER_OFFSET       0x044   /* Clear RX_UNDER Interrupt */
#define SG2002_IC_CLR_RX_OVER_OFFSET        0x048   /* Clear RX_OVER Interrupt */
#define SG2002_IC_CLR_TX_OVER_OFFSET        0x04c   /* Clear TX_OVER Interrupt */
#define SG2002_IC_CLR_RD_REQ_OFFSET         0x050   /* Clear RD_REQ Interrupt */
#define SG2002_IC_CLR_TX_ABRT_OFFSET        0x054   /* Clear TX_ABRT Interrupt */
#define SG2002_IC_CLR_RX_DONE_OFFSET        0x058   /* Clear RX_DONE Interrupt */
#define SG2002_IC_CLR_ACTIVITY_OFFSET       0x05c   /* Clear ACTIVITY Interrupt */
#define SG2002_IC_CLR_STOP_DET_OFFSET       0x060   /* Clear STOP_DET Interrupt */
#define SG2002_IC_CLR_START_DET_OFFSET      0x064   /* Clear START_DET Interrupt */
#define SG2002_IC_CLR_GEN_CALL_OFFSET       0x068   /* Clear GEN_CALL Interrupt */
#define SG2002_IC_ENABLE_OFFSET             0x06c   /* I2C Enable */
#define SG2002_IC_STATUS_OFFSET             0x070   /* I2C Status register */
#define SG2002_IC_TXFLR_OFFSET              0x074   /* Transmit FIFO Level Register */
#define SG2002_IC_RXFLR_OFFSET              0x078   /* Receive FIFO Level Register */
#define SG2002_IC_SDA_HOLD_OFFSET           0x07c   /* SDA hold time length register */
#define SG2002_IC_TX_ABRT_SOURCE_OFFSET     0x080   /* I2C Transmit Abort Status Register */
#define SG2002_IC_SLV_DATA_NACK_ONLY_OFFSET 0x084   /* Generate SLV_DATA_NACK Register */
#define SG2002_IC_DMA_CR_OFFSET             0x088   /* DMA Control Register for transmit and receive handshaking interface */
#define SG2002_IC_DMA_TDLR_OFFSET           0x08c   /* DMA Transmit Data Level */
#define SG2002_IC_DMA_RDLR_OFFSET           0x090   /* DMA Receive Data Level */
#define SG2002_IC_SDA_SETUP_OFFSET          0x094   /* I2C SDA Setup Register */
#define SG2002_IC_ACK_GENERAL_CALL_OFFSET   0x098   /* I2C ACK General Call Register */
#define SG2002_IC_ENABLE_STATUS_OFFSET      0x09c   /* I2C Enable Status Regist */
#define SG2002_IC_FS_SPKLEN_OFFSET          0x0a0   /* ISS and FS spike suppression limit */
#define SG2002_IC_HS_SPKLEN_OFFSET          0x0a4   /* HS spike suppression limit */

typedef enum {
    SG2002_I2C_BUS_MODE_STANDARD    = 1,
    SG2002_I2C_BUS_MODE_FAST        = 2,
} SG2002_I2C_BUS_MODE_TypeDef;

typedef enum {
    SG2002_I2C_BUS_WRITE            = 0,
    SG2002_I2C_BUS_READ             = 1,
} SG2002_I2C_BUS_TR_TypeDef;

typedef union {
    struct {
        uint32_t master_mode            : 1;    /* bit 0 :      enable master mode */
        uint32_t speed                  : 2;    /* bit 2 ~ 1 : 
                                                                1 - standard mode
                                                                2 - fast     mode */

        uint32_t ic_10bitaddr_slave     : 1;    /* bit 3 :      enable 10bit slave  address mode */
        uint32_t ic_10bitaddr_master    : 1;    /* bit 4 :      enable 10bit master address mode */
        uint32_t ic_restart_en          : 1;    /* bit 5 :      enable I2C master to be able generate restart */
        uint32_t ic_slave_disable       : 1;    /* bit 6 :      
                                                                0 - slave is enabled
                                                                1 - slave is disabled */

        uint32_t reserved               : 25;   /* bit 7 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_CON_Reg_TypeDef;

typedef union {
    struct {
        uint32_t ic_tar                 : 10;   /* bit 0 ~ 9 :  i2c target address register */
        uint32_t gc_or_start            : 1;    /* bit 10 :     If bit SPECIAL is set to 1, then this bit indicates whether a General Call or START byte command
                                                                0: general call
                                                                1: start byte */
        uint32_t SPECIAL                : 1;    /* bit 11 :     Used to issue General Call or START BYTE*/
        uint32_t reserved               : 20;   /* bit 12 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_TAR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t ic_sar                 : 10;   /* bit 0 ~ 9 : i2c slave address register */
        uint32_t reserved               : 22;   /* bit 10 ~ 31*/
    } field;

    uint32_t val;
} SG2002_IC_SAR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t dat                    : 8;    /* bit 0 ~ 7 :   transmitted or received data port */
        uint32_t cmd                    : 1;    /* bit 8 :
                                                            0 - write
                                                            1 - read */
        uint32_t stop                   : 1;    /* bit 9 :      issue stop */
        uint32_t restart                : 1;    /* bit 10 :     issue restart */
        uint32_t reserved               : 21;   /* bit 11 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_DATA_CMD_Reg_TypeDef;

typedef union {
    struct {
        uint32_t ic_ss_scl_hcnt         : 16;   /* bit 0 ~ 15 : standard speed i2c clock high count register */
        uint32_t reserved               : 16;   /* bit 16 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_SS_SCL_HCNT_Reg_TypeDef;

typedef union {
    struct {
        uint32_t ic_ss_scl_lcnt         : 16;   /* bit 0 ~ 15 : standard speed i2c clock low count register */
        uint32_t reserved               : 16;   /* bit 16 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_SS_SCL_LCNT_Reg_TypeDef;

typedef union {
    struct {
        uint32_t ic_fs_scl_hcnt         : 16;   /* bit 0 ~ 15 : fast speed i2c clock scl high count register */
        uint32_t reserved               : 16;   /* bit 16 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_FS_SCL_HCNT_Reg_TypeDef;

typedef union {
    struct {
        uint32_t ic_fs_scl_lcnt         : 16;   /* bit 0 ~ 15 : fast speed i2c clock scl low count register */
        uint32_t reserved               : 16;   /* bit 16 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_FS_SCL_LCNT_Reg_TypeDef;

typedef union {
    struct {
        uint32_t ic_hs_scl_hcnt         : 16;   /* bit 0 ~ 15 : high speed i2c clock scl high count register */
        uint32_t reserved               : 16;   /* bit 16 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_HS_SCL_HCNT_Reg_TypeDef;

typedef union {
    struct {
        uint32_t ic_hs_scl_lcnt         : 16;   /* bit 0 ~ 15 : high speed i2c clock scl low count register */
        uint32_t reserved               : 16;   /* bit 16 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_HS_SCL_LCNT_Reg_TypeDef;

typedef union {
    struct {
        uint32_t r_rx_under             : 1;    /* bit 0 :  corresponding masked interrupt staus, please reference I2C Raw Interrupt Status */
        uint32_t r_rx_over              : 1;    /* bit 1 :  corresponding masked interrupt staus, please reference I2C Raw Interrupt Status */
        uint32_t r_rx_full              : 1;    /* bit 2 :  corresponding masked interrupt staus, please reference I2C Raw Interrupt Status */
        uint32_t r_tx_over              : 1;    /* bit 3 :  corresponding masked interrupt staus, please reference I2C Raw Interrupt Status */
        uint32_t r_tx_empty             : 1;    /* bit 4 :  corresponding masked interrupt staus, please reference I2C Raw Interrupt Status */
        uint32_t r_rd_req               : 1;    /* bit 5 :  corresponding masked interrupt staus, please reference I2C Raw Interrupt Status */
        uint32_t r_tx_abrt              : 1;    /* bit 6 :  corresponding masked interrupt staus, please reference I2C Raw Interrupt Status */
        uint32_t r_rx_done              : 1;    /* bit 7 :  corresponding masked interrupt staus, please reference I2C Raw Interrupt Status */
        uint32_t r_activity             : 1;    /* bit 8 :  corresponding masked interrupt staus, please reference I2C Raw Interrupt Status */
        uint32_t r_stop_det             : 1;    /* bit 9 :  corresponding masked interrupt staus, please reference I2C Raw Interrupt Status */
        uint32_t r_start_det            : 1;    /* bit 10 : corresponding masked interrupt staus, please reference I2C Raw Interrupt Status */
        uint32_t r_gen_call             : 1;    /* bit 11 : corresponding masked interrupt staus, please reference I2C Raw Interrupt Status */
        uint32_t reserved               : 20;   /* bit 12 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_INTR_STAT_Reg_TypeDef;  /* read only */

typedef union {
    struct {
        uint32_t m_rx_under             : 1;    /* bit 0 :  corresponding interrupt staus mask, please reference I2C Raw Interrupt Status */
        uint32_t m_rx_over              : 1;    /* bit 1 :  corresponding interrupt staus mask, please reference I2C Raw Interrupt Status */
        uint32_t m_rx_full              : 1;    /* bit 2 :  corresponding interrupt staus mask, please reference I2C Raw Interrupt Status */
        uint32_t m_tx_over              : 1;    /* bit 3 :  corresponding interrupt staus mask, please reference I2C Raw Interrupt Status */
        uint32_t m_tx_empty             : 1;    /* bit 4 :  corresponding interrupt staus mask, please reference I2C Raw Interrupt Status */
        uint32_t m_rd_req               : 1;    /* bit 5 :  corresponding interrupt staus mask, please reference I2C Raw Interrupt Status */
        uint32_t m_tx_abrt              : 1;    /* bit 6 :  corresponding interrupt staus mask, please reference I2C Raw Interrupt Status */
        uint32_t m_rx_done              : 1;    /* bit 7 :  corresponding interrupt staus mask, please reference I2C Raw Interrupt Status */
        uint32_t m_activity             : 1;    /* bit 8 :  corresponding interrupt staus mask, please reference I2C Raw Interrupt Status */
        uint32_t m_stop_det             : 1;    /* bit 9 :  corresponding interrupt staus mask, please reference I2C Raw Interrupt Status */
        uint32_t m_start_det            : 1;    /* bit 10 : corresponding interrupt staus mask, please reference I2C Raw Interrupt Status */
        uint32_t m_gen_call             : 1;    /* bit 11 : corresponding interrupt staus mask, please reference I2C Raw Interrupt Status */
        uint32_t reserved               : 20;   /* bit 12 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_INTR_MASK_Reg_TypeDef;

typedef union {
    struct {
        uint32_t ist_rx_under           : 1;    /* bit 0 :  when receive buffer is empty by reading from the IC_DATA_CMD register */
        uint32_t ist_rx_over            : 1;    /* bit 1 :  */
        uint32_t ist_rx_full            : 1;    /* bit 2 :  */
        uint32_t ist_tx_over            : 1;    /* bit 3 :  */
        uint32_t ist_tx_empty           : 1;    /* bit 4 :  */
        uint32_t ist_rd_req             : 1;    /* bit 5 :  */
        uint32_t ist_tx_abrt            : 1;    /* bit 6 :  */
        uint32_t ist_rx_done            : 1;    /* bit 7 :  */
        uint32_t ist_activity           : 1;    /* bit 8 :  */
        uint32_t ist_stop_det           : 1;    /* bit 9 :  */
        uint32_t ist_start_det          : 1;    /* bit 10 : */
        uint32_t ist_gen_call           : 1;    /* bit 11 : */
        uint32_t reserved               : 20;   /* bit 12 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_RAW_INTR_STAT_Reg_TypeDef;  /* read only */

typedef union {
    struct {
        uint32_t rx_tl                  : 8;    /* bit 0 ~ 7 : Receive FIFO Threshold Level */
        uint32_t reserved               : 24;   /* bit 8 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_RX_TL_Reg_TypeDef;

typedef union {
    struct {
        uint32_t tx_tl                  : 8;    /* bit 0 ~ 7 : Transmit FIFO Threshold Level */
        uint32_t reserved               : 24;   /* bit 8 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_TX_TL_Reg_TypeDef;

typedef union {
    struct {
        uint32_t clr_intr               : 1;    /* bit 0 : read to clear corresponding all raw staus */
        uint32_t reserved               : 31;   /* bit 1 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_CLR_INTR_Reg_TypeDef;       /* read only */

typedef union {
    struct {
        uint32_t clr_rx_under           : 1;    /* bit 0 : read to clear corresponding interupt raw staus, please reference I2C Raw Interrupt Status */
        uint32_t reserved               : 31;   /* bit 1 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_CLR_RX_UNDER_Reg_TypeDef;   /* read only */

typedef union {
    struct {
        uint32_t clr_rx_over            : 1;    /* bit 0 : read to clear corresponding interupt raw staus, please reference I2C Raw Interrupt Status */
        uint32_t reserved               : 31;   /* bit 1 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_CLR_RX_OVER_Reg_TypeDef;    /* read only */

typedef union {
    struct {
        uint32_t clr_tx_over            : 1;    /* bit 0 : read to clear corresponding interupt raw staus, please reference I2C Raw Interrupt Status */
        uint32_t reserved               : 31;   /* bit 1 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_CLR_TX_OVER_Reg_TypeDef;    /* read only */

typedef union {
    struct {
        uint32_t clr_rd_req             : 1;    /* bit 0 : read to clear corresponding interupt raw staus, please reference I2C Raw Interrupt Status */
        uint32_t reserved               : 31;   /* bit 1 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_CLR_RD_REQ_Reg_TypeDef;     /* read only */

typedef union {
    struct {
        uint32_t clr_tx_abrt            : 1;    /* bit 0 : read to clear corresponding interupt raw staus, please reference I2C Raw Interrupt Status */
        uint32_t reserved               : 31;   /* bit 1 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_CLR_TX_ABRT_Reg_TypeDef;    /* read only */

typedef union {
    struct {
        uint32_t clr_rx_done            : 1;    /* bit 0 : read to clear corresponding interupt raw staus, please reference I2C Raw Interrupt Status */
        uint32_t reserved               : 31;   /* bit 1 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_CLR_RX_DONE_Reg_TypeDef;    /* read only */

typedef union {
    struct {
        uint32_t clr_activity           : 1;    /* bit 0 : read to clear corresponding interupt raw staus, please reference I2C Raw Interrupt Status */
        uint32_t reserved               : 31;   /* bit 1 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_CLR_ACTIVITY_Reg_TypeDef;   /* read only */

typedef union {
    struct {
        uint32_t clr_stop_det           : 1;    /* bit 0 : read to clear corresponding interupt raw staus, please reference I2C Raw Interrupt Status */
        uint32_t reserved               : 31;   /* bit 1 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_CLR_STOP_DET_Reg_TypeDef;   /* read only */

typedef union {
    struct {
        uint32_t clr_start_det          : 1;    /* bit 0 : read to clear corresponding interupt raw staus, please reference I2C Raw Interrupt Status */
        uint32_t reserved               : 31;   /* bit 1 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_CLR_START_DET_Reg_TypeDef;  /* read only */

typedef union {
    struct {
        uint32_t clr_gen_call           : 1;    /* bit 0 : read to clear corresponding interupt raw staus, please reference I2C Raw Interrupt Status */
        uint32_t reserved               : 31;   /* bit 1 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_CLR_GEN_CALL_Reg_TypeDef;    /* read only */

typedef union {
    struct {
        uint32_t enable                 : 1;    /* bit 0 : Enables I2C controller */
        uint32_t reserved               : 31;   /* bit 1 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_ENABLE_Reg_TypeDef;

typedef union {
    struct {
        uint32_t st_activity            : 1;    /* bit 0 : I2C Activity Status */
        uint32_t st_tfnf                : 1;    /* bit 1 : Transmit FIFO Not Full */
        uint32_t st_tfe                 : 1;    /* bit 2 : Transmit FIFO Completely Empty */
        uint32_t st_rfne                : 1;    /* bit 3 : Receive FIFO Not Empty */
        uint32_t st_rff                 : 1;    /* bit 4 : Receive FIFO Completely Full */
        uint32_t st_mst_activity        : 1;    /* bit 5 : Master FSM Activity Status */
        uint32_t st_slv_activity        : 1;    /* bit 6 : Slave FSM Activity Status */
        uint32_t reserved               : 25;   /* bit 7 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_STATUS_Reg_TypeDef;         /* read only */

typedef union {
    struct {
        uint32_t txflr                  : 7;    /* bit 0 ~ 6 : I2C Transmit FIFO Level */
        uint32_t reserved               : 25;   /* bit 7 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_TXFLR_Reg_TypeDef;          /* read only */

typedef union {
    struct {
        uint32_t rxflr                  : 7;    /* bit 0 ~ 6 : I2C Receive FIFO Level Register */
        uint32_t reserved               : 25;   /* bit 7 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_RXFLR_Reg_TypeDef;          /* read only */

typedef union {
    struct {
        uint32_t ic_sda_hold            : 16;   /* bit 0 ~ 15 : Sets the required SDA hold time in units of IP clock */
        uint32_t reserved               : 16;   /* bit 16 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_SDA_HOLD_Reg_TypeDef;

typedef union {
    struct {
        uint32_t tx_abrt_source         : 16;   /* bit 0 ~ 15 : I2C Transmit Abort Source Register */
        uint32_t reserved               : 16;   /* bit 16 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_TX_ABRT_Reg_TypeDef;

typedef union {
    struct {
        uint32_t nack                   : 1;    /* bit 0 : generate a NACK in slave-receiver mode */
        uint32_t reserved               : 31;   /* bit 1 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_SLV_DATA_NACK_ONLY_Reg_TypeDef;

typedef union {
    struct {
        uint32_t rdmae                  : 1;    /* bit 0 : Rx DMA Enable */
        uint32_t tdmae                  : 1;    /* bit 1 : Tx DMA Enable */
        uint32_t reserved               : 30;   /* bit 2 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_DMA_CR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t dmatdl                 : 6;    /* bit 0 ~ 5 : the dma_tx_req signal is generated when the number of valid data entries in the transmit FIFO is equal to or below this field value */
        uint32_t reserved               : 26;   /* bit 6 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_DMA_TDLR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t dmardl                 : 6;    /* bit 0 ~ 5 : dma_rx_req is generated when the number of valid data entries in the receive FIFO is equal to or more than this field value + 1 */
        uint32_t reserved               : 26;   /* bit 6 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_DMA_RDLR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t sda_setup              : 1;    /* bit 0 : SDA Setup time config register */
        uint32_t reserved               : 31;   /* bit 1 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_SDA_SETUP_Reg_TypeDef;

typedef union {
    struct {
        uint32_t ack_gen_call           : 1;    /* bit 0 : 
                                                            1 - DW_apb_i2c responds with a ACK when it receives a General Call. 
                                                            0 - the IP does not generate General Call interrupts */
        uint32_t reserved               : 31;   /* bit 1 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_ACK_GENERAL_CALL_Reg_TypeDef;

typedef union {
    struct {
        uint32_t ic_en                  : 1;    /* bit 0 : I2C Enable Status Register */
        uint32_t slv_disabled_while_busy: 1;    /* bit 1 : Slave Disabled While Busy (Transmit, Receive) */
        uint32_t slv_rx_data_lost       : 1;    /* bit 2 : Slave Received Data Lost */
        uint32_t reserved               : 29;   /* bit 3 ~ 31 */
    } field;

    uint32_t val;
} SG2002_IC_ENABLE_STATUS_Reg_TypeDef;  /* read only */

typedef union {
    struct {
        uint32_t ic_fs_spklen           : 8;    /* bit 0 ~ 7 : I2C SS and FS Spike Suppression Limit Register */
        uint32_t reserved               : 24;   /* bit 8 ~ 32 */
    } field;

    uint32_t val;
} SG2002_IC_FS_SPKLEN_Reg_TypeDef;

typedef union {
    struct {
        uint32_t ic_hs_spklen           : 8;    /* bit 0 ~ 7 : I2C HS Spike Suppression Limit Register */
        uint32_t reserved               : 24;   /* bit 8 ~ 32 */
    } field;

    uint32_t val;
} SG2002_IC_HS_SPKLEN_Reg_TypeDef;



#endif

