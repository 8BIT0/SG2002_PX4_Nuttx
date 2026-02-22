#ifndef __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_SPI_H
#define __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_SPI_H

#define SG2002_SPI_BUS_NUM                      1

#define SG2002_SPI_1_BASE                       0x04190000ul
#define SG2002_SPI_2_BASE                       0x041A0000ul

#define SG2002_SPI_CTRLR0_REG_OFFSET            0x000 /* Control Register 0 */
#define SG2002_SPI_CTRLR1_REG_OFFSET            0x004 /* Control Register 1 */
#define SG2002_SPI_SPIENR_REG_OFFSET            0x008 /* SPI Enable Register */
#define SG2002_SPI_MWCR_REG_OFFSET              0x00c /* Microwire Control Register */
#define SG2002_SPI_SER_REG_OFFSET               0x010 /* Slave Enable Register */
#define SG2002_SPI_BAUDR_REG_OFFSET             0x014 /* Baud Rate Select */
#define SG2002_SPI_TXFTLR_REG_OFFSET            0x018 /* Transmit FIFO Threshold Level */
#define SG2002_SPI_RXFTLR_REG_OFFSET            0x01c /* Receive FIFO Threshold Level */
#define SG2002_SPI_TXFLR_REG_OFFSET             0x020 /* Transmit FIFO Level Register */
#define SG2002_SPI_RXFLR_REG_OFFSET             0x024 /* Receive FIFO Level Register */
#define SG2002_SPI_SR_REG_OFFSET                0x028 /* Status Register */
#define SG2002_SPI_IMR_REG_OFFSET               0x02c /* Interrupt Mask Register */
#define SG2002_SPI_ISR_REG_OFFSET               0x030 /* Interrupt Status Register */
#define SG2002_SPI_RISR_REG_OFFSET              0x034 /* Raw Interrupt Status Register */
#define SG2002_SPI_TXOICR_REG_OFFSET            0x038 /* Transmit FIFO Overflow Interrupt Clear Register */
#define SG2002_SPI_RXOICR_REG_OFFSET            0x03c /* Receive FIFO Overflow Interrupt Clear Register */
#define SG2002_SPI_RXUICR_REG_OFFSET            0x040 /* Receive FIFO Underflow Interrupt Clear Register */
#define SG2002_SPI_MSTICR_REG_OFFSET            0x044 /* Multi-Master Interrupt Clear Register */
#define SG2002_SPI_ICR_REG_OFFSET               0x048 /* Interrupt Clear Register */
#define SG2002_SPI_DMACR_REG_OFFSET             0x04c /* DMA Control Register */
#define SG2002_SPI_DMATDLR_REG_OFFSET           0x050 /* DMA Transmit Data Level */
#define SG2002_SPI_DMARDLR_REG_OFFSET           0x054 /* DMA Receive Data Level */
#define SG2002_SPI_DATA_REG_OFFSET              0x060 /* (36 groups) Data Register */
#define SG2002_SPI_RX_SAMPLE_DLY_REG_OFFSET     0x0f0 /* Rx Sample Delay Register */

typedef enum {
    SG2002_SPI_1 = 0,
    SG2002_SPI_2,
} SG2002_SPI_List_TypeDef;

typedef union {
    struct {
        uint32_t data_frame_size : 4;       /* bit 3 ~ 0
                                               Selects the data frame length. When the data frame size is programmed to be less than 16 bits,
                                               the receive data are automatically right-justified by the receive logic, with the upper bits of the receive FIFO zero-padded. You must right-justify
                                               transmit data before writing into the transmit FIFO. The transmit logic ignores the upper unused bits when transmitting the data. */
                                            /* 0000 Reserved - undefined operation */
                                            /* 0001 Reserved - undefined operation */
                                            /* 0010 Reserved - undefined operation */
                                            /* 0011 4-bit serial data transfer */
                                            /* 0100 5-bit serial data transfer */
                                            /* 0101 6-bit serial data transfer */
                                            /* 0110 7-bit serial data transfer */
                                            /* 0111 8-bit serial data transfer */
                                            /* 1000 9-bit serial data transfer */
                                            /* 1001 10-bit serial data transfer */
                                            /* 1010 11-bit serial data transfer */
                                            /* 1011 12-bit serial data transfer */
                                            /* 1100 13-bit serial data transfer */
                                            /* 1101 14-bit serial data transfer */
                                            /* 1110 15-bit serial data transfer */
                                            /* 1111 16-bit serial data transfer */

        uint32_t frame_format : 2;          /* bit 5 ~ 4
                                               Selects which serial protocol transfers the data. */
                                            /* 00 - Motorola SPI */
                                            /* 01 - Texas Instruments SSP */
                                            /* 10 - National Semiconductors Microwire */
                                            /* 11 - Reserved */

        uint32_t serial_clock_phase : 1;    /* bit 6
                                               Valid when the frame format (FRF) is set to Motorola SPI. The serial clock phase selects the relationship of the serial clock with the slave select signal.
                                               When SCPH = 0, data are captured on the first edge of the serial clock. When SCPH = 1, the serial clock starts toggling one cycle after
                                               the slave select line is activated, and data are captured on the second edge of the serial clock. */
                                            /* 0: Serial clock toggles in middle of first data bit */
                                            /* 1: Serial clock toggles at start of first data bit */

        uint32_t serial_lcock_polarity : 1; /* bit 7
                                               Valid when the frame format (FRF) is set to Motorola SPI. 
                                               Used to select the polarity of the inactive serial clock, which is held inactive when the 
                                               SPI master is not actively transferring data on the serial bus. */
                                            /* 0 - Inactive state of serial clock is low */
                                            /* 1 - Inactive state of serial clock is high */

        uint32_t transfer_mode : 2;         /* bit 9 ~ 8
                                               Selects the mode of transfer for serial communication. This field does not affect the transfer
                                               duplicity. Only indicates whether the receive or transmit data are valid.
                                               In transmit-only mode, data received from the external device is not valid and is not stored in
                                               the receive FIFO memory; it is overwritten on the next transfer.
                                               In receive-only mode, transmitted data are not valid. After the first write to the transmit FIFO,
                                               the same word is retransmitted for the duration of the transfer.
                                               In transmit-and-receive mode, both transmit and receive data are valid. The transfer continues until the transmit FIFO is empty. Data received
                                               from the external device are stored into the receive FIFO memory, where it can be accessed by the host processor.
                                               In eeprom-read mode, receive data is not valid while control data is being transmitted.
                                               When all control data is sent to the EEPROM, receive data becomes valid and transmit data becomes invalid. All data in the transmit FIFO is
                                               considered control data in this mode. */
                                            /* 00 - Transmit & Receive */
                                            /* 01 - Transmit Only */
                                            /* 10 - Receive Only */
                                            /* 11 - EEPROM Re */

        uint32_t slave_mode_bit : 1;        /* bit 10
                                              only for slave mode */

        uint32_t shift_register_loop : 1;   /* bit 11
                                               Used for testing purposes only. When internally
                                               active, connects the transmit shift register output
                                               to the receive shift register input. Can be used in
                                               both serialslave and serial-master modes. */
                                            /* 0 - Normal Mode Operation */
                                            /* 1 - Test Mode Operation */

        uint32_t control_frame_size : 3;    /* bit 15 ~ 12
                                               Selects the length of the control word for the Microwire frame format. */
                                            /* 0000 1-bit control word */
                                            /* 0001 2-bit control word */
                                            /* 0010 3-bit control word */
                                            /* 0011 4-bit control word */
                                            /* 0100 5-bit control word */
                                            /* 0101 6-bit control word */
                                            /* 0110 7-bit control word */
                                            /* 0111 8-bit control word */
                                            /* 1000 9-bit control word */
                                            /* 1001 10-bit control word */
                                            /* 1010 11-bit control word */
                                            /* 1011 12-bit control word */
                                            /* 1100 13-bit control word */
                                            /* 1101 14-bit control word */
                                            /* 1110 15-bit control word */
                                            /* 1111 16-bit control word */

        uint32_t reserved : 16;             /* bit 31 ~ 16
                                               reserved */
    } field;

    uint32_t val;
} SG2002_SPI_CTRLR0_Reg_TypeDef;

typedef union {
    struct {
        uint32_t ctrlr1 : 16;               /* bit 15 ~ 0
                                               Number of Data Frames.
                                               When TMOD = 10 or TMOD = 11, this register field sets the number of data frames to be continuously received by the SPI.
                                               The SPI continues to receive serial data until the number of data frames received is equal to this register value plus 1,
                                               which enables you to receive up to 64 KB of data in a continuous transfer. */
        uint32_t reserved : 16;
    } field;

    uint32_t val;
} SG2002_SPI_CTRLR1_Reg_TypeDef;

typedef union {
    struct {
        uint32_t spienr : 1;
        uint32_t reserved : 31;
    } field;

    uint32_t val;
} SG2002_SPI_SPIENR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t transfer_mode : 1;         /* bit 0
                                               Microwire Transfer Mode
                                               Defines whether the Microwire transfer is sequential or non-sequential. When sequential mode is used,
                                               only one control word is needed to transmit or receive a block of data words. When nonsequential mode is used,
                                               there must be a control word for each data word that is transmitted or received. */
                                            /* 0 –non-sequential transfer */
                                            /* 1 –sequential transfer */

        uint32_t control : 1;               /* bit 1
                                               Microwire Control
                                               Defines the direction of the data word when the Microwire serial protocol is used.
                                               When this bit is set to 0, the data word is received by the SPI from the external serial device.
                                               When this bit is set to 1, the data word is transmitted from the SPI to the external serial device. */

        uint32_t handshaking : 1;           /* bit 2
                                               Microwire Handshaking.
                                               Relevant only when the SPI is configured as a serial-master device. When configured as a serial slave,
                                               this bit field has no functionality.
                                               Used to enable and disable the “busy/ready” handshaking interface for the Microwire protocol.
                                               When enabled, the SPI checks for a ready status from the target slave, after the transfer of the last
                                               data/control bit, before clearing the BUSY status in the SR register */ 
                                               
        uint32_t reserved : 29;
    } field;

    uint32_t val;
} SG2002_SPI_MWCR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t ser : 1;                   /* bit 0
                                               Slave Select Enable Flag
                                               This register corresponds to a slave select line (ss_x_n]) from the SPI master.
                                               When this register is set (1), the slave select line from the master is activated when a serial transfer begins.
                                               It should be noted that setting or clearing this register have no effect on the corresponding slave select outputs until a transfer is started.
                                               Before beginning a transfer, you should enable this register that corresponds to the slave device with which the master wants to communicate. */
                                            /* 1: Selected */
                                            /* 0: Not Selected */
        uint32_t reserved : 31;
    } field;

    uint32_t val;
} SG2002_SPI_SER_Reg_TypeDef;

typedef union {
    struct {
        uint32_t baudr : 16;                /* bit 15 ~ 0
                                               SPI Clock Divider(SCKDV).
                                               The LSB for this field is always set to 0 and is unaffected by a write operation,
                                               which ensures an even value is held in this register.
                                               If the value is 0, the serial output clock (sclk_out) is disabled.
                                               The frequency of the sclk_out is derived from the following equation:
                                               Fsclk_out/SCKDV = Fssi_clk
                                               where SCKDV is any even value between 2 and 65534.
                                               For example:
                                               for Fssi_clk = 3.6864MHz and SCKDV =2
                                               Fsclk_out = 3.6864/2 = 1.8432MHz */
        uint32_t reserved : 16;
    } field;

    uint32_t val;
} SG2002_SPI_BAUDR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t txftlr : 3;                /* bit 2 ~ 0
                                               Transmit FIFO Threshold.
                                               Controls the level of entries (or below) at which the transmit FIFO controller triggers an interrupt.
                                               The FIFO depth is 8; If you attempt to set this value greater than or equal to the depth of the FIFO,
                                               this field is not written and retains its current value.
                                               When the number of transmit FIFO entries is less than or equal to this value,
                                               the transmit FIFO empty interrupt is triggered. */
        uint32_t reserved : 29;
    } field;

    uint32_t val;
} SG2002_SPI_TXFTLR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t rxftlr : 3;                /* bit 2 ~ 0
                                               Receive FIFO Threshold.
                                               Controls the level of entries (or above) at which the receive FIFO controller triggers an interrupt.
                                               The FIFO depth is 8.
                                               If you attempt to set this value greater than the depth of the FIFO,
                                               this field is not written and retains its current value.
                                               When the number of receive FIFO entries is greater than or equal to this value + 1,
                                               the receive FIFO full interrupt is triggered. */
        uint32_t reserved : 29;
    } field;

    uint32_t val;
} SG2002_SPI_RXFTLR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t txflr : 4;                 /* bit 3 ~ 0
                                               Transmit FIFO Level.
                                               Contains the number of valid data entries in the transmit FIFO. */
        uint32_t reserved : 28;
    } field;

    uint32_t val;
} SG2002_SPI_TXFLR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t rxflr : 4;                 /* bit 3 ~ 0
                                               Receive FIFO Level.
                                               Contains the number of valid data entries in the receive FIFO. */
        uint32_t reserved : 28;
    } field;

    uint32_t val;
} SG2002_SPI_RXFLR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t busy : 1;                      /* bit 0
                                                   When set, indicates that a serial transfer is in progress;
                                                   when cleared indicates that the SPI is idle or disabled. */
                                                /* 0 - SPI is idle or disabled */
                                                /* 1 - SPI is actively transferring data */

        uint32_t transmit_fifo_not_empty : 1;   /* bit 1
                                                   Set when the transmit FIFO contains one or more empty locations, and is cleared when the FIFO is full.
                                                /* 0 - Transmit FIFO is full */
                                                /* 1 - Transmit FIFO is not full */

        uint32_t transmit_fifo_empty : 1;       /* bit 2
                                                   When the transmit FIFO is completely empty, this bit is set.
                                                   When the transmit FIFO contains one or more valid entries, this bit is cleared.
                                                   This bit field does not request an interrupt. */
                                                /* 0 - Transmit FIFO is not empty */
                                                /* 1 - Transmit FIFO is empty */

        uint32_t receive_fifo_not_empty : 1;    /* bit 3
                                                   Set when the receive FIFO contains one or more entries and is cleared when the receive FIFO is empty.
                                                   This bit can be polled by software to completely empty the receive FIFO.
                                                /* 0 - Receive FIFO is empty */
                                                /* 1 - Receive FIFO is not empty */

        uint32_t receive_fifo_full : 1;         /* bit 4
                                                   When the receive FIFO is completely full, this bit is set.
                                                   When the receive FIFO contains one or more empty location, this bit is cleared. */
                                                /* 0 - Receive FIFO is not full */
                                                /* 1 - Receive FIFO is full */

        uint32_t transmission_error : 1;        /* bit 5
                                                   Set if the transmit FIFO is empty when a transfer is started.
                                                   Data from the previous transmission is resent on the txd line. This bit is cleared when read.
                                                /* 0 - No error */
                                                /* 1 - Transmission error */

        uint32_t data_collision_error : 1;      /* bit 6
                                                   This bit is set if the SPI master is actively transmitting when another master selects this device as a slave.
                                                   This informs the processor that the last data transfer was halted before completion.
                                                   This bit is cleared when read. */
                                                /* 0 - No error */
                                                /* 1 - Transmit data collision error */

        uint32_t reserved : 25;
    } field;

    uint32_t val;
} SG2002_SPI_SR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t transmit_fifo_empty_int_mask : 1;      /* bit 0
                                                           Transmit FIFO Empty Interrupt Mask */
                                                        /* 0 - spi_txe_intr interrupt is masked */
                                                        /* 1 - spi_txe_intr interrupt is not masked */

        uint32_t transmit_fifo_overflow_int_mask : 1;   /* bit 1
                                                           Transmit FIFO Overflow Interrupt Mask */
                                                        /* 0 - spi_txo_intr interrupt is masked */
                                                        /* 1 - spi_txo_intr interrupt is not masked */

        uint32_t receive_fifo_underflow_int_mask : 1;   /* bit 2
                                                           Receive FIFO Underflow Interrupt Mask */
                                                        /* 0 - spi_rxu_intr interrupt is masked */
                                                        /* 1 - spi_rxu_intr interrupt is not masked */

        uint32_t receive_fifo_overflow_int_mask : 1;    /* bit 3
                                                           Receive FIFO Overflow Interrupt Mask */
                                                        /* 0 - spi_rxo_intr interrupt is masked */
                                                        /* 1 - spi_rxo_intr interrupt is not masked */

        uint32_t receive_fifo_full_int_mask : 1;        /* bit 4
                                                           Receive FIFO Full Interrupt Mask */
                                                        /* 0 - spi_rxf_intr interrupt is masked */
                                                        /* 1 - spi_rxf_intr interrupt is not masked */

        uint32_t multi_master : 1;                      /* bit 5
                                                           Multi-Master Contention Interrupt Mask. */
                                                        /* 0 - spi_mst_intr interrupt is masked */
                                                        /* 1 - spi_mst_intr interrupt is not masked */

        uint32_t reserved : 26;
    } field;

    uint32_t val;
} SG2002_SPI_IMR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t transmit_fifo_empty_int_status : 1;        /* bit 0
                                                               Transmit FIFO Empty Interrupt Status */
                                                            /* 0 = spi_txe_intr interrupt is not active after masking */
                                                            /* 1 = spi_txe_intr interrupt is active after masking */

        uint32_t transmit_fifo_overflow_int_status : 1;     /* bit 1
                                                               Transmit FIFO Overflow Interrupt Status */
                                                            /* 0 = spi_txo_intr interrupt is not active after masking */
                                                            /* 1 = spi_txo_intr interrupt is active after masking */

        uint32_t receive_fifo_underflow_int_status : 1;     /* bit 2 
                                                               Receive FIFO Underflow Interrupt Status */
                                                            /* 0 = spi_rxu_intr interrupt is not active after masking */
                                                            /* 1 = spi_rxu_intr interrupt is active after masking */

        uint32_t receive_fifo_overflow_int_status : 1;      /* bit 3 
                                                               Receive FIFO Overflow Interrupt Status */
                                                            /* 0 = spi_rxo_intr interrupt is not active after masking */
                                                            /* 1 = spi_rxo_intr interrupt is active after masking */

        uint32_t receive_fifo_full_int_status : 1;          /* bit 4 
                                                               Receive FIFO Full Interrupt Status
                                                            /* 0 = spi_rxf_intr interrupt is not active after masking */
                                                            /* 1 = spi_rxf_intr interrupt is full after masking*/

        uint32_t multi_master_contention_int_status : 1;    /* bit 5
                                                               Multi-Master Contention Interrupt Status.
                                                            /* 0 = spi_mst_intr interrupt not active after masking */
                                                            /* 1 = spi_mst_intr interrupt is active after masking */

        uint32_t reserved : 26;
    } field;

    uint32_t val;
} SG2002_SPI_ISR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t transmit_fifo_empty_raw_int_status : 1;    /* bit 0
                                                               Transmit FIFO Empty Raw Interrupt Status
                                                            /* 0 = spi_txe_intr interrupt is not active prior to masking */
                                                            /* 1 = spi_txe_intr interrupt is active prior masking */

        uint32_t transmit_fifo_overflow_raw_int_status : 1; /* bit 1 
                                                               Transmit FIFO Overflow Raw Interrupt Status */
                                                            /* 0 = spi_txo_intr interrupt is not active prior to masking */
                                                            /* 1 = spi_txo_intr interrupt is active prior masking */

        uint32_t receive_fifo_underflow_raw_int_status : 1; /* bit 2
                                                               Receive FIFO Underflow Raw Interrupt Status */
                                                            /* 0 = spi_rxu_intr interrupt is not active prior to masking */
                                                            /* 1 = spi_rxu_intr interrupt is active prior to masking */

        uint32_t receive_fifo_overflow_raw_int_status : 1;  /* bit 3
                                                               Receive FIFO Overflow Raw Interrupt Status
                                                            /* 0 = spi_rxo_intr interrupt is not active prior to masking */
                                                            /* 1 = spi_rxo_intr interrupt is active prior masking */

        uint32_t transmit_fifo_empty_int_status : 1;        /* bit 4
                                                               Transmit FIFO Empty Interrupt Status */
                                                            /* 0 = spi_txe_intr interrupt is not active after masking */
                                                            /* 1 = spi_txe_intr interrupt is active after masking */

        uint32_t transmit_fifo_empty_int_status : 1;        /* bit 5
                                                               Transmit FIFO Empty Interrupt Status */
                                                            /* 0 = spi_txe_intr interrupt is not active after masking */
                                                            /* 1 = spi_txe_intr interrupt is active after masking */

        uint32_t reserved : 26;
    } field;

    uint32_t val;
} SG2002_SPI_RISR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t txoicr : 1;                            /* bit 0
                                                           Clear Transmit FIFO Overflow Interrupt.
                                                           This register reflects the status of the interrupt. 
                                                           A read from this register clears the spi_txo_intr interrupt;
                                                           writing has no effect. */

        uint32_t reserved : 31;
    } field;

    uint32_t val;
} SG2002_SPI_TXOICR_Reg_TypeDef; /* read only */

typedef union {
    struct {
        uint32_t rxoicr : 1;                            /* bit 0
                                                           Clear Receive FIFO Overflow Interrupt.
                                                           This register reflects the status of the interrupt.
                                                           A read from this register clears the spi_rxo_intr interrupt;
                                                           writing has no effect. */

        uint32_t reserved : 31;
    } field;

    uint32_t val;
} SG2002_SPI_RXOICR_Reg_TypeDef; /* read only */

typedef union {
    struct {
        uint32_t roicr : 1;                             /* bit 0
                                                           Clear Receive FIFO Overflow Interrupt.
                                                           This register reflects the status of the interrupt.
                                                           A read from this register clears the spi_rxo_intr interrupt;
                                                           writing has no effect. */

        uint32_t reserved : 31;
    } field;

    uint32_t val;
} SG2002_SPI_RXOICR_Reg_TypeDef; /* read only */

typedef union {
    struct {
        uint32_t rxuicr : 1;                            /* bit 0
                                                           Clear Receive FIFO Underflow Interrupt.
                                                           This register reflects the status of the interrupt.
                                                           A read from this register clears the spi_rxu_intr interrupt;
                                                           writing has no effect. */

        uint32_t reserved : 31;
    } field;

    uint32_t val;
} SG2002_SPI_RXUICR_Reg_TypeDef; /* read only */

typedef union {
    struct {
        uint32_t msticr : 1;                            /* bit 0
                                                           Clear Multi-Master Contention Interrupt.
                                                           This register reflects the status of the interrupt.
                                                           A read from this register clears the spi_mst_intr interrupt;
                                                           writing has no effect. */

        uint32_t reserved : 31;
    } field;

    uint32_t val;
} SG2002_SPI_MSTICR_Reg_TypeDef; /* read only */

typedef union {
    struct {
        uint32_t icr : 1;                               /* bit 0 
                                                           Clear Interrupts.
                                                           This register is set if any of the interrupts below are active.
                                                           A read clears the spi_txo_intr, spi_rxu_intr, spi_rxo_intr, and the spi_mst_intr interrupts.
                                                           Writing to this register has no effect. */

        uint32_t reserved : 31;
    } field;

    uint32_t val;
} SG2002_SPI_ICR_Reg_TypeDef; /* read only */

typedef union {
    struct {
        uint32_t receive_dma_en : 1;                    /* bit 0
                                                           Receive DMA Enable.
                                                           This bit enables/disables the receive FIFO DMA channel */
                                                        /* 0 = Receive DMA disabled */
                                                        /* 1 = Receive DMA enabled */

        uint32_t transmit_dma_en : 1;                   /* bit 1
                                                           Transmit DMA Enable.
                                                           This bit enables/disables the transmit FIFO DMA channel.
                                                        /* 0 = Transmit DMA disabled */
                                                        /* 1 = Transmit DMA enabled */

        uint32_t reserved : 30;
    } field;

    uint32_t val;
} SG2002_DMACR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t dmatdlr : 3;                           /* bit 2 ~ 0
                                                           Transmit Data Level.
                                                           This bit field controls the level at which a DMA request is made by the transmit logic.
                                                           It is equal to the watermark level;
                                                           that is, the dma_tx_req signal is generated when the number of valid data entries in the transmit FIFO is equal to or below
                                                           this field value, and TDMAE = 1. */

        uint32_t reserved : 29;
    } field;

    uint32_t val;
} SG2002_DMATDLR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t dmardlr : 3;                           /* bit 2 ~ 0
                                                           Receive Data Level.
                                                           This bit field controls the level at which a DMA request is made by the receive logic.
                                                           The watermark level = DMARDL+1; that is, dma_rx_req is generated when the number of valid data entries
                                                           in the receive FIFO is equal to or above this field value + 1, and RDMAE=1. */

        uint32_t reserved : 29;
    } field;

    uint32_t val;
} SG2002_SPI_DMARDLR_Reg_TypeDef;

typedef union {
    struct {
        uint32_t data : 16;                             /* bit 15 ~ 0 
                                                           Data Register.
                                                           When writing to this register, you must rightjustify the data.
                                                           Read data are automatically right-justified.
                                                           Read = Receive FIFO buffer
                                                           Write = Transmit FIFO buffer */

        uint32_t reserved : 16;
    } field;
    
    uint32_t val;
} SG2002_SPI_Data_Reg_TypeDef;

typedef union {
    struct {
        uint32_t rx_sample_dly : 8;                     /* bit 7 ~ 0 
                                                           Receive Data (rxd) Sample Delay.
                                                           This register is used to delay the sample of the rxd input signal.
                                                           Each value represents a single ssi_clk delay on the sample of the rxd signal.
                                                           NOTE: If this register is programmed with a value that exceeds the depth of the internal shift
                                                           registers (DEPTH = 8), a zero (0) delay will be applied to the rxd sample. */

        uint32_t reserved : 24;
    } field;

    uint32_t val;
} SG2002_SPI_RxSampleDly_Reg_TypeDef;

#endif

