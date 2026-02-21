#ifndef __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_SPI_H
#define __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_SPI_H

#define SG2002_SPI_BUS_NUM                      1

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


#endif

