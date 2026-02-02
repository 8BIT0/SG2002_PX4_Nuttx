#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/init.h>
#include <nuttx/power/pm.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "chip.h"
#include "riscv_internal.h"
#include "hardware/sg2002_mmio.h"

#define SG2002_UART_TIMEOUT                 200

#define SG2002_UART_SIM_BASE                0x10000000
#define SG2002_UART1_BASE                   0x04150000
#define SG2002_UART2_BASE                   0x04160000

#define SG2002_UART_LCR_WLS_MSK             0x03 /* character length select mask */
#define SG2002_UART_LCR_WLS_5               0x00 /* 5 bit character length */
#define SG2002_UART_LCR_WLS_6               0x01 /* 6 bit character length */
#define SG2002_UART_LCR_WLS_7               0x02 /* 7 bit character length */
#define SG2002_UART_LCR_WLS_8               0x03 /* 8 bit character length */
#define SG2002_UART_LCR_STB                 0x04 /* # stop Bits, off=1, on=1.5 or 2) */
#define SG2002_UART_LCR_PEN                 0x08 /* Parity eneble */
#define SG2002_UART_LCR_EPS                 0x10 /* Even Parity Select */
#define SG2002_UART_LCR_STKP                0x20 /* Stick Parity */
#define SG2002_UART_LCR_SBRK                0x40 /* Set Break */
#define SG2002_UART_LCR_BKSE                0x80 /* Bank select enable */
#define SG2002_UART_LCR_DLAB                0x80 /* Divisor latch access bit */

#define SG2002_UART_MCR_DTR                 0x01 /* DTR   */
#define SG2002_UART_MCR_RTS                 0x02 /* RTS   */

#define SG2002_UART_LSR_THRE                0x20 /* Transmit-hold-register empty */
#define SG2002_UART_LSR_DR                  0x01 /* Receiver data ready */
#define SG2002_UART_LSR_TEMT                0x40 /* Xmitter empty */

#define SG2002_UART_FCR_FIFO_EN             0x01 /* Fifo enable */
#define SG2002_UART_FCR_RXSR                0x02 /* Receiver soft reset */
#define SG2002_UART_FCR_TXSR                0x04 /* Transmitter soft reset */

#define SG2002_UART_IRQ_RECEIVER_DATA_AVAILABLE 0x04
#define SG2002_UART_IRQ_THR_EMPTY               0x02

#define SG2002_UART_MCRVAL                  (SG2002_UART_MCR_DTR | SG2002_UART_MCR_RTS) /* RTS/DTR */
#define SG2002_UART_FCR_DEFVAL              (SG2002_UART_FCR_FIFO_EN | SG2002_UART_FCR_RXSR | SG2002_UART_FCR_TXSR)
#define SG2002_UART_FCR_RST                 (SG2002_UART_FCR_RXSR | SG2002_UART_FCR_TXSR)
#define SG2002_UART_LCR_8N1                 0x03

#ifdef CONFIG_SG2002_QEMU_SIM
#define reg_type    uint8_t
#define reg_val     uint8_t
#else
#define reg_type    uint32_t
#define reg_val     uint32_t
#endif

#define thr rbr
#define iir fcr
#define dll rbr
#define dlm ier

struct dw_regs {
	volatile reg_type rbr;          /* 0x00 Data register */
	volatile reg_type ier;          /* 0x04 Interrupt Enable Register */
	volatile reg_type fcr;          /* 0x08 FIFO Control Register */
	volatile reg_type lcr;          /* 0x0C Line control register */
	volatile reg_type mcr;          /* 0x10 Modem control register */
	volatile reg_type lsr;          /* 0x14 Line Status Register */
	volatile reg_type msr;          /* 0x18 Modem Status Register */
	volatile reg_type lpdll;        /* 0x20 Low Power Divisor Latch Low Register */
#ifndef CONFIG_SG2002_QEMU_SIM
    volatile reg_type lpdlh;        /* 0x24 Low Power Divisor Latch High Register */
    volatile reg_type res_1[2];
    volatile reg_type srbr_sthr;    /* 0x30 Shadow Receive/Transmit Buffer Register */
    volatile reg_type res_2[4];
    volatile reg_type far;          /* 0x70 FIFO Access Register */
    volatile reg_type tfr;          /* 0x74 Transmit FIFO Read Register */
    volatile reg_type rfw;          /* 0x78 Receive FIFO Write Register */
    volatile reg_type usr;          /* 0x7C UART Status Register */
    volatile reg_type tfl;          /* 0x80 Transmit FIFO Level Register */
    volatile reg_type rfl;          /* 0x84 Receive FIFO Level Register */
    volatile reg_type srr;          /* 0x88 Software Reset Register */
    volatile reg_type srts;         /* 0x8C Shadow Request To Send Register */
    volatile reg_type sbcr;         /* 0x90 Shadow Break Control Register */
    volatile reg_type sdmam;        /* 0x94 Shadow DMA Mode Register */
    volatile reg_type sfe;          /* 0x98 Shadow FIFO Enable Register */
    volatile reg_type srt;          /* 0x9C Shadow RCVR Trigger Register */
    volatile reg_type stet;         /* 0xA0 Shadow TX Empty Trigger Register */
    volatile reg_type htx;          /* 0xA4 Halt TX Register */
    volatile reg_type dmasa;        /* 0xA8 DMA Software Acknowladge */
#endif
};

typedef struct {
    struct      uart_dev_s dev;
    uint32_t    uartbase;
    uint32_t	clock;
    uint32_t    baud;
    uint32_t 	irq;
    uint8_t     parity;
    uint8_t     bits;
    bool        stopbits;
    int8_t      port_id;
} SG2002_Uart_TypeDef;

static inline bool sg2002_check_uart_base(uint32_t base);
static int sg2002_interrupt(int irq, void *context, void *arg);

static bool dummy9(struct uart_dev_s *dev);
static int sg2002_attach(struct uart_dev_s *dev);
static void sg2002_detach(struct uart_dev_s *dev);
static int sg2002_uart_setup(struct uart_dev_s *dev);
static void sg2002_send(struct uart_dev_s *dev,  int ch);
static void sg2002_txint(struct uart_dev_s *dev, bool enable);
static void sg2002_rxint(struct uart_dev_s *dev, bool enable);
static int sg2002_receive(struct uart_dev_s *dev, uint32_t *status);
static bool sg2002_rxavailable(struct uart_dev_s *dev);
static inline void sg2002_disableuartint(struct uart_dev_s *dev);
static int sg2002_ioctl(struct file *filep, int cmd, unsigned long arg);
static bool sg2002_txready(struct uart_dev_s *dev);

static const struct uart_ops_s g_uart_ops = {
    .setup          = sg2002_uart_setup,
    .shutdown       = sg2002_disableuartint,
    .attach         = sg2002_attach,
    .detach         = sg2002_detach,
    .ioctl          = sg2002_ioctl,
    .receive        = sg2002_receive,
    .rxint          = sg2002_rxint,
    .rxavailable    = sg2002_rxavailable,
    .send           = sg2002_send,
    .txint          = sg2002_txint,
    .txready        = sg2002_txready,
    .txempty        = dummy9,
};

#ifdef CONFIG_DEV_CONSOLE
#define CONFIG_CONSOLE_RXBUFSIZE 2048
#define CONFIG_CONSOLE_TXBUFSIZE 2048

static char g_console_rxbuffer[CONFIG_CONSOLE_RXBUFSIZE] = {0};
static char g_console_txbuffer[CONFIG_CONSOLE_TXBUFSIZE] = {0};

static SG2002_Uart_TypeDef Console_Dev = {
    .dev = {
        .recv = {
            .size       = CONFIG_CONSOLE_RXBUFSIZE,
            .buffer     = g_console_rxbuffer,
        },
        .xmit = {
            .size       = CONFIG_CONSOLE_TXBUFSIZE,
            .buffer     = g_console_txbuffer,
        },
        .ops            = &g_uart_ops,
    },
    
#ifdef CONFIG_SG2002_QEMU_SIM
    .uartbase           = SG2002_UART_SIM_BASE,
    .irq                = SG2002_IRQ_SIM_BASE,
#else
    .uartbase           = SG2002_UART1_BASE,
    .irq                = SG2002_IRQ_UART1_BASE,
    .port_id            = sg2002_pinmux_uart1,
#endif
    .clock              = 24883200,
    .baud               = 115200,
    .parity             = 0,    // unused
    .bits               = 0,    // unused
    .stopbits           = 0,    // unused
};
#endif

static inline bool sg2002_check_uart_base(uint32_t base) {
#ifdef CONFIG_SG2002_QEMU_SIM
    return true;
#endif

    if ((base == SG2002_UART1_BASE) || \
        (base == SG2002_UART2_BASE))
        return true;

    return false;
}

static inline void sg2002_disableuartint(struct uart_dev_s *dev) {
    irqstate_t  flags = 0;
    volatile struct dw_regs *uart = NULL;
	SG2002_Uart_TypeDef *priv = (SG2002_Uart_TypeDef *)dev;

    if ((priv == NULL) || !sg2002_check_uart_base(priv->uartbase))
        return;

    uart = (volatile struct dw_regs *)((uintptr_t)(priv->uartbase));

    flags = enter_critical_section();
    uart->lcr &= ~(1 << 7);
    uart->ier = 0;
    leave_critical_section(flags);
}

static int sg2002_attach(struct uart_dev_s *dev) {
	SG2002_Uart_TypeDef *priv = (SG2002_Uart_TypeDef *)dev;
    int ret = OK;

    ret = irq_attach(priv->irq, sg2002_interrupt, dev);
    if (ret == OK)
    {
        up_enable_irq(priv->irq);
    }

    return ret;
}

static void sg2002_detach(struct uart_dev_s *dev) {
    SG2002_Uart_TypeDef *priv = (SG2002_Uart_TypeDef *)dev;

    up_disable_irq(priv->irq);
    irq_detach(priv->irq);
}

static int sg2002_uart_setup(struct uart_dev_s *dev) {
	SG2002_Uart_TypeDef *priv = (SG2002_Uart_TypeDef *)dev;
    volatile struct dw_regs *uart = NULL;

    if ((priv == NULL) || !sg2002_check_uart_base(priv->uartbase)) {
        return -1;
    }

#ifndef CONFIG_SG2002_QEMU_SIM
    /* pin config */
    sg2002_pinmux_config((sg2002_pinmux_list)(priv->port_id));
#endif

    int divisor = (priv->clock + 8 * priv->baud) / (16 * priv->baud);
    
    uart = (volatile struct dw_regs *)((uintptr_t)(priv->uartbase));

	uart->lcr = uart->lcr | SG2002_UART_LCR_DLAB | SG2002_UART_LCR_8N1;
	uart->dll = divisor & 0xff;
	uart->dlm = (divisor >> 8) & 0xff;
	uart->lcr = uart->lcr & (~SG2002_UART_LCR_DLAB);

    uart->ier = 0;
	uart->mcr = SG2002_UART_MCRVAL;
	uart->fcr = SG2002_UART_FCR_DEFVAL;

	uart->lcr = 3;

    return 0;
}

static bool sg2002_txready(struct uart_dev_s *dev) {
    SG2002_Uart_TypeDef *priv = (SG2002_Uart_TypeDef *)dev;
    volatile struct dw_regs *uart = NULL;

    if ((priv == NULL) || (priv->uartbase == 0))
        return false;

    uart = (volatile struct dw_regs *)((uintptr_t)(priv->uartbase));
    return (uart->lsr & SG2002_UART_LSR_THRE);
}

static void sg2002_rxint(struct uart_dev_s *dev, bool enable) {
    SG2002_Uart_TypeDef *priv = (SG2002_Uart_TypeDef *)dev;
    volatile struct dw_regs *uart = NULL;
    irqstate_t flags;

    if ((priv == NULL) || !sg2002_check_uart_base(priv->uartbase))
        return;
    
    uart = (volatile struct dw_regs *)((uintptr_t)(priv->uartbase));

    flags = enter_critical_section();
    if (enable)
    {
        /* Enable the RX interrupt */
        uart->ier |= (1 << 0);
    }
    else
    {
        /* Disable the RX interrupt */
        uart->ier &= ~(1 << 0);
    }

    leave_critical_section(flags);
}

static void sg2002_txint(struct uart_dev_s *dev, bool enable) {
    SG2002_Uart_TypeDef *priv = (SG2002_Uart_TypeDef *)dev;
    volatile struct dw_regs *uart = NULL;
    irqstate_t flags;

    if ((priv == NULL) || !sg2002_check_uart_base(priv->uartbase))
        return;
    
    uart = (volatile struct dw_regs *)((uintptr_t)(priv->uartbase));

    flags = enter_critical_section();
    if (enable) {
        /* Enable the TX interrupt */
        uart->ier |= (1 << 1);

        uart_xmitchars(dev);
    }
    else {
        /* Disable the TX interrupt */
        uart->ier &= ~(1 << 1);
    }

    leave_critical_section(flags);
}

static void sg2002_send(struct uart_dev_s *dev, int ch) {
    SG2002_Uart_TypeDef *priv = (SG2002_Uart_TypeDef *)dev;
    volatile struct dw_regs *uart = NULL;

    if ((priv == NULL) || !sg2002_check_uart_base(priv->uartbase))
        return;

    uart = (volatile struct dw_regs *)((uintptr_t)(priv->uartbase));
    uart->rbr = ch;
}

static int sg2002_receive(struct uart_dev_s *dev, uint32_t *status) {
    SG2002_Uart_TypeDef *priv = (SG2002_Uart_TypeDef *)dev;
    volatile struct dw_regs *uart = NULL;
    uint32_t data = 0;

    if ((priv == NULL) || !sg2002_check_uart_base(priv->uartbase) || (status == NULL))
        return 0;

    *status = -1;
    uart = (volatile struct dw_regs *)((uintptr_t)(priv->uartbase));
    
    if (uart->lsr & 0x01) {
        data = uart->rbr;
        *status = 0;
        return data;
    }

    return 0;
}

static bool sg2002_rxavailable(struct uart_dev_s *dev) {
    SG2002_Uart_TypeDef *priv = (SG2002_Uart_TypeDef *)dev;
    volatile struct dw_regs *uart = NULL;

    if ((priv == NULL) || !sg2002_check_uart_base(priv->uartbase))
        return false;

    uart = (volatile struct dw_regs *)((uintptr_t)(priv->uartbase));

    if (uart->lsr & SG2002_UART_LSR_DR)
        return true;

    return false;
}

static int sg2002_interrupt(int irq, void *context, void *arg) {
    struct uart_dev_s *dev = (struct uart_dev_s *)arg;
    SG2002_Uart_TypeDef *priv = (SG2002_Uart_TypeDef *)dev;
    volatile struct dw_regs *uart = NULL;
    uint32_t irq_id = 0;

    if ((priv == NULL) || !sg2002_check_uart_base(priv->uartbase))
        return ERROR;

    uart = (volatile struct dw_regs *)((uintptr_t)(priv->uartbase));
    irq_id = uart->fcr & 0x0F;

    switch (irq_id) {
        case SG2002_UART_IRQ_RECEIVER_DATA_AVAILABLE: uart_recvchars(dev); break;
        case SG2002_UART_IRQ_THR_EMPTY: uart_xmitchars(dev); break;
        default: break;
    }

    return OK;
}

static int sg2002_ioctl(struct file *filep, int cmd, unsigned long arg) {
    if ((filep == NULL) || \
        (filep->f_inode == NULL) || \
        (filep->f_inode->i_private == NULL))
        return -1;

    switch (cmd)
    {
        case TCGETS:
        {
            struct termios *termiosp = (struct termios *)arg;
            if (termiosp == NULL)
                return -EINVAL;
            
            /* uart 8bit */
            termiosp->c_cflag = CS8;
            cfsetispeed(termiosp, 115200);
        }
        break;

        case TCSETS: up_putc('S'); break;
        default: return ENOTTY;
    }

    return OK;
}

void riscv_earlyserialinit(void) {
#ifdef CONFIG_DEV_CONSOLE
    Console_Dev.dev.isconsole = true;
    sg2002_uart_setup(&Console_Dev.dev);
#endif
}

void riscv_serialinit(void) {
  /* Register the console */
#ifdef CONFIG_DEV_CONSOLE
    uart_register("/dev/console", &Console_Dev.dev);
#endif
}

int up_putc(int ch) {
#ifdef CONFIG_DEV_CONSOLE
    SG2002_Uart_TypeDef *priv = (SG2002_Uart_TypeDef *)&Console_Dev.dev;
    volatile struct dw_regs *uart = NULL;

    if ((priv == NULL) || !sg2002_check_uart_base(priv->uartbase))
        return -1;

    uart = (volatile struct dw_regs *)((uintptr_t)(priv->uartbase));
    
    while (!(uart->lsr & SG2002_UART_LSR_THRE));
    uart->rbr = ch;

    return 0;
#endif
}

static bool dummy9(struct uart_dev_s *dev) {
    up_putc('9');
    return false;
}



