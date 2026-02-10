#pragma once
#ifndef __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_MAILBOX_H
#define __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_MAILBOX_H

#define SG2002_MAILBOX_MAX_NUM          8

#define SG2002_MAILBOX_REG_ADDR         0x01900000ul
#define SG2002_MAILBOX_DONE_REG_ADDR    0x01900002ul
#define SG2002_MAILBOX_CONTEXT_ADDR	    0x01900400ul

#define SG2002_RECEIVE_CPU              2 /* c906L */
#define SG2002_SEND_TO_CPU              1 /* c906B */

#define SG2002_LINUX_VALID_VALID		1

#define H26X_BITSTREAM_ADDR 			0x89500000  /* offset 149.0MiB */
#define H26X_BITSTREAM_SIZE 			0x200000  	/* 2.0MiB */
#define H26X_ENC_BUFF_ADDR 				0x89700000  /* offset 151.0MiB */
#define H26X_ENC_BUFF_SIZE 				0x0  		/* 0.0KiB */
#define ISP_MEM_BASE_ADDR 				0x89700000  /* offset 151.0MiB */
#define ISP_MEM_BASE_SIZE 				0x1400000  	/* 20.0MiB */

typedef enum {
	MCU_STATUS_NONOS_INIT = 1,
	MCU_STATUS_NONOS_RUNNING,
	MCU_STATUS_NONOS_DONE,
	MCU_STATUS_RTOS_T1_INIT,  // before linux running
	MCU_STATUS_RTOS_T1_RUNNING,
	MCU_STATUS_RTOS_T2_INIT,  // after linux running
	MCU_STATUS_RTOS_T2_RUNNING,
	MCU_STATUS_LINUX_INIT,
	MCU_STATUS_LINUX_RUNNING,
} SG2002_McuStats_TypeDef;

typedef enum {
	SYS_CMD_INFO_TRANS = 0x50,
	SYS_CMD_INFO_LINUX_INIT_DONE,
	SYS_CMD_INFO_RTOS_INIT_DONE,
	SYS_CMD_INFO_STOP_ISR,
	SYS_CMD_INFO_STOP_ISR_DONE,
	SYS_CMD_INFO_LINUX,
	SYS_CMD_INFO_RTOS,
	SYS_CMD_SYNC_TIME,
	SYS_CMD_INFO_DUMP_MSG,
	SYS_CMD_INFO_DUMP_EN,
	SYS_CMD_INFO_DUMP_DIS,
	SYS_CMD_INFO_TRACE_SNAPSHOT_START,
	SYS_CMD_INFO_TRACE_SNAPSHOT_STOP,
	SYS_CMD_INFO_TRACE_STREAM_START,
	SYS_CMD_INFO_TRACE_STREAM_STOP,
	SYS_CMD_INFO_LIMIT,
} SG2002_Mailbox_SysCMDID_TypeDef;

typedef union {
	uint8_t _info;
	uint32_t val;
} SG2002_MailboxInfo_TypeDef;

typedef union {
	uint8_t _clr;
	uint32_t val;
} SG2002_MailboxIntClr_TypeDef;

typedef union {
	uint8_t _mask;
	uint32_t val;
} SG2002_MailboxIntMask_TypeDef;

typedef union {
	uint8_t _int;
	uint32_t val;
} SG2002_MailboxInt_TypeDef;

typedef union {
	uint8_t _int_raw;
	uint32_t val;
} SG2002_MailboxIntRaw_TypeDef;

typedef union {
	uint8_t _set;
	uint32_t val;
} SG2002_MailboxSet_TypeDef;

typedef union {
	uint8_t _status;
	uint32_t val;
} SG2002_MailboxStatus_TypeDef;

typedef struct {
	SG2002_MailboxIntClr_TypeDef    mb_clr;
	SG2002_MailboxIntMask_TypeDef   mb_mask;
	SG2002_MailboxInt_TypeDef       mb_int;
	SG2002_MailboxIntRaw_TypeDef    mb_raw;
} SG2002_CPU_MailboxInt_TypeDef;

typedef struct {
	SG2002_MailboxInfo_TypeDef      cpu_mbox_en[4];     /* [ 0x00 - 0x0c ] */
	SG2002_CPU_MailboxInt_TypeDef   cpu_mbox_set[4];    /* [ 0x10 - 0x4C ] */
	uint32_t                        reserved[4];        /* [ 0x50 - 0x5C ] */
	SG2002_MailboxSet_TypeDef       mbox_set;           /* [ 0x60 ]        */
	SG2002_MailboxStatus_TypeDef    mbox_status;        /* [ 0x64 ]        */
	uint32_t                        reserved2[2];       /* [ 0x68 - 0x6C ] */
	SG2002_MailboxStatus_TypeDef    cpu_mbox_status[4]; /* [ 0x70 ]        */
} SG2002_MailboxSet_Reg_TypeDef; 

typedef struct {
	SG2002_MailboxInfo_TypeDef      cpu_mbox_done_en[4];
	SG2002_CPU_MailboxInt_TypeDef   cpu_mbox_done[4];
} SG2002_MailboxSetDone_Reg_TypeDef;

typedef union {
    struct {
        uint8_t linux_valid;
        uint8_t rtos_valid;
    } valid;
	uint16_t ms; // 0 : noblock, -1 : block infinite
} __attribute__((packed)) SG2002_Resv_TypeDef;

typedef struct {
	uint8_t             ip_id;
	uint8_t             cmd_id : 7;
	uint8_t             block : 1;
	SG2002_Resv_TypeDef resv;
	uint32_t            param_ptr;
} __attribute__((packed)) __attribute__((aligned(0x8))) SG2002_CMDQU_TypeDef;

typedef struct {
	uint32_t conf_magic;
	uint32_t conf_size;  //conf_size exclude mcu_status & linux_status
	uint32_t isp_buffer_addr;
	uint32_t isp_buffer_size;
	uint32_t encode_img_addr;
	uint32_t encode_img_size;
	uint32_t encode_buf_addr;
	uint32_t encode_buf_size;
	uint8_t  dump_print_enable;
	uint8_t  dump_print_size_idx;
	uint16_t image_type;
	uint16_t checksum; // checksum exclude mcu_status & linux_status
	uint8_t  mcu_status;
	uint8_t  linux_status;
} __attribute__((packed)) __attribute__((aligned(0x40))) transfer_config_t;

#endif
