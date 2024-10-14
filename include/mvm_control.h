/* SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __MVM_CONTROL_H
#define __MVM_CONTROL_H

enum mvm_ctrl_msg_type {
	MVM_DEBUG = 0,          /* Control message for debug framework */
	MVM_POLICY,             /* Control message for load balancing policy */
	MVM_CPU_FREQ,           /* Control message for DSVC */
	MVM_POWER,              /* Control message for power collapse */
	MVM_TRIGGER_SSR,        /* Control message to trigger SSR of MVM */
	MVM_MAX_RATE,           /* Control message to send max allowed rate */
	MVM_CHANGE_CLK_FREQ,    /* Control message to change the clock frequency */
	MVM_SET_P1_INT_MOD,     /* Control message to set p1_int_mod(number of message) */
	MVM_SET_P1_INT_DELAY,   /* Control message to set p1 msg interrupt delay */
	MVM_FW_VERSION,         /* Control message with MVM firmware version numbers */
	MVM_MAX,
};

enum mvm_log_policy {
	/* Flush log to DDR when APSS sends ctrl message - DMA local log buffer */
	MVM_FLUSH_DDR_ON_DEMAND = 0,
	MVM_FLUSH_DDR_PER_MSG,         /* Flush log to DDR as it arrives - DDR memory mapped */
	MVM_FLUSH_DDR_OVERFLOW,        /* Flush log to DDR when buffer is full */
	MVM_FLUSH_QDSS,                /* Flush log to QDSS */
	MVM_FLUSH_PERIODIC             /* Flush log to QDSS */
};

enum mvm_log_level {
	MVM_LOG_LEVEL_DEBUG = 0,
	MVM_LOG_LEVEL_INFO             /* Flush log info message */
};

enum mvm_balance_policy {
	MVM_PKE_ALL = 0,              /* Equally distribute jobs to all PKE's */
	MVM_PKE_1,                    /* Send all jobs to PKE 1 */
	MVM_PKE_1_2                   /* Equally distribute jobs between PKE 1 and 2 */
};

typedef enum {
	MVM_DEBUG_LOG_POLICY = 0,
	MVM_DEBUG_LOG_TRANSFER_REQUEST,
	MVM_DEBUG_LOG_TRANSFER_COMPLETE,
	MVM_DEBUG_SET_LOG_LEVEL
} mvm_debug_msg_type;

typedef enum {
	NOMINAL,
	SVS,
	LOW_SVS,
} mvm_clk_freq;

/*
 * struct mvm_debug
 * size = 4*5 = 20 bytes
 */
struct mvm_debug {
	union transfer_msg {
		enum mvm_log_policy log_policy;   /* Logging policy */
		enum mvm_log_level log_level;     /* Logging policy */
		uint32_t active_buffer_index;     /* Micro_ulog buffer index */
		uint32_t ddr_log_buf_addr;        /* DDR address to push logs to */									
		uint32_t flush_period;            /* Time in msecs to flush logs to DDR when log_policy == MVM_FLUSH_PERIODIC */
	} transfer_msg;
	uint32_t ddr_buf_len;                 /* Length of buffer in DDR */
	mvm_debug_msg_type  msg_type;
	uint32_t num_bytes_transferred;
} __attribute__((packed));

/*
 * struct mvm_policy
 * size = 4*1 = 4 bytes
 */
struct mvm_policy {
	enum mvm_balance_policy balance;      /* Load balancing policy to use */
};

struct mvm_cpu_freq {
	uint32_t freq;
};

struct mvm_power {
	uint8_t enter_pwr_collapse;
};

struct mvm_log_transfer {
	bool is_transfer;
};

struct mvm_trigger_ssr {
	bool trigger_ssr;
};

struct mvm_max_rate {
        uint32_t max_rate;
};

struct mvm_clk_freq {
	mvm_clk_freq clk_freq;
};

struct mvm_p1_int_mod {
	uint32_t new_p1_int_mod;
};

struct mvm_p1_int_delay {
	uint32_t new_p1_int_delay;
};

struct mvm_fw_version {
	uint8_t major;
	uint8_t minor;
	uint8_t patch;
};

/*
 * struct mvm_control
 * Max Size possible = 240 bytes
 * size = 4+20 = 24 bytes
 */
struct mvm_control {
	enum mvm_ctrl_msg_type type;
	union mvm_ctrl_msg {
		struct mvm_debug debug;
		struct mvm_policy policy;
		struct mvm_cpu_freq cpu;
		struct mvm_power power;
		struct mvm_trigger_ssr ssr;
		struct mvm_max_rate max_rate;
		struct mvm_clk_freq mvm_clk_freq;
		struct mvm_p1_int_mod p1_int_mod;
		struct mvm_p1_int_delay p1_int_delay;
		struct mvm_fw_version fw_version;
	} mvm_ctrl_msg;
} __attribute__((packed));

#endif /* __MVM_CONTROL_H */
