/* SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __MVM_CONTROL_H
#define __MVM_CONTROL_H

enum mvm_ctrl_msg_type {
	MVM_DEBUG = 0,       /* Control message for debug framework */
	MVM_POLICY,          /* Control message for load balancing policy */
	MVM_CPU_FREQ,        /* Control message for DSVC */
	MVM_POWER,           /* Control message for power collapse */
	MVM_MAX
};

enum mvm_log_policy {
	/* Flush log to DDR when APSS sends ctrl message - DMA local log buffer */
	MVM_FLUSH_DDR_ON_DEMAND = 0,
	MVM_FLUSH_DDR_PER_MSG,       /* Flush log to DDR as it arrives - DDR memory mapped */
	MVM_FLUSH_DDR_OVERFLOW,       /* Flush log to DDR when buffer is full */
	MVM_FLUSH_QDSS                /* Flush log to QDSS */
};

enum mvm_balance_policy {
	MVM_PKE_ALL = 0,              /* Equally distribute jobs to all PKE's */
	MVM_PKE_1,                    /* Send all jobs to PKE 1 */
	MVM_PKE_1_2                   /* Equally distribute jobs between PKE 1 and 2 */
};

/*
 * struct mvm_debug
 * size = 4*5 = 20 bytes
 */
struct mvm_debug {
	enum mvm_log_policy log_policy;       /* Logging policy */
	uint32_t ddr_log_buf_addr_high;  /* DDR address to push logs to - higher word */
	uint32_t ddr_log_buf_addr_low;   /* DDR address to push logs to - lower word */
	uint32_t ddr_buf_len;            /* Length of buffer in DDR */
	/* Time in msecs to flush logs to DDR when log_policy == MVM_FLUSH_PERIODIC */
	uint32_t flush_period;
};

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
	} mvm_ctrl_msg;
};

#endif /* __MVM_CONTROL_H */
