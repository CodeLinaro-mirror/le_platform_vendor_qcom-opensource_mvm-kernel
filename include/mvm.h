// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef MVM_H_
#define MVM_H_
/* Input msg data len is 5 * 384 bits = 1920 / 32 = 60 words = 240 bytes*/
#define INP_MSG_DATA_LEN                240
/* Output msg data len is 2 * 384 bits = 768 / 32 = 24 words = 96 bytes*/
#define VER_RES_DATA_LEN                96

struct input_msg {
	uint32_t client_id      :4;
	uint32_t msg_id         :12;
	uint32_t opcode         :3;
	uint32_t priority       :1;
	uint32_t interrupt      :1;
	uint32_t curve_id       :3;
	uint32_t msg_count      :4;
	uint32_t reserved       :4;
	uint8_t data[INP_MSG_DATA_LEN];
};

struct output_msg {
	uint32_t client_id:4;
	uint32_t msg_id:12;
	uint32_t opcode:3;
	uint32_t result:4;
	uint32_t err_status:9;
	uint8_t data[VER_RES_DATA_LEN];
};

#define MVM_DEV_NODE      	"/dev/mvm"
#define GET_CLIENT_ID     	100
#define SET_TIMEOUT_MS    	101
#define READY_AFTER_SSR   	102
#define GET_MVM_CAPACITY  	103
#define GET_MVM_STATS_MSG_COUNT	104
#define GET_MVM_STATE		105
#define CTL_MSG_OPCODE    	0x02

#endif
