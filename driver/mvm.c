// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "mvm.h"
#include <linux/completion.h>
#include <linux/jiffies.h>
#include <linux/iopoll.h>
#include "mvm_control.h"
#include <linux/qcom_scm.h>
#include <linux/soc/qcom/mdt_loader.h>
#include <linux/firmware.h>
#include <linux/of_address.h>
#include <linux/sysfs.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/iommu.h>
#include <soc/qcom/boot_stats.h>

#define SIG_MVM_STATE           SIGRTMAX
#define DDR_FIFO_COUNT			2
#define DDR_FIFO_SIZE			128
#define MSG_PRIORITY_BIT		BIT(19)
#define OUT_BUFF_SIZE			32
#define TIMEOUT_MS			10000
#define MAX_CLIENT_COUNT		15
/*TODO Remove this once PIL validation is complete */
#define KEEP_FW_IN_DDR			1

/* CSR to enable WFI interrupt from E21 to APPS */
#define MVMSS_CSR_RVSS_CFG_OFFSET	0x02000010
#define MVMSS_CSR_WFI_EN_MASK		0xFFFFFFFB
#define MVMSS_CSR_WFI_EN_SHIFT		2

#define MVMSS_CSR_MVMSS_APSS		0x02000028
#define IRQ_APSS0			0x00100000
#define IRQ_APSS1                       0x00200000
#define IRQ_APSS2                       0x00400000
#define IRQ_APSS3                       0x00800000

#define MVMSS_CSR_INPUT_RING0_BASE_ADDR		0x02001000
#define MVMSS_CSR_INPUT_RING0_BUFFER_LENGTH	0x02001004
#define MVMSS_CSR_OUTPUT_RING0_BASE_ADDR	0x02001008
#define MVMSS_CSR_OUTPUT_RING0_BUFFER_LENGTH	0x0200100C
#define MVMSS_CSR_INPUT_RING1_BASE_ADDR		0x02003000
#define MVMSS_CSR_INPUT_RING1_BUFFER_LENGTH	0x02003004
#define MVMSS_CSR_OUTPUT_RING1_BASE_ADDR	0x02003008
#define MVMSS_CSR_OUTPUT_RING1_BUFFER_LENGTH	0x0200300C
#define MVMSS_CSR_INPUT_RING0_TAIL_PTR_OFFSET	0x02002000
#define MVMSS_CSR_INPUT_RING0_HEAD_PTR_OFFSET	0x02007000
#define MVMSS_CSR_INPUT_RING1_TAIL_PTR_OFFSET	0x02004000
#define MVMSS_CSR_INPUT_RING1_HEAD_PTR_OFFSET	0x02008000

#define MVMSS_CSR_OUTPUT_RING0_TAIL_PTR_OFFSET	0x02007004
#define MVMSS_CSR_OUTPUT_RING0_HEAD_PTR_OFFSET	0x02002004
#define MVMSS_CSR_OUTPUT_RING1_TAIL_PTR_OFFSET	0x02008004
#define MVMSS_CSR_OUTPUT_RING1_HEAD_PTR_OFFSET	0x02004004

#define MVMSS_CSR_APSS_MVM_SCRATCH_PAD0 	0x02005000
#define MVMSS_CSR_APSS_MVM_SCRATCH_PAD1 	0x02005004
#define MVM_INIT_DONE_COOKIE			0xc0deba5e
#define SW_COLLAPSE_MASK			0xFFFFFFFE
#define MVM_CC_MVM_GDSCR_OFFSET			0x02028004
#define MVM_CC_AHB_CORE_CBCR_OFFSET     	0x02028120
#define MVM_CC_E21CPU_CC_CBCR_OFFSET       	0x02028030
#define CLK_EN_MASK				0xFFFFFFFE

/* MVMSS CSRs */

/* CSR to enable WFI interrupt from E21 to APPS */
#define MVMSS_CSR_RVSS_CFG_OFFSET       0x02000010
#define MVMSS_CSR_RVSS_STS_OFFSET	0x02000014
#define MVMSS_CSR_WFI_EN_MASK           0xFFFFFFFB
#define MVMSS_CSR_WFI_EN_SHIFT          2

/* MVM_CC CSRs for mvm core power collapse/restore */

#define CLK_EN_MASK                     0xFFFFFFFE
#define MVM_CC_MVM_GDSCR_OFFSET         0x02028004
#define RETAIN_FF_ENABLE_MASK           0xFFFFF7FF
#define RETAIN_FF_ENABLE_SHIFT          11
#define SW_COLLAPSE_MASK                0xFFFFFFFE
#define GDSC_PWR_DOWN_COMPLETE          15
#define GDSC_PWR_UP_COMPLETE            16
#define MVM_CC_MVM_CFG_GDSCR_OFFSET     0x02028008
#define MVM_CC_AHB_CORE_CBCR_OFFSET     0x02028120

/* MVM_CC CSRs to switch RCGs to XO */
#define MVM_CC_E21CPU_CFG_RCGR_OFFSET   0x0202801C
#define MVM_CC_SRC_DIV_DISABLE          0xFFFFF8E0
#define MVM_CC_PKE_CFG_RCGR_BASE_OFFSET 0x02028068
#define MVM_CC_PKEn_CFG_RCGR_OFFSET(pke)        MVM_CC_PKE_CFG_RCGR_BASE_OFFSET + (24 * pke)
#define MVM_CC_BUS_CFG_RCGR_OFFSET      0x020280D4
#define MVM_CC_SLEEP_CFG_RCGR_OFFSET    0x02028128
#define MVM_CC_XO_CFG_RCGR_OFFSET       0x02028144

#define MVM_CC_AHB_CORE_CBCR_OFFSET     0x02028120

#define GDSC_POWER_SLEEP_US             10
#define GDSC_POWER_TIMEOUT_US           1000000
#define MVM_ULOG_BUFFER_SEGMENT         4
#define MVM_ULOG_BUFFER_SIZE            2048
#define MVM_DUMP_COLL_TIMEOUT_MS        3000
#define MVM_PROC_ID                     0x2B
#define MVM_CRASH_DUMP_SIZE		0x100F0
#define MVM_FW_SIZE			0x10000
#define RING_BUFF_IOVA			0x40000000
#define MVM_DUMP_BUFF_IOVA		0x50000000
#define LOG_BUFF_IOVA			0x60000000
/* The offset in the ring buffer structures from where
each of the actual P0 and P1 buffer starts */
#define BASE_ADDR_OFFSET		0x10
#define MAX_LOG_BUFFER_SIZE		0x1800

/**
 * enum mvm_state - state of mvm subsystem
 * @MVM_OFFLINE: MVM firmware is not loaded/authenticated yet.
 * @MVM_ONLINE: The MVM firmware has been loaded, authenticate and MVMSS is up and running.
 * @MVM_CRASHED: Watchdog bite is received from MVM to APSS.
 * @MVM_RESTARTING: The mvm dump has been collected and ssr_done_irq is received from MVM to APSS.
 * @MVM_SLEEP: The gdsc core collapse sequence has been completed from APSS.
**/

enum mvm_state {
	MVM_OFFLINE,
	MVM_ONLINE,
	MVM_CRASHED,
	MVM_RESTARTING,
	MVM_SLEEP,
};

static const char * const mvm_states[] = {
	[MVM_OFFLINE] = "OFFLINE",
	[MVM_ONLINE] = "ONLINE",
	[MVM_CRASHED] = "CRASHED",
	[MVM_RESTARTING] = "RESTARTING",
	[MVM_SLEEP] = "SLEEP",
};

struct input_fifo {
	unsigned int head;
	unsigned int tail;
	unsigned int count;
	unsigned int size;
	struct input_msg base[128];
};

struct output_fifo {
	unsigned int head;
	unsigned int tail;
	unsigned int count;
	unsigned int size;
	struct output_msg base[128];
};

struct output_buffer {
	unsigned int head;
	unsigned int tail;
	unsigned int count;
	unsigned int size;
	struct output_msg out_msg[OUT_BUFF_SIZE];
};

struct ring_buffers {
	struct input_fifo in_fifo[DDR_FIFO_COUNT];
	struct output_fifo out_fifo[DDR_FIFO_COUNT];
};

struct mvm_crashdump_buffer {
	uint32_t mvm_dump_buffer[MVM_CRASH_DUMP_SIZE/4];
};

struct mvmlog_buffers {
	uint32_t mvmlog_buffer[MVM_ULOG_BUFFER_SIZE];
};

struct mvm_client {
	unsigned int client_id;
	unsigned int timeout_ms;
	struct mvm_device *mvm_dev;
	struct list_head list;
	bool client_ready;
	struct output_buffer *out_buff;
	enum mvm_log_policy log_policy;
	struct task_struct *task;
};

struct mvm_device {
	bool resume_frm_pwr_collapse;
	DECLARE_BITMAP(client_id_bitmap, MAX_CLIENT_COUNT);
	int mvm_ssr_done_irq;
	int mvm_ssr_done_hw_irq;
	int mvm_verif_done_irq;
	int mvm_verif_done_hw_irq;
	int wfi_irq;
	int wdog_irq;
	unsigned int incoming_msgs;
	unsigned int outgoing_results;
	dev_t mvm_cdev_devid;
	void __iomem *mvm_base;
	void __iomem *apss_shared_base;
	struct list_head client_list;
	struct cdev mvm_cdev;
	struct device *dev;
	struct class *mvm_class;
	struct mutex mvm_cli_lock;
	struct mutex mvm_csr_lock;
	struct mutex in_fifo_lock;
	struct mutex out_fifo_lock;
	wait_queue_head_t mvm_waitqueue;
	wait_queue_head_t log_poll_wait;
	wait_queue_head_t ssr_poll_wait;
	struct work_struct drain_out_fifo_work;
	struct work_struct trigger_ssr_work;
	struct completion p0_fifo_slot_available;
	struct completion p1_fifo_slot_available;
	struct completion mvm_wfi_irq_recvd;
	struct completion mvm_dump_collection_done;
	struct kobject *kobj;
	struct kobj_attribute attr;
	struct dentry *dir;
	uint32_t *mvm_fw;
	struct mvm_crashdump_buffer *dump_buff;
	struct ring_buffers *ring_buff;
	dma_addr_t ring_buff_dma;
	struct mvmlog_buffers *log_buff;
	dma_addr_t mvm_fw_dma;
	dma_addr_t mvm_dump_dma;
	struct clk *xo;
	struct clk *cnoc_s_ahb_clk;
	struct clk *snoc_m_axi_clk;
	struct clk *sysnoc_mvmss_clk;
	enum mvm_state state;
	dma_addr_t mvmlog_buff_dma;
	uint32_t *ddr_head_pos;
	uint32_t *ddr_tail_pos;
	uint32_t *ddr_current_addr;
	int active_buffer_index;
	uint32_t ddr_buf_len;
	uint32_t filled_dma_bytes;
	bool pending_dump_read;
	struct iommu_domain *domain;
};

static int enable_gcc_clocks(struct mvm_device *mvm_dev);
int qcom_pil_info_store(const char *image, phys_addr_t base, size_t size);

static void send_mvm_state_to_user(struct mvm_device *mvm_dev)
{
	struct kernel_siginfo info;
	struct mvm_client *mvm_cli;

	memset(&info, 0, sizeof(struct kernel_siginfo));
	info.si_signo = SIG_MVM_STATE;
	info.si_int = mvm_dev->state;

	mutex_lock(&mvm_dev->mvm_cli_lock);
	list_for_each_entry(mvm_cli, &mvm_dev->client_list, list) {
		if (mvm_cli->task != NULL) {
			if(send_sig_info(SIG_MVM_STATE, &info, mvm_cli->task) < 0)
				dev_err(mvm_dev->dev, "Unable to send mvm state change signal to userspace\n");
			dev_dbg(mvm_dev->dev, "Sent state change signal to client %d\n",mvm_cli->client_id);
		}
	}
	mutex_unlock(&mvm_dev->mvm_cli_lock);
}

static void enable_wfi_int(struct mvm_device *mvm_dev, bool enable)
{

	unsigned int reg_val, val;

	val = enable ? 1 : 0;
	reg_val = readl_relaxed(mvm_dev->mvm_base + MVMSS_CSR_RVSS_CFG_OFFSET);
	val <<= MVMSS_CSR_WFI_EN_SHIFT;
	writel_relaxed((reg_val & MVMSS_CSR_WFI_EN_MASK) | val,
			mvm_dev->mvm_base + MVMSS_CSR_RVSS_CFG_OFFSET);
}

static int enable_mvm_gdsc(struct mvm_device *mvm_dev, bool powerup)
{
	uint32_t reg_val, val;
	int ret = 0;

	val = powerup ? 0 : 1;
	reg_val = readl_relaxed(mvm_dev->mvm_base + MVM_CC_MVM_GDSCR_OFFSET);
	writel_relaxed((reg_val & SW_COLLAPSE_MASK)| val,
			mvm_dev->mvm_base + MVM_CC_MVM_GDSCR_OFFSET);

	reg_val = readl_relaxed(mvm_dev->mvm_base + MVM_CC_MVM_CFG_GDSCR_OFFSET);
	if (powerup) {
		ret = readl_poll_timeout(mvm_dev->mvm_base + MVM_CC_MVM_CFG_GDSCR_OFFSET,
					reg_val,
					(((reg_val >> 16) & 1) == 1),
					GDSC_POWER_SLEEP_US,
					GDSC_POWER_TIMEOUT_US);
		if (ret < 0)
			dev_err(mvm_dev->dev, "mvm gdsc couldn't power up\n");
		else
			dev_info(mvm_dev->dev, "MVM GSDC powered up\n");
	} else {
		ret = readl_poll_timeout(mvm_dev->mvm_base + MVM_CC_MVM_CFG_GDSCR_OFFSET,
					reg_val,
					(((reg_val >> 15 )& 1) == 1),
					GDSC_POWER_SLEEP_US,
					GDSC_POWER_TIMEOUT_US);
		if (ret < 0)
			dev_err(mvm_dev->dev, "mvm gdsc couldn't power down\n");
		else
			dev_info(mvm_dev->dev, "mvm GDSC powered down\n");
	}
	return ret;
}

static void collapse_mvm_core(struct mvm_device *mvm_dev)
{
	uint32_t reg_val;

	reg_val = readl_relaxed(mvm_dev->mvm_base + MVM_CC_E21CPU_CC_CBCR_OFFSET);
	writel_relaxed(((reg_val & SW_COLLAPSE_MASK)| 0) ,  mvm_dev->mvm_base + MVM_CC_E21CPU_CC_CBCR_OFFSET);
	enable_mvm_gdsc(mvm_dev, false);
}

static void restore_mvm_core(struct mvm_device *mvm_dev)
{
	uint32_t reg_val;
	reg_val = readl_relaxed(mvm_dev->mvm_base + MVM_CC_E21CPU_CC_CBCR_OFFSET);
	writel_relaxed((reg_val & SW_COLLAPSE_MASK)| 1 ,  mvm_dev->mvm_base + MVM_CC_E21CPU_CC_CBCR_OFFSET);
	enable_mvm_gdsc(mvm_dev, true);
}

static ssize_t mvm_state_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	struct mvm_device *mvm_dev = container_of(attr,
						struct mvm_device,
						attr);
	return snprintf(buf, 0x10, "%s\n", mvm_states[mvm_dev->state]);
}

static int mvm_sysfs_init(struct mvm_device *mvm_dev)
{
	int ret = 0;
	mvm_dev->kobj = kobject_create_and_add("mvm", kernel_kobj);
	if (!mvm_dev->kobj) {
		dev_err(mvm_dev->dev, "%s: sysfs creation failed\n",
					__func__);
		return -ENOMEM;
	}

	sysfs_attr_init(&mvm_dev->attr.attr);
	mvm_dev->attr.attr.mode = 0444;
	mvm_dev->attr.attr.name = "mvm_state";
	mvm_dev->attr.show = mvm_state_show;
	mvm_dev->attr.store = NULL;

	ret = sysfs_create_file(mvm_dev->kobj, &mvm_dev->attr.attr);
	if (ret)        {
		dev_err(mvm_dev->dev, "%s: sysfs_create_file failed\n",
							__func__);
		goto fail_sysfs;
	}
	return 0;

fail_sysfs:
	kobject_put(mvm_dev->kobj);
	return ret;
}

static bool fifo_full(unsigned int fifo_head, unsigned int fifo_size, unsigned int fifo_tail)
{
	unsigned int head = fifo_head + 1;

	if (head == fifo_size)
		head = 0;
	return head == fifo_tail;
}

static int send_ctrl_msg_to_mvm(struct mvm_control *mvm_ctrl, struct mvm_device *mvm_dev)
{
	struct input_msg *inp_msg;
	unsigned int *in_fifo_addr = NULL;
	bool full;
	int ret = 0;

	mvm_dev->ring_buff->in_fifo[0].tail =
		readl_relaxed(mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING0_TAIL_PTR_OFFSET);
	mvm_dev->ring_buff->in_fifo[0].head =
		readl_relaxed(mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING0_HEAD_PTR_OFFSET);

	full = fifo_full(mvm_dev->ring_buff->in_fifo[0].head,
			 mvm_dev->ring_buff->in_fifo[0].size,
			 mvm_dev->ring_buff->in_fifo[0].tail);

	if (full) {
		ret = -1;
		dev_err(mvm_dev->dev, "Couldn't send control message\n");
		return ret;
	}

	in_fifo_addr = (unsigned int *) &mvm_dev->ring_buff->in_fifo[0].base[mvm_dev->ring_buff->in_fifo[0].head];
	inp_msg = kzalloc(sizeof(struct input_msg), GFP_KERNEL);
	inp_msg->priority = 0;
	inp_msg->opcode = CTL_MSG_OPCODE;
	memcpy(in_fifo_addr, inp_msg, sizeof(struct input_msg));

	memcpy(in_fifo_addr + 1, mvm_ctrl, sizeof(struct mvm_control));
	mvm_dev->ring_buff->in_fifo[0].head = (mvm_dev->ring_buff->in_fifo[0].head + 1) % mvm_dev->ring_buff->in_fifo[0].size;
	writel_relaxed(mvm_dev->ring_buff->in_fifo[0].head, mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING0_HEAD_PTR_OFFSET);
	writel_relaxed(IRQ_APSS1, mvm_dev->apss_shared_base);
	return 0;
}

static ssize_t mvm_log_file_read(struct file *filp, char __user *buff, size_t count, loff_t *offset)
{
	struct mvm_device *mvm_dev = filp->private_data;
	ssize_t actual_length = 0;
	ssize_t ret = 0;
	char *buff_total;
	ssize_t length_tail_to_bufferend;

	if (mvm_dev->ddr_tail_pos >= &mvm_dev->log_buff->mvmlog_buffer[MVM_ULOG_BUFFER_SIZE]) {
		mvm_dev->ddr_tail_pos = &mvm_dev->log_buff->mvmlog_buffer[0];
	}

	if (mvm_dev->ddr_head_pos > mvm_dev->ddr_tail_pos) {
		if (count > (mvm_dev->ddr_head_pos - mvm_dev->ddr_tail_pos)) {
			if (copy_to_user(buff, mvm_dev->ddr_tail_pos, (mvm_dev->ddr_head_pos - mvm_dev->ddr_tail_pos)*4)) {//multiplied by 4 to calculate size of int
				return actual_length;//Actual length will be zero if copy to user not success
			}

			actual_length = (mvm_dev->ddr_head_pos - mvm_dev->ddr_tail_pos)*4;
			mvm_dev->ddr_tail_pos = mvm_dev->ddr_head_pos;
			return actual_length;
		} else  {
			if (copy_to_user(buff, mvm_dev->ddr_tail_pos, count)) {
				return actual_length;
			}//Actual length will be zero if copy to user not success
			return count;
		}
	} else if (mvm_dev->ddr_head_pos < mvm_dev->ddr_tail_pos) {
		ret = copy_to_user(buff, mvm_dev->ddr_tail_pos, (&mvm_dev->log_buff->mvmlog_buffer[MVM_ULOG_BUFFER_SIZE] - mvm_dev->ddr_tail_pos));
		length_tail_to_bufferend = &mvm_dev->log_buff->mvmlog_buffer[MVM_ULOG_BUFFER_SIZE] - mvm_dev->ddr_tail_pos;
		if (length_tail_to_bufferend == (&mvm_dev->log_buff->mvmlog_buffer[MVM_ULOG_BUFFER_SIZE] - mvm_dev->ddr_tail_pos)) {
			buff_total = (buff+length_tail_to_bufferend);
			ret = copy_to_user(buff_total, &mvm_dev->log_buff->mvmlog_buffer[0], (mvm_dev->ddr_head_pos - &mvm_dev->log_buff->mvmlog_buffer[0]));
			actual_length = length_tail_to_bufferend + (mvm_dev->ddr_head_pos - &mvm_dev->log_buff->mvmlog_buffer[0]);
		}
		mvm_dev->ddr_tail_pos = mvm_dev->ddr_head_pos;
		return actual_length;
	}

	return actual_length;
}

static __poll_t mvm_log_poll(struct file *filp, struct poll_table_struct *wait)
{
	unsigned int events = 0;
	struct mvm_device *mvm_dev = filp->private_data;

	events = mvm_dev->ddr_head_pos - mvm_dev->ddr_tail_pos;
	if (events) {
		events = POLLIN | POLLPRI;
		return events;
	}
	poll_wait(filp, &mvm_dev->log_poll_wait, wait);
	return events;
}

static int mvm_log_open(struct inode *inode, struct file *filp)
{
	struct mvm_device *mvm_dev = inode->i_private;
	filp->private_data = mvm_dev;
	return 0;
}

static const struct file_operations debugfs_mvm_log_ops = {
	.owner     = THIS_MODULE,
	.open      = mvm_log_open,
	.read      = mvm_log_file_read,
	.poll      = mvm_log_poll
};

static int mvm_log_policy_open(struct inode *inode, struct file *filp)
{
	struct mvm_device *mvm_dev = inode->i_private;
	filp->private_data = mvm_dev;
	return 0;
}

static ssize_t mvm_log_policy_store(struct file *filp, const char __user *ubuf, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	struct mvm_control *mvm_ctrl;
	struct mvm_device *mvm_dev = filp->private_data;

	mvm_ctrl = kzalloc(sizeof(struct mvm_control), GFP_KERNEL);
	mvm_ctrl->type = MVM_DEBUG;
	if (copy_from_user(&mvm_ctrl->mvm_ctrl_msg.debug.transfer_msg.log_policy, ubuf, count)) {
		kfree(mvm_ctrl);
		ret = -EFAULT;
		return ret;
	}

	ret = send_ctrl_msg_to_mvm(mvm_ctrl, mvm_dev);
	kfree(mvm_ctrl);
	return ret;
}

static const struct file_operations debugfs_mvm_policy_ops = {
	.owner      = THIS_MODULE,
	.open     = mvm_log_policy_open,
	.write    = mvm_log_policy_store,
};

static int mvm_log_transfer_open(struct inode *inode, struct file *filp)
{
	struct mvm_device *mvm_dev = inode->i_private;
	filp->private_data = mvm_dev;
	return 0;
}

static ssize_t mvm_log_transfer_store(struct file *filp, const char __user *ubuf, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	struct mvm_control *mvm_ctrl;
	struct mvm_device *mvm_dev = filp->private_data;

	mvm_ctrl = kzalloc(sizeof(struct mvm_control), GFP_KERNEL);
	mvm_ctrl->type = MVM_DEBUG;
	mvm_dev->filled_dma_bytes = mvm_dev->filled_dma_bytes + (mvm_dev->ddr_buf_len * 0x4);
	if (mvm_dev->filled_dma_bytes > MAX_LOG_BUFFER_SIZE) { //for on-demand check if 2kb memory left in ddr
		mvm_dev->filled_dma_bytes = 0;//Reset filled ddr bytes to zero
		mvm_dev->ddr_head_pos = &mvm_dev->log_buff->mvmlog_buffer[0];
	}
	mvm_ctrl->mvm_ctrl_msg.debug.transfer_msg.ddr_log_buf_addr = LOG_BUFF_IOVA + mvm_dev->filled_dma_bytes;
	if (copy_from_user(&mvm_ctrl->mvm_ctrl_msg.debug.msg_type, ubuf, count)) {
		ret = -EFAULT;
	} else {
		mvm_ctrl->mvm_ctrl_msg.debug.msg_type = MVM_DEBUG_LOG_TRANSFER_REQUEST;
		ret = send_ctrl_msg_to_mvm(mvm_ctrl, mvm_dev);
	}

	kfree(mvm_ctrl);
	return ret;
}

static const struct file_operations debugfs_mvm_transfer_ops = {
	.owner      = THIS_MODULE,
	.open     = mvm_log_transfer_open,
	.write    = mvm_log_transfer_store,
};

static int mvm_trigger_ssr_open(struct inode *inode, struct file *filp)
{
	struct mvm_device *mvm_dev = inode->i_private;
	filp->private_data = mvm_dev;
	return 0;
}

static ssize_t mvm_trigger_ssr_store(struct file *filp, const char __user *ubuf, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	unsigned int ssr = 0;
	struct mvm_control *mvm_ctrl;
	struct mvm_device *mvm_dev = filp->private_data;

	ret = kstrtouint_from_user(ubuf, count, 10, &ssr);
	if (ret)
		return ret;

	if (ssr == 1) {
		mvm_ctrl = kzalloc(sizeof(struct mvm_control), GFP_KERNEL);
		mvm_ctrl->type = MVM_TRIGGER_SSR;
		mvm_ctrl->mvm_ctrl_msg.ssr.trigger_ssr = ssr;
		ret = send_ctrl_msg_to_mvm(mvm_ctrl, mvm_dev);
		kfree(mvm_ctrl);
	}

	return count;
}

static const struct file_operations debugfs_mvm_trigger_ssr_ops = {
	.owner = THIS_MODULE,
	.open = mvm_trigger_ssr_open,
	.write = mvm_trigger_ssr_store,
};

static int mvm_crash_dump_open(struct inode *inode, struct file *filp)
{
	struct mvm_device *mvm_dev = inode->i_private;
	filp->private_data = mvm_dev;
	return 0;
}

static unsigned int mvm_crash_dump_poll(struct file *filp, struct poll_table_struct *pt)
{
	unsigned int events = 0;
	struct mvm_device *mvm_dev = filp->private_data;

	if (mvm_dev->pending_dump_read == true) {
		events = POLLIN | POLLPRI;
		goto ret;
	}

	poll_wait(filp, &mvm_dev->ssr_poll_wait, pt);
ret:
	return events;
}

static ssize_t mvm_crash_dump_read(struct file *filp, char __user *buff, size_t count, loff_t *offset)
{
	struct mvm_device *mvm_dev = filp->private_data;
	int ret;

	ret = copy_to_user(buff, &mvm_dev->dump_buff->mvm_dump_buffer[0], MVM_CRASH_DUMP_SIZE);
	if (ret > 0)
		dev_err(mvm_dev->dev, "couldnt copy crash dump to userspace\n");

	mvm_dev->pending_dump_read = false;
	return MVM_CRASH_DUMP_SIZE - ret;
}

static const struct file_operations debugfs_mvm_crash_dump_ops = {
	.owner = THIS_MODULE,
	.open = mvm_crash_dump_open,
	.poll = mvm_crash_dump_poll,
	.read = mvm_crash_dump_read,
};

static int mvm_log_level_open(struct inode *inode, struct file *filp)
{
        struct mvm_device *mvm_dev = inode->i_private;
        filp->private_data = mvm_dev;
        return 0;
}

static ssize_t mvm_log_level_store(struct file *filp, const char __user *ubuf, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	struct mvm_control *mvm_ctrl;
	struct mvm_device *mvm_dev = filp->private_data;

	mvm_ctrl = kzalloc(sizeof(struct mvm_control), GFP_KERNEL);
	mvm_ctrl->type = MVM_DEBUG;

	mvm_ctrl->mvm_ctrl_msg.debug.msg_type = MVM_DEBUG_SET_LOG_LEVEL;
	if (copy_from_user(&mvm_ctrl->mvm_ctrl_msg.debug.transfer_msg.log_level, ubuf, count)) {
		kfree(mvm_ctrl);
		ret = -EFAULT;
		return ret;
	}

	ret = send_ctrl_msg_to_mvm(mvm_ctrl, mvm_dev);
	kfree(mvm_ctrl);
	return ret;
}

static const struct file_operations debugfs_mvm_log_level_ops = {
	.owner      = THIS_MODULE,
	.open     = mvm_log_level_open,
	.write    = mvm_log_level_store,
};

static int mvm_debugfs_init(struct mvm_device *mvm_dev)
{
	struct dentry *file;

	mvm_dev->dir = debugfs_create_dir("mvm", NULL);
	if (IS_ERR_OR_NULL(mvm_dev->dir))
		return -ENOMEM;

	file = debugfs_create_file("mvm_log", 0644, mvm_dev->dir, mvm_dev,
						&debugfs_mvm_log_ops);
	if (!file)
		debugfs_remove(mvm_dev->dir);

	file = debugfs_create_file("mvm_policy", 0644, mvm_dev->dir, mvm_dev,
						&debugfs_mvm_policy_ops);
	if (!file)
		debugfs_remove(mvm_dev->dir);

	file = debugfs_create_file("mvm_transfer", 0644, mvm_dev->dir, mvm_dev,
						&debugfs_mvm_transfer_ops);
	if (!file)
		debugfs_remove(mvm_dev->dir);

	file = debugfs_create_file("trigger_ssr", 0644, mvm_dev->dir, mvm_dev,
						&debugfs_mvm_trigger_ssr_ops);
	if (!file)

		debugfs_remove(mvm_dev->dir);

	file = debugfs_create_file("mvm_crash_dump", 0644, mvm_dev->dir, mvm_dev,
						&debugfs_mvm_crash_dump_ops);
	if (!file)
		debugfs_remove(mvm_dev->dir);

	file = debugfs_create_file("mvm_log_level", 0644, mvm_dev->dir, mvm_dev,
		&debugfs_mvm_log_level_ops);
	if (!file)
		debugfs_remove(mvm_dev->dir);

	mvm_dev->ddr_current_addr = &mvm_dev->log_buff->mvmlog_buffer[0];
	mvm_dev->ddr_head_pos = &mvm_dev->log_buff->mvmlog_buffer[0];
	mvm_dev->ddr_tail_pos = &mvm_dev->log_buff->mvmlog_buffer[0];
	mvm_dev->active_buffer_index = -1;
	mvm_dev->ddr_buf_len = 0;
	mvm_dev->filled_dma_bytes = 0;
	return 0;
}

static void process_control_message(struct mvm_control *mvm_ctrl_recv, struct mvm_device *mvm_dev)
{
	ssize_t ret = 0;
	struct mvm_control *mvm_ctrl_send;

	mvm_ctrl_send = kzalloc(sizeof(struct mvm_control), GFP_KERNEL);
	mvm_ctrl_send->type = MVM_DEBUG;
	if ( mvm_ctrl_recv->mvm_ctrl_msg.debug.msg_type == MVM_DEBUG_LOG_TRANSFER_REQUEST) {
	    if ( mvm_ctrl_recv->mvm_ctrl_msg.debug.msg_type == MVM_FLUSH_DDR_OVERFLOW) {
			wake_up_interruptible_poll(&mvm_dev->log_poll_wait, POLLIN | POLLPRI);//wake up poll for logging if transfer complete
			mvm_ctrl_send->mvm_ctrl_msg.debug.transfer_msg.ddr_log_buf_addr = LOG_BUFF_IOVA + mvm_dev->filled_dma_bytes;
			mvm_ctrl_send->mvm_ctrl_msg.debug.transfer_msg.active_buffer_index = mvm_dev->active_buffer_index;
			ret = send_ctrl_msg_to_mvm(mvm_ctrl_send, mvm_dev);
		}
	} else if ( mvm_ctrl_recv->mvm_ctrl_msg.debug.msg_type == MVM_DEBUG_LOG_TRANSFER_COMPLETE) {
		mvm_dev->ddr_buf_len = mvm_ctrl_recv->mvm_ctrl_msg.debug.num_bytes_transferred /4;
		mvm_dev->ddr_head_pos = &mvm_dev->ddr_head_pos[mvm_dev->ddr_buf_len];
		if( (mvm_dev->ddr_head_pos >= &mvm_dev->log_buff->mvmlog_buffer[MVM_ULOG_BUFFER_SIZE]) || ((mvm_dev->filled_dma_bytes + mvm_ctrl_recv->mvm_ctrl_msg.debug.num_bytes_transferred) > MAX_LOG_BUFFER_SIZE)) {
			//if will check filled_dma_bytes(total byte logs collected) not greater than 6kb.make sure 2k memory always available for e21 to write logs.
			mvm_dev->filled_dma_bytes = 0;//Reset filled ddr bytes to zero
			mvm_dev->ddr_head_pos = &mvm_dev->log_buff->mvmlog_buffer[0];//reset head position to start reading ddr from log base address.
		}
		mvm_dev->ddr_current_addr = mvm_dev->ddr_head_pos;
		wake_up_interruptible_poll(&mvm_dev->log_poll_wait, POLLIN | POLLPRI);
	}
	kfree(mvm_ctrl_send);
}

static bool drain_out_fifo(struct mvm_device *mvm_dev, unsigned int fifo_index, unsigned int count)
{
	struct mvm_client *mvm_cli;
	unsigned int client_id;
	unsigned int opcode;
	bool full = false;
	bool out_buff_written = false;
	bool control_message_written = false;
	unsigned int *out_fifo_addr = NULL;
	struct mvm_control *mvm_ctrl_recv;
	while (count && !full) {
		client_id =
		mvm_dev->ring_buff->out_fifo[fifo_index].base[mvm_dev->ring_buff->out_fifo[fifo_index].tail].client_id;
		dev_dbg(mvm_dev->dev, "client_id is %d\n", client_id);
		opcode =
			mvm_dev->ring_buff->out_fifo[fifo_index].base[mvm_dev->ring_buff->out_fifo[fifo_index].tail].opcode;
		dev_dbg(mvm_dev->dev, "opcode is %d\n", opcode);
		if (opcode == CTL_MSG_OPCODE) {
			mvm_ctrl_recv = kzalloc(sizeof(struct mvm_control), GFP_KERNEL);
			out_fifo_addr = (unsigned int *) mvm_dev->ring_buff->out_fifo[fifo_index].base[mvm_dev->ring_buff->out_fifo[fifo_index].tail].data;
			memcpy(mvm_ctrl_recv, out_fifo_addr, sizeof(struct mvm_control));
			switch (mvm_ctrl_recv->type) {
			case MVM_DEBUG:
				mvm_dev->active_buffer_index = mvm_ctrl_recv->mvm_ctrl_msg.debug.transfer_msg.active_buffer_index;
				process_control_message(mvm_ctrl_recv, mvm_dev);
				control_message_written = true;
				break;
			case MVM_POLICY:
				break;
			case MVM_CPU_FREQ:
				break;
			case MVM_POWER:
				break;
			default:
				break;
			}
			kfree(mvm_ctrl_recv);
		}
		else {
			bool match = false;
			list_for_each_entry(mvm_cli, &mvm_dev->client_list, list) {
				if (mvm_cli->client_id == client_id) {
					match = true;
					full = fifo_full(mvm_cli->out_buff->head, mvm_cli->out_buff->size,
									mvm_cli->out_buff->tail);
					if (!full) {
						memcpy(&mvm_cli->out_buff->out_msg[mvm_cli->out_buff->head],
						&mvm_dev->ring_buff->out_fifo[fifo_index].base[
						mvm_dev->ring_buff->out_fifo[fifo_index].tail],
						sizeof(struct output_msg));
						mvm_cli->out_buff->head =
							(mvm_cli->out_buff->head+1) % (mvm_cli->out_buff->size);
						mvm_cli->out_buff->count++;
						mvm_dev->outgoing_results++;
						out_buff_written = true;
						break;
					}
				}
			}
			if(!match){
				dev_err(mvm_dev->dev,"outring message client id did not match connected client list\n");
			}
		}
		if(!full){
			mvm_dev->ring_buff->out_fifo[fifo_index].tail =
				(mvm_dev->ring_buff->out_fifo[fifo_index].tail+1) %
				mvm_dev->ring_buff->out_fifo[fifo_index].size;
			count--;
		}
	}
	if (out_buff_written || control_message_written) {
		writel_relaxed(mvm_dev->ring_buff->out_fifo[0].tail,
			mvm_dev->mvm_base + MVMSS_CSR_OUTPUT_RING0_TAIL_PTR_OFFSET);
		writel_relaxed(mvm_dev->ring_buff->out_fifo[1].tail,
			mvm_dev->mvm_base + MVMSS_CSR_OUTPUT_RING1_TAIL_PTR_OFFSET);

		dev_dbg(mvm_dev->dev, "%s OUTPUT_RING0_TAIL_PTR_OFFSET %d OUTPUT_RING1_TAIL_PTR_OFFSET %d\n",__func__,
			readl_relaxed(mvm_dev->mvm_base + MVMSS_CSR_OUTPUT_RING0_TAIL_PTR_OFFSET),
			readl_relaxed(mvm_dev->mvm_base + MVMSS_CSR_OUTPUT_RING1_TAIL_PTR_OFFSET));
	}

	return out_buff_written;
}

bool out_fifo_get_results(struct mvm_device *mvm_dev, unsigned int fifo_index)
{
	bool ret = false;
	unsigned int count;
	//Read OUTPUT_RING CSRS
	if (fifo_index == 0) {
		mvm_dev->ring_buff->out_fifo[0].tail =
			readl_relaxed(mvm_dev->mvm_base + MVMSS_CSR_OUTPUT_RING0_TAIL_PTR_OFFSET);
		mvm_dev->ring_buff->out_fifo[0].head =
			readl_relaxed(mvm_dev->mvm_base + MVMSS_CSR_OUTPUT_RING0_HEAD_PTR_OFFSET);
	} else {
		mvm_dev->ring_buff->out_fifo[1].tail =
			readl_relaxed(mvm_dev->mvm_base + MVMSS_CSR_OUTPUT_RING1_TAIL_PTR_OFFSET);
		mvm_dev->ring_buff->out_fifo[1].head =
			readl_relaxed(mvm_dev->mvm_base + MVMSS_CSR_OUTPUT_RING1_HEAD_PTR_OFFSET);
	}

	dev_dbg(mvm_dev->dev, "%s fifo_index %d tail is %d head is %d\n", __func__,fifo_index,
				mvm_dev->ring_buff->out_fifo[fifo_index].tail,
				mvm_dev->ring_buff->out_fifo[fifo_index].head);
	if (mvm_dev->ring_buff->out_fifo[fifo_index].tail <= mvm_dev->ring_buff->out_fifo[fifo_index].head)
		count = mvm_dev->ring_buff->out_fifo[fifo_index].head - mvm_dev->ring_buff->out_fifo[fifo_index].tail;
	else
		count = (mvm_dev->ring_buff->out_fifo[fifo_index].size - mvm_dev->ring_buff->out_fifo[fifo_index].tail) +
							mvm_dev->ring_buff->out_fifo[fifo_index].head;


	if (count > 0)
		ret = drain_out_fifo(mvm_dev, fifo_index, count);

	return ret;
}

static void drain_out_fifo_work_hdlr(struct work_struct *work)
{
	struct mvm_device *mvm_dev = container_of(work, struct mvm_device, drain_out_fifo_work);
	bool p0_fifo_has_results, p1_fifo_has_results, wake_up;

	mvm_dev->ring_buff->in_fifo[0].tail =
		readl_relaxed(mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING0_TAIL_PTR_OFFSET);
	mvm_dev->ring_buff->in_fifo[1].tail =
		readl_relaxed(mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING1_TAIL_PTR_OFFSET);

	if(!fifo_full(mvm_dev->ring_buff->in_fifo[0].head,
			mvm_dev->ring_buff->in_fifo[0].size,
			mvm_dev->ring_buff->in_fifo[0].tail)){
		complete(&mvm_dev->p0_fifo_slot_available);
	}

	if(!fifo_full(mvm_dev->ring_buff->in_fifo[1].head,
			mvm_dev->ring_buff->in_fifo[1].size,
			mvm_dev->ring_buff->in_fifo[1].tail)){
		complete(&mvm_dev->p1_fifo_slot_available);
	}

	mutex_lock(&mvm_dev->out_fifo_lock);
	p0_fifo_has_results = out_fifo_get_results(mvm_dev, 0);
	if (p0_fifo_has_results)
		wake_up = true;

	p1_fifo_has_results = out_fifo_get_results(mvm_dev, 1);
	if (p1_fifo_has_results)
		wake_up = true;
	mutex_unlock(&mvm_dev->out_fifo_lock);

	if (wake_up)
		wake_up_interruptible_poll(&mvm_dev->mvm_waitqueue, POLLIN | POLLPRI);
}

static irqreturn_t mvm_ssr_done_irq_handler(int irq, void *dev_id)
{
	struct mvm_device *mvm_dev = dev_id;
	uint32_t value;

	value = readl_relaxed(mvm_dev->mvm_base+MVMSS_CSR_MVMSS_APSS);
	value = (value & (~(1 << ((mvm_dev->mvm_ssr_done_hw_irq - 432)))));
	writel_relaxed(value, mvm_dev->mvm_base+MVMSS_CSR_MVMSS_APSS);

	complete(&mvm_dev->mvm_dump_collection_done);
	return IRQ_HANDLED;
}

static irqreturn_t mvm_verif_done_irq_handler(int irq, void *dev_id)
{
	struct mvm_device *mvm_dev = dev_id;
	uint32_t value;

	value = readl_relaxed(mvm_dev->mvm_base+MVMSS_CSR_MVMSS_APSS);
	value = (value & (~(1 << ((mvm_dev->mvm_verif_done_hw_irq - 432)))));
	writel_relaxed(value, mvm_dev->mvm_base+MVMSS_CSR_MVMSS_APSS);

	schedule_work(&mvm_dev->drain_out_fifo_work);
	return IRQ_HANDLED;
}

static int mvm_open(struct inode *inode, struct file *filp)
{
	unsigned long i;
	int ret = 0;
	struct mvm_device *mvm_dev = container_of(inode->i_cdev,
					struct mvm_device, mvm_cdev);
	struct mvm_client *mvm_cli;

	mutex_lock(&mvm_dev->mvm_cli_lock);
	if (bitmap_full(mvm_dev->client_id_bitmap, MAX_CLIENT_COUNT)) {
		dev_err(mvm_dev->dev, "Cannot accept new connections\n");
		ret = -EUSERS;
		goto mutex_unlock;
	}

	mvm_cli = kzalloc(sizeof(*mvm_cli), GFP_KERNEL);
	if (!mvm_cli) {
		ret = -ENOMEM;
		goto mutex_unlock;
	}

	INIT_LIST_HEAD(&mvm_cli->list);
	mvm_cli->mvm_dev = mvm_dev;
	i = find_first_zero_bit(mvm_dev->client_id_bitmap, MAX_CLIENT_COUNT);
	set_bit(i, mvm_dev->client_id_bitmap);
	mvm_cli->client_id = i+1;
	mvm_cli->task = get_current();
	mvm_cli->client_ready = true;
	list_add_tail(&mvm_cli->list, &mvm_dev->client_list);
	mutex_unlock(&mvm_dev->mvm_cli_lock);

	mvm_cli->out_buff = kzalloc(sizeof(struct output_buffer), GFP_KERNEL);
	mvm_cli->out_buff->size = OUT_BUFF_SIZE;
	filp->private_data = mvm_cli;
	goto exit;

mutex_unlock:
	mutex_unlock(&mvm_dev->mvm_cli_lock);

exit:
	return ret;
}

static int mvm_release(struct inode *inode, struct file *filp)
{
	struct mvm_device *mvm_dev = container_of(inode->i_cdev,
	struct mvm_device, mvm_cdev);
	struct mvm_client *mvm_cli = filp->private_data;

	mutex_lock(&mvm_dev->mvm_cli_lock);
	clear_bit(mvm_cli->client_id - 1, mvm_dev->client_id_bitmap);
	list_del(&mvm_cli->list);
	mutex_unlock(&mvm_dev->mvm_cli_lock);
	mvm_cli->mvm_dev = NULL;
	kfree(mvm_cli->out_buff);
	kfree(mvm_cli);
	complete_all(&mvm_dev->p0_fifo_slot_available);
	complete_all(&mvm_dev->p1_fifo_slot_available);
	return 0;
}

static long mvm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct mvm_client *mvm_cli = filp->private_data;

	switch (cmd) {
	case GET_CLIENT_ID:
		if (copy_to_user((unsigned int *)arg, &mvm_cli->client_id,
					sizeof(mvm_cli->client_id)))
			ret = -EFAULT;
		break;

	case SET_TIMEOUT_MS:
		if (copy_from_user(&mvm_cli->timeout_ms, (unsigned int *) arg,
					sizeof(mvm_cli->timeout_ms)))
			ret = -EFAULT;
		break;

	case READY_AFTER_SSR:
		mvm_cli->client_ready= true;
		mvm_cli->out_buff->head = 0;
		mvm_cli->out_buff->tail = 0;
		break;

	default:
		break;
	}

	return ret;
}

static ssize_t mvm_write(
	struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
	struct input_msg *inp_msg;
	unsigned int i, fifo_index;
	struct mvm_client *mvm_cli = filp->private_data;
	struct mvm_device *mvm_dev = mvm_cli->mvm_dev;
	bool full;
	int rc, ret;
	unsigned int no_of_msgs_written = 0;
	unsigned int count;

	if (!mvm_cli->client_ready)
		goto ret;

	mutex_lock(&mvm_dev->in_fifo_lock);
	inp_msg = kzalloc(sizeof(struct input_msg), GFP_KERNEL);
	ret = copy_from_user(inp_msg, buf, sizeof(struct input_msg));
	if (ret)
		count = (len - ret) / sizeof(struct input_msg);
	else
		count = len / sizeof(struct input_msg);
	fifo_index = inp_msg[0].priority;
	dev_dbg(mvm_dev->dev, "priority is %d count is %d\n",
					fifo_index, count);
	kfree(inp_msg);

	//Read INPUT_RING CSRS
	if (fifo_index == 0) {
		mvm_dev->ring_buff->in_fifo[0].tail =
			readl_relaxed(mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING0_TAIL_PTR_OFFSET);
		mvm_dev->ring_buff->in_fifo[0].head =
			readl_relaxed(mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING0_HEAD_PTR_OFFSET);
		dev_dbg(mvm_dev->dev, "%s : before INPUT_RING0_TAIL %d INPUT_RING0_HEAD %d\n", __func__,
			 mvm_dev->ring_buff->in_fifo[0].tail, mvm_dev->ring_buff->in_fifo[0].head);
	} else {
		mvm_dev->ring_buff->in_fifo[1].tail =
			readl_relaxed(mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING1_TAIL_PTR_OFFSET);
		mvm_dev->ring_buff->in_fifo[1].head =
			readl_relaxed(mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING1_HEAD_PTR_OFFSET);
		dev_dbg(mvm_dev->dev, "%s : before INPUT_RING1_TAIL %d INPUT_RING1_HEAD %d\n", __func__,
			 mvm_dev->ring_buff->in_fifo[1].tail, mvm_dev->ring_buff->in_fifo[1].head);
	}
	for (i = 0; i < count; i++) {
push_to_input_ring:
		dev_dbg(mvm_dev->dev, "Head %d tail %d size %d\n",
				       mvm_dev->ring_buff->in_fifo[fifo_index].head,
				       mvm_dev->ring_buff->in_fifo[fifo_index].tail,
				       mvm_dev->ring_buff->in_fifo[fifo_index].size);
		full = fifo_full(mvm_dev->ring_buff->in_fifo[fifo_index].head,
				 mvm_dev->ring_buff->in_fifo[fifo_index].size,
				 mvm_dev->ring_buff->in_fifo[fifo_index].tail);
		dev_dbg(mvm_dev->dev, "fifo %d is %d\n", fifo_index, full);
		if (full) {
			if (fifo_index == 0) {
				reinit_completion(&mvm_dev->p0_fifo_slot_available);
				rc = wait_for_completion_interruptible_timeout(
					&mvm_dev->p0_fifo_slot_available,
					msecs_to_jiffies(mvm_cli->timeout_ms));
			} else {
				reinit_completion(&mvm_dev->p1_fifo_slot_available);
				rc = wait_for_completion_interruptible_timeout(
					&mvm_dev->p1_fifo_slot_available,
					msecs_to_jiffies(mvm_cli->timeout_ms));
			}
			if (rc > 0)
				goto push_to_input_ring;
			else if (rc == 0) {
				dev_err(mvm_dev->dev,
					"Timed out waiting for empty space in fifo %d\n",
					fifo_index);
				goto ret;
			}
		} else {
			ret = copy_from_user(
			&mvm_dev->ring_buff->in_fifo[fifo_index].base[mvm_dev->ring_buff->in_fifo[fifo_index].head],
			(buf+(i * sizeof(struct input_msg))),
			sizeof(struct input_msg));
			if (!ret) {
				mvm_dev->ring_buff->in_fifo[fifo_index].head =
					(mvm_dev->ring_buff->in_fifo[fifo_index].head + 1)
					% mvm_dev->ring_buff->in_fifo[fifo_index].size;
				no_of_msgs_written++;
				mvm_dev->incoming_msgs++;
				if (fifo_index == 0)
					writel_relaxed(mvm_dev->ring_buff->in_fifo[0].head,
					mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING0_HEAD_PTR_OFFSET);
				else
					writel_relaxed(mvm_dev->ring_buff->in_fifo[1].head,
					mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING1_HEAD_PTR_OFFSET);
			}
		}
	}
ret:
	dev_dbg(mvm_dev->dev, "%s : after INPUT_RING0_HEAD_PTR_OFFSET %d INPUT_RING1_HEAD_PTR_OFFSET %d\n", __func__,
	readl_relaxed(mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING0_HEAD_PTR_OFFSET),
	readl_relaxed(mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING1_HEAD_PTR_OFFSET));

	dev_dbg(mvm_dev->dev, "no_of_msgs_written %d\n", no_of_msgs_written);
	mutex_unlock(&mvm_dev->in_fifo_lock);

	if (no_of_msgs_written > 0)
		writel_relaxed(IRQ_APSS1, mvm_dev->apss_shared_base);
	return (no_of_msgs_written * sizeof(struct input_msg));
}

static unsigned int mvm_poll(struct file *filp,
				struct poll_table_struct *pt)
{
	unsigned int events = 0;
	struct mvm_client *mvm_cli = filp->private_data;
	struct mvm_device *mvm_dev = mvm_cli->mvm_dev;

	if (mvm_cli->out_buff->count >= 1) {
		events = POLLIN | POLLPRI;
		return events;
	}

	poll_wait(filp, &mvm_dev->mvm_waitqueue, pt);
	return events;
}

static ssize_t mvm_read(struct file *filp,
		char __user *buf, size_t count, loff_t *off)
{
	struct mvm_client *mvm_cli = filp->private_data;
	struct mvm_device *mvm_dev = mvm_cli->mvm_dev;
	unsigned int size, out_buff_rem_count;
	int ret = 0, bytes_copied = 0;
	unsigned int bytes_to_copy = 0;

	if (!buf || count < 1)
		return -EINVAL;

	mutex_lock(&mvm_dev->out_fifo_lock);
	if ((mvm_cli->out_buff->tail + mvm_cli->out_buff->count) <= OUT_BUFF_SIZE) {
		if (count <= (mvm_cli->out_buff->count * sizeof(struct output_msg))) {
			bytes_to_copy = count;
		}
		else {
			bytes_to_copy = mvm_cli->out_buff->count * sizeof(struct output_msg);
		}
		ret = copy_to_user(buf,
				&mvm_cli->out_buff->out_msg[mvm_cli->out_buff->tail],
				bytes_to_copy);
		bytes_copied = bytes_to_copy - ret;
		goto update_tail;
	} else {
		size = (OUT_BUFF_SIZE - mvm_cli->out_buff->tail);
		if (count <= (size * sizeof(struct output_msg))) {
			bytes_to_copy = count;
		}
		else {
			bytes_to_copy = size * sizeof(struct output_msg);
		}
		ret = copy_to_user(buf,
				   &mvm_cli->out_buff->out_msg[mvm_cli->out_buff->tail],
				   bytes_to_copy);
		bytes_copied += bytes_to_copy - ret;

		if (ret || (bytes_copied == count))
			goto update_tail;

		count = count - bytes_copied;
		out_buff_rem_count = mvm_cli->out_buff->count - ((bytes_copied / sizeof(struct output_msg)));

		if (count <= (out_buff_rem_count * sizeof(struct output_msg))) {
			bytes_to_copy = count;
		}
		else {
			bytes_to_copy = out_buff_rem_count * sizeof(struct output_msg);
		}

		ret = copy_to_user(buf + bytes_copied,
			     &mvm_cli->out_buff->out_msg[0],
			     bytes_to_copy);
		bytes_copied += bytes_to_copy - ret;
	}
update_tail:
	mvm_cli->out_buff->tail = (mvm_cli->out_buff->tail +
				  (bytes_copied / sizeof(struct output_msg))) %
				   mvm_cli->out_buff->size;
	mvm_cli->out_buff->count -= (bytes_copied / sizeof(struct output_msg));
	dev_dbg(mvm_dev->dev, "mvm_read bytes_copied %x sizeof(struct output_msg) %x\n",bytes_copied,
					 sizeof(struct output_msg));
	dev_dbg(mvm_dev->dev, "mvm_read out_buff_count is %d\n",mvm_cli->out_buff->count);
	mutex_unlock(&mvm_dev->out_fifo_lock);
	schedule_work(&mvm_dev->drain_out_fifo_work);
	return bytes_copied;
}

static irqreturn_t mvm_wfi_irq_handler(int irq, void *dev_id)
{
	struct mvm_device *mvm_dev = dev_id;

	complete(&mvm_dev->mvm_wfi_irq_recvd);
	return IRQ_HANDLED;
}

static void initialise_fifos(struct mvm_device *mvm_dev)
{
	uint32_t infifo_size, outfifo_size;
	infifo_size = sizeof(struct input_fifo);
	outfifo_size = sizeof(struct output_fifo);

	/*initialise P0 Input Ring buffer pointers */
	mvm_dev->ring_buff->in_fifo[0].head = 0;
	writel_relaxed(mvm_dev->ring_buff->in_fifo[0].head,
			mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING0_HEAD_PTR_OFFSET);
	mvm_dev->ring_buff->in_fifo[0].tail = 0;
	writel_relaxed(mvm_dev->ring_buff->in_fifo[0].tail,
			mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING0_TAIL_PTR_OFFSET);
	mvm_dev->ring_buff->in_fifo[0].count = 0;
	mvm_dev->ring_buff->in_fifo[0].size = DDR_FIFO_SIZE;
	writel_relaxed(mvm_dev->ring_buff->in_fifo[0].size,
			mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING0_BUFFER_LENGTH);
	writel_relaxed(RING_BUFF_IOVA + BASE_ADDR_OFFSET,
			mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING0_BASE_ADDR);

	/*initialise P1 Input Ring buffer pointers */
	mvm_dev->ring_buff->in_fifo[1].head = 0;
	writel_relaxed(mvm_dev->ring_buff->in_fifo[1].head,
		mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING1_HEAD_PTR_OFFSET);
	mvm_dev->ring_buff->in_fifo[1].tail = 0;
	writel_relaxed(mvm_dev->ring_buff->in_fifo[1].tail,
		mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING1_TAIL_PTR_OFFSET);
	mvm_dev->ring_buff->in_fifo[1].count = 0;
	mvm_dev->ring_buff->in_fifo[1].size = DDR_FIFO_SIZE;
	writel_relaxed(mvm_dev->ring_buff->in_fifo[1].size,
		mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING1_BUFFER_LENGTH);
	writel_relaxed(RING_BUFF_IOVA + infifo_size + BASE_ADDR_OFFSET,
		mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING1_BASE_ADDR);

	/*initialise P0 Output Ring buffer pointers */
	mvm_dev->ring_buff->out_fifo[0].head = 0;
	writel_relaxed(mvm_dev->ring_buff->out_fifo[0].head,
		mvm_dev->mvm_base + MVMSS_CSR_OUTPUT_RING0_HEAD_PTR_OFFSET);
	mvm_dev->ring_buff->out_fifo[0].tail = 0;
	writel_relaxed(mvm_dev->ring_buff->out_fifo[0].tail,
		mvm_dev->mvm_base + MVMSS_CSR_OUTPUT_RING0_TAIL_PTR_OFFSET);
	mvm_dev->ring_buff->out_fifo[0].count = 0;
	mvm_dev->ring_buff->out_fifo[0].size = DDR_FIFO_SIZE;
	writel_relaxed(mvm_dev->ring_buff->out_fifo[0].size,
		mvm_dev->mvm_base + MVMSS_CSR_OUTPUT_RING0_BUFFER_LENGTH);
	writel_relaxed(RING_BUFF_IOVA + (2 * infifo_size) + BASE_ADDR_OFFSET,
		mvm_dev->mvm_base + MVMSS_CSR_OUTPUT_RING0_BASE_ADDR);

	/*initialise P1 Output Ring buffer pointers */
	mvm_dev->ring_buff->out_fifo[1].head = 0;
	writel_relaxed(mvm_dev->ring_buff->out_fifo[1].head,
		mvm_dev->mvm_base + MVMSS_CSR_OUTPUT_RING1_HEAD_PTR_OFFSET);
	mvm_dev->ring_buff->out_fifo[1].tail = 0;
	writel_relaxed(mvm_dev->ring_buff->out_fifo[1].tail,
		mvm_dev->mvm_base + MVMSS_CSR_OUTPUT_RING1_TAIL_PTR_OFFSET);
	mvm_dev->ring_buff->out_fifo[1].count = 0;
	mvm_dev->ring_buff->out_fifo[1].size = DDR_FIFO_SIZE;
        writel_relaxed(mvm_dev->ring_buff->out_fifo[1].size,
		mvm_dev->mvm_base + MVMSS_CSR_OUTPUT_RING1_BUFFER_LENGTH);
	writel_relaxed(RING_BUFF_IOVA + (2 * infifo_size) + outfifo_size + BASE_ADDR_OFFSET,
		mvm_dev->mvm_base + MVMSS_CSR_OUTPUT_RING1_BASE_ADDR);

	writel_relaxed(MVM_INIT_DONE_COOKIE,
				mvm_dev->mvm_base + MVMSS_CSR_APSS_MVM_SCRATCH_PAD1);
}

static int ioremap_resources(struct platform_device *pdev)
{
	struct resource *res;
	struct mvm_device *mvm_dev;

	mvm_dev = platform_get_drvdata(pdev);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mvm_base");
	mvm_dev->mvm_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mvm_dev->mvm_base)) {
		dev_err(mvm_dev->dev, "ioremap of mvm_base failed\n");
		return -ENOMEM;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "apss_shared_base");
	mvm_dev->apss_shared_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mvm_dev->apss_shared_base)) {
		dev_err(mvm_dev->dev, "ioremap of apss_shared_basefailed\n");
		return -ENOMEM;
	}
	return 0;
}

static int mvm_load_fw(struct mvm_device *mvm_dev)
{
	const struct firmware *fw;
	char fw_name[32];
	void *virt;
	int ret = 0;

	mvm_dev->mvm_fw = dma_alloc_coherent(mvm_dev->dev, MVM_FW_SIZE,
						&mvm_dev->mvm_fw_dma, GFP_KERNEL);
	if (!mvm_dev->mvm_fw) {
		dev_err(mvm_dev->dev,
			"dma_alloc_coherent of the firmware memory failed %d\n");
		ret = -ENOMEM;
		goto fw_dma_mem_fail;
	}

	scnprintf(fw_name, ARRAY_SIZE(fw_name), "mvm_ecc.mdt");
	ret = request_firmware(&fw, fw_name, mvm_dev->dev);
	if (ret) {
		dev_err(mvm_dev->dev, "request_firmware for mvm failed\n");
		goto release_fw_dma_mem;
	}

	virt = mvm_dev->mvm_fw;
	if (!virt) {
		dev_err(mvm_dev->dev, "Failed to remap firmware memory\n");
		goto out_release_firmware;

	}

	update_marker("M - Loading MVM firmware");

	ret = qcom_mdt_load(mvm_dev->dev, fw, fw_name, MVM_PROC_ID,
			    virt, mvm_dev->mvm_fw_dma, MVM_FW_SIZE, NULL);
	if (ret) {
		dev_err(mvm_dev->dev, "Failed to load mvm firmware\n");
		goto out_release_firmware;
	}

	ret = qcom_scm_pas_auth_and_reset(MVM_PROC_ID);
	if (ret) {
		dev_err(mvm_dev->dev, "Error authenticating mvm firmware\n");
		goto out_release_firmware;
	}
	dev_info(mvm_dev->dev, "MVM subsystem brought out of reset\n");
	update_marker("M - MVM subsystem brought out of reset");

	/* qcom_pil_info_store writes the PIL info to the IMEM address so that
	 * MVM SDI dump collection will be enabled. If this imem write returns
	 * error, MVM SDI dump collection will fail.But MVM will still continue to
	 * perform message verification.
	 */

	ret = qcom_pil_info_store("mvm", mvm_dev->mvm_fw_dma, MVM_FW_SIZE);
	if (ret)
		dev_err(mvm_dev->dev, "Couldnt store MVM PIL info in IMEM\n");

	/* Write crash dump DDR location to SCRATCH_PAD0 register so that E21 can store the crash
	 * dump information here.
	 */
	writel_relaxed(MVM_DUMP_BUFF_IOVA,
			mvm_dev->mvm_base + MVMSS_CSR_APSS_MVM_SCRATCH_PAD0);

	initialise_fifos(mvm_dev);
	/*Send an interrupt to MVM to indicate MVM_Init done */
	writel_relaxed(IRQ_APSS0, mvm_dev->apss_shared_base);
	mvm_dev->state = MVM_ONLINE;
	send_mvm_state_to_user(mvm_dev);
#ifdef KEEP_FW_IN_DDR
	goto success;
#else
	goto release_fw_dma_mem;
#endif

out_release_firmware:
	release_firmware(fw);

release_fw_dma_mem:
	dma_free_coherent(mvm_dev->dev, MVM_FW_SIZE, mvm_dev->mvm_fw, mvm_dev->mvm_fw_dma);

fw_dma_mem_fail:
success:
	return ret;
}

static void trigger_ssr_work_hdlr(struct work_struct *work)
{
	struct mvm_device *mvm_dev = container_of(work, struct mvm_device, trigger_ssr_work);
	int ret = 0;
	struct mvm_client *mvm_cli;
	bool p0_fifo_has_results =0 ,p1_fifo_has_results =0;

	ret = qcom_scm_pas_shutdown(MVM_PROC_ID);
	if (ret) {
		dev_err(mvm_dev->dev, "Error sending shutdown request to MVM\n");
		return;
	}

	list_for_each_entry(mvm_cli, &mvm_dev->client_list, list) {
		mvm_cli->client_ready= false;
		dev_dbg(mvm_dev->dev, "Set client flag %d\n",mvm_cli->client_id);
	}
	reinit_completion(&mvm_dev->mvm_dump_collection_done);
	mutex_lock(&mvm_dev->out_fifo_lock);
	p0_fifo_has_results = out_fifo_get_results(mvm_dev, 0);
	p1_fifo_has_results = out_fifo_get_results(mvm_dev, 1);
	mutex_unlock(&mvm_dev->out_fifo_lock);
	if (p0_fifo_has_results || p1_fifo_has_results)
		dev_info(mvm_dev->dev, "mvm outfifo has results and draining out fifo started\n");
	wake_up_interruptible_poll(&mvm_dev->mvm_waitqueue, POLLIN | POLLPRI);
	send_mvm_state_to_user(mvm_dev);//Send crash signal to clients

	ret = wait_for_completion_interruptible_timeout(
			&mvm_dev->mvm_dump_collection_done,
			msecs_to_jiffies(MVM_DUMP_COLL_TIMEOUT_MS));
	if (ret == 0) {
		dev_err(mvm_dev->dev, "Timed out as mvm dump collection is not complete\n");
	}
	mvm_dev->pending_dump_read = true;
	wake_up_interruptible_poll(&mvm_dev->ssr_poll_wait, POLLIN | POLLPRI);

	dev_info(mvm_dev->dev, "MVM subsystem is restarting after SSR\n");
	enable_gcc_clocks(mvm_dev);
	enable_mvm_gdsc(mvm_dev, true);

	mvm_dev->state = MVM_RESTARTING;
	send_mvm_state_to_user(mvm_dev);

	mvm_load_fw(mvm_dev);

	return;
}

static irqreturn_t mvm_wdog_irq_handler(int irq, void *dev_id)
{
	struct mvm_device *mvm_dev = dev_id;

	dev_info(mvm_dev->dev, "Received watchdog bite from MVM\n");
	mvm_dev->state = MVM_CRASHED;
	dev_dbg(mvm_dev->dev, "The current state of MVM is CRASHED\n");
	schedule_work(&mvm_dev->trigger_ssr_work);

	return IRQ_HANDLED;
}

static int register_isrs(struct platform_device *pdev)
{
	int ret = -1;
	struct mvm_device *mvm_dev;
	struct irq_data *data;

	mvm_dev = platform_get_drvdata(pdev);

	mvm_dev->mvm_ssr_done_irq = platform_get_irq_byname(pdev, "mvm_ssr_done");
	if (mvm_dev->mvm_ssr_done_irq < 0) {
		dev_err(mvm_dev->dev, "mvm_ssr_done irq not defined\n");
		return ret;
	}

	ret = devm_request_threaded_irq(mvm_dev->dev, mvm_dev->mvm_ssr_done_irq,
					NULL, mvm_ssr_done_irq_handler,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT |
					IRQF_NO_SUSPEND, "mvm_ssr_done", mvm_dev);
	if (ret < 0) {
		dev_err(mvm_dev->dev,
			"devm_request_threaded_irq of mvm_ssr_done failed %d\n", ret);
		return ret;
	}
	data = irq_get_irq_data(mvm_dev->mvm_ssr_done_irq);
	mvm_dev->mvm_ssr_done_hw_irq = data->hwirq;

	dev_dbg(mvm_dev->dev, "mvm_ssr_done irq registered\n");

	mvm_dev->mvm_verif_done_irq = platform_get_irq_byname(pdev, "mvm_verif_done");
	if (mvm_dev->mvm_verif_done_irq < 0) {
		dev_err(mvm_dev->dev, "mvm_verification_done irq not defined\n");
		return ret;
	}
	ret = devm_request_threaded_irq(mvm_dev->dev, mvm_dev->mvm_verif_done_irq,
					NULL, mvm_verif_done_irq_handler,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT |
					IRQF_NO_SUSPEND, "mvm_verif_done", mvm_dev);
	if (ret < 0) {
		dev_err(mvm_dev->dev,
			"devm_request_threaded_irq of mvm_verif_done_irq failed %d\n", ret);
		return ret;
	}
	data = irq_get_irq_data(mvm_dev->mvm_verif_done_irq);
	mvm_dev->mvm_verif_done_hw_irq = data->hwirq;
	dev_dbg(mvm_dev->dev, "mvm_verif_done registered\n");

	mvm_dev->wfi_irq = platform_get_irq_byname(pdev, "wfi");
	if (mvm_dev->wfi_irq < 0) {
		dev_err(mvm_dev->dev, "wfi irq not defined\n");
		return ret;
	}

	ret = devm_request_threaded_irq(mvm_dev->dev, mvm_dev->wfi_irq,
					NULL, mvm_wfi_irq_handler,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT |
					IRQF_NO_SUSPEND, "mvm_wfi", mvm_dev);
	if (ret < 0) {
		dev_err(mvm_dev->dev,
			"devm_request_threaded_irq of wfi_irq failed %d\n", ret);
		return ret;
	}
	dev_dbg(mvm_dev->dev, "wfi_irq registered\n");

	mvm_dev->wdog_irq = platform_get_irq_byname(pdev, "wdog");
	if (mvm_dev->wdog_irq < 0) {
		dev_err(mvm_dev->dev, "wdog irq not defined\n");
		return ret;
	}

	ret = devm_request_threaded_irq(mvm_dev->dev, mvm_dev->wdog_irq,
					NULL, mvm_wdog_irq_handler,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"mvm_dog", mvm_dev);
	if (ret) {
		dev_err(mvm_dev->dev, "mvm_wdog irq request failed\n");
		return ret;
	}
	dev_dbg(mvm_dev->dev, "wdog irq registered\n");

	return 0;
}

static void mvm_iommu_release(struct mvm_device *mvm_dev)
{
	iommu_unmap(mvm_dev->domain, RING_BUFF_IOVA, round_up(sizeof(struct ring_buffers), PAGE_SIZE));
	iommu_unmap(mvm_dev->domain, MVM_DUMP_BUFF_IOVA, round_up(sizeof(struct mvm_crashdump_buffer), PAGE_SIZE));
	iommu_unmap(mvm_dev->domain, LOG_BUFF_IOVA, round_up(sizeof(struct mvmlog_buffers), PAGE_SIZE));
	iommu_detach_device(mvm_dev->domain, mvm_dev->dev);
	iommu_domain_free(mvm_dev->domain);
}

static int mvm_iommu_init(struct mvm_device *mvm_dev)
{
	int ret = 0;

	mvm_dev->domain = iommu_domain_alloc(mvm_dev->dev->bus);
	if (!mvm_dev->domain) {
		dev_err(mvm_dev->dev, "failed to allocate iommu domain\n");
		ret = -ENODEV;
		goto fail;
	}

	ret = iommu_attach_device(mvm_dev->domain, mvm_dev->dev);
	if (ret) {
		dev_err(mvm_dev->dev, "failed to attach device ret = %d\n", ret);
		goto attach_device_fail;
	}

	ret = iommu_map(mvm_dev->domain, RING_BUFF_IOVA, mvm_dev->ring_buff_dma,
			round_up(sizeof(struct ring_buffers), PAGE_SIZE), IOMMU_READ | IOMMU_WRITE);
	if (ret) {
		dev_err(mvm_dev->dev, "iommu_map for ring_buffers failed\n");
		goto ring_buff_iommu_map_fail;
	}

	ret = iommu_map(mvm_dev->domain, MVM_DUMP_BUFF_IOVA, mvm_dev->mvm_dump_dma,
			round_up(sizeof(struct mvm_crashdump_buffer), PAGE_SIZE), IOMMU_READ | IOMMU_WRITE);
	if (ret) {
		dev_err(mvm_dev->dev, "iommu_map for dump_buffers failed\n");
		goto dump_buff_iommu_map_fail;
	}

	ret = iommu_map(mvm_dev->domain, LOG_BUFF_IOVA, mvm_dev->mvmlog_buff_dma,
			round_up(sizeof(struct mvmlog_buffers), PAGE_SIZE), IOMMU_READ | IOMMU_WRITE);
	if (ret) {
		dev_err(mvm_dev->dev, "iommu_map for log buffers failed\n");
		goto log_buff_iommu_map_fail;
	}

	return 0;

log_buff_iommu_map_fail:
	iommu_unmap(mvm_dev->domain, MVM_DUMP_BUFF_IOVA, round_up(sizeof(struct mvm_crashdump_buffer), PAGE_SIZE));
dump_buff_iommu_map_fail:
	iommu_unmap(mvm_dev->domain, RING_BUFF_IOVA, round_up(sizeof(struct ring_buffers), PAGE_SIZE));
ring_buff_iommu_map_fail:
	iommu_detach_device(mvm_dev->domain, mvm_dev->dev);
attach_device_fail:
	iommu_domain_free(mvm_dev->domain);
fail:
	return ret;
}

static void mvm_dma_mem_free(struct mvm_device *mvm_dev)
{
	dma_free_coherent(mvm_dev->dev, round_up(sizeof(struct mvmlog_buffers), PAGE_SIZE),
					mvm_dev->log_buff, mvm_dev->mvmlog_buff_dma);
	dma_free_coherent(mvm_dev->dev, round_up(sizeof(struct mvm_crashdump_buffer), PAGE_SIZE),
					mvm_dev->dump_buff, mvm_dev->mvm_dump_dma);
	dma_free_coherent(mvm_dev->dev, round_up(sizeof(struct ring_buffers), PAGE_SIZE),
					mvm_dev->ring_buff, mvm_dev->ring_buff_dma);
}

static int mvm_dma_mem_alloc(struct mvm_device *mvm_dev)
{
	int ret = 0;

        mvm_dev->ring_buff = dma_alloc_coherent(mvm_dev->dev, round_up(sizeof(struct ring_buffers), PAGE_SIZE),
								&mvm_dev->ring_buff_dma, GFP_KERNEL);
        if (!mvm_dev->ring_buff) {
		dev_err(mvm_dev->dev, "dma_alloc_coherent of ring buffers failed\n");
		ret = -ENOMEM;
		goto ring_buff_dma_mem_alloc_fail;
	}

        mvm_dev->dump_buff = dma_alloc_coherent(mvm_dev->dev, round_up(sizeof(struct mvm_crashdump_buffer), PAGE_SIZE),
								&mvm_dev->mvm_dump_dma, GFP_KERNEL);
	if (!mvm_dev->dump_buff) {
		dev_err(mvm_dev->dev, "dma_alloc_coherent of dump buffers failed\n");
		ret = -ENOMEM;
		goto dump_dma_mem_alloc_fail;
	}

	mvm_dev->log_buff = dma_alloc_coherent(mvm_dev->dev, round_up(sizeof(struct mvmlog_buffers), PAGE_SIZE),
								&mvm_dev->mvmlog_buff_dma, GFP_KERNEL);
	if (!mvm_dev->log_buff) {
		dev_err(mvm_dev->dev, "dma_alloc_coherent of log buffers failed\n");
		ret = -ENOMEM;
		goto log_buff_dma_mem_alloc_fail;
	}

	return 0;

log_buff_dma_mem_alloc_fail:
	dma_free_coherent(mvm_dev->dev, round_up(sizeof(struct mvm_crashdump_buffer), PAGE_SIZE),
					mvm_dev->dump_buff, mvm_dev->mvm_dump_dma);
dump_dma_mem_alloc_fail:
	dma_free_coherent(mvm_dev->dev, round_up(sizeof(struct ring_buffers), PAGE_SIZE),
					mvm_dev->ring_buff, mvm_dev->ring_buff_dma);
ring_buff_dma_mem_alloc_fail:
	return ret;
}

static const struct file_operations mvm_fileops = {
	.open = mvm_open,
	.release = mvm_release,
	.write = mvm_write,
	.unlocked_ioctl = mvm_ioctl,
	.poll = mvm_poll,
	.read = mvm_read,
	.owner = THIS_MODULE,
};

static void disable_gcc_clocks(struct mvm_device *mvm_dev)
{
	clk_disable_unprepare(mvm_dev->snoc_m_axi_clk);
	clk_disable_unprepare(mvm_dev->cnoc_s_ahb_clk);
	clk_disable_unprepare(mvm_dev->sysnoc_mvmss_clk);
}

static int enable_gcc_clocks(struct mvm_device *mvm_dev)
{
	int ret = 0;

	mvm_dev->xo = devm_clk_get(mvm_dev->dev, "xo");
	if (IS_ERR(mvm_dev->xo))
		return PTR_ERR(mvm_dev->xo);

	mvm_dev->cnoc_s_ahb_clk = devm_clk_get(mvm_dev->dev, "mvmss_cnoc_ahb_clk");
	if (IS_ERR(mvm_dev->cnoc_s_ahb_clk))
		return PTR_ERR(mvm_dev->cnoc_s_ahb_clk);

	mvm_dev->snoc_m_axi_clk = devm_clk_get(mvm_dev->dev, "mvmss_snoc_axi_clk");
	if (IS_ERR(mvm_dev->snoc_m_axi_clk))
		return PTR_ERR(mvm_dev->snoc_m_axi_clk);

	mvm_dev->sysnoc_mvmss_clk = devm_clk_get(mvm_dev->dev, "sysnoc_mvmss_clk");
	if (IS_ERR(mvm_dev->sysnoc_mvmss_clk))
		return PTR_ERR(mvm_dev->sysnoc_mvmss_clk);

	ret = clk_prepare_enable(mvm_dev->xo);
	if (ret) {
		dev_err(mvm_dev->dev, "Failed to vote for XO clk\n");
		return ret;
	}
	ret = clk_prepare_enable(mvm_dev->cnoc_s_ahb_clk);
	if (ret) {
		dev_err(mvm_dev->dev, "Failed to vote for cnoc_s_ahb_clk\n");
		goto unprepare_xo;
	}
	ret = clk_prepare_enable(mvm_dev->snoc_m_axi_clk);
	if (ret) {
		dev_err(mvm_dev->dev, "Failed to vote for snoc_m_axi_clk\n");
		goto unprepare_cnoc_s_ahb;
	}
	ret = clk_prepare_enable(mvm_dev->sysnoc_mvmss_clk);
	if (ret) {
		dev_err(mvm_dev->dev, "Failed to vote for sysnoc_mvmss_clk\n");
		goto unprepare_snoc_m_axi;
	}
	return 0;

unprepare_snoc_m_axi:
	clk_disable_unprepare(mvm_dev->snoc_m_axi_clk);
unprepare_cnoc_s_ahb:
	clk_disable_unprepare(mvm_dev->cnoc_s_ahb_clk);
unprepare_xo:
	clk_disable_unprepare(mvm_dev->xo);
	return ret;
}

static int mvm_suspend(struct device *dev)
{
	struct mvm_device *mvm_dev = dev_get_drvdata(dev);
	int is_suspend = 0;

	mutex_lock(&mvm_dev->mvm_csr_lock);

	if (mvm_dev->incoming_msgs == mvm_dev->outgoing_results) {
		/* prepare power collapse control message */
		struct mvm_control *mvm_ctrl;
		reinit_completion(&mvm_dev->mvm_wfi_irq_recvd);
		mvm_ctrl = kzalloc(sizeof(struct mvm_control), GFP_KERNEL);
		mvm_ctrl->type = MVM_POWER;
		mvm_ctrl->mvm_ctrl_msg.power.enter_pwr_collapse = 1;
		is_suspend= send_ctrl_msg_to_mvm(mvm_ctrl, mvm_dev);
		kfree(mvm_ctrl);
		if (is_suspend) {
			is_suspend = -EBUSY;
		} else {
			mvm_dev->resume_frm_pwr_collapse = false;
			is_suspend = wait_for_completion_interruptible_timeout(
				&mvm_dev->mvm_wfi_irq_recvd,
				msecs_to_jiffies(TIMEOUT_MS));
			if (is_suspend == 0) {
				mvm_dev->resume_frm_pwr_collapse = true;
				enable_wfi_int(mvm_dev, false);
				disable_irq(mvm_dev->wfi_irq);
				dev_err(mvm_dev->dev, "Timed out waiting for mvm core collapse\n");
				is_suspend = -EBUSY;
			} else {
				collapse_mvm_core(mvm_dev);
				disable_gcc_clocks(mvm_dev);
				is_suspend = 0;
				dev_dbg(mvm_dev->dev, "MVM subsystem in power collapse mode\n");
			}
		}
		/*TODO
		 * 1. Vote for power collapse to aop
		 */
	} else {
		is_suspend = -EBUSY;
		dev_err(mvm_dev->dev, "E21 has pending message for verification,can't suspend now\n");
	}
	mutex_unlock(&mvm_dev->mvm_csr_lock);
	mvm_dev->state = MVM_SLEEP;
	return is_suspend;
}

static int mvm_resume(struct device *dev)
{
	struct mvm_device *mvm_dev = dev_get_drvdata(dev);
	struct mvm_control *mvm_ctrl;
	int is_resume = 0;

	mvm_dev->resume_frm_pwr_collapse = true;
	/*TODO
	 * 1.vote for aop
	 */
	is_resume = enable_gcc_clocks(mvm_dev);
	if (is_resume) {
		dev_err(mvm_dev->dev, "Failed to turn on gcc clocks\n");
		is_resume = -EHOSTDOWN;
	} else {
		restore_mvm_core(mvm_dev);
		mvm_ctrl = kzalloc(sizeof(struct mvm_control), GFP_KERNEL);
		mvm_ctrl->type = MVM_POWER;
		mvm_ctrl->mvm_ctrl_msg.power.enter_pwr_collapse = 0;
		is_resume = send_ctrl_msg_to_mvm(mvm_ctrl, mvm_dev);
		kfree(mvm_ctrl);
		enable_wfi_int(mvm_dev, false);
	}
	dev_dbg(mvm_dev->dev, "MVM subsystem in Restored\n");
	return is_resume;
}

static int mvm_probe(struct platform_device *pdev)
{
	struct device_node *node;
	struct mvm_device *mvm_dev;
	struct device *dev;
	int ret;

	node = pdev->dev.of_node;
	mvm_dev = devm_kzalloc(&pdev->dev, sizeof(*mvm_dev), GFP_KERNEL);
	if (!mvm_dev)
		return -ENOMEM;

	mvm_dev->dev = &pdev->dev;

	platform_set_drvdata(pdev, mvm_dev);

	ret = alloc_chrdev_region(&mvm_dev->mvm_cdev_devid, 0, 1, "mvm");
	if (ret < 0) {
		dev_err(mvm_dev->dev,
			"can't allocate major number, %d\n", ret);
		goto drv_err;
	}

	cdev_init(&mvm_dev->mvm_cdev, &mvm_fileops);
	cdev_add(&mvm_dev->mvm_cdev, mvm_dev->mvm_cdev_devid, 1);
	mvm_dev->mvm_class = class_create(THIS_MODULE, "mvm");
	if (IS_ERR(mvm_dev->mvm_class)) {
		dev_err(mvm_dev->dev,
			"can't create rmt_sys_evt class, %d\n",
			-ENOMEM);
		goto class_fail;
	}

	dev = device_create(mvm_dev->mvm_class, &pdev->dev,
				mvm_dev->mvm_cdev_devid, mvm_dev,
				"mvm");
	if (IS_ERR(dev)) {
		dev_err(mvm_dev->dev,
				"can't create rmt_sys_evt device, %d\n",
				-ENOMEM);
		goto device_fail;
	}

	dev_dbg(mvm_dev->dev, "mvm character device driver created\n");

	ret = mvm_dma_mem_alloc(mvm_dev);
	if (ret)
		goto dma_mem_fail;

	ret = mvm_iommu_init(mvm_dev);
	if (ret)
		goto iommu_init_fail;

	ret = ioremap_resources(pdev);
	if (ret) {
		dev_err(mvm_dev->dev, "Cant ioremap resources\n");
		goto ioremap_fail;
	}

	ret = register_isrs(pdev);
	if (ret < 0) {
		dev_err(mvm_dev->dev,
			"registration of isrs failed\n");
		goto ioremap_fail;
	}
	mutex_init(&mvm_dev->mvm_cli_lock);
	mutex_init(&mvm_dev->mvm_csr_lock);
	mutex_init(&mvm_dev->in_fifo_lock);
	mutex_init(&mvm_dev->out_fifo_lock);
	INIT_LIST_HEAD(&mvm_dev->client_list);
	init_waitqueue_head(&mvm_dev->mvm_waitqueue);
	init_waitqueue_head(&mvm_dev->log_poll_wait);
	init_waitqueue_head(&mvm_dev->ssr_poll_wait);
	bitmap_zero(mvm_dev->client_id_bitmap, MAX_CLIENT_COUNT);
	INIT_WORK(&mvm_dev->drain_out_fifo_work, drain_out_fifo_work_hdlr);
	INIT_WORK(&mvm_dev->trigger_ssr_work, trigger_ssr_work_hdlr);
	init_completion(&mvm_dev->p0_fifo_slot_available);
	init_completion(&mvm_dev->p1_fifo_slot_available);
	init_completion(&mvm_dev->mvm_wfi_irq_recvd);
	ret = mvm_debugfs_init(mvm_dev);
	init_completion(&mvm_dev->mvm_dump_collection_done);
	mvm_dev->pending_dump_read = false;

	ret = mvm_sysfs_init(mvm_dev);
	if (ret) {
		dev_err(mvm_dev->dev, "mvm sysfs initialisation failed\n");
		goto mutex_err;
	}
	mvm_dev->state = MVM_OFFLINE;
	send_mvm_state_to_user(mvm_dev);
	dev_dbg(mvm_dev->dev, "The current state of MVM is OFFLINE\n");

	ret = enable_gcc_clocks(mvm_dev);
        if (ret) {
                dev_err(mvm_dev->dev, "Failed to turn on gcc clocks\n");
		goto mutex_err;
	}

	ret = enable_mvm_gdsc(mvm_dev, true);
	if (ret) {
		dev_err(mvm_dev->dev, "Failed to turn on mvm gdsc\n");
		goto gdsc_err;
	}

	ret = mvm_load_fw(mvm_dev);
	if (ret)
		goto gdsc_err;

	mvm_dev->resume_frm_pwr_collapse = true;
	return 0;

gdsc_err:
	clk_disable_unprepare(mvm_dev->cnoc_s_ahb_clk);
	clk_disable_unprepare(mvm_dev->xo);
	clk_disable_unprepare(mvm_dev->snoc_m_axi_clk);
	clk_disable_unprepare(mvm_dev->sysnoc_mvmss_clk);
mutex_err:
	mutex_destroy(&mvm_dev->mvm_csr_lock);
	mutex_destroy(&mvm_dev->mvm_cli_lock);
ioremap_fail:
	mvm_iommu_release(mvm_dev);
iommu_init_fail:
	mvm_dma_mem_free(mvm_dev);
dma_mem_fail:
	device_destroy(mvm_dev->mvm_class, mvm_dev->mvm_cdev_devid);
device_fail:
	class_destroy(mvm_dev->mvm_class);
class_fail:
	cdev_del(&mvm_dev->mvm_cdev);
	unregister_chrdev_region(mvm_dev->mvm_cdev_devid, 1);
drv_err:
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int mvm_remove(struct platform_device *pdev)
{
	struct mvm_device *mvm_dev;

	mvm_dev = dev_get_drvdata(&pdev->dev);
	sysfs_remove_file(mvm_dev->kobj, &mvm_dev->attr.attr);
	kobject_put(mvm_dev->kobj);
	mutex_destroy(&mvm_dev->mvm_csr_lock);
	mutex_destroy(&mvm_dev->mvm_cli_lock);
	mvm_iommu_release(mvm_dev);
	mvm_dma_mem_free(mvm_dev);
	device_destroy(mvm_dev->mvm_class, mvm_dev->mvm_cdev_devid);
	class_destroy(mvm_dev->mvm_class);
	cdev_del(&mvm_dev->mvm_cdev);
	unregister_chrdev_region(mvm_dev->mvm_cdev_devid, 1);
	return 0;
}

static const struct dev_pm_ops mvm_pm_ops = {
	.suspend        =    mvm_suspend,
	.resume         =    mvm_resume,
};

static const struct of_device_id mvm_of_match[] = {
	{ .compatible = "qcom,mvm"},
	{},
};

MODULE_DEVICE_TABLE(of, mvm_of_match);

static struct platform_driver mvm_driver = {
	.probe		= mvm_probe,
	.remove		= mvm_remove,
	.driver = {
		.name	= "mvm",
		.of_match_table = mvm_of_match,
		.pm = &mvm_pm_ops,
	},
};

static int __init mvm_register(void)
{
	return platform_driver_register(&mvm_driver);
}
module_init(mvm_register);

static void __exit mvm_unregister(void)
{
	platform_driver_unregister(&mvm_driver);
}
module_exit(mvm_unregister);
MODULE_LICENSE("GPL v2");
