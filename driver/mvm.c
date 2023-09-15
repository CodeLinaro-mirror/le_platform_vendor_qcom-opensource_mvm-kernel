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
#include <soc/qcom/secure_buffer.h>
#include <linux/gunyah/gh_rm_drv.h>
#include <linux/gunyah/gh_vm.h>
#include <linux/qcom-iommu-util.h>
#include <linux/gunyah/gh_dbl.h>
#include <linux/suspend.h>
#include <linux/kthread.h>
#include <linux/timekeeping.h>
#include <linux/moduleparam.h>

#define MVM_NOTIFY_SIGNO		SIGRTMAX
#define MVM_NOTIFY_VALUE_CAPACITY	11
#define DDR_FIFO_COUNT			2
#define DDR_FIFO_SIZE			128
#define MSG_PRIORITY_BIT		BIT(19)
#define OUT_BUFF_SIZE			32
#define TIMEOUT_MS			10000
#define WFI_TIMEOUT_MS				5000
#define MAX_CLIENT_COUNT			15
#define MIN_CLOCK_CHANGE_TIME_MS	500

/* CSR to enable WFI interrupt from E21 to APPS */
#define MVMSS_CSR_RVSS_CFG_OFFSET	0x00000010
#define MVMSS_CSR_WFI_EN_MASK		0xFFFFFFFB
#define MVMSS_CSR_WFI_EN_SHIFT		2

#define MVMSS_CSR_MVMSS_APSS		0x00000028
#define IRQ_APSS0			0x00100000
#define IRQ_APSS1                       0x00200000
#define IRQ_APSS2                       0x00400000
#define IRQ_APSS3                       0x00800000

#define MVMSS_REG_BASE				0x12000000
#define MVMSS_REG_SIZE				0x30000
#define MVMSS_LEND_SIZE				0x9000
#define APSS_SHARED_BASE_START			0x17400000
#define APSS_SHARED_BASE_END			0x1000
#define MVMSS_CSR_INPUT_RING0_BASE_ADDR		0x00001000
#define MVMSS_CSR_INPUT_RING0_BUFFER_LENGTH	0x00001004
#define MVMSS_CSR_OUTPUT_RING0_BASE_ADDR	0x00001008
#define MVMSS_CSR_OUTPUT_RING0_BUFFER_LENGTH	0x0000100C
#define MVMSS_CSR_INPUT_RING1_BASE_ADDR		0x00003000
#define MVMSS_CSR_INPUT_RING1_BUFFER_LENGTH	0x00003004
#define MVMSS_CSR_OUTPUT_RING1_BASE_ADDR	0x00003008
#define MVMSS_CSR_OUTPUT_RING1_BUFFER_LENGTH	0x0000300C
#define MVMSS_CSR_INPUT_RING0_TAIL_PTR_OFFSET	0x00002000
#define MVMSS_CSR_INPUT_RING0_HEAD_PTR_OFFSET	0x00007000
#define MVMSS_CSR_INPUT_RING1_TAIL_PTR_OFFSET	0x00004000
#define MVMSS_CSR_INPUT_RING1_HEAD_PTR_OFFSET	0x00008000

#define MVMSS_CSR_OUTPUT_RING0_TAIL_PTR_OFFSET	0x00007004
#define MVMSS_CSR_OUTPUT_RING0_HEAD_PTR_OFFSET	0x00002004
#define MVMSS_CSR_OUTPUT_RING1_TAIL_PTR_OFFSET	0x00008004
#define MVMSS_CSR_OUTPUT_RING1_HEAD_PTR_OFFSET	0x00004004

#define MVMSS_CSR_APSS_MVM_SCRATCH_PAD0 	0x00005000
#define MVMSS_CSR_APSS_MVM_SCRATCH_PAD1 	0x00005004
#define MVM_INIT_DONE_COOKIE			0xc0deba5e
#define SW_COLLAPSE_MASK			0xFFFFFFFE
#define MVM_CC_MVM_GDSCR_OFFSET			0x00028004
#define MVM_CC_AHB_CORE_CBCR_OFFSET     	0x00028120
#define MVM_CC_E21CPU_CC_CBCR_OFFSET       	0x00028030
#define CLK_EN_MASK				0xFFFFFFFE

/* MVMSS CSRs */

/* CSR to enable WFI interrupt from E21 to APPS */
#define MVMSS_CSR_RVSS_CFG_OFFSET       0x00000010
#define MVMSS_CSR_RVSS_STS_OFFSET	0x00000014
#define MVMSS_CSR_WFI_EN_MASK           0xFFFFFFFB
#define MVMSS_CSR_WFI_EN_SHIFT          2

/* MVM_CC CSRs for mvm core power collapse/restore */

#define CLK_EN_MASK                     0xFFFFFFFE
#define MVM_CC_MVM_GDSCR_OFFSET         0x00028004
#define RETAIN_FF_ENABLE_MASK           0xFFFFF7FF
#define RETAIN_FF_ENABLE_SHIFT          11
#define SW_COLLAPSE_MASK                0xFFFFFFFE
#define GDSC_PWR_DOWN_COMPLETE          15
#define GDSC_PWR_UP_COMPLETE            16
#define MVM_CC_MVM_CFG_GDSCR_OFFSET     0x00028008
#define MVM_CC_AHB_CORE_CBCR_OFFSET     0x00028120

/* MVM_CC CSRs to switch RCGs to XO */
#define MVM_CC_E21CPU_CFG_RCGR_OFFSET   0x0002801C
#define MVM_CC_SRC_DIV_DISABLE          0xFFFFF8E0
#define MVM_CC_PKE_CFG_RCGR_BASE_OFFSET 0x00028068
#define MVM_CC_PKEn_CFG_RCGR_OFFSET(pke)        MVM_CC_PKE_CFG_RCGR_BASE_OFFSET + (24 * pke)
#define MVM_CC_BUS_CFG_RCGR_OFFSET      0x000280D4
#define MVM_CC_SLEEP_CFG_RCGR_OFFSET    0x00028128
#define MVM_CC_XO_CFG_RCGR_OFFSET       0x00028144

#define MVM_CC_AHB_CORE_CBCR_OFFSET     0x00028120

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
#define AC_VM_HLOS                      3
#define APSS_SHARED_IPC_INTERRUPT_OFFSET	0xC
#define DBL_MASK			0x1
#define MVM_SHUTDOWN_DBL_MASK		0x1
#define MVM_LOAD_FW_DBL_MASK		0x2
#define MVM_INIT_FIFOS_DBL_MASK		0x3
#define MVM_DO_SUSPEND_DBL_MASK		0x4
#define MVM_DO_RESUME_DBL_MASK		0x5
#define MVM_SUSPEND_DONE_DBL_MASK	0x6
#define MVM_RESUME_DONE_DBL_MASK	0x7
#define MAX_CURVES			5
#define MAX_FREQ_PLAN			3
#define MVM_STATS_TIMEOUT_MS		100
#define MVM_INITIAL_CAPACITY 		100

#define CLK_CHANGE_MIN_CAPACITY		3001
#define PKE_UTIL_ADJUST_THRESH		90
#define PKE_UTIL_UPPER_THRESH		70
#define PKE_UTIL_LOWER_THRESH		40
#define MAX_PKE_UTIL_LOW_SVS		100
#define PKE_UTIL_ADJUST_FACTOR_LOWSVS(x)	(x/2)
#define PKE_UTIL_ADJUST_FACTOR_SVS(x)		((x*3)/4)
#define MVM_STATS_TIMER_DEFAULT_INTERVAL_MS	300
#define PKE_UTIL_PERIOD_TICKS (19200 * mvm_stats_timer_interval_ms)
#define PKE_COUNT 3
#define LOW_SVS_THRESH_RANGE	1 ... 50
#define SVS_THRESH_RANGE		51 ... 70
#define NOMINAL_THRESH_RANGE	71 ... 100
#define LOWSVS_RATE_LIMIT		3000

/**
 * enum mvm_state - state of mvm subsystem
 * @MVM_OFFLINE: MVM firmware is not loaded/authenticated yet.
 * @MVM_ONLINE: The MVM firmware has been loaded, authenticate and MVMSS is up and running.
 * @MVM_CRASHED: Watchdog bite is received from MVM to APSS.
 * @MVM_RESTARTING: The mvm dump has been collected and ssr_done_irq is received from MVM to APSS.
 * @MVM_SUSPEND:send suspend notification to client so client will halt the task.
**/

enum mvm_state {
	MVM_OFFLINE,
	MVM_ONLINE,
	MVM_CRASHED,
	MVM_RESTARTING,
	MVM_SUSPEND,
};

enum vm_variant {
	PVM_ONLY = 1,
	HOSTVM,
	TELEVM,
};

enum client_state {
	CLIENT_READY,
	CLIENT_SSR,
	CLIENT_DISCONNECTING,
	CLIENT_SUSPEND
};

enum mvm_curve {
	CURVE_NISTP256,
	CURVE_BP256R1,
	CURVE_SM2,
	CURVE_NISTP384,
	CURVE_BP384R1,
};

static const char * const mvm_states[] = {
	[MVM_OFFLINE] = "OFFLINE",
	[MVM_ONLINE] = "ONLINE",
	[MVM_CRASHED] = "CRASHED",
	[MVM_RESTARTING] = "RESTARTING",
	[MVM_SUSPEND] = "SUSPEND",
};

static const char * const mvm_clk_freq_plan[] = {
	[NOMINAL] = "NOMINAL",
	[SVS] = "SVS",
	[LOW_SVS] = "LOW_SVS",
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
	unsigned int pke_time_accumulator;
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
	enum client_state state;
	struct output_buffer *out_buff;
	enum mvm_log_policy log_policy;
	struct task_struct *task;
	unsigned int msgs_sent;
	unsigned int results_recvd;
};

struct mvm_stats_client {
	struct list_head list;
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
	struct mutex in_fifo_lock;
	struct mutex out_fifo_lock;
	wait_queue_head_t mvm_waitqueue;
	wait_queue_head_t log_poll_wait;
	wait_queue_head_t ssr_poll_wait;
	struct work_struct drain_out_fifo_work;
	struct work_struct trigger_ssr_work;
	struct work_struct change_clk_freq_work;
	struct completion p0_fifo_slot_available;
	struct completion p1_fifo_slot_available;
	struct completion mvm_wfi_irq_recvd;
	struct completion mvm_dump_collection_done;
	struct completion mvm_suspend_done;
	struct completion mvm_resume_done;
	struct kobject *kobj;
	struct kobj_attribute mvm_state_attr;
	struct kobj_attribute mvm_capacity_attr;
	struct kobj_attribute mvm_curr_clk_attr;
	struct kobj_attribute mvm_max_clk_attr;
	struct kobj_attribute mvm_rate_lut_attr;
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
	enum vm_variant vm_variant;
	uint32_t iomem_gunyah_label;
	uint32_t mvm_ring_buff_shm_label;
	uint32_t mvm_dump_buff_shm_label;
	uint32_t mvm_log_buff_shm_label;
	gh_memparcel_handle_t mvmss_mem_handle;
	gh_memparcel_handle_t apss_mem_handle;
	gh_memparcel_handle_t ring_buff_mem_handle;
	gh_memparcel_handle_t dump_buff_mem_handle;
	gh_memparcel_handle_t log_buff_mem_handle;
	void *televm_tx_dbl;
	void *televm_rx_dbl;
	void *hostvm_tx_dbl;
	void *hostvm_rx_dbl;
	const struct firmware *fw;
	struct notifier_block pm_notifier;
	uint32_t max_rate;
	uint32_t mvm_capacity[MAX_CURVES];
	mvm_clk_freq req_clk;
	mvm_clk_freq curr_clk;
	mvm_clk_freq max_clk;
	enum mvm_curve curve;
	struct timer_list mvm_stats_timer;
	uint32_t mvm_in_msg_count[MAX_CURVES];
	unsigned int prev_pke_time;
	unsigned int curr_pke_util ;
	unsigned int prev_pke_util;
	dev_t mvm_stats_cdev_devid;
	struct list_head mvm_stats_client_list;
	struct cdev mvm_stats_cdev;
	struct class *mvm_stats_class;
	struct device *mvm_stats_dev;
	ktime_t clk_time_elapsed;
	struct notifier_block rm_nb;
	gh_vmid_t televm_vmid;
};

static const uint32_t mvm_rate_lut[MAX_FREQ_PLAN][MAX_CURVES] = {
	/* NISTP256, BP256, SM2, NISTP384, BP384 */
	{5817, 3014, 6485, 1992, 1004},    /* LOW_SVS */
	{9551, 4955, 10706, 3291, 1663},   /* SVS */
	{14285, 7434, 16064, 4938, 2493}}; /*NOMINAL */

// Static copies for crashscope to access these variables
static struct mvm_crashdump_buffer *crashdump_buffer = NULL;
__attribute__((used))
static dma_addr_t crashdump_dma_addr = 0;
__attribute__((used))
static size_t crashdump_size = MVM_CRASH_DUMP_SIZE;

static int enable_gcc_clocks(struct mvm_device *mvm_dev);
int qcom_pil_info_store(const char *image, phys_addr_t base, size_t size);
static int mvm_stats_timer_interval_ms = MVM_STATS_TIMER_DEFAULT_INTERVAL_MS;//set 300ms as default value

static void send_mvm_state_to_user(struct mvm_device *mvm_dev, enum mvm_state state)
{
	struct kernel_siginfo info;
	struct mvm_client *mvm_cli;

	memset(&info, 0, sizeof(struct kernel_siginfo));
	info.si_signo = MVM_NOTIFY_SIGNO;
	info.si_int = state;

	mutex_lock(&mvm_dev->mvm_cli_lock);
	list_for_each_entry(mvm_cli, &mvm_dev->client_list, list) {
		if ((mvm_cli->task != NULL) && (mvm_cli->state != CLIENT_DISCONNECTING )) {
			if(send_sig_info(MVM_NOTIFY_SIGNO, &info, mvm_cli->task) < 0)
				dev_err(mvm_dev->dev, "Unable to send mvm state change signal to userspace\n");
			dev_dbg(mvm_dev->dev, "Sent state change signal to client %d\n",mvm_cli->client_id);
		}
	}
	mutex_unlock(&mvm_dev->mvm_cli_lock);
}

static void send_mvm_capacity_to_user(struct mvm_device *mvm_dev)
{
	struct kernel_siginfo info;
	struct mvm_stats_client *mvm_stats_cli;
	int i;
	uint8_t rate_idx = 0;

	memset(&info, 0, sizeof(struct kernel_siginfo));
	info.si_signo = MVM_NOTIFY_SIGNO;
	info.si_int = MVM_NOTIFY_VALUE_CAPACITY;

	if (mvm_dev->max_rate > LOWSVS_RATE_LIMIT)
		rate_idx = MAX_FREQ_PLAN-1;

	for (i=0; i< MAX_CURVES; i++)
		mvm_dev->mvm_capacity[i] = min(mvm_dev->max_rate, mvm_rate_lut[rate_idx][i]);

	list_for_each_entry(mvm_stats_cli, &mvm_dev->mvm_stats_client_list, list) {
		if (mvm_stats_cli->task != NULL) {
			if(send_sig_info(MVM_NOTIFY_SIGNO, &info, mvm_stats_cli->task) < 0)
				dev_err(mvm_dev->dev, "Unable to send mvm capacity signal to userspace\n");
			dev_info(mvm_dev->dev, "Sent capacity state change to client\n");
		}
	}
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
				struct kobj_attribute *mvm_state_attr,
				char *buf)
{
	struct mvm_device *mvm_dev = container_of(mvm_state_attr,
						struct mvm_device,
						mvm_state_attr);
	return snprintf(buf, 10, "%s\n", mvm_states[mvm_dev->state]);
}

static ssize_t mvm_curr_clk_show(struct kobject *kobj,
                                struct kobj_attribute *mvm_curr_clk_attr,
                                char *buf)
{
	struct mvm_device *mvm_dev = container_of(mvm_curr_clk_attr,
											struct mvm_device,
											mvm_curr_clk_attr);
	return snprintf(buf, 10, "%s\n", mvm_clk_freq_plan[mvm_dev->curr_clk]);
}

static ssize_t mvm_capacity_show(struct kobject *kobj,
                                struct kobj_attribute *mvm_capacity_attr,
                                char *buf)
{
	struct mvm_device *mvm_dev = container_of(mvm_capacity_attr,
											struct mvm_device,
											mvm_capacity_attr);
	return snprintf(buf, 26, "%d %d %d %d %d\n", mvm_dev->mvm_capacity[0], mvm_dev->mvm_capacity[1],
							mvm_dev->mvm_capacity[2], mvm_dev->mvm_capacity[3], mvm_dev->mvm_capacity[4]);
}

static ssize_t mvm_max_clk_show(struct kobject *kobj,
                                struct kobj_attribute *mvm_max_clk_attr,
                                char *buf)
{
	struct mvm_device *mvm_dev = container_of(mvm_max_clk_attr,
											struct mvm_device,
											mvm_max_clk_attr);
	return snprintf(buf, 10, "%s\n", mvm_clk_freq_plan[mvm_dev->max_clk]);
}

static ssize_t mvm_rate_lut_show(struct kobject *kobj,
				struct kobj_attribute *mvm_rate_lut_attr,
				char *buf)
{
        return snprintf(buf, 80, "%d %d %d %d %d\n%d %d %d %d %d\n%d %d %d %d %d",
				mvm_rate_lut[0][0], mvm_rate_lut[0][1], mvm_rate_lut[0][2],
				mvm_rate_lut[0][3], mvm_rate_lut[0][4], mvm_rate_lut[1][0],
				mvm_rate_lut[1][1], mvm_rate_lut[1][2], mvm_rate_lut[1][3],
				mvm_rate_lut[1][4], mvm_rate_lut[2][0], mvm_rate_lut[2][1],
				mvm_rate_lut[2][2], mvm_rate_lut[2][3], mvm_rate_lut[2][4]);
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

	sysfs_attr_init(&mvm_dev->mvm_state_attr.attr);
	mvm_dev->mvm_state_attr.attr.mode = 0444;
	mvm_dev->mvm_state_attr.attr.name = "mvm_state";
	mvm_dev->mvm_state_attr.show = mvm_state_show;
	mvm_dev->mvm_state_attr.store = NULL;
	ret = sysfs_create_file(mvm_dev->kobj, &mvm_dev->mvm_state_attr.attr);
	if (ret)        {
		dev_err(mvm_dev->dev, "%s: sysfs_create_file failed\n",
							__func__);
		goto fail_sysfs_mvm_state;
	}
	if (mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == TELEVM) {
		sysfs_attr_init(&mvm_dev->mvm_curr_clk_attr.attr);
		mvm_dev->mvm_curr_clk_attr.attr.mode = 0444;
		mvm_dev->mvm_curr_clk_attr.attr.name = "mvm_curr_clk";
		mvm_dev->mvm_curr_clk_attr.show = mvm_curr_clk_show;
		mvm_dev->mvm_curr_clk_attr.store = NULL;
		ret = sysfs_create_file(mvm_dev->kobj, &mvm_dev->mvm_curr_clk_attr.attr);
		if (ret)        {
			dev_err(mvm_dev->dev, "%s: sysfs_create_file mvm_curr_clk failed\n",
								__func__);
			goto fail_sysfs_mvm_curr_clk;
		}
		sysfs_attr_init(&mvm_dev->mvm_capacity_attr.attr);
		mvm_dev->mvm_capacity_attr.attr.mode = 0444;
		mvm_dev->mvm_capacity_attr.attr.name = "mvm_capacity";
		mvm_dev->mvm_capacity_attr.show = mvm_capacity_show;
		mvm_dev->mvm_capacity_attr.store = NULL;

		ret = sysfs_create_file(mvm_dev->kobj, &mvm_dev->mvm_capacity_attr.attr);
		if (ret)        {
			dev_err(mvm_dev->dev, "%s: sysfs_create_file mvm_capacity failed\n",
								__func__);
			goto fail_sysfs_mvm_capacity;
		}
		sysfs_attr_init(&mvm_dev->mvm_max_clk_attr.attr);
		mvm_dev->mvm_max_clk_attr.attr.mode = 0444;
		mvm_dev->mvm_max_clk_attr.attr.name = "mvm_max_clk";
		mvm_dev->mvm_max_clk_attr.show = mvm_max_clk_show;
		mvm_dev->mvm_max_clk_attr.store = NULL;

		ret = sysfs_create_file(mvm_dev->kobj, &mvm_dev->mvm_max_clk_attr.attr);
		if (ret)        {
			dev_err(mvm_dev->dev, "%s: sysfs_create_file mvm_max_clk failed\n",
								__func__);
			goto fail_sysfs_mvm_max_clk;
		}
		sysfs_attr_init(&mvm_dev->mvm_rate_lut_attr.attr);
		mvm_dev->mvm_rate_lut_attr.attr.mode = 0444;
		mvm_dev->mvm_rate_lut_attr.attr.name = "mvm_rate_lut";
		mvm_dev->mvm_rate_lut_attr.show = mvm_rate_lut_show;
		mvm_dev->mvm_rate_lut_attr.store = NULL;

		ret = sysfs_create_file(mvm_dev->kobj, &mvm_dev->mvm_rate_lut_attr.attr);
		if (ret)        {
			dev_err(mvm_dev->dev, "%s: sysfs_create_file mvm_rate_lut failed\n",
							__func__);
			goto fail_sysfs_mvm_rate_lut;
		}

		mvm_dev->clk_time_elapsed =0;
	}
	return 0;

fail_sysfs_mvm_rate_lut:
	sysfs_remove_file(mvm_dev->kobj, &mvm_dev->mvm_max_clk_attr.attr);
fail_sysfs_mvm_max_clk:
	sysfs_remove_file(mvm_dev->kobj, &mvm_dev->mvm_capacity_attr.attr);
fail_sysfs_mvm_capacity:
	sysfs_remove_file(mvm_dev->kobj, &mvm_dev->mvm_curr_clk_attr.attr);
fail_sysfs_mvm_curr_clk:
	sysfs_remove_file(mvm_dev->kobj, &mvm_dev->mvm_state_attr.attr);
fail_sysfs_mvm_state:
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
	if (inp_msg) {
		inp_msg->priority = 0;
		inp_msg->opcode = CTL_MSG_OPCODE;
		memcpy(in_fifo_addr, inp_msg, sizeof(struct input_msg));
		memcpy(in_fifo_addr + 1, mvm_ctrl, sizeof(struct mvm_control));
		mvm_dev->ring_buff->in_fifo[0].head = (mvm_dev->ring_buff->in_fifo[0].head + 1) % mvm_dev->ring_buff->in_fifo[0].size;
		writel_relaxed(mvm_dev->ring_buff->in_fifo[0].head, mvm_dev->mvm_base + MVMSS_CSR_INPUT_RING0_HEAD_PTR_OFFSET);
		writel_relaxed(IRQ_APSS1, mvm_dev->apss_shared_base + APSS_SHARED_IPC_INTERRUPT_OFFSET);
	}
	else {
		dev_err(mvm_dev->dev, "send_ctrl_msg_to_mvm failed with null inp_msg pointer\n");
		ret = -1;
	}
	return ret;
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

static int mvm_generic_file_open(struct inode *inode, struct file *filp)
{
	struct mvm_device *mvm_dev = inode->i_private;
	filp->private_data = mvm_dev;
	return 0;
}

static const struct file_operations debugfs_mvm_log_ops = {
	.owner     = THIS_MODULE,
	.open      = mvm_generic_file_open,
	.read      = mvm_log_file_read,
	.poll      = mvm_log_poll
};

static ssize_t mvm_log_policy_store(struct file *filp, const char __user *ubuf, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	struct mvm_control *mvm_ctrl;
	struct mvm_device *mvm_dev = filp->private_data;

	mvm_ctrl = kzalloc(sizeof(struct mvm_control), GFP_KERNEL);
	if (mvm_ctrl) {
		mvm_ctrl->type = MVM_DEBUG;
		if (copy_from_user(&mvm_ctrl->mvm_ctrl_msg.debug.transfer_msg.log_policy, ubuf, count)) {
			ret = -EFAULT;
		} else {
			ret = send_ctrl_msg_to_mvm(mvm_ctrl, mvm_dev);
			if (ret == 0)
				ret = count;
		}
		kfree(mvm_ctrl);
	}
	else {
		dev_err(mvm_dev->dev, "mvm_log_policy_store failed with null pointer\n");
		ret = -1;
	}
	return ret;
}

static const struct file_operations debugfs_mvm_policy_ops = {
	.owner      = THIS_MODULE,
	.open     = mvm_generic_file_open,
	.write    = mvm_log_policy_store,
};

static ssize_t mvm_log_transfer_store(struct file *filp, const char __user *ubuf, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	struct mvm_control *mvm_ctrl;
	struct mvm_device *mvm_dev = filp->private_data;

	mvm_ctrl = kzalloc(sizeof(struct mvm_control), GFP_KERNEL);
	if (mvm_ctrl) {
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
	}
	else {
		dev_err(mvm_dev->dev, "mvm_log_transfer_store failed with null mvm_ctrl pointer\n");
		ret = -1;
	}
	return ret;
}

static const struct file_operations debugfs_mvm_transfer_ops = {
	.owner      = THIS_MODULE,
	.open     = mvm_generic_file_open,
	.write    = mvm_log_transfer_store,
};

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
		if(mvm_ctrl) {
			mvm_ctrl->type = MVM_TRIGGER_SSR;
			mvm_ctrl->mvm_ctrl_msg.ssr.trigger_ssr = ssr;
			ret = send_ctrl_msg_to_mvm(mvm_ctrl, mvm_dev);
			kfree(mvm_ctrl);
		}
		else {
			dev_err(mvm_dev->dev, "mvm_trigger_ssr_store failed with null mvm_ctrl pointer\n");
			count = -1;
		}
	}

	return count;
}

static const struct file_operations debugfs_mvm_trigger_ssr_ops = {
	.owner = THIS_MODULE,
	.open = mvm_generic_file_open,
	.write = mvm_trigger_ssr_store,
};

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
	.open = mvm_generic_file_open,
	.poll = mvm_crash_dump_poll,
	.read = mvm_crash_dump_read,
};

static ssize_t mvm_log_level_store(struct file *filp, const char __user *ubuf, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	struct mvm_control *mvm_ctrl;
	struct mvm_device *mvm_dev = filp->private_data;

	mvm_ctrl = kzalloc(sizeof(struct mvm_control), GFP_KERNEL);
	if (mvm_ctrl) {
		mvm_ctrl->type = MVM_DEBUG;
		mvm_ctrl->mvm_ctrl_msg.debug.msg_type = MVM_DEBUG_SET_LOG_LEVEL;
		if (copy_from_user(&mvm_ctrl->mvm_ctrl_msg.debug.transfer_msg.log_level, ubuf, count)) {
			kfree(mvm_ctrl);
			ret = -EFAULT;
			return ret;
		}
		ret = send_ctrl_msg_to_mvm(mvm_ctrl, mvm_dev);
		kfree(mvm_ctrl);
	}
	else {
		dev_err(mvm_dev->dev, "mvm_log_level_store failed with null mvm_ctrl pointer \n");
		ret = -1;
	}
	return ret;
}

static const struct file_operations debugfs_mvm_log_level_ops = {
	.owner      = THIS_MODULE,
	.open     = mvm_generic_file_open,
	.write    = mvm_log_level_store,
};

static ssize_t pke_time_accumulator_read(struct file *filp, char __user *buff, size_t count, loff_t *offset)
{
	if(*offset == 0){
		char string[12] = {0};
		struct mvm_device *mvm_dev = filp->private_data;

		int to_copy = snprintf(string,12,"0x%x\n",mvm_dev->ring_buff->pke_time_accumulator);
		unsigned long remain = copy_to_user(buff,string,to_copy);
		*offset += (to_copy - remain);
		return *offset;
	}
	return 0;
}

static const struct file_operations debugfs_mvm_pke_time_ops = {
	.owner     = THIS_MODULE,
	.open      = mvm_generic_file_open,
	.read      = pke_time_accumulator_read,
};

static ssize_t pke_util_read(struct file *filp, char __user *buff, size_t count, loff_t *offset)
{
	if(*offset == 0){
		char string[12] = {0};
		struct mvm_device *mvm_dev = filp->private_data;

		int to_copy = snprintf(string,12,"0x%x\n",mvm_dev->curr_pke_util);
		unsigned long remain = copy_to_user(buff,string,to_copy);
		*offset += (to_copy - remain);
		return *offset;
	}
	return 0;
}

static const struct file_operations debugfs_mvm_pke_util_ops = {
	.owner     = THIS_MODULE,
	.open      = mvm_generic_file_open,
	.read      = pke_util_read,
};

static int mvm_debugfs_init(struct mvm_device *mvm_dev)
{
	struct dentry *file;
	int ret = 0;

	mvm_dev->dir = debugfs_create_dir("mvm", NULL);
	if (IS_ERR_OR_NULL(mvm_dev->dir)) {
		ret = -ENOMEM;
		goto ret;
	}

	file = debugfs_create_file("mvm_log", 0644, mvm_dev->dir, mvm_dev,
						&debugfs_mvm_log_ops);
	if (!file)
		goto file_err;

	file = debugfs_create_file("mvm_policy", 0644, mvm_dev->dir, mvm_dev,
						&debugfs_mvm_policy_ops);
	if (!file)
		goto file_err;

	file = debugfs_create_file("mvm_transfer", 0644, mvm_dev->dir, mvm_dev,
						&debugfs_mvm_transfer_ops);
	if (!file)
		goto file_err;

	file = debugfs_create_file("trigger_ssr", 0644, mvm_dev->dir, mvm_dev,
						&debugfs_mvm_trigger_ssr_ops);
	if (!file)
		goto file_err;

	file = debugfs_create_file("mvm_crash_dump", 0644, mvm_dev->dir, mvm_dev,
						&debugfs_mvm_crash_dump_ops);
	if (!file)
		goto file_err;

	file = debugfs_create_file("mvm_log_level", 0644, mvm_dev->dir, mvm_dev,
		&debugfs_mvm_log_level_ops);
	if (!file)
		goto file_err;

	file = debugfs_create_file("pke_time_accumulator", 0644, mvm_dev->dir, mvm_dev,
		&debugfs_mvm_pke_time_ops);
	if (!file)
		goto file_err;

	file = debugfs_create_file("pke_utilization", 0644, mvm_dev->dir, mvm_dev,
		&debugfs_mvm_pke_util_ops);
	if (!file)
		goto file_err;

	mvm_dev->ddr_current_addr = &mvm_dev->log_buff->mvmlog_buffer[0];
	mvm_dev->ddr_head_pos = &mvm_dev->log_buff->mvmlog_buffer[0];
	mvm_dev->ddr_tail_pos = &mvm_dev->log_buff->mvmlog_buffer[0];
	mvm_dev->active_buffer_index = -1;
	mvm_dev->ddr_buf_len = 0;
	mvm_dev->filled_dma_bytes = 0;
	goto ret;

file_err:
	ret = -ENOENT;
	debugfs_remove_recursive(mvm_dev->dir);
ret:
	return ret;
}

static void process_control_message(struct mvm_control *mvm_ctrl_recv, struct mvm_device *mvm_dev)
{
	ssize_t ret = 0;
	struct mvm_control *mvm_ctrl_send;

	mvm_ctrl_send = kzalloc(sizeof(struct mvm_control), GFP_KERNEL);
	if (mvm_ctrl_send) {
		mvm_ctrl_send->type = MVM_DEBUG;
		if ( mvm_ctrl_recv->mvm_ctrl_msg.debug.msg_type == MVM_DEBUG_LOG_TRANSFER_REQUEST) {
			mvm_ctrl_send->mvm_ctrl_msg.debug.transfer_msg.ddr_log_buf_addr = LOG_BUFF_IOVA + mvm_dev->filled_dma_bytes;
			mvm_ctrl_send->mvm_ctrl_msg.debug.msg_type = MVM_DEBUG_LOG_TRANSFER_REQUEST;
			ret = send_ctrl_msg_to_mvm(mvm_ctrl_send, mvm_dev);
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
	else {
		dev_err(mvm_dev->dev,"process_control_message failed with mvm_ctrl_send null pointer\n");
	}
}

static void mvm_client_remove(struct mvm_device *mvm_dev,struct mvm_client *mvm_cli)
{
	mutex_lock(&mvm_dev->mvm_cli_lock);
	clear_bit(mvm_cli->client_id - 1, mvm_dev->client_id_bitmap);
	list_del(&mvm_cli->list);
	mutex_unlock(&mvm_dev->mvm_cli_lock);
	mvm_cli->mvm_dev = NULL;
	kfree(mvm_cli->out_buff);
	kfree(mvm_cli);
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
			if (mvm_ctrl_recv) {
				out_fifo_addr = (unsigned int *) mvm_dev->ring_buff->out_fifo[fifo_index].base[mvm_dev->ring_buff->out_fifo[fifo_index].tail].data;
				memcpy(mvm_ctrl_recv, out_fifo_addr, sizeof(struct mvm_control));
				switch (mvm_ctrl_recv->type) {
				case MVM_DEBUG:
					mvm_dev->active_buffer_index = mvm_ctrl_recv->mvm_ctrl_msg.debug.transfer_msg.active_buffer_index;
					process_control_message(mvm_ctrl_recv, mvm_dev);
					break;
				case MVM_POLICY:
					break;
				case MVM_CPU_FREQ:
					break;
				case MVM_POWER:
					break;
				case MVM_MAX_RATE:
					mvm_dev->max_rate = mvm_ctrl_recv->mvm_ctrl_msg.max_rate.max_rate;
					dev_info(mvm_dev->dev, "MVM License file is read and the max rate supported by MVM is %d\n",mvm_dev->max_rate);
					if (mvm_dev->max_rate > LOWSVS_RATE_LIMIT)
						mvm_dev->max_clk = 0;
					send_mvm_capacity_to_user(mvm_dev);
					break;
				case MVM_CHANGE_CLK_FREQ:
					dev_dbg(mvm_dev->dev, "Clock freq changed from %s to %s\n",
							mvm_clk_freq_plan[mvm_dev->curr_clk],
							mvm_clk_freq_plan[mvm_ctrl_recv->mvm_ctrl_msg.mvm_clk_freq.clk_freq]);
					mvm_dev->curr_clk = mvm_ctrl_recv->mvm_ctrl_msg.mvm_clk_freq.clk_freq;
					break;
				default:
					break;
				}
				control_message_written = true;
				kfree(mvm_ctrl_recv);
			}
			else {
				dev_err(mvm_dev->dev,"drain_out_fifo failed with mvm_ctrl_recv null pointer\n");
			}
		}
		else {
			bool match = false;
			list_for_each_entry(mvm_cli, &mvm_dev->client_list, list) {
				if (mvm_cli->client_id == client_id) {
					match = true;
					if(mvm_cli->state == CLIENT_DISCONNECTING){
						mvm_cli->results_recvd++;
						if(mvm_cli->results_recvd == mvm_cli->msgs_sent){
							dev_dbg(mvm_dev->dev,"received last pending message for disconnected client, removing client\n");
							mvm_client_remove(mvm_dev,mvm_cli);
						}
						mvm_dev->outgoing_results++;
						out_buff_written = true;
						break;
					}
					else{
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
							mvm_cli->results_recvd++;
							mvm_dev->outgoing_results++;
							out_buff_written = true;
							break;
						}
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
	bool p0_fifo_has_results, p1_fifo_has_results, wake_up = false;

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

	if (wake_up){
		wake_up_interruptible_poll(&mvm_dev->mvm_waitqueue, POLLIN | POLLPRI);
	}
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
	mvm_cli->state = CLIENT_READY;
	list_add_tail(&mvm_cli->list, &mvm_dev->client_list);
	mutex_unlock(&mvm_dev->mvm_cli_lock);

	mvm_cli->out_buff = kzalloc(sizeof(struct output_buffer), GFP_KERNEL);
	if (mvm_cli->out_buff) {
		mvm_cli->out_buff->size = OUT_BUFF_SIZE;
	}
	else {
		dev_err(mvm_dev->dev, "mvm_open failed with null mvm_cli->out_buff pointer \n");
		ret = -1;
	}
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

	mvm_cli->state = CLIENT_DISCONNECTING;
	if(mvm_cli->msgs_sent == mvm_cli->results_recvd)
	{
		dev_dbg(mvm_dev->dev, "client disconnected with no pending messages, removing immediately\n");
		mvm_client_remove(mvm_dev,mvm_cli);
	}
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
		mvm_cli->state = CLIENT_READY;
		mvm_cli->out_buff->head = 0;
		mvm_cli->out_buff->tail = 0;
		break;
	}

	return ret;
}

static ssize_t mvm_write(
	struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
	struct input_msg *inp_msg;
	unsigned int i, fifo_index = 0;
	struct mvm_client *mvm_cli = filp->private_data;
	struct mvm_device *mvm_dev = mvm_cli->mvm_dev;
	bool full;
	int rc, ret;
	unsigned int no_of_msgs_written = 0;
	unsigned int count = 0;
	unsigned int curve_id;

	if (!(mvm_cli->state == CLIENT_READY))
		return 0;

	mutex_lock(&mvm_dev->in_fifo_lock);
	inp_msg = kzalloc(sizeof(struct input_msg), GFP_KERNEL);
	if (inp_msg) {
		ret = copy_from_user(inp_msg, buf, sizeof(struct input_msg));
		if (ret)
			count = (len - ret) / sizeof(struct input_msg);
		else
			count = len / sizeof(struct input_msg);
		fifo_index = inp_msg[0].priority;
		dev_dbg(mvm_dev->dev, "priority is %d count is %d\n",
						fifo_index, count);
		kfree(inp_msg);
	}
	else {
		dev_err(mvm_dev->dev, "mvm_write failed with null inp_msg pointer \n");
		ret = -1;
		goto ret;
	}
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
			curve_id = mvm_dev->ring_buff->in_fifo[fifo_index].base[mvm_dev->ring_buff->in_fifo[fifo_index].head].curve_id;
			mvm_dev->mvm_in_msg_count[curve_id]++;
			if (!ret) {
				mvm_dev->ring_buff->in_fifo[fifo_index].head =
					(mvm_dev->ring_buff->in_fifo[fifo_index].head + 1)
					% mvm_dev->ring_buff->in_fifo[fifo_index].size;
				no_of_msgs_written++;
				mvm_dev->incoming_msgs++;
				mvm_cli->msgs_sent++;
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
		writel_relaxed(IRQ_APSS1, mvm_dev->apss_shared_base + APSS_SHARED_IPC_INTERRUPT_OFFSET);
	if (ret == 0)
		ret = no_of_msgs_written * sizeof(struct input_msg);
	return ret;
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
	writel_relaxed(MVM_DUMP_BUFF_IOVA,
				mvm_dev->mvm_base + MVMSS_CSR_APSS_MVM_SCRATCH_PAD0);
	/*Send an interrupt to MVM to indicate MVM_Init done */
	writel_relaxed(IRQ_APSS0, mvm_dev->apss_shared_base + APSS_SHARED_IPC_INTERRUPT_OFFSET);
	mvm_dev->state = MVM_ONLINE;
	dev_info(mvm_dev->dev, "The current state of MVM is ONLINE\n");
	update_marker("M - MVM is ONLINE");
	send_mvm_state_to_user(mvm_dev, MVM_ONLINE);
}

static int mvm_televm_map_shared_mem(struct mvm_device *mvm_dev, char *compat, uint32_t shm_label)
{
	struct device_node *np = NULL, *shm_np;
	struct resource res;
	uint32_t label;
	int ret = 0;

	while ((np = of_find_compatible_node(np, NULL, compat))) {
		ret = of_property_read_u32(np, "qcom,label", &label);
		if (ret) {
			of_node_put(np);
			continue;
		}
		if (label == shm_label) {
			break;
		}
		of_node_put(np);
	}

	if (!np) {
		ret = -EINVAL;
		goto err;
	}

	shm_np = of_parse_phandle(np, "memory-region", 0);
	if (!shm_np) {
		dev_err(mvm_dev->dev, "can't parse mvm-shm node for ring buffers\n");
		ret = -EINVAL;
		goto put_np;
	}

	ret = of_address_to_resource(shm_np, 0, &res);
	if (ret) {
		dev_err(mvm_dev->dev, "of_address_to_resource of ring buffers failed\n");
		ret = -EINVAL;
		goto put_shm_np;
	}

	if (label == mvm_dev->mvm_ring_buff_shm_label) {
		mvm_dev->ring_buff = devm_ioremap_wc(mvm_dev->dev, res.start, resource_size(&res));
		if (IS_ERR(mvm_dev->ring_buff)) {
			ret = -ENOMEM;
			dev_err(mvm_dev->dev, "ioremap of ring buffers failed\n");
		}
	} else if (label == mvm_dev->mvm_dump_buff_shm_label) {
		mvm_dev->dump_buff = devm_ioremap_wc(mvm_dev->dev, res.start, resource_size(&res));
		if (IS_ERR(mvm_dev->dump_buff)) {
			ret = -ENOMEM;
			dev_err(mvm_dev->dev, "ioremap of dump buffers failed\n");
			goto ioremap_dump_buff_fail;
		}
	} else if (label == mvm_dev->mvm_log_buff_shm_label) {
		mvm_dev->log_buff = devm_ioremap_wc(mvm_dev->dev, res.start, resource_size(&res));
		if (IS_ERR(mvm_dev->log_buff)) {
			ret = -ENOMEM;
			dev_err(mvm_dev->dev, "ioremap of log buffers failed\n");
			goto ioremap_log_buff_fail;
		}
	}

	goto put_shm_np;

ioremap_log_buff_fail:
	devm_iounmap(mvm_dev->dev, mvm_dev->dump_buff);
ioremap_dump_buff_fail:
	devm_iounmap(mvm_dev->dev, mvm_dev->ring_buff);
put_shm_np:
	of_node_put(shm_np);
put_np:
	of_node_put(np);
err:
	return ret;
}

static int hyp_unassign_mem_reclaim(struct mvm_device *mvm_dev, dma_addr_t dma_addr, uint32_t label,
				     uint32_t size, gh_memparcel_handle_t handle) {
	int srcVMperm[1] = {PERM_READ | PERM_WRITE};
	int srcVM[2] = {AC_VM_HLOS, mvm_dev->televm_vmid};
	int destVM[1] = {AC_VM_HLOS};
	int ret;

	ret = gh_rm_mem_reclaim(handle, 0);
	if (ret)
		dev_err(mvm_dev->dev, "mem reclaim for label %u failed with ret = %d\n", label, ret);
	else
		ret = hyp_assign_phys(dma_addr, size, srcVM, 2, destVM, srcVMperm, 1);

	return ret;
}

static int hyp_assign_mem_share(struct mvm_device *mvm_dev, struct gh_acl_desc *mvm_acl_desc,
 				struct gh_sgl_desc *mvm_sgl_desc, dma_addr_t dma_addr,
				uint32_t size, uint32_t label, gh_memparcel_handle_t *handle)
{
	int srcVMperm[1] = {PERM_READ | PERM_WRITE};
	int destVMperm[2] = {PERM_READ | PERM_WRITE, PERM_READ | PERM_WRITE};
	int srcVM[1] = {AC_VM_HLOS};
	int destVM[2] = {AC_VM_HLOS, mvm_dev->televm_vmid};
	int ret = 0;

        mvm_sgl_desc->n_sgl_entries = 1;
        mvm_sgl_desc->sgl_entries[0].ipa_base = dma_addr;
        mvm_sgl_desc->sgl_entries[0].size = size;

        ret = hyp_assign_phys(dma_addr, size, srcVM, 1, destVM, destVMperm, 2);
        if (ret) {
                dev_err(mvm_dev->dev, "Couldnt hyp_assign ring buffers from hostvm to televm\n");
                goto ret;
        }

        ret = gh_rm_mem_share(GH_RM_MEM_TYPE_NORMAL, 0, label, mvm_acl_desc, mvm_sgl_desc,
				NULL, handle);

        if (ret) {
                dev_err(mvm_dev->dev, "mem_share of ring buffers from hostvm to televm failed\n");
                goto mem_share_fail;
        }

	goto ret;

mem_share_fail:
	hyp_assign_phys(dma_addr, size, destVM, 2, srcVM, srcVMperm, 1);
ret:
	return ret;

}

static int read_shm_labels(struct mvm_device *mvm_dev)
{
	int ret = 0;
	struct device_node *node;

	node = mvm_dev->dev->of_node;
	ret = of_property_read_u32(node, "qcom,mvm-ring-buff-shm-label", &mvm_dev->mvm_ring_buff_shm_label);
	if (ret) {
		dev_err(mvm_dev->dev, "qcom,mvm-ring-buff-shm-label not defined\n");
		ret = -EINVAL;
		goto ret;
	}

	ret = of_property_read_u32(node, "qcom,mvm-dump-buff-shm-label", &mvm_dev->mvm_dump_buff_shm_label);
	if (ret) {
		dev_err(mvm_dev->dev, "qcom,mvm-dump-buff-shm-label not defined\n");
		ret = -EINVAL;
		goto ret;
	}

	ret = of_property_read_u32(node, "qcom,mvm-log-buff-shm-label", &mvm_dev->mvm_log_buff_shm_label);
	if (ret) {
		dev_err(mvm_dev->dev, "qcom,mvm-log-buff-shm-label not defined\n");
		ret = -EINVAL;
		goto ret;
	}

ret:
	return ret;
}

static int mvm_hostvm_mem_share(struct mvm_device *mvm_dev)
{
	struct gh_acl_desc *mvm_acl_desc;
	struct gh_sgl_desc *mvm_sgl_desc;
	int srcVMperm[1] = {PERM_READ | PERM_WRITE};
	int srcVM[1] = {AC_VM_HLOS};
	int destVM[2] = {AC_VM_HLOS, mvm_dev->televm_vmid};
	int ret = 0;

	mvm_dev->ring_buff_mem_handle = 0;
	mvm_dev->dump_buff_mem_handle = 0;
	mvm_dev->log_buff_mem_handle = 0;

	mvm_acl_desc = kzalloc(offsetof(struct gh_acl_desc, acl_entries[2]), GFP_KERNEL);
	if (!mvm_acl_desc) {
		ret = -ENOMEM;
		goto acl_alloc_fail;
	}

	mvm_sgl_desc = kzalloc(offsetof(struct gh_sgl_desc, sgl_entries[1]), GFP_KERNEL);
	if (!mvm_sgl_desc) {
		ret = -ENOMEM;
		goto sgl_alloc_fail;
	}

        mvm_acl_desc->n_acl_entries = 2;
        mvm_acl_desc->acl_entries[0].vmid = AC_VM_HLOS;
        mvm_acl_desc->acl_entries[0].perms = GH_RM_ACL_R | GH_RM_ACL_W;
        mvm_acl_desc->acl_entries[1].vmid = mvm_dev->televm_vmid;
        mvm_acl_desc->acl_entries[1].perms = GH_RM_ACL_R | GH_RM_ACL_W;

        /* Share ring buffers from hostvm to televm */
	ret = hyp_assign_mem_share(mvm_dev, mvm_acl_desc, mvm_sgl_desc,
				   mvm_dev->ring_buff_dma,
				   round_up(sizeof(struct ring_buffers), PAGE_SIZE),
				   mvm_dev->mvm_ring_buff_shm_label,
				   &mvm_dev->ring_buff_mem_handle);
	if (ret)
		goto free_mem;

	/* Share dump buffers from hostvm to televm */
	ret = hyp_assign_mem_share(mvm_dev, mvm_acl_desc, mvm_sgl_desc,
				   mvm_dev->mvm_dump_dma,
				   round_up(sizeof(struct mvm_crashdump_buffer), PAGE_SIZE),
				   mvm_dev->mvm_dump_buff_shm_label,
				   &mvm_dev->dump_buff_mem_handle);
	if (ret)
		goto mem_share_dump_buff_fail;

	/* Share log buffer from hostvm to televm */
	ret = hyp_assign_mem_share(mvm_dev, mvm_acl_desc, mvm_sgl_desc,
				   mvm_dev->mvmlog_buff_dma,
				   round_up(sizeof(struct mvmlog_buffers), PAGE_SIZE),
				   mvm_dev->mvm_log_buff_shm_label,
				   &mvm_dev->log_buff_mem_handle);
	if (ret)
		goto mem_share_log_buff_fail;
	else
		goto free_mem;


mem_share_log_buff_fail:
	gh_rm_mem_reclaim(mvm_dev->dump_buff_mem_handle,0);
	hyp_assign_phys(mvm_dev->mvm_dump_dma, round_up(sizeof(struct mvm_crashdump_buffer), PAGE_SIZE),
				destVM, 2, srcVM, srcVMperm, 1);
mem_share_dump_buff_fail:
	gh_rm_mem_reclaim(mvm_dev->ring_buff_mem_handle, 0);
	hyp_assign_phys(mvm_dev->ring_buff_dma, round_up(sizeof(struct ring_buffers), PAGE_SIZE),
					destVM, 2, srcVM, srcVMperm, 1);
free_mem:
	kfree(mvm_sgl_desc);
sgl_alloc_fail:
	kfree(mvm_acl_desc);
acl_alloc_fail:
	return ret;
}

static int mvm_hostvm_io_lend(struct mvm_device *mvm_dev)
{
	int ret = 0;
	struct gh_acl_desc *mvm_acl_desc;
	struct gh_sgl_desc *mvm_sgl_desc;
	struct device_node *node;

	mvm_dev->mvmss_mem_handle = 0;
	mvm_dev->apss_mem_handle = 0;
        node = mvm_dev->dev->of_node;

	ret = of_property_read_u32(node, "qcom,iomem-gunyah-label", &mvm_dev->iomem_gunyah_label);
	if (ret) {
		dev_err(mvm_dev->dev, "qcom,iomem-gunyah-label not defined\n");
		ret = -EINVAL;
		goto ret;
	}

	mvm_acl_desc = kzalloc(offsetof(struct gh_acl_desc, acl_entries[2]), GFP_KERNEL);
	if (!mvm_acl_desc) {
		ret = -ENOMEM;
		goto ret;
	}

	mvm_sgl_desc = kzalloc(offsetof(struct gh_sgl_desc, sgl_entries[1]), GFP_KERNEL);
	if (!mvm_sgl_desc) {
		ret = -ENOMEM;
		goto sgl_alloc_fail;
	}

	mvm_acl_desc->n_acl_entries = 1;
	mvm_acl_desc->acl_entries[0].vmid = mvm_dev->televm_vmid;
	mvm_acl_desc->acl_entries[0].perms = GH_RM_ACL_R | GH_RM_ACL_W;

	mvm_sgl_desc->n_sgl_entries = 1;
	mvm_sgl_desc->sgl_entries[0].ipa_base = MVMSS_REG_BASE;
	mvm_sgl_desc->sgl_entries[0].size = MVMSS_LEND_SIZE;

	/*Lend mvm address space from hostvm to televm */
	ret = gh_rm_mem_lend(GH_RM_MEM_TYPE_IO, 0, mvm_dev->iomem_gunyah_label,
			mvm_acl_desc, mvm_sgl_desc, NULL, &mvm_dev->mvmss_mem_handle);
	if (ret) {
		dev_err(mvm_dev->dev, "gh_rm_mem_lend for mvm address space failed\n");
		goto free_mem;
	}

	mvm_acl_desc->n_acl_entries = 2;
	mvm_acl_desc->acl_entries[0].vmid = AC_VM_HLOS;
	mvm_acl_desc->acl_entries[0].perms = GH_RM_ACL_R | GH_RM_ACL_W;
	mvm_acl_desc->acl_entries[1].vmid = mvm_dev->televm_vmid;
	mvm_acl_desc->acl_entries[1].perms = GH_RM_ACL_R | GH_RM_ACL_W;

	mvm_sgl_desc->n_sgl_entries = 1;
	mvm_sgl_desc->sgl_entries[0].ipa_base = APSS_SHARED_BASE_START;
	mvm_sgl_desc->sgl_entries[0].size = APSS_SHARED_BASE_END;

	/*Share apss shared base address space between hostvm and televm */
	ret = gh_rm_mem_share(GH_RM_MEM_TYPE_IO, 0, mvm_dev->iomem_gunyah_label,
		mvm_acl_desc, mvm_sgl_desc, NULL, &mvm_dev->apss_mem_handle);
	if (ret) {
		dev_err(mvm_dev->dev, "gh_rm_mem_share for apps shared base failed\n");
		goto mem_share_err;
	}
	goto free_mem;

mem_share_err:
	gh_rm_mem_reclaim(mvm_dev->mvmss_mem_handle, 0);
free_mem:
	kfree(mvm_sgl_desc);
sgl_alloc_fail:
	kfree(mvm_acl_desc);
ret:
	return ret;
}

static void mvm_hostvm_unshare_mem(struct mvm_device *mvm_dev)
{
	int ret;

	ret = hyp_unassign_mem_reclaim(mvm_dev, mvm_dev->ring_buff_dma, mvm_dev->mvm_ring_buff_shm_label,
				round_up(sizeof(struct ring_buffers), PAGE_SIZE), mvm_dev->ring_buff_mem_handle);
	if (ret)
		dev_err(mvm_dev->dev,"hyp_unassign_mem_reclaim failed for label %u with ret = %d\n",
					mvm_dev->mvm_ring_buff_shm_label, ret);

	ret = hyp_unassign_mem_reclaim(mvm_dev, mvm_dev->mvm_dump_dma, mvm_dev->mvm_dump_buff_shm_label,
				round_up(sizeof(struct mvm_crashdump_buffer), PAGE_SIZE), mvm_dev->dump_buff_mem_handle);
	if (ret)
		dev_err(mvm_dev->dev, "hyp_unassign_mem_reclaim failed for label %u with ret = %d\n",
					mvm_dev->mvm_dump_buff_shm_label, ret);


	ret = hyp_unassign_mem_reclaim(mvm_dev, mvm_dev->mvmlog_buff_dma, mvm_dev->mvm_log_buff_shm_label,
				round_up(sizeof(struct mvmlog_buffers), PAGE_SIZE), mvm_dev->log_buff_mem_handle);
	if (ret)
		dev_err(mvm_dev->dev,"hyp_unassign_mem_reclaim failed for label %u with ret = %d\n",
					mvm_dev->mvm_log_buff_shm_label, ret);
}

static int qcom_mvm_rm_cb(struct notifier_block *nb, unsigned long cmd,
						void *data)
{
	struct gh_rm_notif_vm_status_payload *vm_status_payload;
	struct mvm_device *mvm_dev;
	int ret;
	gh_vmid_t vmid;

	mvm_dev = container_of(nb, struct mvm_device, rm_nb);
	vm_status_payload = data;
	ret = gh_rm_get_vmid(GH_TELE_VM, &vmid);
	if (ret) {
		dev_err(mvm_dev->dev, "gh_rm_get_vmid failed\n");
		return NOTIFY_DONE;
	}
	if (vm_status_payload->vmid == vmid && cmd == GH_VM_BEFORE_POWERUP) {
		mvm_dev->televm_vmid = vmid;

		ret = read_shm_labels(mvm_dev);
		if (ret) {
			dev_err(mvm_dev->dev, "read_shm_labels failed\n");
			return NOTIFY_DONE;
		}
		ret = mvm_hostvm_io_lend(mvm_dev);
		if (ret) {
			dev_err(mvm_dev->dev, "mvm_hostvm_io_lend failed\n");
			return NOTIFY_DONE;
		}

		ret = mvm_hostvm_mem_share(mvm_dev);
		if (ret) {
			dev_err(mvm_dev->dev, "mvm_hostvm_mem_share failed\n");
			goto hostvm_mem_share_fail;
		}
	}
	else if (vm_status_payload->vmid == mvm_dev->televm_vmid && cmd == GH_VM_POWEROFF) {
		ret = gh_rm_mem_reclaim(mvm_dev->mvmss_mem_handle, 0);
		if (ret)
			dev_err(mvm_dev->dev,"mem reclaim of mvmss_mem_handle failed with ret = %d\n",ret);

		ret = gh_rm_mem_reclaim(mvm_dev->apss_mem_handle, 0);
		if (ret)
			dev_err(mvm_dev->dev, "mem reclaim of apss_mem_handle failed with ret = %d\n",ret);

		mvm_hostvm_unshare_mem(mvm_dev);
	}
	return NOTIFY_DONE;

hostvm_mem_share_fail:
	gh_rm_mem_reclaim(mvm_dev->mvmss_mem_handle, 0);
	gh_rm_mem_reclaim(mvm_dev->apss_mem_handle, 0);

	return NOTIFY_DONE;
}

static int ioremap_resources(struct platform_device *pdev)
{
	struct mvm_device *mvm_dev;
	struct device_node *node;
	int ret = 0;

	mvm_dev = platform_get_drvdata(pdev);
	node = pdev->dev.of_node;

	mvm_dev->mvm_base = devm_ioremap(&pdev->dev, MVMSS_REG_BASE, MVMSS_REG_SIZE);
	if (IS_ERR(mvm_dev->mvm_base)) {
		dev_err(mvm_dev->dev, "ioremap of mvm_base failed\n");
		ret = -ENOMEM;
		goto err;
	}

	mvm_dev->apss_shared_base = devm_ioremap(&pdev->dev, APSS_SHARED_BASE_START,
							APSS_SHARED_BASE_END);
	if (IS_ERR(mvm_dev->apss_shared_base)) {
		dev_err(mvm_dev->dev, "ioremap of apss_shared_basefailed\n");
		devm_iounmap(mvm_dev->dev, mvm_dev->mvm_base);
		ret = -ENOMEM;
	}

err:
	return ret;
}

static int mvm_load_fw(struct mvm_device *mvm_dev)
{
	int ret = 0;

	ret = enable_gcc_clocks(mvm_dev);
	if (ret) {
		dev_err(mvm_dev->dev, "Failed to turn on gcc clocks\n");
		goto ret;
	}

	ret = enable_mvm_gdsc(mvm_dev, true);
	if (ret) {
		dev_err(mvm_dev->dev, "Failed to turn on mvm gdsc\n");
		goto gdsc_err;
	}

	mvm_dev->resume_frm_pwr_collapse = true;

	update_marker("M - Loading MVM firmware");

	ret = qcom_mdt_load(mvm_dev->dev, mvm_dev->fw, "mvm_ecc.mdt", MVM_PROC_ID,
			    mvm_dev->mvm_fw, mvm_dev->mvm_fw_dma, MVM_FW_SIZE, NULL);
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
	if (ret) {
		dev_err(mvm_dev->dev, "Couldnt store MVM PIL info in IMEM\n");
		goto out_release_firmware;
	}

	if (mvm_dev->vm_variant == HOSTVM) {
		if (mvm_dev->state == MVM_OFFLINE)
			mvm_dev->state = MVM_ONLINE;
		else if (mvm_dev->state == MVM_RESTARTING) {
			gh_dbl_flags_t dbl_mask = MVM_INIT_FIFOS_DBL_MASK;
			ret = gh_dbl_send(mvm_dev->hostvm_tx_dbl, &dbl_mask, 0);
			if (ret) {
				dev_err(mvm_dev->dev, "Failed to send MVM_INIT_FIFOS_DBL to televm %d\n", ret);
				goto ret;
			}
			dev_dbg(mvm_dev->dev, "Sent MVM_INIT_FIFOS_DBL to televm\n");
			mvm_dev->state = MVM_ONLINE;
		}
	}

	goto ret;

out_release_firmware:
	release_firmware(mvm_dev->fw);
gdsc_err:
	clk_disable_unprepare(mvm_dev->cnoc_s_ahb_clk);
	clk_disable_unprepare(mvm_dev->xo);
	clk_disable_unprepare(mvm_dev->snoc_m_axi_clk);
	clk_disable_unprepare(mvm_dev->sysnoc_mvmss_clk);
ret:
	return ret;
}

static void disable_gcc_clocks(struct mvm_device *mvm_dev)
{
	clk_disable_unprepare(mvm_dev->snoc_m_axi_clk);
	clk_disable_unprepare(mvm_dev->cnoc_s_ahb_clk);
	clk_disable_unprepare(mvm_dev->sysnoc_mvmss_clk);
	clk_disable_unprepare(mvm_dev->xo);
}

static int send_pwr_collpase_ctrl_msg(struct mvm_device *mvm_dev, bool pwr_collapse) {
	struct mvm_control *mvm_ctrl;
	int ret = 0;
	mvm_ctrl = kzalloc(sizeof(struct mvm_control), GFP_KERNEL);
	if (mvm_ctrl) {
		mvm_ctrl->type = MVM_POWER;
		mvm_ctrl->mvm_ctrl_msg.power.enter_pwr_collapse = pwr_collapse;
		ret = send_ctrl_msg_to_mvm(mvm_ctrl, mvm_dev);
		kfree(mvm_ctrl);
	}
	else {
		dev_err(mvm_dev->dev, "send_pwr_collpase_ctrl_msg failed with null mvm_ctrl pointer\n");
		ret = -1;
	}
	return ret;
}

static void mvm_hostvm_rx_dbl_cb(int irq, void *data)
{
	gh_dbl_flags_t dbl_mask = DBL_MASK;
	struct mvm_device *mvm_dev;
	int ret;

	mvm_dev = data;
	ret = gh_dbl_read_and_clean(mvm_dev->hostvm_rx_dbl, &dbl_mask, GH_DBL_NONBLOCK);
	if(ret){
		dev_err(mvm_dev->dev, "Error reading from doorbell\n");
	}
	dev_dbg(mvm_dev->dev, "Received mvm_hostvm_rx_dbl_cb with mask %x\n", dbl_mask);

	if (dbl_mask == MVM_SHUTDOWN_DBL_MASK) {
		mvm_dev->state = MVM_CRASHED;
		ret = qcom_scm_pas_shutdown(MVM_PROC_ID);
		if (ret) {
			dev_err(mvm_dev->dev, "Error sending shutdown request to MVM\n");
			return;
		}
	} else if (dbl_mask == MVM_LOAD_FW_DBL_MASK) {
		mvm_dev->state = MVM_RESTARTING;
		mvm_load_fw(mvm_dev);
	} else if (dbl_mask == MVM_DO_SUSPEND_DBL_MASK) {
		collapse_mvm_core(mvm_dev);
		disable_gcc_clocks(mvm_dev);
		dbl_mask = MVM_SUSPEND_DONE_DBL_MASK;
		ret = gh_dbl_send(mvm_dev->hostvm_tx_dbl, &dbl_mask, 0);
		if (ret)
			dev_err(mvm_dev->dev, "Couldnt send MVM_SUSPEND_DONE_DBL to televm\n");
	} else if (dbl_mask == MVM_DO_RESUME_DBL_MASK) {
		int is_resume;
		is_resume = enable_gcc_clocks(mvm_dev);
	        if (is_resume)
	                dev_err(mvm_dev->dev, "Failed to turn on gcc clocks\n");
		else {
			restore_mvm_core(mvm_dev);
			dbl_mask = MVM_RESUME_DONE_DBL_MASK;
			ret = gh_dbl_send(mvm_dev->hostvm_tx_dbl, &dbl_mask, 0);
			if (ret)
				dev_err(mvm_dev->dev, "Couldnt send MVM_RESUME_DONE_DBL to televm\n");
		}
	}
	ret = gh_dbl_reset(mvm_dev->hostvm_rx_dbl, GH_DBL_NONBLOCK);
	if(ret){
		dev_err(mvm_dev->dev, "Error resetting rx doorbell\n");
	}
}

static void mvm_televm_rx_dbl_cb(int irq, void *data)
{
	gh_dbl_flags_t dbl_mask = DBL_MASK;
	struct mvm_device *mvm_dev =data;
	int ret;

	mvm_dev = data;
	ret = gh_dbl_read_and_clean(mvm_dev->televm_rx_dbl, &dbl_mask, GH_DBL_NONBLOCK);
	if(ret){
		dev_err(mvm_dev->dev, "Error reading from doorbell\n");
	}
	dev_dbg(mvm_dev->dev, "Received mvm_televm_rx_dbl_cb with mask %x\n", dbl_mask);

	if (dbl_mask == MVM_INIT_FIFOS_DBL_MASK) {
		initialise_fifos(mvm_dev);
	} else if (dbl_mask == MVM_SUSPEND_DONE_DBL_MASK) {
		dev_info(mvm_dev->dev, "televm suspend completed\n");
		complete(&mvm_dev->mvm_suspend_done);
	} else if (dbl_mask == MVM_RESUME_DONE_DBL_MASK) {
		dev_info(mvm_dev->dev, "televm resume completed\n");
		complete(&mvm_dev->mvm_resume_done);
		enable_wfi_int(mvm_dev, false);
		ret =  send_pwr_collpase_ctrl_msg(mvm_dev, 0);
	}

	ret = gh_dbl_reset(mvm_dev->televm_rx_dbl, GH_DBL_NONBLOCK);
	if(ret){
		dev_err(mvm_dev->dev, "Error resetting rx doorbell\n");
	}
}

static int mvm_doorbell_register(struct mvm_device *mvm_dev)
{
	int ret = 0;

	if (mvm_dev->vm_variant == TELEVM) {
		mvm_dev->televm_tx_dbl = gh_dbl_tx_register(GH_DBL_MVM_TELEVM_TX_LABEL);
		if (IS_ERR_OR_NULL(mvm_dev->televm_tx_dbl)) {
			ret = PTR_ERR(mvm_dev->televm_tx_dbl);
			dev_err(mvm_dev->dev, "Failed to register televm_tx_dbl %d\n", ret);
		}

		mvm_dev->televm_rx_dbl = gh_dbl_rx_register(GH_DBL_MVM_TELEVM_RX_LABEL, mvm_televm_rx_dbl_cb, mvm_dev);
		if (IS_ERR_OR_NULL(mvm_dev->televm_rx_dbl)) {
			ret = PTR_ERR(mvm_dev->televm_rx_dbl);
			gh_dbl_tx_unregister(mvm_dev->televm_tx_dbl);
			dev_err(mvm_dev->dev, "Failed to register televm_rx_dbl %d\n", ret);
		}
	} else {
		mvm_dev->hostvm_tx_dbl = gh_dbl_tx_register(GH_DBL_MVM_TELEVM_RX_LABEL);
		if (IS_ERR_OR_NULL(mvm_dev->hostvm_tx_dbl)) {
			ret = PTR_ERR(mvm_dev->hostvm_tx_dbl);
			dev_err(mvm_dev->dev, "Failed to register hostvm_tx_dbl %d\n", ret);
		}

		mvm_dev->hostvm_rx_dbl = gh_dbl_rx_register(GH_DBL_MVM_TELEVM_TX_LABEL, mvm_hostvm_rx_dbl_cb, mvm_dev);
		if (IS_ERR_OR_NULL(mvm_dev->hostvm_rx_dbl)) {
			ret = PTR_ERR(mvm_dev->hostvm_rx_dbl);
			gh_dbl_tx_unregister(mvm_dev->hostvm_tx_dbl);
			dev_err(mvm_dev->dev, "Failed to register hostvm_rx_dbl %d\n",ret);
		}
	}

	return ret;
}

static void trigger_ssr_work_hdlr(struct work_struct *work)
{
	struct mvm_device *mvm_dev = container_of(work, struct mvm_device, trigger_ssr_work);
	int ret = 0;
	struct mvm_client *mvm_cli;
	struct mvm_client *mvm_cli_temp;
	bool p0_fifo_has_results =0 ,p1_fifo_has_results =0;
	gh_dbl_flags_t dbl_mask;

	del_timer_sync(&mvm_dev->mvm_stats_timer);
	if (mvm_dev->vm_variant == TELEVM) {
		dbl_mask = MVM_SHUTDOWN_DBL_MASK;
		ret = gh_dbl_send(mvm_dev->televm_tx_dbl, &dbl_mask, 0);
		if (ret) {
			dev_err(mvm_dev->dev, "failed to send MVM_SHUTDOWN_DBL to hostvm %d\n", ret);
			return;
		}
		dev_dbg(mvm_dev->dev, "Sent MVM_SHUTDOWN_DBL to hostvm\n");
	} else if (mvm_dev->vm_variant == PVM_ONLY) {
		ret = qcom_scm_pas_shutdown(MVM_PROC_ID);
		if (ret) {
			dev_err(mvm_dev->dev, "Error sending shutdown request to MVM\n");
			return;
		}
	}

	reinit_completion(&mvm_dev->mvm_dump_collection_done);
	mutex_lock(&mvm_dev->out_fifo_lock);
	p0_fifo_has_results = out_fifo_get_results(mvm_dev, 0);
	p1_fifo_has_results = out_fifo_get_results(mvm_dev, 1);
	mutex_unlock(&mvm_dev->out_fifo_lock);
	if (p0_fifo_has_results || p1_fifo_has_results)
		dev_dbg(mvm_dev->dev, "mvm outfifo has results and draining out fifo started\n");
	wake_up_interruptible_poll(&mvm_dev->mvm_waitqueue, POLLIN | POLLPRI);


	list_for_each_entry_safe(mvm_cli, mvm_cli_temp, &mvm_dev->client_list, list) {
		if (mvm_cli->state == CLIENT_DISCONNECTING ) {
			mvm_client_remove(mvm_dev,mvm_cli);
		}
		else {
			mvm_cli->state = CLIENT_SSR;
			dev_dbg(mvm_dev->dev, "Set client flag %d\n",mvm_cli->client_id);
		}
	}

	send_mvm_state_to_user(mvm_dev, MVM_CRASHED);//Send crash signal to clients
	ret = wait_for_completion_interruptible_timeout(
			&mvm_dev->mvm_dump_collection_done,
			msecs_to_jiffies(MVM_DUMP_COLL_TIMEOUT_MS));
	if (ret == 0) {
		dev_err(mvm_dev->dev, "Timed out as mvm dump collection is not complete\n");
	}
	mvm_dev->pending_dump_read = true;
	wake_up_interruptible_poll(&mvm_dev->ssr_poll_wait, POLLIN | POLLPRI);
	dev_info(mvm_dev->dev, "MVM subsystem is restarting after SSR\n");

	mvm_dev->state = MVM_RESTARTING;
	send_mvm_state_to_user(mvm_dev, MVM_RESTARTING);
	if (mvm_dev->vm_variant == TELEVM) {
		dbl_mask = MVM_LOAD_FW_DBL_MASK;
		ret = gh_dbl_send(mvm_dev->televm_tx_dbl, &dbl_mask, 0);
		if (ret) {
			dev_err(mvm_dev->dev, "failed to send MVM_LOAD_FW_DBL to hostvm %d\n", ret);
			return;
		}
		dev_dbg(mvm_dev->dev, "Sent MVM_LOAD_FW_DBL to hostvm\n");
	} else if (mvm_dev->vm_variant == PVM_ONLY)
		mvm_load_fw(mvm_dev);

	if (mvm_dev->vm_variant == PVM_ONLY)
		initialise_fifos(mvm_dev);

	mod_timer(&mvm_dev->mvm_stats_timer,jiffies + msecs_to_jiffies(mvm_stats_timer_interval_ms));
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
		goto err;
	}

	ret = devm_request_threaded_irq(mvm_dev->dev, mvm_dev->mvm_ssr_done_irq,
					NULL, mvm_ssr_done_irq_handler,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					"mvm_ssr_done", mvm_dev);
	if (ret < 0) {
		dev_err(mvm_dev->dev,
			"devm_request_threaded_irq of mvm_ssr_done failed %d\n", ret);
		goto err;
	}
	data = irq_get_irq_data(mvm_dev->mvm_ssr_done_irq);
	mvm_dev->mvm_ssr_done_hw_irq = data->hwirq;

	dev_dbg(mvm_dev->dev, "mvm_ssr_done irq registered\n");

	mvm_dev->mvm_verif_done_irq = platform_get_irq_byname(pdev, "mvm_verif_done");
	if (mvm_dev->mvm_verif_done_irq < 0) {
		dev_err(mvm_dev->dev, "mvm_verification_done irq not defined\n");
		goto err;
	}
	ret = devm_request_threaded_irq(mvm_dev->dev, mvm_dev->mvm_verif_done_irq,
					NULL, mvm_verif_done_irq_handler,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					"mvm_verif_done", mvm_dev);
	if (ret < 0) {
		dev_err(mvm_dev->dev,
			"devm_request_threaded_irq of mvm_verif_done_irq failed %d\n", ret);
		goto err;
	}
	data = irq_get_irq_data(mvm_dev->mvm_verif_done_irq);
	mvm_dev->mvm_verif_done_hw_irq = data->hwirq;
	dev_dbg(mvm_dev->dev, "mvm_verif_done registered\n");

	mvm_dev->wfi_irq = platform_get_irq_byname(pdev, "wfi");
	if (mvm_dev->wfi_irq < 0) {
		dev_err(mvm_dev->dev, "wfi irq not defined\n");
		goto err;
	}

	ret = devm_request_threaded_irq(mvm_dev->dev, mvm_dev->wfi_irq,
					NULL, mvm_wfi_irq_handler,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"mvm_wfi", mvm_dev);
	if (ret < 0) {
		dev_err(mvm_dev->dev,
			"devm_request_threaded_irq of wfi_irq failed %d\n", ret);
		goto err;
	}
	dev_dbg(mvm_dev->dev, "wfi_irq registered\n");

	mvm_dev->wdog_irq = platform_get_irq_byname(pdev, "wdog");
	if (mvm_dev->wdog_irq < 0) {
		dev_err(mvm_dev->dev, "wdog irq not defined\n");
		goto err;
	}

	ret = devm_request_threaded_irq(mvm_dev->dev, mvm_dev->wdog_irq,
					NULL, mvm_wdog_irq_handler,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"mvm_dog", mvm_dev);
	if (ret) {
		dev_err(mvm_dev->dev, "mvm_wdog irq request failed\n");
		goto err;
	}
	dev_dbg(mvm_dev->dev, "wdog irq registered\n");

err:
	return ret;
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
		dev_err(mvm_dev->dev, "failed to attach iommu device ret = %d\n", ret);
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
	// Static copies for crashscope to access these variables
	crashdump_buffer = mvm_dev->dump_buff;
	crashdump_dma_addr = mvm_dev->mvm_dump_dma;

	mvm_dev->log_buff = dma_alloc_coherent(mvm_dev->dev, round_up(sizeof(struct mvmlog_buffers), PAGE_SIZE),
								&mvm_dev->mvmlog_buff_dma, GFP_KERNEL);
	if (!mvm_dev->log_buff) {
		dev_err(mvm_dev->dev, "dma_alloc_coherent of log buffers failed\n");
		ret = -ENOMEM;
		goto log_buff_dma_mem_alloc_fail;
	}

	mvm_dev->mvm_fw = dma_alloc_coherent(mvm_dev->dev, MVM_FW_SIZE,
						&mvm_dev->mvm_fw_dma, GFP_KERNEL);
	if (!mvm_dev->mvm_fw) {
		dev_err(mvm_dev->dev,
				"dma_alloc_coherent of the firmware memory failed %d\n");
		ret = -ENOMEM;
		goto fw_mem_alloc_fail;
	}

	return 0;

fw_mem_alloc_fail:
	dma_free_coherent(mvm_dev->dev, round_up(sizeof(struct mvmlog_buffers), PAGE_SIZE),
					mvm_dev->log_buff, mvm_dev->mvmlog_buff_dma);
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

static long mvm_stats_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct mvm_device *mvm_dev = filp->private_data;

	switch (cmd) {
	case GET_MVM_CAPACITY:
		if (copy_to_user((unsigned int *)arg, mvm_dev->mvm_capacity, sizeof(mvm_dev->mvm_capacity)))
			ret = -EFAULT;
		break;

	case GET_MVM_STATS_MSG_COUNT:
		if (copy_to_user((unsigned int *)arg, mvm_dev->mvm_in_msg_count, sizeof(mvm_dev->mvm_in_msg_count)))
			ret = -EFAULT;
		break;

	default:
		break;
	}

	return ret;
}

static int mvm_stats_open(struct inode *inode, struct file *filp)
{
	struct mvm_device *mvm_dev = container_of(inode->i_cdev,
				struct mvm_device, mvm_stats_cdev);
	struct mvm_stats_client *mvm_stats_cli;
	int ret = 0;

	mvm_stats_cli = kzalloc(sizeof(*mvm_stats_cli), GFP_KERNEL);
	if (!mvm_stats_cli) {
		ret = -ENOMEM;
		goto ret;
	}

	INIT_LIST_HEAD(&mvm_stats_cli->list);
	mvm_stats_cli->task = get_current();
	list_add_tail(&mvm_stats_cli->list, &mvm_dev->mvm_stats_client_list);
	filp->private_data = mvm_dev;

ret:
	return 0;
}

static const struct file_operations mvm_stats_fileops = {
	.open = mvm_stats_open,
	.unlocked_ioctl = mvm_stats_ioctl,
	.owner = THIS_MODULE,
};

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


static int mvm_pm_notify(struct notifier_block *notifier,
			     unsigned long mode, void *_unused)
{
	struct mvm_device *mvm_dev =
			container_of(notifier, struct mvm_device,
			pm_notifier);
	int ret = 0;
	struct mvm_client *mvm_cli;
	switch (mode) {
	case PM_SUSPEND_PREPARE:
		if (mvm_dev->resume_frm_pwr_collapse ) {
			if (mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == TELEVM) {
				mvm_dev->resume_frm_pwr_collapse = false;
				mvm_dev->state = MVM_SUSPEND;
				send_mvm_state_to_user(mvm_dev, MVM_SUSPEND);
				del_timer_sync(&mvm_dev->mvm_stats_timer);
				reinit_completion(&mvm_dev->mvm_wfi_irq_recvd);
				ret =  send_pwr_collpase_ctrl_msg(mvm_dev, 1);
				list_for_each_entry(mvm_cli, &mvm_dev->client_list, list) {
					if (mvm_cli->state != CLIENT_DISCONNECTING) {
						mvm_cli->state = CLIENT_SUSPEND;
						dev_dbg(mvm_dev->dev, "disable mvm write clients at suspend  %d\n",mvm_cli->client_id);
					}
				}
			}
		}
		break;
	default:
		break;
	}
	return 0;
}

static int wait_for_mvmss_wfi_interrupt(struct mvm_device *mvm_dev)
{
	int is_suspend = 0;
	is_suspend = wait_for_completion_interruptible_timeout(
		&mvm_dev->mvm_wfi_irq_recvd,
		msecs_to_jiffies(WFI_TIMEOUT_MS));
	if (is_suspend == 0) {//if wfi interrupt not recieved,dont suspend
			mvm_dev->resume_frm_pwr_collapse = true;
			enable_wfi_int(mvm_dev, false);
			dev_err(mvm_dev->dev, "Timed out waiting for mvm core collapse\n");
			is_suspend = -EBUSY;
	}
	else {
		is_suspend = 0;
	}
	return is_suspend;
}

static int mvm_suspend(struct device *dev)
{
	struct mvm_device *mvm_dev = dev_get_drvdata(dev);
	int is_suspend = 0, ret = 0;
	struct mvm_client *mvm_cli;

	switch (mvm_dev->vm_variant) {
	case HOSTVM:
		is_suspend = 0;//return 0 since hostvm no need take any actions for suspend
		break;
	case PVM_ONLY:
		dev_info(mvm_dev->dev, "mvm_info:pvm suspend recieved \n");
		is_suspend = wait_for_mvmss_wfi_interrupt(mvm_dev);
		if (is_suspend == 0) {
			collapse_mvm_core(mvm_dev);
			disable_gcc_clocks(mvm_dev);
			dev_dbg(mvm_dev->dev, "MVM subsystem in power collapse mode\n");
		}
		if (is_suspend)
		{
			reinit_completion(&mvm_dev->mvm_wfi_irq_recvd);
			mvm_dev->resume_frm_pwr_collapse = true;
			enable_wfi_int(mvm_dev, false);
			list_for_each_entry(mvm_cli, &mvm_dev->client_list, list) { //if suspend is failed ,then keep client enabled
				if (mvm_cli->state != CLIENT_DISCONNECTING) {
					mvm_cli->state = CLIENT_READY;
					dev_dbg(mvm_dev->dev, "mvm suspend failed : enable clients for mvm %d\n",mvm_cli->client_id);
				}
			}
			mod_timer(&mvm_dev->mvm_stats_timer,jiffies + msecs_to_jiffies(mvm_stats_timer_interval_ms));
		}
		break;
	case TELEVM:
		is_suspend = wait_for_mvmss_wfi_interrupt(mvm_dev);
		if (is_suspend == 0) {
			gh_dbl_flags_t dbl_mask = MVM_DO_SUSPEND_DBL_MASK;
			dbl_mask = MVM_DO_SUSPEND_DBL_MASK;
			dev_info(mvm_dev->dev, "mvm_info:televm suspend recieved \n");
			reinit_completion(&mvm_dev->mvm_suspend_done);
			ret = gh_dbl_send(mvm_dev->televm_tx_dbl, &dbl_mask, 0);
			if (ret) {
				dev_err(mvm_dev->dev,
					"Failed to send MVM_SUSPEND_DBL to hostvm %d\n", ret);
			}
			else
				dev_info(mvm_dev->dev, "televm sent MVM_SUSPEND_DBL to suspend \n");
			is_suspend = wait_for_completion_interruptible_timeout(
						&mvm_dev->mvm_suspend_done,
						msecs_to_jiffies(TIMEOUT_MS));
			if (is_suspend == 0) {
				dev_err(mvm_dev->dev, "tele Timed out waiting for mvm core collapse\n");
				is_suspend = -EBUSY;
			} else {
				dev_info(mvm_dev->dev, "MVM subsystem in power collapse mode\n");
				is_suspend = 0;
			}
		}
		if (is_suspend)
		{
			reinit_completion(&mvm_dev->mvm_wfi_irq_recvd);
			mvm_dev->resume_frm_pwr_collapse = true;
			enable_wfi_int(mvm_dev, false);
			list_for_each_entry(mvm_cli, &mvm_dev->client_list, list) { //if suspend is failed ,then keep client enabled
				if (mvm_cli->state != CLIENT_DISCONNECTING) {
					mvm_cli->state = CLIENT_READY;
					dev_dbg(mvm_dev->dev, "mvm suspend failed : enable clients for mvm %d\n",mvm_cli->client_id);
				}
			}
			mod_timer(&mvm_dev->mvm_stats_timer,jiffies + msecs_to_jiffies(mvm_stats_timer_interval_ms));
		}
		break;
	default:
		break;
	}

	return is_suspend;
}

static int mvm_resume(struct device *dev)
{
	struct mvm_device *mvm_dev = dev_get_drvdata(dev);
	gh_dbl_flags_t dbl_mask = MVM_DO_RESUME_DBL_MASK;
	int is_resume = 0, ret = 0;
	struct mvm_client *mvm_cli;

	mvm_dev->resume_frm_pwr_collapse = true;
	switch (mvm_dev->vm_variant) {
	case HOSTVM:
		is_resume = 0;//return 0 since hostvm no need take any actions for resume
		break;
	case PVM_ONLY:
		dev_info(mvm_dev->dev, "pvm resume recieved \n");
		is_resume = enable_gcc_clocks(mvm_dev);
		if (is_resume)
			dev_err(mvm_dev->dev, "Failed to turn on gcc clocks\n");
		else {
			restore_mvm_core(mvm_dev);
			is_resume = 0;
		}
		enable_wfi_int(mvm_dev, false);
		ret =  send_pwr_collpase_ctrl_msg(mvm_dev, 0);
		break;
	case TELEVM:
		reinit_completion(&mvm_dev->mvm_resume_done);
		ret = gh_dbl_send(mvm_dev->televm_tx_dbl, &dbl_mask, 0);
		if (ret) {
			dev_err(mvm_dev->dev,
				"Failed to send MVM_RESUME_DBL to hostvm %d\n", ret);
		}
		is_resume = wait_for_completion_interruptible_timeout(
					&mvm_dev->mvm_resume_done,
					msecs_to_jiffies(TIMEOUT_MS));
		if (is_resume == 0)
			dev_err(mvm_dev->dev, "Couldnt restore mvm subsystem resume done wait failed\n");
		else {
			dev_info(mvm_dev->dev, "MVM subsystem in Restored\n");
			is_resume =0;
		}
		break;
	default:
		break;
	}
	if(mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == TELEVM) {
		list_for_each_entry(mvm_cli, &mvm_dev->client_list, list) {
			if (mvm_cli->state != CLIENT_DISCONNECTING) {
				mvm_cli->state = CLIENT_READY;
				dev_dbg(mvm_dev->dev, "mvm is restoring and enable clients for mvm %d\n",mvm_cli->client_id);
			}
		}
		mvm_dev->state = MVM_ONLINE;
		send_mvm_state_to_user(mvm_dev, MVM_ONLINE);
		mvm_dev->curr_clk = LOW_SVS;
		mvm_dev->req_clk = LOW_SVS;
		mvm_dev->prev_pke_time = 0;
		mod_timer(&mvm_dev->mvm_stats_timer,jiffies + msecs_to_jiffies(mvm_stats_timer_interval_ms));
	}
	return is_resume;
}

static void change_clk_freq_work_hdlr(struct work_struct *work)
{
	struct mvm_device *mvm_dev = container_of(work, struct mvm_device, change_clk_freq_work);
	int ret = 0;
	unsigned int pke_utility_now = 0;
	bool change_clock = true;
	ktime_t now,de_bounce_time;

	/*Do not change clock frequency when entering suspend or when MVM has crashed */
	if (mvm_dev->state == MVM_CRASHED || mvm_dev->state == MVM_SUSPEND)
		return;

	pke_utility_now = mvm_dev->ring_buff->pke_time_accumulator;
	mvm_dev->curr_pke_util= (100 * (pke_utility_now - mvm_dev->prev_pke_time)) / (PKE_COUNT  * PKE_UTIL_PERIOD_TICKS);
	mvm_dev->curr_pke_util = (mvm_dev->prev_pke_util + mvm_dev->curr_pke_util)/2;//take average of pke_prev_util and pke_curr_util
	mvm_dev->prev_pke_util = mvm_dev->curr_pke_util;
	mvm_dev->prev_pke_time = pke_utility_now;
	pke_utility_now = mvm_dev->curr_pke_util;
	now = ktime_get();
	if (mvm_dev->max_rate < CLK_CHANGE_MIN_CAPACITY) {//When max_rate is 3000 or less, operate at LOW_SVS.
		change_clock = false;
	}
	if (mvm_dev->curr_clk == LOW_SVS) {
		if (mvm_dev->curr_pke_util < MAX_PKE_UTIL_LOW_SVS)
			change_clock = false;//operate in low_svs untill utilization upto MAX_PKE_UTIL_LOW_SVS
		else
			pke_utility_now = PKE_UTIL_ADJUST_FACTOR_LOWSVS(pke_utility_now);
		/*Once pke utilisation goes beyond MAX_PKE_UTIL_LOW_SVS, then clock will be switched to SVS.
		Since we are running at a higher frequency compared to the previous run, utilisation will drop now.
		Hence adjust the pke utilisation with the division factor to avoid switching to LOW_SVS or NOMINAL*/
	}
	else if((mvm_dev->curr_clk == SVS)&& (mvm_dev->curr_pke_util > PKE_UTIL_UPPER_THRESH) &&(mvm_dev->curr_pke_util < PKE_UTIL_ADJUST_THRESH)){
		/*When operating at SVS, do not switch the clock unless the pke utilisation is more than PKE_UTIL_ADJUST_THRESH.*/
		pke_utility_now = PKE_UTIL_ADJUST_FACTOR_SVS(pke_utility_now);//Adjust the pke utilisation value by PKE_UTIL_ADJUST_FACTOR_SVS to ensure we dont switch clock unnecessarily.
	}
	de_bounce_time = ktime_sub(now, mvm_dev->clk_time_elapsed);
	if((mvm_dev->req_clk == mvm_dev->curr_clk) && (ktime_to_ms(de_bounce_time) > MIN_CLOCK_CHANGE_TIME_MS) && (change_clock)){
		switch (pke_utility_now) {
		 case LOW_SVS_THRESH_RANGE:
		 	mvm_dev->req_clk = LOW_SVS;
		 	break;
		 case SVS_THRESH_RANGE:
			mvm_dev->req_clk = SVS;
		 	break;
		 case NOMINAL_THRESH_RANGE:
		 	mvm_dev->req_clk = NOMINAL;
		 	break;
		 default:
		 	break;
		}
		if (mvm_dev->req_clk != mvm_dev->curr_clk) {
			/* Send control message to change the clock frequency */
			struct mvm_control *mvm_ctrl = kzalloc(sizeof(struct mvm_control), GFP_KERNEL);
			if (mvm_ctrl) {
				mvm_ctrl->type = MVM_CHANGE_CLK_FREQ;
				mvm_ctrl->mvm_ctrl_msg.mvm_clk_freq.clk_freq = mvm_dev->req_clk;
				ret = send_ctrl_msg_to_mvm(mvm_ctrl, mvm_dev);
				dev_dbg(mvm_dev->dev, "time taken for changing clock rate %lld %d\n",ktime_to_ms(de_bounce_time));
				mvm_dev->clk_time_elapsed = ktime_get();
			}
			else {
				dev_err(mvm_dev->dev, "change_clk_freq_work_hdlr failed with null mvm_ctrl pointer\n");
			}
		}
	}
	mod_timer(&mvm_dev->mvm_stats_timer,jiffies + msecs_to_jiffies(mvm_stats_timer_interval_ms));
}

static void mvm_stats_monitor_timeout_handler(struct timer_list *t)
{
	struct mvm_device *mvm_dev = from_timer(mvm_dev, t, mvm_stats_timer);

	schedule_work(&mvm_dev->change_clk_freq_work);
}

static int mvm_stats_init(struct mvm_device *mvm_dev)
{
	int i, ret = 0;

	ret = alloc_chrdev_region(&mvm_dev->mvm_stats_cdev_devid, 0, 2, "mvm_stats");
	if (ret < 0) {
		dev_err(mvm_dev->dev,
			"can't allocate major number, %d\n", ret);
		goto ret;
	}

	cdev_init(&mvm_dev->mvm_stats_cdev, &mvm_stats_fileops);
	cdev_add(&mvm_dev->mvm_stats_cdev, mvm_dev->mvm_stats_cdev_devid, 1);

	mvm_dev->mvm_stats_class = class_create(THIS_MODULE, "mvm_stats");
	if (IS_ERR(mvm_dev->mvm_stats_class)) {
		dev_err(mvm_dev->dev,
			"can't create mvm_stats class, %d\n",
			-ENOMEM);
		goto class_fail;
	}

	mvm_dev->mvm_stats_dev = device_create(mvm_dev->mvm_stats_class, NULL,
				mvm_dev->mvm_stats_cdev_devid, NULL, "mvm_stats");
	if (IS_ERR(mvm_dev->mvm_stats_dev)) {
		dev_err(mvm_dev->dev,
			"can't create mvm_stats device, %d\n",
			-ENOMEM);
		goto device_fail;
	}

	dev_info(mvm_dev->dev, "mvm_stats device created\n");

	for(i=0; i<MAX_CURVES; i++) {
		mvm_dev->mvm_capacity[i] = MVM_INITIAL_CAPACITY;
		mvm_dev->mvm_in_msg_count[i] = 0;
	}

	mvm_dev->curr_clk = LOW_SVS;
	mvm_dev->req_clk = LOW_SVS;
	mvm_dev->max_clk = LOW_SVS;

	timer_setup(&mvm_dev->mvm_stats_timer, mvm_stats_monitor_timeout_handler, 0);
	mvm_dev->mvm_stats_timer.expires = jiffies + msecs_to_jiffies(mvm_stats_timer_interval_ms);
	add_timer(&mvm_dev->mvm_stats_timer);

	return 0;

device_fail:
	class_destroy(mvm_dev->mvm_stats_class);
class_fail:
	cdev_del(&mvm_dev->mvm_stats_cdev);
	unregister_chrdev_region(mvm_dev->mvm_stats_cdev_devid, 1);
ret:
	return ret;
}

static int mvm_probe(struct platform_device *pdev)
{
	struct device_node *node;
	struct mvm_device *mvm_dev;
	int ret;

	node = pdev->dev.of_node;
	mvm_dev = devm_kzalloc(&pdev->dev, sizeof(*mvm_dev), GFP_KERNEL);
	if (!mvm_dev)
		return -ENOMEM;

	mvm_dev->dev = &pdev->dev;

	platform_set_drvdata(pdev, mvm_dev);

	ret = of_property_read_u32(node, "qcom,vm-variant", &mvm_dev->vm_variant);
	if (ret) {
		dev_err(mvm_dev->dev, "qcom,vm_variant property not defined\n");
	}

	if (mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == TELEVM) {
		struct device *dev;

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
				-ENOMEM)	;
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
	}

	ret = ioremap_resources(pdev);
	if (ret) {
		dev_err(mvm_dev->dev, "Cant ioremap resources\n");
		goto ioremap_fail;
	}

	ret = mvm_sysfs_init(mvm_dev);
	if (ret) {
		dev_err(mvm_dev->dev, "mvm sysfs initialisation failed\n");
		goto sysfs_fail;
	}

	mvm_dev->state = MVM_OFFLINE;
	dev_info(mvm_dev->dev, "The current state of MVM is OFFLINE\n");

	if (mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == HOSTVM) {
		ret = mvm_dma_mem_alloc(mvm_dev);
		if (ret)
			goto dma_mem_fail;

		ret = mvm_iommu_init(mvm_dev);
		if (ret)
			goto iommu_init_fail;
	}

	if(mvm_dev->vm_variant == HOSTVM) {
		mvm_dev->rm_nb.notifier_call = qcom_mvm_rm_cb;
		mvm_dev->rm_nb.priority = INT_MAX;
#ifdef CONFIG_GUNYAH
		gh_register_vm_notifier(&mvm_dev->rm_nb);
#endif /* CONFIG_GUNYAH */
	} else if (mvm_dev->vm_variant == TELEVM) {
		ret = read_shm_labels(mvm_dev);
		if (ret) {
			dev_err(mvm_dev->dev, "read_shm_labels failed\n");
			goto shm_label_fail;
		}

		ret = mvm_televm_map_shared_mem(mvm_dev, "mvm-ring-buff-shm",
						mvm_dev->mvm_ring_buff_shm_label);
		if (ret) {
			dev_err(mvm_dev->dev, "Couldnt map ring buffers from hostvm to televm\n");
			ret = -ENOMEM;
			goto shm_label_fail;
		}

		ret = mvm_televm_map_shared_mem(mvm_dev, "mvm-dump-buff-shm", mvm_dev->mvm_dump_buff_shm_label);
		if (ret) {
			dev_err(mvm_dev->dev, "Couldnt map ring buffers from hostvm to televm\n");
			ret = -ENOMEM;
			goto shm_label_fail;
		}

		ret = mvm_televm_map_shared_mem(mvm_dev, "mvm-log-buff-shm", mvm_dev->mvm_log_buff_shm_label);
		if (ret) {
			dev_err(mvm_dev->dev, "Couldnt map ring buffers from hostvm to televm\n");
			ret = -ENOMEM;
			goto shm_label_fail;
		}
	}

	if (mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == TELEVM) {
		ret = register_isrs(pdev);
		if (ret) {
			dev_err(mvm_dev->dev,
				"registration of isrs failed\n");
			goto isrs_fail;
		}

		ret = mvm_debugfs_init(mvm_dev);
		if (ret)
			goto isrs_fail;

		mutex_init(&mvm_dev->mvm_cli_lock);
		mutex_init(&mvm_dev->in_fifo_lock);
		mutex_init(&mvm_dev->out_fifo_lock);

		init_completion(&mvm_dev->p0_fifo_slot_available);
		init_completion(&mvm_dev->p1_fifo_slot_available);
		init_completion(&mvm_dev->mvm_dump_collection_done);
		init_completion(&mvm_dev->mvm_wfi_irq_recvd);
		init_completion(&mvm_dev->mvm_suspend_done);
		init_completion(&mvm_dev->mvm_resume_done);

		INIT_LIST_HEAD(&mvm_dev->client_list);
		INIT_LIST_HEAD(&mvm_dev->mvm_stats_client_list);
		init_waitqueue_head(&mvm_dev->mvm_waitqueue);
		init_waitqueue_head(&mvm_dev->log_poll_wait);
		init_waitqueue_head(&mvm_dev->ssr_poll_wait);
		INIT_WORK(&mvm_dev->drain_out_fifo_work, drain_out_fifo_work_hdlr);
		INIT_WORK(&mvm_dev->trigger_ssr_work, trigger_ssr_work_hdlr);
		INIT_WORK(&mvm_dev->change_clk_freq_work, change_clk_freq_work_hdlr);

		bitmap_zero(mvm_dev->client_id_bitmap, MAX_CLIENT_COUNT);
		mvm_dev->pending_dump_read = false;
	}
	if (mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == TELEVM) {
		mvm_dev->pm_notifier.notifier_call = mvm_pm_notify;
		ret = register_pm_notifier(&mvm_dev->pm_notifier);
		if (ret)
			goto pm_err;
	}

	if (mvm_dev->vm_variant == HOSTVM || mvm_dev->vm_variant == TELEVM) {
		ret = mvm_doorbell_register(mvm_dev);
		if (ret) {
			dev_err(mvm_dev->dev, "mvm_doorbell_register failed\n");
			goto doorbell_fail;
		}
	}
	mvm_dev->resume_frm_pwr_collapse = true;
	if (mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == HOSTVM) {

		ret = request_firmware(&mvm_dev->fw, "mvm_ecc.mdt", mvm_dev->dev);
		if (ret) {
			dev_err(mvm_dev->dev, "request_firmware for mvm failed\n");
			goto doorbell_fail;
		}

		ret = mvm_load_fw(mvm_dev);
		if (ret)
			goto doorbell_fail;
	}

	if (mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == TELEVM) {
		ret = mvm_stats_init(mvm_dev);
		if (ret) {
			dev_err(mvm_dev->dev, "mvm_stats_init failed\n");
			goto doorbell_fail;
		}
		initialise_fifos(mvm_dev);
	}

	return 0;

doorbell_fail:
	if (mvm_dev->vm_variant == PVM_ONLY) {
		mutex_destroy(&mvm_dev->in_fifo_lock);
		mutex_destroy(&mvm_dev->out_fifo_lock);
		mutex_destroy(&mvm_dev->mvm_cli_lock);
	}
	if(mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == TELEVM) {
		unregister_pm_notifier(&mvm_dev->pm_notifier);
	}
pm_err:
	if (mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == TELEVM)
		debugfs_remove_recursive(mvm_dev->dir);
isrs_fail:
	if (mvm_dev->vm_variant == TELEVM) {
		devm_iounmap(mvm_dev->dev, mvm_dev->ring_buff);
		devm_iounmap(mvm_dev->dev, mvm_dev->dump_buff);
		devm_iounmap(mvm_dev->dev, mvm_dev->log_buff);
	}
shm_label_fail:
	if (mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == HOSTVM)
		mvm_iommu_release(mvm_dev);
iommu_init_fail:
	if (mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == HOSTVM)
		mvm_dma_mem_free(mvm_dev);
dma_mem_fail:
	if(mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == TELEVM) {
		sysfs_remove_file(mvm_dev->kobj, &mvm_dev->mvm_rate_lut_attr.attr);
		sysfs_remove_file(mvm_dev->kobj, &mvm_dev->mvm_capacity_attr.attr);
		sysfs_remove_file(mvm_dev->kobj, &mvm_dev->mvm_max_clk_attr.attr);
		sysfs_remove_file(mvm_dev->kobj, &mvm_dev->mvm_curr_clk_attr.attr);
	}
	sysfs_remove_file(mvm_dev->kobj, &mvm_dev->mvm_state_attr.attr);
	kobject_put(mvm_dev->kobj);
sysfs_fail:
	devm_iounmap(mvm_dev->dev, mvm_dev->mvm_base);
	devm_iounmap(mvm_dev->dev, mvm_dev->apss_shared_base);
ioremap_fail:
	if (mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == TELEVM)
		device_destroy(mvm_dev->mvm_class, mvm_dev->mvm_cdev_devid);
device_fail:
	if (mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == TELEVM)
		class_destroy(mvm_dev->mvm_class);
class_fail:
	if (mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == TELEVM) {
		cdev_del(&mvm_dev->mvm_cdev);
		unregister_chrdev_region(mvm_dev->mvm_cdev_devid, 1);
	}
drv_err:
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int mvm_remove(struct platform_device *pdev)
{
	struct mvm_device *mvm_dev;

	mvm_dev = dev_get_drvdata(&pdev->dev);

	if (mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == TELEVM) {
		mutex_destroy(&mvm_dev->in_fifo_lock);
		mutex_destroy(&mvm_dev->out_fifo_lock);
		mutex_destroy(&mvm_dev->mvm_cli_lock);
		device_destroy(mvm_dev->mvm_stats_class, mvm_dev->mvm_stats_cdev_devid);
		class_destroy(mvm_dev->mvm_stats_class);
		cdev_del(&mvm_dev->mvm_stats_cdev);
		unregister_chrdev_region(mvm_dev->mvm_stats_cdev_devid, 1);
		del_timer_sync(&mvm_dev->mvm_stats_timer);
	}

        if (mvm_dev->vm_variant == HOSTVM) {
                gh_rm_mem_reclaim(mvm_dev->mvmss_mem_handle, 0);
                gh_rm_mem_reclaim(mvm_dev->apss_mem_handle, 0);
		mvm_hostvm_unshare_mem(mvm_dev);
		gh_dbl_tx_unregister(mvm_dev->hostvm_tx_dbl);
		gh_dbl_rx_unregister(mvm_dev->hostvm_rx_dbl);
        }

	if (mvm_dev->vm_variant == TELEVM) {
		gh_dbl_tx_unregister(mvm_dev->televm_tx_dbl);
		gh_dbl_rx_unregister(mvm_dev->televm_rx_dbl);
	}

	if (mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == HOSTVM) {
		mvm_iommu_release(mvm_dev);
                mvm_dma_mem_free(mvm_dev);
		clk_disable_unprepare(mvm_dev->cnoc_s_ahb_clk);
		clk_disable_unprepare(mvm_dev->xo);
		clk_disable_unprepare(mvm_dev->snoc_m_axi_clk);
		clk_disable_unprepare(mvm_dev->sysnoc_mvmss_clk);
	}
	if(mvm_dev->vm_variant == PVM_ONLY || mvm_dev->vm_variant == TELEVM) {
		unregister_pm_notifier(&mvm_dev->pm_notifier);
		sysfs_remove_file(mvm_dev->kobj, &mvm_dev->mvm_rate_lut_attr.attr);
		sysfs_remove_file(mvm_dev->kobj, &mvm_dev->mvm_capacity_attr.attr);
		sysfs_remove_file(mvm_dev->kobj, &mvm_dev->mvm_max_clk_attr.attr);
		sysfs_remove_file(mvm_dev->kobj, &mvm_dev->mvm_curr_clk_attr.attr);
		debugfs_remove_recursive(mvm_dev->dir);
	}
	sysfs_remove_file(mvm_dev->kobj, &mvm_dev->mvm_state_attr.attr);
	kobject_put(mvm_dev->kobj);
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
module_param(mvm_stats_timer_interval_ms, int, 0644);
module_exit(mvm_unregister);
MODULE_LICENSE("GPL v2");
