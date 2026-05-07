// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL62X1 DVB-S/S2/S2X demodulator driver
 * Supports AVL6221 and AVL6261. NOT AVL6211
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/vmalloc.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/bitrev.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/seq_file.h>
#include <linux/dvb/frontend.h>
#include <media/dvb_frontend.h>
#include <linux/slab.h>
#include <linux/ioctl.h>

#include "avl62x1.h"
#include "avl62x1_api.h"
#include "avl_tuner.h"
#include "av201x.h"


#define INCLUDE_STDOUT	0

#define p_debug(fmt, args...)					\
	do							\
	{							\
		if (debug)					\
			printk("avl62x1:%s DEBUG: " fmt "\n",	\
				__func__, ##args);		\
	} while (0);

#define p_debug_lvl(lvl,fmt, args...)				\
	do							\
	{							\
		if (debug >= lvl)				\
			printk("avl62x1:%s DEBUG: " fmt "\n",	\
				__func__, ##args);		\
	} while (0);

#define p_error(fmt, args...)					\
	do							\
	{							\
		printk("avl62x1:%s ERROR: " fmt "\n",		\
			__func__, ##args);			\
	} while (0);

#define p_info(fmt, args...)					\
	do							\
	{							\
		printk("avl62x1:%s INFO: " fmt "\n",		\
			__func__, ##args);			\
	} while (0);

#define p_warn(fmt, args...)					\
	do							\
	{							\
		printk("avl62x1:%s WARN: " fmt "\n",		\
			__func__, ##args);			\
	} while (0);

#define safe_mutex_unlock(m)					\
	if(mutex_is_locked(m)) {				\
		mutex_unlock(m);				\
	} else {						\
		p_warn("Tried to unlock mutex that wasn't locked"); \
	}

static struct avl62x1_priv *global_priv = NULL;
static char sel_fw[256] = {0};

//--- module params ---
static int debug = 0;
char fw_path[256] = {0};
//-------------------

#define MHz 1000000UL
#define kHz 1000UL

/* Mutexes for I2C bus arbitration between demodulator and tuner.
 * Previously declared as part of the i2cctl character device block.
 * Retained here as they are used throughout the driver for I2C access control. */
static DEFINE_MUTEX(i2cctl_fe_mutex);
static DEFINE_MUTEX(i2cctl_tuneri2c_mutex);

/* MODCOD lookup tables for DVB-S2 and DVB-S2X */
static unsigned char modcod[32][2] = //indexed by PLS/4
{
	{QAM_AUTO,	FEC_AUTO}, //0 DUMMY PLFRAME

#ifdef AVL_S2X_ENUMS //Even though 1/4 and 1/3 are NOT S2X specific!!!
	{QPSK,		FEC_1_4}, //1
	{QPSK,		FEC_1_3}, //2
#else
	{QPSK,		FEC_AUTO},
	{QPSK,		FEC_AUTO},
#endif
	{QPSK,		FEC_2_5}, //3
	{QPSK,		FEC_1_2},
	{QPSK,		FEC_3_5},
	{QPSK,		FEC_2_3},
	{QPSK,		FEC_3_4},
	{QPSK,		FEC_4_5},
	{QPSK,		FEC_5_6},
	{QPSK,		FEC_8_9},
	{QPSK,		FEC_9_10}, //11

	{PSK_8,		FEC_3_5}, //12
	{PSK_8,		FEC_2_3},
	{PSK_8,		FEC_3_4},
	{PSK_8,		FEC_5_6},
	{PSK_8,		FEC_8_9},
	{PSK_8,		FEC_9_10},

	{APSK_16,	FEC_2_3}, //18
	{APSK_16,	FEC_3_4},
	{APSK_16,	FEC_4_5},
	{APSK_16,	FEC_5_6},
	{APSK_16,	FEC_8_9},
	{APSK_16,	FEC_9_10},

	{APSK_32,	FEC_3_4}, //24
	{APSK_32,	FEC_4_5},
	{APSK_32,	FEC_5_6},
	{APSK_32,	FEC_8_9},
	{APSK_32,	FEC_9_10},

	{QAM_AUTO,	FEC_AUTO}, //29 Reserved
	{QAM_AUTO,	FEC_AUTO}, //30 Reserved
	{QAM_AUTO,	FEC_AUTO}, //31 Reserved
};

#ifdef AVL_S2X_ENUMS
static unsigned char s2x_modcod[][2] = //indexed by (PLS-128)/2
{
	{QAM_AUTO,	FEC_AUTO}, //128 VL SNR set 1 UNIMP
	{QAM_AUTO,	FEC_AUTO}, //130 VL SNR set 2 UNIMP

	{QPSK,		FEC_13_45}, //132
	{QPSK,		FEC_9_20},
	{QPSK,		FEC_11_20},

	{APSK_8_L,	FEC_5_9}, //138
	{APSK_8_L,	FEC_26_45},
	{PSK_8,		FEC_23_36}, //142
	{PSK_8,		FEC_25_36},
	{PSK_8,		FEC_13_18},

	{APSK_16_L,	FEC_1_2}, //148
	{APSK_16_L,	FEC_8_15},
	{APSK_16_L,	FEC_5_9},
	{APSK_16,	FEC_26_45}, //154
	{APSK_16,	FEC_3_5},
	{APSK_16_L,	FEC_3_5}, //158
	{APSK_16,	FEC_28_45}, //160
	{APSK_16,	FEC_23_36},
	{APSK_16_L,	FEC_2_3}, //164
	{APSK_16,	FEC_25_36}, //166
	{APSK_16,	FEC_13_18},
	{APSK_16,	FEC_7_9},
	{APSK_16,	FEC_77_90},

	{APSK_32_L,	FEC_2_3}, //174
	{QAM_AUTO,	FEC_AUTO}, //176 UNUSED
	{APSK_32,	FEC_32_45}, //178
	{APSK_32,	FEC_11_15},
	{APSK_32,	FEC_7_9},

	{APSK_64_L,	FEC_32_45}, //184
	{APSK_64,	FEC_11_15},
	{QAM_AUTO,	FEC_AUTO}, //188 UNUSED
	{APSK_64,	FEC_7_9}, //190
	{QAM_AUTO,	FEC_AUTO}, //192 UNUSED
	{APSK_64,	FEC_4_5}, //194
	{QAM_AUTO,	FEC_AUTO}, //196 UNUSED
	{APSK_64,	FEC_5_6}, //198

	{QAM_AUTO,	FEC_AUTO}, //200-202 UNIMP 128APSK
	{QAM_AUTO,	FEC_AUTO},

	{QAM_AUTO,	FEC_AUTO}, //204 UNIMP 256APSK(L)
	{QAM_AUTO,	FEC_AUTO},
	{QAM_AUTO,	FEC_AUTO},
	{QAM_AUTO,	FEC_AUTO},
	{QAM_AUTO,	FEC_AUTO},
	{QAM_AUTO,	FEC_AUTO}, //214

	{QPSK,		FEC_AUTO}, //216 short
	{QPSK,		FEC_AUTO},
	{QPSK,		FEC_AUTO},
	{QPSK,		FEC_AUTO},
	{QPSK,		FEC_8_15},
	{QPSK,		FEC_32_45},

	{PSK_8,		FEC_AUTO}, //228
	{PSK_8,		FEC_8_15},
	{PSK_8,		FEC_26_45},
	{PSK_8,		FEC_32_45},

	{APSK_16,	FEC_AUTO}, //236
	{APSK_16,	FEC_8_15},
	{APSK_16,	FEC_26_45},
	{APSK_16,	FEC_3_5},
	{APSK_16,	FEC_32_45},

	{APSK_32,	FEC_2_3}, //246
	{APSK_32,	FEC_32_45}
};
#endif

static int avl62x1_blindscan_props(struct dtv_frontend_properties *props,
				   struct avl62x1_carrier_info *carrier_info,
				   struct avl62x1_stream_info *stream_info)
{
	props->frequency = carrier_info->rf_freq_khz;

	props->symbol_rate = carrier_info->symbol_rate_hz;

	props->cnr.len = 1;
	props->cnr.stat[0].scale = FE_SCALE_DECIBEL;
	props->cnr.stat[0].svalue = carrier_info->snr_db_x100 * 10;

	props->scrambling_sequence_index = carrier_info->pl_scrambling & AVL62X1_PL_SCRAM_XSTATE_MASK;
	if (carrier_info->pl_scrambling & AVL62X1_PL_SCRAM_XSTATE)
		__avl62x1_conv_xlfsr_state_to_n(props->scrambling_sequence_index, &props->scrambling_sequence_index);

	if (carrier_info->signal_type == avl62x1_dvbs) {
		props->delivery_system = SYS_DVBS;
		props->modulation = QPSK;
		props->pilot = PILOT_OFF;

		switch (carrier_info->code_rate.dvbs_code_rate) {
		case avl62x1_dvbs_cr_1_2:
			props->fec_inner = FEC_1_2;
			break;
		case avl62x1_dvbs_cr_2_3:
			props->fec_inner = FEC_2_3;
			break;
		case avl62x1_dvbs_cr_3_4:
			props->fec_inner = FEC_3_4;
			break;
		case avl62x1_dvbs_cr_5_6:
			props->fec_inner = FEC_5_6;
			break;
		case avl62x1_dvbs_cr_7_8:
			props->fec_inner = FEC_7_8;
			break;
		default:
			break;
		}

		props->rolloff = ROLLOFF_35;
		props->stream_id = NO_STREAM_ID_FILTER;
		props->AVL62X1_T2MI_CTRL_PROP = NO_STREAM_ID_FILTER;
	} else if (carrier_info->signal_type == avl62x1_dvbs2) {
		props->delivery_system = SYS_DVBS2;

		if (carrier_info->dvbs2_ccm_acm == avl62x1_dvbs2_ccm) {
#ifdef AVL_S2X_ENUMS
			if (carrier_info->pls_acm >= 128) {
				int idx = (carrier_info->pls_acm - 128) / 2;
				idx = (idx >= ARRAY_LENGTH(s2x_modcod)) ? 0 : idx;
				props->modulation = s2x_modcod[idx][0];
				props->fec_inner = s2x_modcod[idx][1];
			} else
#endif
			{
				int idx = carrier_info->pls_acm / 4;
				idx = (idx >= ARRAY_LENGTH(modcod)) ? 0 : idx;
				props->modulation = modcod[idx][0];
				props->fec_inner = modcod[idx][1];
			}
			props->pilot = (carrier_info->pls_acm & 1) ? PILOT_ON : PILOT_OFF;
		} else {
			props->modulation = QAM_AUTO;
			props->fec_inner = FEC_AUTO;
			props->pilot = PILOT_ON;
		}

		switch (carrier_info->code_rate.dvbs2_code_rate) {
		case avl62x1_dvbs2_cr_2_5:
			props->fec_inner = FEC_2_5;
			break;
		case avl62x1_dvbs2_cr_1_2:
			props->fec_inner = FEC_1_2;
			break;
		case avl62x1_dvbs2_cr_3_5:
			props->fec_inner = FEC_3_5;
			break;
		case avl62x1_dvbs2_cr_2_3:
			props->fec_inner = FEC_2_3;
			break;
		case avl62x1_dvbs2_cr_3_4:
			props->fec_inner = FEC_3_4;
			break;
		case avl62x1_dvbs2_cr_4_5:
			props->fec_inner = FEC_4_5;
			break;
		case avl62x1_dvbs2_cr_5_6:
			props->fec_inner = FEC_5_6;
			break;
		case avl62x1_dvbs2_cr_8_9:
			props->fec_inner = FEC_8_9;
			break;
		case avl62x1_dvbs2_cr_9_10:
			props->fec_inner = FEC_9_10;
			break;
		default:
			props->fec_inner = FEC_AUTO;
			break;
		}

		props->rolloff = ROLLOFF_AUTO;

		if (carrier_info->sis_mis == avl62x1_mis)
			props->stream_id = stream_info->isi;
		else
			props->stream_id = NO_STREAM_ID_FILTER;

		if (stream_info->stream_type == avl62x1_t2mi) {
			props->AVL62X1_T2MI_CTRL_PROP  = AVL62X1_T2MI_CTRL_VALID_STREAM_MASK;
			props->AVL62X1_T2MI_CTRL_PROP |= stream_info->t2mi.plp_id;
			props->AVL62X1_T2MI_CTRL_PROP |= stream_info->t2mi.pid << AVL62X1_T2MI_PID_SHIFT;
		} else {
			props->AVL62X1_T2MI_CTRL_PROP = NO_STREAM_ID_FILTER;
		}
	}

	return 0;
}

static int avl62x1_blindscan_get_streams(struct dvb_frontend *fe,
					 struct avl62x1_carrier_info *carrier)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;
	struct avl62x1_stream_info *streams, *stream;
	uint8_t num_streams;
	uint8_t num_splp_mplps;
	uint16_t cntr;
	avl62x1_lock_status lock;
	const uint16_t timeout = 10;
	const uint32_t delay = 20;
	int i, j, ret;

	ret = avl62x1_get_num_streams(&num_streams, priv->chip);
	if (ret != AVL_EC_OK)
		return 0;

	streams = kvcalloc(num_streams, sizeof(*stream), GFP_KERNEL);
	if (!streams)
		return -ENOMEM;

	ret = avl62x1_blindscan_get_stream_list(carrier, streams, num_streams, priv->chip);
	if (ret != AVL_EC_OK)
		goto err_free_streams;

	for (i = 0; i < num_streams; i++) {
		stream = streams + i;

		if (stream->stream_type == avl62x1_t2mi) {
			ret = avl62x1_switch_stream(stream, priv->chip);

			lock = avl62x1_status_unlocked;
			cntr = 0;
			do {
				avl_bsp_delay(delay);
				ret = avl62x1_get_lock_status(&lock, priv->chip);
			} while ((lock == avl62x1_status_unlocked) && (cntr < timeout) && (ret == AVL_EC_OK));

			if (ret != AVL_EC_OK || cntr >= timeout)
				continue;

			avl62x1_get_t2mi_plp_list(&stream->t2mi.plp_list, priv->chip);

			num_splp_mplps = stream->t2mi.plp_list.list_size;
		} else {
			num_splp_mplps = 1;
		}

		for (j = 0; j < num_splp_mplps; j++) {
			struct dtv_frontend_properties *props = state->props + state->num_props;

			if (stream->stream_type == avl62x1_t2mi)
				stream->t2mi.plp_id = stream->t2mi.plp_list.list[j];

			avl62x1_blindscan_props(props, carrier, stream);

			state->num_props++;
		}
	}

err_free_streams:
	kvfree(streams);

	return 0;
}

static int avl62x1_blindscan_get_carriers(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;
	struct avl62x1_carrier_info *carriers, *carrier;
	uint16_t cntr;
	enum avl62x1_discovery_status status;
	const uint16_t timeout = 20;
	const uint32_t delay = 100;
	int i, ret;

	carriers = kvcalloc(state->info.num_carriers, sizeof(*carrier), GFP_KERNEL);
	if (!carriers)
		return -ENOMEM;

	ret = avl62x1_blindscan_get_carrier_list(&state->params, &state->info, carriers, priv->chip);
	if (ret != AVL_EC_OK)
		goto err_free_carriers;

	for (i = 0; i < state->info.num_carriers; i++) {
		carrier = carriers + i;

		ret = avl62x1_blindscan_confirm_carrier(&state->params, carrier, priv->chip);

		status = avl62x1_discovery_running;
		cntr = 0;
		do {
			ret = avl62x1_get_discovery_status(&status, priv->chip);
			avl_bsp_delay(delay);
			cntr++;
		} while ((status == avl62x1_discovery_running) && (cntr < timeout) && (ret == AVL_EC_OK));

		if (ret != AVL_EC_OK || cntr >= timeout)
			continue;

		ret = avl62x1_blindscan_get_streams(fe, carrier);
		if (ret)
			goto err_free_carriers;
	}

err_free_carriers:
	kvfree(carriers);

	return 0;
}

static int avl62x1_blindscan_thread(void *data)
{
	struct dvb_frontend *fe = data;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;
	uint16_t bw_ratio, bs_min_sr;
	u32 bs_min_symbol_rate = 1000000;
	u32 frequency = state->min_prop.frequency;
	uint16_t cntr;
	const uint16_t timeout = 200;
	const uint32_t delay = 100;
	int i, j, k, ret;

	/* TODO: workaround */
	avl_bms_read16(priv->chip->chip_pub->i2c_addr,
		       c_AVL62X1_S2X_BWfilt_to_Rsamp_ratio_saddr, &bw_ratio);
	avl_bms_read16(priv->chip->chip_pub->i2c_addr,
		       c_AVL62X1_S2X_blind_scan_min_sym_rate_kHz_saddr, &bs_min_sr);

	if (state->min_prop.symbol_rate < bs_min_symbol_rate)
		state->min_prop.symbol_rate = bs_min_symbol_rate;

	while (frequency < state->max_prop.frequency) {
		if (kthread_should_stop())
			break;

		priv->chip->chip_pub->tuner->rf_freq_hz = frequency * 1000;
		priv->chip->chip_pub->tuner->lpf_hz = 0xffffffff;
		priv->chip->chip_pub->tuner->blindscan_mode = 1;
		avl62x1_optimize_carrier(priv->chip->chip_pub->tuner, NULL, priv->chip);
		c->frequency = frequency;
		c->symbol_rate = 0;
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 1);
		if (fe->ops.tuner_ops.set_params)
			fe->ops.tuner_ops.set_params(fe);
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 0);
		avl_bsp_delay(250);

		state->params.tuner_center_freq_100khz = frequency / 100;
		state->params.tuner_lpf_100khz = priv->chip->chip_pub->tuner->lpf_hz / 100000;
		state->params.min_symrate_khz = state->min_prop.symbol_rate / 1000;

		ret = avl62x1_blindscan_start(&state->params, priv->chip);
		if (ret != AVL_EC_OK)
			break;

		state->info.finished = 0;
		cntr = 0;
		do {
			if (kthread_should_stop()) {
				avl62x1_blindscan_cancel(priv->chip);
				break;
			}

			avl_bsp_delay(delay);
			ret = avl62x1_blindscan_get_status(&state->info, priv->chip);
			cntr++;
		} while (!state->info.finished && (cntr < timeout) && (ret == AVL_EC_OK));

		if (ret != AVL_EC_OK || cntr >= timeout) {
			frequency += (priv->chip->chip_pub->tuner->lpf_hz / 1000);
			continue;
		}

		if (state->info.num_carriers > 0) {
			ret = avl62x1_blindscan_get_carriers(fe);
			if (ret)
				break;
		}

		frequency += state->info.next_freq_step_hz / 1000;
		if (frequency > state->max_prop.frequency)
			frequency = state->max_prop.frequency;
	}

	for (i = 0; i < state->num_props; i++) {
		for (j = i + 1; j < state->num_props; j++) {
			struct dtv_frontend_properties *p = state->props + i;
			struct dtv_frontend_properties *n = state->props + j;

			if ((AVL_abs(p->frequency - n->frequency) <= 5000) &&
			    (AVL_abs(p->symbol_rate - n->symbol_rate) <= 500000) &&
			    (p->scrambling_sequence_index == n->scrambling_sequence_index) &&
			    (p->stream_id == n->stream_id) &&
			    (p->AVL62X1_T2MI_CTRL_PROP == n->AVL62X1_T2MI_CTRL_PROP)) {
				for (k = j; k < state->num_props; k++)
					*(state->props + k) = *(state->props + (k + 1));

				state->num_props--;

				j--;
			}
		}
	}

	/* TODO: workaround */
	avl_bms_write16(priv->chip->chip_pub->i2c_addr,
			c_AVL62X1_S2X_BWfilt_to_Rsamp_ratio_saddr, bw_ratio);
	avl_bms_write16(priv->chip->chip_pub->i2c_addr,
			c_AVL62X1_S2X_blind_scan_min_sym_rate_kHz_saddr, bs_min_sr);
	avl_bms_write32(priv->chip->chip_pub->i2c_addr,
			c_AVL62X1_S2X_bs_cent_freq_tuner_Hz_iaddr, 0);

	refcount_set(&state->running, 0);

	return 0;
}

static int avl62x1_blindscan_ctrl_show(struct seq_file *m, void *v)
{
	struct dvb_frontend *fe = m->private;
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;

	seq_printf(m, "%d %d %d \n",
		   refcount_read(&state->running),
		   state->num_props,
		   state->progress);

	return 0;
}

static int avl62x1_blindscan_ctrl_open(struct inode *inode, struct file *file)
{
	return single_open(file, avl62x1_blindscan_ctrl_show, PDE_DATA(inode));
}

static ssize_t avl62x1_blindscan_ctrl_write(struct file *file,
					    const char __user *buf,
					    size_t count, loff_t *ppos)
{
	struct dvb_frontend *fe = PDE_DATA(file_inode(file));
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;
	u32 val[5];
	char *p;

	p = memdup_user_nul(buf, count);
	if (IS_ERR(p))
		return PTR_ERR(p);

	if (sscanf(p, "%u%u%u%u%u", &val[0], &val[1], &val[2], &val[3], &val[4]) != 5) {
		kfree(p);
		return -EINVAL;
	}

	kfree(p);

	if (val[0]) {
		if (val[1] < 950 || val[2] > 2150 || val[3] < 1 || val[4] > 60)
			return -EINVAL;

		if (signal_pending(current))
			return -EINTR;

		mb();

		if (refcount_read(&state->running))
			return -EBUSY;

		refcount_set(&state->running, 1);

		state->min_prop.frequency = val[1] * 1000;
		state->max_prop.frequency = val[2] * 1000;
		state->min_prop.symbol_rate = val[3] * 1000000;
		state->max_prop.symbol_rate = val[4] * 1000000;
		state->progress = 0;
		state->index = 0;

		memset(state->props, 0, array_size(1000, sizeof(struct dtv_frontend_properties)));
		state->num_props = 0;

		state->thread = kthread_run(avl62x1_blindscan_thread, fe, "blindscand");
		if (IS_ERR(state->thread)) {
			refcount_set(&state->running, 0);
			return PTR_ERR(state->thread);
		}
	} else {
		if (refcount_read(&state->running))
			kthread_stop(state->thread);
	}

	return count;
}

static struct proc_ops avl62x1_blindscan_ctrl_fops = {
	.proc_open = avl62x1_blindscan_ctrl_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
	.proc_write = avl62x1_blindscan_ctrl_write,
};

static int avl62x1_blindscan_info_show(struct seq_file *m, void *v)
{
	struct dvb_frontend *fe = m->private;
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;
	struct dtv_frontend_properties *props = state->props + state->index;
	u32 t2mi_plp_id;
	u32 t2mi_pid;

	if (props->AVL62X1_T2MI_CTRL_PROP != NO_STREAM_ID_FILTER &&
	    props->AVL62X1_T2MI_CTRL_PROP & AVL62X1_T2MI_CTRL_VALID_STREAM_MASK) {
		t2mi_plp_id = props->AVL62X1_T2MI_CTRL_PROP & 0xff;
		t2mi_pid = (props->AVL62X1_T2MI_CTRL_PROP >> AVL62X1_T2MI_PID_SHIFT) & 0x1fff;
	} else {
		t2mi_plp_id = NO_STREAM_ID_FILTER;
		t2mi_pid = 0x1000;
	}

	seq_printf(m, "%d %u %u %d %d %d %d %d %d %d %d %d %d %d \n",
		   state->index,
		   props->frequency,
		   props->symbol_rate,
		   props->delivery_system,
		   INVERSION_AUTO,
		   props->pilot,
		   props->fec_inner,
		   props->modulation,
		   props->rolloff,
		   1,
		   props->stream_id,
		   props->scrambling_sequence_index,
		   t2mi_plp_id,
		   t2mi_pid);

	return 0;
}

static int avl62x1_blindscan_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, avl62x1_blindscan_info_show, PDE_DATA(inode));
}

static ssize_t avl62x1_blindscan_info_write(struct file *file,
					    const char __user *buf,
					    size_t count, loff_t *ppos)
{
	struct dvb_frontend *fe = PDE_DATA(file_inode(file));
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;
	int val;
	int ret;

	ret = kstrtoint_from_user(buf, count, 10, &val);
	if (ret)
		return ret;

	if (val < 0 || val >= state->num_props)
		return -EINVAL;

	state->index = val;

	return count;
}

static struct proc_ops avl62x1_blindscan_info_fops = {
	.proc_open = avl62x1_blindscan_info_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
	.proc_write = avl62x1_blindscan_info_write,
};

static int avl62x1_t2mi_show(struct seq_file *m, void *v)
{
	return 0;
}

int avl62x1_blindscan_attach(struct dvb_frontend *fe,
			     struct proc_dir_entry *root_dir)
{
	if (!proc_create_data("bs_ctrl", S_IRUGO | S_IWUSR, root_dir,
			      &avl62x1_blindscan_ctrl_fops, fe))
		return -ENOMEM;

	if (!proc_create_data("bs_info", S_IRUGO | S_IWUSR, root_dir,
			      &avl62x1_blindscan_info_fops, fe))
		return -ENOMEM;

	if (!proc_create_single_data("t2mi", S_IRUGO | S_IWUSR, root_dir,
				     avl62x1_t2mi_show, fe))
		return -ENOMEM;

	return 0;
}
EXPORT_SYMBOL(avl62x1_blindscan_attach);

//-------------- stdout character device -----------------------------------
#if INCLUDE_STDOUT
#include "read_stdout_62x1.c"
#define STDOUT_DEVICE_NAME "avl62x1_stdout"
#define STDOUT_CLASS_NAME  "avl62x1_stdout"
static DEFINE_MUTEX(stdout_dev_mutex);
static int stdout_maj_num;
static struct class*  stdout_class  = NULL;
static struct device* stdout_device = NULL;
static int     stdout_dev_open(struct inode *, struct file *);
static int     stdout_dev_release(struct inode *, struct file *);
static ssize_t stdout_dev_read(struct file *, char *, size_t, loff_t *);
static struct file_operations stdout_fops =
    {
	.owner = THIS_MODULE,
	.open = stdout_dev_open,
	.read = stdout_dev_read,
	.release = stdout_dev_release};
int init_avl62x1_stdout_device(struct avl62x1_priv *priv)
{
	stdout_maj_num = register_chrdev(0, STDOUT_DEVICE_NAME, &stdout_fops);
	if (stdout_maj_num < 0)
	{
		p_error("failed to register a major number");
		return stdout_maj_num;
	}

	// Register the device class
	stdout_class = class_create(THIS_MODULE, STDOUT_CLASS_NAME);
	if (IS_ERR(stdout_class))
	{ // Check for error and clean up if there is
		unregister_chrdev(stdout_maj_num, STDOUT_DEVICE_NAME);
		p_error("Failed to register device class");
		return PTR_ERR(stdout_class); // Correct way to return an error on a pointer
	}

	// Register the device driver
	stdout_device = device_create(stdout_class, NULL, MKDEV(stdout_maj_num, 0), NULL, STDOUT_DEVICE_NAME);
	if (IS_ERR(stdout_device))
	{				     // Clean up if there is an error
		class_destroy(stdout_class); // Repeated code but the alternative is goto statements
		unregister_chrdev(stdout_maj_num, STDOUT_DEVICE_NAME);
		p_error("Failed to create the device");
		return PTR_ERR(stdout_device);
	}

	mutex_init(&stdout_dev_mutex);
	return 0;
}

void deinit_avl62x1_stdout_device(struct avl62x1_priv *priv)
{
	device_destroy(stdout_class, MKDEV(stdout_maj_num, 0)); // remove the device
	class_unregister(stdout_class);			     // unregister the device class
	class_destroy(stdout_class);			     // remove the device class
	unregister_chrdev(stdout_maj_num, STDOUT_DEVICE_NAME);	     // unregister the major number

	mutex_destroy(&stdout_dev_mutex);
}

static int stdout_dev_open(struct inode *inodep, struct file *filep)
{
	p_debug_lvl(4,"");
	if (!mutex_trylock(&stdout_dev_mutex))
	{	/// Try to acquire the mutex (i.e., put the lock on/down)
		/// returns 1 if successful and 0 if there is contention
		p_error("Device in use by another process");
		return -EBUSY;
	}
	return 0;
}

static ssize_t stdout_dev_read(
    struct file *filep,
    char *buffer,   /* The buffer to fill with data */
    size_t len,	    /* The length of the buffer     */
    loff_t *offset) /* Our offset in the file       */
{

	char *stdout_buf;
	char *sp_stdout_buf;
	int ttl;
	stdout_buf = read_stdout(&global_priv->frontend);
	sp_stdout_buf = read_sp_stdout(&global_priv->frontend);
	ttl = simple_read_from_buffer(buffer,len,offset,stdout_buf,strlen(stdout_buf));
	ttl += simple_read_from_buffer(&buffer[strlen(stdout_buf)],len,offset,sp_stdout_buf,strlen(sp_stdout_buf));
	return ttl;
}

static int stdout_dev_release(struct inode *inodep, struct file *filep)
{
	p_debug_lvl(4,"");
	safe_mutex_unlock(&stdout_dev_mutex);
	return 0;
}
#endif
//--------------------------------------------------------------------------

static int diseqc_set_voltage(struct dvb_frontend *fe,
			      enum fe_sec_voltage voltage);
int init_error_stat(struct avl62x1_priv *priv)
{
	uint16_t r = AVL_EC_OK;
	struct avl62x1_error_stats_config err_stats_config;
	struct avl62x1_ber_config ber_config;

	err_stats_config.error_stats_mode = avl62x1_error_stats_auto;
	err_stats_config.auto_error_stats_type = avl62x1_error_stats_time;
	err_stats_config.time_threshold_ms = 3000;
	err_stats_config.bytes_threshold = 0;

	r = avl62x1_config_error_stats(&err_stats_config, priv->chip);

	ber_config.test_pattern = avl62x1_test_lfsr_23;
	ber_config.fb_inversion = avl62x1_lfsr_fb_inverted;
	ber_config.lfsr_sync = 0;
	ber_config.lfsr_start_pos = 4;
	r |= avl62x1_reset_ber(&ber_config, priv->chip);

	return r;
}

static int avl68x2_init_diseqc(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_diseqc_params params;
	uint16_t r = AVL_EC_OK;

	params.tone_freq_khz = 22;
	params.tx_gap = avl62x1_dtxg_15ms;
	params.tx_waveform = avl62x1_dwm_normal;
	params.rx_timeout = avl62x1_drt_150ms;
	params.rx_waveform = avl62x1_dwm_normal;

	r |= avl62x1_init_diseqc(&params, priv->chip);
	if (AVL_EC_OK != r)
	{
		p_debug("Diseqc Init failed !\n");
	}

	diseqc_set_voltage(fe, SEC_VOLTAGE_OFF);

	return r;
}

//called from dvb_frontend::dvb_frontend_init
// and from ::set_frontend
static int i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	uint16_t ret = AVL_EC_OK;

	p_debug("%d", enable);

	mutex_lock(&i2cctl_fe_mutex);


	if (enable)
	{
		mutex_lock(&i2cctl_tuneri2c_mutex);
		ret = avl62x1_enable_tuner_i2c(priv->chip);
	}
	else
	{
		ret = avl62x1_disable_tuner_i2c(priv->chip);
		safe_mutex_unlock(&i2cctl_tuneri2c_mutex);
	}

	safe_mutex_unlock(&i2cctl_fe_mutex);
	
	return ret;
}

static int acquire_dvbs_s2(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	uint16_t r = AVL_EC_OK;
	struct avl62x1_carrier_info carrier_info;
	struct avl62x1_stream_info stream_info;
	
	p_debug("Freq:%d khz,sym:%d hz", c->frequency, c->symbol_rate);

	carrier_info.rf_freq_khz = c->frequency;
	carrier_info.carrier_freq_offset_hz = 0;
	carrier_info.symbol_rate_hz = c->symbol_rate;
	//"-1" means ISI not specified and is very unlikely to be a valid
	//  decoding of the other t2mi fields
	if ((c->stream_id != -1) &&
	    ((c->stream_id >> AVL62X1_BS_IS_T2MI_SHIFT) & 0x1))
	{
		stream_info.stream_type = avl62x1_t2mi;
		stream_info.t2mi.pid =
		    (c->stream_id >> AVL62X1_BS_T2MI_PID_SHIFT) & 0x1FFF;
		stream_info.t2mi.plp_id =
		    (c->stream_id >> AVL62X1_BS_T2MI_PLP_ID_SHIFT) & 0xFF;
		stream_info.t2mi.raw_mode = 0;
		stream_info.isi = c->stream_id & 0xFF;

		carrier_info.pl_scrambling = AVL62X1_PL_SCRAM_AUTO;
		printk("Acquire T2MI\n");
	}
	else
	{
		stream_info.stream_type = avl62x1_transport;
		stream_info.isi = (c->stream_id == -1) ? 0 : (c->stream_id & 0xFF);
#if DVB_VER_ATLEAST(5, 11)
		//use scrambling_sequence_index if it's not the default n=0 val
		if (c->scrambling_sequence_index)
		{
			carrier_info.pl_scrambling =
			    c->scrambling_sequence_index;
		}
		else
		{
			carrier_info.pl_scrambling = AVL62X1_PL_SCRAM_AUTO;
		}

#else
		carrier_info.pl_scrambling = AVL62X1_PL_SCRAM_AUTO;
#endif
		printk("Acquire TS\n");
	}

	r = avl62x1_lock_tp(&carrier_info,
			    &stream_info,
			    AVL_FALSE, /* don't do blind symbol rate */
			    priv->chip);

	return r;
}

static int set_dvb_mode(struct dvb_frontend *fe,
			enum fe_delivery_system delsys,
			bool force_reload)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	uint16_t ret = AVL_EC_OK;
	uint16_t r = AVL_EC_OK;
	struct avl62x1_ver_info ver_info;

	/* already in desired mode */
	if (!force_reload && (priv->delivery_system == delsys))
		return 0;

	p_debug("initing demod for delsys=%d", delsys);

	switch (priv->delivery_system)
	{
	case SYS_DVBS:
		if (delsys == SYS_DVBS2)
			return 0;
		break;
	case SYS_DVBS2:
		if (delsys == SYS_DVBS)
			return 0;
		break;
	default:
		break;
	}
	priv->delivery_system = delsys;

	//Reset Demod
	r = avl_bsp_reset();
	if (AVL_EC_OK != r)
	{
		p_debug("Failed to Resed demod via BSP!\n");
		return r;
	}

	// boot the firmware here
	r |= avl62x1_initialize(priv->chip);
	if (AVL_EC_OK != r)
	{
		p_debug("AVL_AVL62X1_Initialize failed !\n");
		return (r);
	}

	r |= avl62x1_get_version(&ver_info, priv->chip);
	if (AVL_EC_OK != r)
	{
		p_debug("avl62x1_get_version failed\n");
		return (r);
	}
	p_debug("FW version %d.%d.%d\n",
		ver_info.firmware.major,
		ver_info.firmware.minor,
		ver_info.firmware.build);
	p_debug("Driver version %d.%d.%d\n",
		ver_info.driver.major,
		ver_info.driver.minor,
		ver_info.driver.build);

	switch (priv->delivery_system)
	{
	case SYS_DVBS:
	case SYS_DVBS2:
	default:
		ret |= avl68x2_init_diseqc(fe);
		break;
	}

	ret |= init_error_stat(priv);

	if (ret)
	{
		p_error("demod init failed");
	}

	return ret;
}

uint16_t diseqc_send_cmd(struct avl62x1_priv *priv,
			 uint8_t *cmd,
			 uint8_t cmdsize)
{
	uint16_t r = AVL_EC_OK;
	struct avl62x1_diseqc_tx_status tx_status;

	p_debug(" %*ph", cmdsize, cmd);

	mutex_lock(&i2cctl_fe_mutex);
	
	r = avl62x1_send_diseqc_data(cmd, cmdsize, priv->chip);
	if (r != AVL_EC_OK)
	{
		printk("diseqc_send_cmd failed !\n");
	}
	else
	{
		do
		{
			msleep(5);
			r |= avl62x1_get_diseqc_tx_status(&tx_status,
							  priv->chip);
		} while (tx_status.tx_complete != 1);
		if (r == AVL_EC_OK)
		{
		}
		else
		{
			printk("diseqc_send_cmd Err. !\n");
		}
	}

	safe_mutex_unlock(&i2cctl_fe_mutex);

	return (int)(r);
}

static int diseqc_send_master_cmd(struct dvb_frontend *fe,
				  struct dvb_diseqc_master_cmd *cmd)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;

	return diseqc_send_cmd(priv, cmd->msg, cmd->msg_len);
}

static int diseqc_send_burst(struct dvb_frontend *fe,
			     enum fe_sec_mini_cmd burst)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	int ret;
	uint8_t tone = burst == SEC_MINI_A ? 1 : 0;
	uint8_t count = 1;

	p_debug("");

	mutex_lock(&i2cctl_fe_mutex);

	ret = (int)avl62x1_send_diseqc_tone(tone, count, priv->chip);

	safe_mutex_unlock(&i2cctl_fe_mutex);

	return ret;
}

static int diseqc_set_tone(struct dvb_frontend *fe, enum fe_sec_tone_mode tone)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	int r = AVL_EC_OK;

	p_debug("tone: %s", tone==SEC_TONE_ON ? "ON" : "OFF");

	mutex_trylock(&i2cctl_fe_mutex);

	switch (tone)
	{
	case SEC_TONE_ON:
		if (priv->chip->chip_priv->diseqc_op_status !=
		    avl62x1_dos_continuous)
		{
			p_debug("call avl62x1_diseqc_tone_on()");
			r = (int)avl62x1_diseqc_tone_on(priv->chip);
		}
		break;
	case SEC_TONE_OFF:
		if (priv->chip->chip_priv->diseqc_op_status ==
		    avl62x1_dos_continuous)
		{
			p_debug("call avl62x1_diseqc_tone_off()");
			r = (int)avl62x1_diseqc_tone_off(priv->chip);
		}
		break;
	default:
		r = -EINVAL;
	}

	if(r != AVL_EC_OK) {
		p_debug("diseqc_set_tone() FAILURE!");
	}

	safe_mutex_unlock(&i2cctl_fe_mutex);

	return r;
}

static int diseqc_set_voltage(struct dvb_frontend *fe,
			      enum fe_sec_voltage voltage)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	avl62x1_gpio_pin_value enable, sel;
	int ret;

	p_debug("volt: %d", voltage);

	mutex_lock(&i2cctl_fe_mutex);

	switch (voltage)
	{
	case SEC_VOLTAGE_OFF:
		enable = avl62x1_gpio_value_logic_0;
		sel = avl62x1_gpio_value_logic_0;
		break;
	case SEC_VOLTAGE_13:
		//power on
		enable = avl62x1_gpio_value_logic_1;
		sel = avl62x1_gpio_value_logic_0;
		break;
	case SEC_VOLTAGE_18:
		//power on
		enable = avl62x1_gpio_value_logic_1;
		sel = avl62x1_gpio_value_high_z;
		break;
	default:
		safe_mutex_unlock(&i2cctl_fe_mutex);
		return -EINVAL;
	}

	p_debug("lnb_pwr_en %d, lnb_pwr_sel %d",enable,sel);

	ret = (int)avl62x1_set_gpio_value(avl62x1_gpio_pin_lnb_pwr_en,
					  enable,
					  priv->chip);
	ret |= (int)avl62x1_set_gpio_value(avl62x1_gpio_pin_lnb_pwr_sel,
					   sel,
					   priv->chip);

	if(ret != AVL_EC_OK) {
		p_debug("diseqc_set_voltage() FAILURE!");
	}

	safe_mutex_unlock(&i2cctl_fe_mutex);

	return ret;
}

static int update_fe_props(
    struct dtv_frontend_properties *props,
    struct avl62x1_carrier_info *carrier_info,
    struct avl62x1_stream_info *stream_info)
{
	uint16_t r = AVL_EC_OK;
	props->frequency = carrier_info->rf_freq_khz +
			   carrier_info->carrier_freq_offset_hz/1000;

	props->inversion =
	    (carrier_info->spectrum_invert == avl62x1_specpol_inverted)
		? INVERSION_ON
		: INVERSION_OFF;

	props->symbol_rate = carrier_info->symbol_rate_hz;

	switch (carrier_info->roll_off)
	{
	case avl62x1_rolloff_35:
		props->rolloff = ROLLOFF_35;
		break;
	case avl62x1_rolloff_25:
		props->rolloff = ROLLOFF_25;
		break;
	case avl62x1_rolloff_20:
		props->rolloff = ROLLOFF_20;
		break;
#ifdef AVL_S2X_ENUMS
	case avl62x1_rolloff_15:
		props->rolloff = ROLLOFF_15;
		break;
	case avl62x1_rolloff_10:
		props->rolloff = ROLLOFF_10;
		break;
	case avl62x1_rolloff_05:
		props->rolloff = ROLLOFF_5;
		break;
#endif
	default:
		props->rolloff = ROLLOFF_20;
	}

	if (carrier_info->signal_type == avl62x1_dvbs2)
	{
		props->delivery_system = SYS_DVBS2;
		if(carrier_info->dvbs2_ccm_acm == avl62x1_dvbs2_ccm)
		{
#ifdef AVL_S2X_ENUMS
			if(carrier_info->pls_acm >= 128)
			{
				int idx = (carrier_info->pls_acm-128)/2;
				idx = (idx >= ARRAY_LENGTH(s2x_modcod)) ? 0 : idx;
				props->modulation = s2x_modcod[idx][0];
				props->fec_inner = s2x_modcod[idx][1];
			}
			else
#endif
			{
				int idx = carrier_info->pls_acm / 4;
				idx = (idx >= ARRAY_LENGTH(modcod)) ? 0 : idx;
				props->modulation = modcod[idx][0];
				props->fec_inner = modcod[idx][1];
			}

			props->pilot = (carrier_info->pls_acm & 1) ? PILOT_ON : PILOT_OFF;
		}
		else
		{
			props->modulation = QAM_AUTO;
			props->fec_inner = FEC_AUTO;
			props->pilot = PILOT_ON;
		}
	}
	else
	{
		props->delivery_system = SYS_DVBS;
		switch (carrier_info->code_rate.dvbs_code_rate)
		{
		case avl62x1_dvbs_cr_1_2:
			props->fec_inner = FEC_1_2;
			break;
		case avl62x1_dvbs_cr_2_3:
			props->fec_inner = FEC_2_3;
			break;
		case avl62x1_dvbs_cr_3_4:
			props->fec_inner = FEC_3_4;
			break;
		case avl62x1_dvbs_cr_5_6:
			props->fec_inner = FEC_5_6;
			break;
		default:
			props->fec_inner = FEC_7_8;
		}
	}

	props->stream_id = stream_info->isi;

	return r;
}

static int get_frontend(struct dvb_frontend *fe,
			struct dtv_frontend_properties *props)
{
	int ret = 0;
	struct avl62x1_priv *priv = fe->demodulator_priv;
	avl62x1_lock_status lock;
	struct avl62x1_carrier_info carrier_info;
	struct avl62x1_stream_info stream_info;
	int16_t snr_db_x100;
	uint16_t sig_strength;
	p_debug_lvl(10,"");

	mutex_lock(&i2cctl_fe_mutex);

	ret = avl62x1_get_lock_status(&lock,
				      priv->chip);

	if ((lock == avl62x1_status_locked) &&
	    (ret == AVL_EC_OK))
	{

		ret |= avl62x1_get_signal_info(&carrier_info,
					priv->chip);
		
		
		ret |= avl62x1_get_stream_info(&stream_info,
					priv->chip);

		ret |= update_fe_props(props, &carrier_info, &stream_info);

		/*  STATS  */
		//SNR
		ret |= avl62x1_get_snr(&snr_db_x100,
				       priv->chip);
		props->cnr.len = 2;
		props->cnr.stat[0].scale = FE_SCALE_DECIBEL; //0.001dB
		props->cnr.stat[0].svalue = snr_db_x100 * 10;
		props->cnr.stat[1].scale = FE_SCALE_RELATIVE;
		//props->cnr.stat[1].uvalue = ((snr_db_x100 + 300) / 10) * 250;
		//max SNR is about 28dB, min is about -3
		props->cnr.stat[1].uvalue = ((snr_db_x100+300) * 0xffff) / (31*100);
		if (props->cnr.stat[1].uvalue > 0xffff) {
			props->cnr.stat[1].uvalue = 0xffff;
		}

		//RF strength
		ret |= avl62x1_get_signal_strength(&sig_strength,
						   priv->chip);

		props->strength.len = 2;
		props->strength.stat[0].scale = FE_SCALE_DECIBEL;
		props->strength.stat[0].svalue = -80 + sig_strength / 2;
		props->strength.stat[1].scale = FE_SCALE_RELATIVE;
		props->strength.stat[1].uvalue = (sig_strength * 0xffff) / 100;
		if(props->strength.stat[1].uvalue > 0xffff) {
			props->strength.stat[1].uvalue = 0xffff;
		}

		//DVB-S pre/post viterbi
		props->pre_bit_error.len = 0;
		props->pre_bit_count.len = 0;
		props->post_bit_error.len = 0;
		props->post_bit_count.len = 0;
		
		//TODO: post outer FEC block errors
		props->block_error.len = 1;
		props->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		props->block_error.stat[0].uvalue = 0;

		props->block_count.len = 1;
		props->block_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		props->block_count.stat[0].uvalue = 0;
	} else {
		//not locked
		props->cnr.len = 1;
		props->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

		props->strength.len = 1;
		props->strength.stat[1].scale = FE_SCALE_NOT_AVAILABLE;

		props->block_error.len = 1;
		props->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

		props->block_count.len = 1;
		props->block_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	}

	safe_mutex_unlock(&i2cctl_fe_mutex);

	return ret;
}

static int read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	int r = AVL_EC_OK;
	struct avl62x1_priv *priv = fe->demodulator_priv;
	avl62x1_lock_status lock;
	p_debug_lvl(10, "");

	if (0) /* blindscan status handled via proc interface */
	{
		if (0)
			*status = FE_HAS_LOCK;
		else
			*status = FE_NONE;
		return AVL_EC_OK;
	}

	mutex_lock(&i2cctl_fe_mutex);

	r = avl62x1_get_lock_status(&lock, priv->chip);

	safe_mutex_unlock(&i2cctl_fe_mutex);

	if (!r && lock == avl62x1_status_locked)
	{
		*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
			  FE_HAS_VITERBI | FE_HAS_SYNC |
			  FE_HAS_LOCK;
		r = get_frontend(fe,
				 &fe->dtv_property_cache);
	}
	else
	{
		*status = FE_HAS_SIGNAL;
	}

	return r;
}

static int read_signal_strength(struct dvb_frontend *fe, uint16_t *strength)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int i;

	//p_debug("ENTER");

	*strength = 0;
	for (i = 0; i < c->strength.len; i++)
		if (c->strength.stat[i].scale == FE_SCALE_RELATIVE)
			*strength = (uint16_t)c->strength.stat[i].uvalue;

	return 0;
}

static int read_snr(struct dvb_frontend *fe, uint16_t *snr)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int i;

	//p_debug("ENTER");

	*snr = 0;
	for (i = 0; i < c->cnr.len; i++)
		if (c->cnr.stat[i].scale == FE_SCALE_RELATIVE)
			*snr = (uint16_t)c->cnr.stat[i].uvalue;

	return 0;
}
static int read_ber(struct dvb_frontend *fe, uint32_t *ber)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	int ret;

	p_debug_lvl(10,"");

	mutex_lock(&i2cctl_fe_mutex);
	
	//FIXME
	*ber = 10e7;
	ret = (int)avl62x1_get_per(ber, priv->chip);
	if (!ret)
		*ber /= 100;

	safe_mutex_unlock(&i2cctl_fe_mutex);

	return ret;
}

static enum dvbfe_algo get_frontend_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_HW;
}





static int set_frontend(struct dvb_frontend *fe)
{
	int ret;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	p_debug("");

	/* tune tuner if necessary*/
	if (fe->ops.tuner_ops.set_params)
	{
		
		ret = fe->ops.i2c_gate_ctrl(fe, 1);
		if (ret)
		{
			p_debug("I2C gate control FAILED\n");
			return ret;
		}

		if(fe->ops.tuner_ops.set_params)
		{
			p_debug("calling tuner_ops.set_params()\n");
			mutex_lock(&i2cctl_fe_mutex);
			ret = fe->ops.tuner_ops.set_params(fe);
			safe_mutex_unlock(&i2cctl_fe_mutex);
			if (ret)
			{
				p_debug("Tuning FAILED\n");
				return ret;
			}
			else
			{
				p_debug("Tuned to %d kHz", c->frequency);
			}
		}

		ret = fe->ops.i2c_gate_ctrl(fe, 0);
		if (ret)
		{
			p_debug("I2C gate control FAILED\n");
			return ret;
		}
		
	}

	mutex_lock(&i2cctl_fe_mutex);
	
	p_debug("ACQUIRE");
	ret = acquire_dvbs_s2(fe);

	safe_mutex_unlock(&i2cctl_fe_mutex);

	return ret;
}

static int tune(struct dvb_frontend *fe,
		bool re_tune,
		unsigned int mode_flags,
		unsigned int *delay,
		enum fe_status *status)
{
	*delay = HZ / 5;
	if (re_tune)
	{
		int ret = set_frontend(fe);
		if (ret)
			return ret;
	}
	return read_status(fe, status);
}

static int init_fe(struct dvb_frontend *fe)
{
	p_debug("");

	//TODO: call tuner resume, then init
	return 0;
}

static int sleep_fe(struct dvb_frontend *fe)
{
	p_debug("");

	//TODO: call tuner suspend, then sleep

	return 0;
}

static void release_fe(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;

	p_debug("");
	if (refcount_read(&state->running))
		kthread_stop(state->thread);
	vfree(state->props);
#if INCLUDE_STDOUT
	deinit_avl62x1_stdout_device(priv);
#endif
	kfree(priv->chip->chip_pub);
	kfree(priv->chip->chip_priv);
	kfree(priv->chip);
	kfree(priv);
}

static struct dvb_frontend_ops avl62x1_ops = {
	.info = {
		.name = "Availink AVL6261",
		.frequency_min_hz =  950 * MHz,
		.frequency_max_hz = 2150 * MHz,
		.frequency_stepsize_hz =  1011 * kHz,
		.frequency_tolerance_hz = 5 * MHz,
		.symbol_rate_min = 0,
		.symbol_rate_max = 60000000,
		.caps = FE_CAN_INVERSION_AUTO |
			FE_CAN_FEC_1_2 |
			FE_CAN_FEC_2_3 |
			FE_CAN_FEC_3_4 |
			FE_CAN_FEC_4_5 |
			FE_CAN_FEC_5_6 |
			FE_CAN_FEC_6_7 |
			FE_CAN_FEC_7_8 |
			FE_CAN_FEC_8_9 |
			FE_CAN_FEC_AUTO |
			FE_CAN_QPSK |
			FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO |
			FE_CAN_HIERARCHY_AUTO |
			FE_CAN_MULTISTREAM |
			FE_CAN_2G_MODULATION |
			FE_CAN_RECOVER
	},

	.delsys = { SYS_DVBS2, SYS_DVBS, },

	.release = release_fe,
	.init = init_fe,
	.sleep = sleep_fe,
	.tune = tune,
	.get_frontend_algo = get_frontend_algo,
	.get_frontend = get_frontend,

	.read_status = read_status,
	.read_ber = read_ber,
	.read_signal_strength = read_signal_strength,
	.read_snr = read_snr,

	.diseqc_send_master_cmd = diseqc_send_master_cmd,
	.diseqc_send_burst = diseqc_send_burst,
	.set_tone = diseqc_set_tone,
	.set_voltage = diseqc_set_voltage,

	.i2c_gate_ctrl = i2c_gate_ctrl,
};

static struct avl_tuner av201x_avl_tuner = {
	.blindscan_mode = 0,
	.more_params = NULL,
	.initialize = NULL,
	.lock = NULL,
	.get_lock_status = NULL,
	.get_rf_strength = NULL,
	.get_max_lpf = &av201x_get_max_lpf,
	.get_min_lpf = &av201x_get_min_lpf,
	.get_lpf_step_size = &av201x_get_lpf_step_size,
	.get_agc_slope = &av201x_get_agc_slope,
	.get_min_gain_voltage = &av201x_get_min_gain_voltage,
	.get_max_gain_voltage = &av201x_get_max_gain_voltage,
	.get_rf_freq_step_size = &av201x_get_rf_freq_step_size,
};

static struct av201x_config av201x_avl_config = {
	.i2c_address = 0x62,
	.id = ID_AV2018,
	.xtal_freq = 27000,
};

static int avl62x1_get_firmware(struct dvb_frontend *fe)
{
	unsigned int fw_maj, fw_min, fw_build;
	int fw_status;
	struct avl62x1_priv *priv = fe->demodulator_priv;

	fw_status = request_firmware(&priv->fw,
				     sel_fw,
				     priv->i2c->dev.parent);
	if (fw_status != 0)
	{
		p_error("firmware file %s not found",sel_fw);
		return fw_status;
	}

	priv->chip->chip_priv->patch_data = (unsigned char *)(priv->fw->data);
	fw_maj = priv->chip->chip_priv->patch_data[24]; //major rev
	fw_min = priv->chip->chip_priv->patch_data[25]; //SDK-FW API rev
	fw_build = (priv->chip->chip_priv->patch_data[26] << 8) |
			priv->chip->chip_priv->patch_data[27]; //internal rev
	if(fw_min != AVL62X1_VER_MINOR)
	{
		//SDK-FW API rev must match
		p_error("Firmware version %d.%d.%d incompatible with this driver version",
			fw_maj, fw_min, fw_build);
		p_error("Firmware minor version must be %d",
			AVL62X1_VER_MINOR);
		release_firmware(priv->fw);
		return 1;
	}
	else
	{
		p_info("Firmware version %d.%d.%d found",
				fw_maj, fw_min, fw_build);
	}

	return 0;
}

struct dvb_frontend *avl62x1_attach(struct avl62x1_config *config,
				    struct i2c_adapter *i2c)
{
	struct avl62x1_priv *priv;
	uint16_t ret;
	uint32_t id;

	p_info("driver version %s\n", AVL62X1_VERSION);

	p_debug("enter %s()", __FUNCTION__);

	priv = kzalloc(sizeof(struct avl62x1_priv), GFP_KERNEL);
	if (priv == NULL)
		goto err;
	global_priv = priv;

	priv->chip = kzalloc(sizeof(struct avl62x1_chip), GFP_KERNEL);
	if (priv->chip == NULL)
		goto err1;

	priv->chip->chip_priv = kzalloc(sizeof(struct avl62x1_chip_priv),
					GFP_KERNEL);
	if (priv->chip->chip_priv == NULL)
		goto err2;

	priv->chip->chip_pub = kzalloc(sizeof(struct avl62x1_chip_pub),
				       GFP_KERNEL);
	if (priv->chip->chip_pub == NULL)
		goto err3;

	memcpy(&priv->frontend.ops, &avl62x1_ops,
	       sizeof(struct dvb_frontend_ops));

	priv->frontend.demodulator_priv = priv;
	priv->i2c = i2c;
	priv->delivery_system = SYS_UNDEFINED;

	/* copy (ephemeral?) public part of chip config into alloc'd area */
	memcpy(priv->chip->chip_pub,
	       config->chip_pub,
	       sizeof(struct avl62x1_chip_pub));

	priv->chip->chip_pub->tuner = &av201x_avl_tuner;

	/* Initialize blindscan state */
	refcount_set(&priv->bs_state.running, 0);
	priv->bs_state.props = vmalloc(array_size(1000,
		sizeof(struct dtv_frontend_properties)));
	if (!priv->bs_state.props)
		goto err;

	if (!av201x_attach(&priv->frontend, &av201x_avl_config, i2c)) {
		p_error("av201x_attach failed");
		vfree(priv->bs_state.props);
		goto err;
	}

	p_debug("Demod ID %d, I2C addr 0x%x",
		(priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) &
		    AVL_DEMOD_ID_MASK,
		priv->chip->chip_pub->i2c_addr & 0xFF);

	// associate demod ID with i2c_adapter
	avl_bsp_assoc_i2c_adapter(priv->chip->chip_pub->i2c_addr, i2c);

	//set up semaphores
	ret = avl62x1_init_chip_object(priv->chip);
	if (ret)
	{
		p_error("chip object init failed");
		goto err4;
	}

	/* get chip id */
	ret = avl62x1_get_chip_id(priv->chip->chip_pub->i2c_addr, &id);
	if (ret)
	{
		p_error("attach failed reading id");
		goto err4;
	}

	p_debug("chip_id 0x%x\n", id);

	if (id != AVL62X1_CHIP_ID)
	{
		p_error("attach failed, id mismatch");
		goto err4;
	}

	p_info("found AVL62x1 id=0x%x", id);

	if(!avl62x1_get_firmware(&priv->frontend)) {
		if(!set_dvb_mode(&priv->frontend, SYS_DVBS2, false))
		{
			p_info("Firmware booted");
			release_firmware(priv->fw);
#if INCLUDE_STDOUT
			init_avl62x1_stdout_device(priv);
#endif
			
			return &priv->frontend;
		}
	}

err4:
	kfree(priv->chip->chip_pub);
err3:
	kfree(priv->chip->chip_priv);
err2:
	kfree(priv->chip);
err1:
	kfree(priv);
err:
	return NULL;
} /* end avl62x1_attach() */
EXPORT_SYMBOL_GPL(avl62x1_attach);


static int __init mod_init(void) {
	p_debug("");

	global_priv = NULL;

	if(strlen(sel_fw) == 0) {
		strlcpy(sel_fw, AVL62X1_FIRMWARE, sizeof(sel_fw));
	}

	return 0;
}
module_init(mod_init);

static void __exit mod_exit(void) {
}
module_exit(mod_exit);

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, " 1: enable debug messages");



static int fw_path_set(const char *val, const struct kernel_param *kp)
{
	char *clean_val = strim((char *)val);
	strlcpy(sel_fw, clean_val, sizeof(sel_fw));
	strlcpy(fw_path, clean_val, sizeof(fw_path));

	if (global_priv != NULL)
	{
		if (!avl62x1_get_firmware(&global_priv->frontend))
		{
			if (!set_dvb_mode(&global_priv->frontend, SYS_DVBS2, true))
			{
				p_info("New firmware %s booted",sel_fw);
				release_firmware(global_priv->fw);
			}
		}
	}

	return 0;
}
static int fw_path_get(char *buffer, const struct kernel_param *kp)
{
	sprintf(buffer, "%s",sel_fw);
	return strlen(buffer);
}
static const struct kernel_param_ops fw_path_ops = {
	.set	= fw_path_set,
	.get	= fw_path_get
};
module_param_cb(fw_path, &fw_path_ops, fw_path, 0644);
MODULE_PARM_DESC(fw_path, "<path>");


MODULE_DESCRIPTION("Availink AVL62X1 DVB-S/S2/S2X demodulator driver");
MODULE_AUTHOR("Availink, Inc. (gpl@availink.com)");
MODULE_LICENSE("GPL");
MODULE_VERSION(AVL62X1_VERSION);
MODULE_FIRMWARE(AVL62X1_FIRMWARE);

