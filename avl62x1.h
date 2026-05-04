// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL62X1 DVB-S/S2/S2X demodulator driver
 * Supports AVL6221 and AVL6261. NOT AVL6211
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#ifndef _AVL62X1_H_
#define _AVL62X1_H_

#include <linux/version.h>
#include <linux/refcount.h>
#include <linux/firmware.h>
#include <linux/dvb/frontend.h>
#include <linux/dvb/version.h>
#include <media/dvb_frontend.h>

#include "avl62x1_lib.h"

#define str(a) #a
#define xstr(a) str(a)

#define ARRAY_LENGTH(x) (sizeof(x) / sizeof((x)[0]))

#define DVB_VER_INT(maj,min) (((maj) << 16) + (min))
#define DVB_VER_ATLEAST(maj, min) \
 (DVB_VER_INT(DVB_API_VERSION,  DVB_API_VERSION_MINOR) >= DVB_VER_INT(maj, min))

#define AVL62X1_FIRMWARE	"availink/dvb-fe-avl62x1.fw"

#define AVL62X1_VERSION xstr(AVL62X1_VER_MAJOR) "." xstr(AVL62X1_VER_MINOR) "." xstr(AVL62X1_VER_BUILD)

#ifndef AVL_S2X_ENUMS
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6,2,0)) //because nobody bothered to rev DVB_API_VERSION_MINOR :shrug: 
#define AVL_S2X_ENUMS
#endif
#endif

#define AVL62X1_BS_CTRL_PROP			isdbt_sb_segment_idx
//isdbt_sb_segment_idx fields
#define AVL62X1_BS_CTRL_VALID_STREAM_MASK	(0x80000000)
/* Aliases for compatibility with Edision blindscan implementation */
#define AVL62X1_T2MI_CTRL_PROP			AVL62X1_BS_CTRL_PROP
#define AVL62X1_T2MI_CTRL_VALID_STREAM_MASK	AVL62X1_BS_CTRL_VALID_STREAM_MASK
#define AVL62X1_T2MI_PID_SHIFT			AVL62X1_BS_T2MI_PID_SHIFT
#define AVL62X1_BS_CTRL_NEW_TUNE_MASK		(0x40000000)
#define AVL62X1_BS_CTRL_MORE_RESULTS_MASK	(0x20000000)

//stream_id fields
#define AVL62X1_BS_IS_T2MI_SHIFT	29
#define AVL62X1_BS_T2MI_PID_SHIFT	16
#define AVL62X1_BS_T2MI_PLP_ID_SHIFT	8

struct i2cctl_ioctl_lock_req {
	int demod;
	int tuner;
};

struct avl62x1_bs_state {
	refcount_t running;
	struct task_struct *thread;
	struct dtv_frontend_properties min_prop;
	struct dtv_frontend_properties max_prop;
	int progress;
	int index;
	struct dtv_frontend_properties *props;
	int num_props;
	struct avl62x1_blind_scan_params params;
	struct avl62x1_blind_scan_info info;
};


struct avl62x1_priv
{
	struct i2c_adapter *i2c;
	struct dvb_frontend frontend;
	enum fe_delivery_system delivery_system;
	struct avl62x1_chip *chip;
	const struct firmware *fw;
	struct avl62x1_bs_state bs_state;
};

struct avl62x1_config
{
	//int i2c_id;	    // i2c adapter (master) id
	//void *i2c_adapter;     // i2c adapter (master)

	//structure of user-configurable parameters
	struct avl62x1_chip_pub *chip_pub;

	//uint8_t demod_address; // demodulator i2c address


};

extern struct dvb_frontend *avl62x1_attach(struct avl62x1_config *config,
					   struct i2c_adapter *i2c);

extern int avl62x1_blindscan_attach(struct dvb_frontend *fe,
				    struct proc_dir_entry *root_dir);

#endif /* _AVL62X1_H_ */
