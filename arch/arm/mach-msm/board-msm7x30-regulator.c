/*
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "board-msm7x30-regulator.h"

#define PCOM_VREG_CONSUMERS(name) \
	static struct regulator_consumer_supply __pcom_vreg_supply_##name[]

#define PCOM_VREG_CONSTRAINT_LVSW(_name, _always_on, _boot_on, _supply_uV) \
{ \
	.name = #_name, \
	.min_uV = 0, \
	.max_uV = 0, \
	.input_uV = _supply_uV, \
	.valid_modes_mask = REGULATOR_MODE_NORMAL, \
	.valid_ops_mask = REGULATOR_CHANGE_STATUS, \
	.apply_uV = 0, \
	.boot_on = _boot_on, \
	.always_on = _always_on \
}

#define PCOM_VREG_CONSTRAINT_DYN(_name, _min_uV, _max_uV, _always_on, \
		_boot_on, _apply_uV, _supply_uV) \
{ \
	.name = #_name, \
	.min_uV = _min_uV, \
	.max_uV = _max_uV, \
	.valid_modes_mask = REGULATOR_MODE_NORMAL, \
	.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS, \
	.input_uV = _supply_uV, \
	.apply_uV = _apply_uV, \
	.boot_on = _boot_on, \
	.always_on = _always_on \
}


#define PCOM_VREG_INIT(_name, _supply, _constraints)\
{ \
	.supply_regulator = _supply, \
	.consumer_supplies = __pcom_vreg_supply_##_name, \
	.num_consumer_supplies = ARRAY_SIZE(__pcom_vreg_supply_##_name), \
	.constraints = _constraints \
}

#define PCOM_VREG_SMP(_name, _id, _supply, _min_uV, _max_uV, _rise_time, \
		_pulldown, _always_on, _boot_on, _apply_uV, _supply_uV) \
{ \
	.init_data = PCOM_VREG_INIT(_name, _supply, \
		PCOM_VREG_CONSTRAINT_DYN(_name, _min_uV, _max_uV, _always_on, \
			_boot_on, _apply_uV, _supply_uV)), \
	.id = _id, \
	.rise_time = _rise_time, \
	.pulldown = _pulldown, \
	.negative = 0, \
}

#define PCOM_VREG_LDO PCOM_VREG_SMP

#define PCOM_VREG_LVS(_name, _id, _supply, _rise_time, _pulldown, _always_on, \
		_boot_on) \
{ \
	.init_data = PCOM_VREG_INIT(_name, _supply, \
		PCOM_VREG_CONSTRAINT_LVSW(_name, _always_on, _boot_on, 0)), \
	.id = _id, \
	.rise_time = _rise_time, \
	.pulldown = _pulldown, \
	.negative = 0, \
}

#define PCOM_VREG_NCP(_name, _id, _supply, _min_uV, _max_uV, _rise_time, \
		_always_on, _boot_on, _apply_uV, _supply_uV) \
{ \
	.init_data = PCOM_VREG_INIT(_name, _supply, \
		PCOM_VREG_CONSTRAINT_DYN(_name, -(_min_uV), -(_max_uV), \
			_always_on, _boot_on, _apply_uV, _supply_uV)), \
	.id = _id, \
	.rise_time = _rise_time, \
	.pulldown = -1, \
	.negative = 1, \
}

PCOM_VREG_CONSUMERS(smps0) = {
	REGULATOR_SUPPLY("smps0",	NULL),
	REGULATOR_SUPPLY("msmc1",	NULL),
};

PCOM_VREG_CONSUMERS(smps1) = {
	REGULATOR_SUPPLY("smps1",	NULL),
	REGULATOR_SUPPLY("msmc2",	NULL),
};

PCOM_VREG_CONSUMERS(smps2) = {
	REGULATOR_SUPPLY("smps2",	NULL),
	REGULATOR_SUPPLY("s2",		NULL),
};

PCOM_VREG_CONSUMERS(smps3) = {
	REGULATOR_SUPPLY("smps3",	NULL),
	REGULATOR_SUPPLY("s3",		NULL),
};

PCOM_VREG_CONSUMERS(smps4) = {
	REGULATOR_SUPPLY("smps4",	NULL),
	REGULATOR_SUPPLY("s4",		NULL),
};

PCOM_VREG_CONSUMERS(ldo00) = {
	REGULATOR_SUPPLY("ldo00",	NULL),
	REGULATOR_SUPPLY("ldo0",	NULL),
	REGULATOR_SUPPLY("gp3",		NULL),
};

#ifdef CONFIG_MACH_MSM7630_U0
/*L2 -NC*/
#else
PCOM_VREG_CONSUMERS(ldo02) = {
	REGULATOR_SUPPLY("ldo02",	NULL),
	REGULATOR_SUPPLY("ldo2",	NULL),
	REGULATOR_SUPPLY("xo_out",	NULL),
};
#endif

PCOM_VREG_CONSUMERS(ldo03) = {
	REGULATOR_SUPPLY("ldo03",	NULL),
	REGULATOR_SUPPLY("ldo3",	NULL),
	REGULATOR_SUPPLY("ruim",	NULL),
};

#ifdef CONFIG_MACH_MSM7630_U0
/*L4 -NC*/
#else
PCOM_VREG_CONSUMERS(ldo04) = {
	REGULATOR_SUPPLY("ldo04",	NULL),
	REGULATOR_SUPPLY("ldo4",	NULL),
	REGULATOR_SUPPLY("tcxo",	NULL),
};
#endif

PCOM_VREG_CONSUMERS(ldo05) = {
	REGULATOR_SUPPLY("ldo05",	NULL),
	REGULATOR_SUPPLY("ldo5",	NULL),
	REGULATOR_SUPPLY("mmc",		NULL),
};

PCOM_VREG_CONSUMERS(ldo06) = {
	REGULATOR_SUPPLY("ldo06",	NULL),
	REGULATOR_SUPPLY("ldo6",	NULL),
	REGULATOR_SUPPLY("usb",		NULL),
};

PCOM_VREG_CONSUMERS(ldo07) = {
	REGULATOR_SUPPLY("ldo07",	NULL),
	REGULATOR_SUPPLY("ldo7",	NULL),
	REGULATOR_SUPPLY("usb2",	NULL),
};

PCOM_VREG_CONSUMERS(ldo08) = {
	REGULATOR_SUPPLY("ldo08",	NULL),
	REGULATOR_SUPPLY("ldo8",	NULL),
	REGULATOR_SUPPLY("gp7",		NULL),
};

PCOM_VREG_CONSUMERS(ldo09) = {
	REGULATOR_SUPPLY("ldo09",	NULL),
	REGULATOR_SUPPLY("ldo9",	NULL),
	REGULATOR_SUPPLY("gp1",		NULL),
};

PCOM_VREG_CONSUMERS(ldo10) = {
	REGULATOR_SUPPLY("ldo10",	NULL),
	REGULATOR_SUPPLY("gp4",		NULL),
};

PCOM_VREG_CONSUMERS(ldo11) = {
	REGULATOR_SUPPLY("ldo11",	NULL),
	REGULATOR_SUPPLY("gp2",		NULL),
};

#ifdef CONFIG_MACH_LGE_M3S
PCOM_VREG_CONSUMERS(touch3v0) = {
	REGULATOR_SUPPLY("touch3v0",	NULL),
	REGULATOR_SUPPLY("gp9", 	NULL),
};
#else
PCOM_VREG_CONSUMERS(ldo12) = {
	REGULATOR_SUPPLY("ldo12",	NULL),
	REGULATOR_SUPPLY("gp9",		NULL),
};
#endif

PCOM_VREG_CONSUMERS(ldo13) = {
	REGULATOR_SUPPLY("ldo13",	NULL),
	REGULATOR_SUPPLY("wlan",	NULL),
};

PCOM_VREG_CONSUMERS(ldo14) = {
	REGULATOR_SUPPLY("ldo14",	NULL),
	REGULATOR_SUPPLY("rf",		NULL),
};

PCOM_VREG_CONSUMERS(ldo15) = {
	REGULATOR_SUPPLY("ldo15",	NULL),
	REGULATOR_SUPPLY("gp6",		NULL),
};

/* L16 - +3V0_MOTOR */
PCOM_VREG_CONSUMERS(ldo16) = {
	REGULATOR_SUPPLY("ldo16",	NULL),
	REGULATOR_SUPPLY("gp10",	NULL),
};


PCOM_VREG_CONSUMERS(ldo17) = {
	REGULATOR_SUPPLY("ldo17",	NULL),
	REGULATOR_SUPPLY("gp11",	NULL),
};

PCOM_VREG_CONSUMERS(ldo18) = {
	REGULATOR_SUPPLY("ldo18",	NULL),
	REGULATOR_SUPPLY("gp12",	NULL),
};


PCOM_VREG_CONSUMERS(ldo19) = {
	REGULATOR_SUPPLY("ldo19",	NULL),
	REGULATOR_SUPPLY("wlan2",	NULL),
};

PCOM_VREG_CONSUMERS(ldo20) = {
	REGULATOR_SUPPLY("ldo20",	NULL),
	REGULATOR_SUPPLY("gp13",	NULL),
};

PCOM_VREG_CONSUMERS(ldo21) = {
	REGULATOR_SUPPLY("ldo21",	NULL),
	REGULATOR_SUPPLY("gp14",	NULL),
};

PCOM_VREG_CONSUMERS(ldo22) = {
	REGULATOR_SUPPLY("ldo22",	NULL),
	REGULATOR_SUPPLY("gp15",	NULL),
};

#ifdef CONFIG_MACH_MSM7630_U0
/*L19 -NC*/
#else
PCOM_VREG_CONSUMERS(ldo23) = {
	REGULATOR_SUPPLY("ldo23",	NULL),
	REGULATOR_SUPPLY("gp5",		NULL),
};
#endif

PCOM_VREG_CONSUMERS(ldo24) = {
	REGULATOR_SUPPLY("ldo24",	NULL),
	REGULATOR_SUPPLY("gp16",	NULL),
};

PCOM_VREG_CONSUMERS(ldo25) = {
	REGULATOR_SUPPLY("ldo25",	NULL),
	REGULATOR_SUPPLY("gp17",	NULL),
};

#ifdef CONFIG_MACH_LGE_M3S
PCOM_VREG_CONSUMERS(touch1v8) = {
	REGULATOR_SUPPLY("touch1v8",	NULL),
	REGULATOR_SUPPLY("lvs0",	NULL),
};
#else
PCOM_VREG_CONSUMERS(lvsw0) = {
	REGULATOR_SUPPLY("lvsw0",	NULL),
	REGULATOR_SUPPLY("lvs0",	NULL),
};
#endif

PCOM_VREG_CONSUMERS(lvsw1) = {
	REGULATOR_SUPPLY("lvsw1",	NULL),
	REGULATOR_SUPPLY("lvs1",	NULL),
};

PCOM_VREG_CONSUMERS(ncp)   = {
	REGULATOR_SUPPLY("ncp",		NULL),
};

/* This list needs to be verified against actual 7x30 hardware requirements. */
static struct proccomm_regulator_info msm7x30_pcom_vreg_info[] = {
	/* Standard regulators (SMPS and LDO)
	 * R = rise time (us)
	 * P = pulldown (1 = pull down, 0 = float, -1 = don't care)
	 * A = always on
	 * B = boot on
	 * V = automatic voltage set (meaningful for single-voltage regs only)
	 * S = supply voltage (uV)
	 *             name  id  supp    min uV    max uV  R   P  A  B  V  S */
	PCOM_VREG_SMP(smps0,  3, NULL,	 375000,  3050000, 0, -1, 0, 0, 0, 0),
	PCOM_VREG_SMP(smps1,  4, NULL,	 375000,  3050000, 0, -1, 0, 0, 0, 0),
	PCOM_VREG_SMP(smps2, 28, NULL,	 375000,  3050000, 0, -1, 0, 0, 0, 0),
	PCOM_VREG_SMP(smps3, 29, NULL,	 375000,  3050000, 0, -1, 0, 0, 0, 0),
	PCOM_VREG_SMP(smps4, 43, NULL,	 375000,  3050000, 0, -1, 0, 0, 0, 0),
	PCOM_VREG_LDO(ldo00,  5, NULL,	 750000,  1525000, 0, -1, 0, 0, 0, 0),
	/*L2 -NC*/
	PCOM_VREG_LDO(ldo03, 19, NULL,	1500000,  3050000, 0, -1, 0, 0, 0, 0), /*L3 - 2V85_RFSW*/
	/*L4 - NC*/
	PCOM_VREG_LDO(ldo05, 18, NULL,	1500000,  3050000, 0, -1, 0, 0, 0, 0), /*L5 - 2V85_eMMC*/
	PCOM_VREG_LDO(ldo06, 16, NULL,	3075000,  3075000, 0, -1, 0, 0, 0, 0), /*L6 - 3V075_USB*/
	PCOM_VREG_LDO(ldo07, 44, NULL,	1500000,  3050000, 0, -1, 0, 0, 0, 0), /*L7 - 1V8_USB*/
	PCOM_VREG_LDO(ldo08, 32, NULL,	1500000,  3050000, 0, -1, 0, 0, 0, 0), /*L8 - 2V85_PROXI*/
	PCOM_VREG_LDO(ldo09,  8, NULL,	1500000,  3050000, 0, -1, 0, 0, 0, 0), /*L9 - 2V05_MSM_ANALOG*/
	PCOM_VREG_LDO(ldo10,  7, NULL,	1500000,  3050000, 0, -1, 0, 0, 0, 0), /*L10 - 2V85_MICRO_SD*/
	PCOM_VREG_LDO(ldo11, 21, NULL,	1500000,  3050000, 0, -1, 0, 0, 0, 0), /*L11 - 1V8_CAM_IO*/
	PCOM_VREG_LDO(touch3v0, 34, NULL,  1500000,  3050000, 0, -1, 0, 0, 0, 0),
	PCOM_VREG_LDO(ldo13, 15, NULL,	1500000,  3050000, 0, -1, 0, 0, 0, 0), /*L13 - 3V0_WCN_PWR*/
	PCOM_VREG_LDO(ldo14, 24, NULL,	1500000,  3050000, 0, -1, 0, 0, 0, 0), /*L14 - 2V8_LCD*/
	PCOM_VREG_LDO(ldo15, 23, NULL,	1500000,  3050000, 0, -1, 0, 0, 0, 0), /*L15 - 2V8_CAM_AF*/
	/* L16 - +3V0_MOTOR */
	PCOM_VREG_LDO(ldo16, 35, NULL,	1500000,  3050000, 0, -1, 0, 0, 0, 0), /*L16 - +3V0_MOTOR*/
	PCOM_VREG_LDO(ldo17, 36, NULL,	1500000,  3050000, 0, -1, 0, 0, 0, 0), /*L17 - 2V8_CAM_AVDD*/
	PCOM_VREG_LDO(ldo18, 37, NULL,	1500000,  3050000, 0, -1, 0, 0, 0, 0), /*L18 - 2V2_ADC_REF*/
	
	PCOM_VREG_LDO(ldo19, 45, NULL,	1500000,  3050000, 0, -1, 0, 0, 0, 0), /*L19 - 1V8_LCD_IO*/
	
	PCOM_VREG_LDO(ldo20, 38, NULL,	1500000,  3050000, 0, -1, 0, 0, 0, 0), /*L20 - 1V8_CAM_DVDD*/
	PCOM_VREG_LDO(ldo21, 39, NULL,	 750000,  1525000, 0, -1, 0, 0, 0, 0), /*L21 - 1V1_MSM_PLL*/
	PCOM_VREG_LDO(ldo22, 40, NULL,	 750000,  1525000, 0, -1, 0, 0, 0, 0), /*L22 - 1V2_WCN*/
	/*L23 - NC*/
	PCOM_VREG_LDO(ldo24, 41, NULL,	 750000,  1525000, 0, -1, 0, 0, 0, 0), /*L24 - 1V2_RF*/
	PCOM_VREG_LDO(ldo25, 42, NULL,	 750000,  1525000, 0, -1, 0, 0, 0, 0), /*L25 - 1V2_POP_VDDQ*/
	PCOM_VREG_LVS(touch1v8, 47, NULL,                     0, -1, 0, 0),
	PCOM_VREG_LVS(lvsw1, 48, NULL,					   0, -1, 0, 0),

	PCOM_VREG_NCP(ncp,   31, NULL, -1800000, -1800000, 0,     0, 0, 0, 0),
};

struct proccomm_regulator_platform_data msm7x30_proccomm_regulator_data = {
	.regs = msm7x30_pcom_vreg_info,
	.nregs = ARRAY_SIZE(msm7x30_pcom_vreg_info)
};
