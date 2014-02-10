/* Copyright (c) 2009, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM__ARCH_OEM_RAPI_CLIENT_H
#define __ASM__ARCH_OEM_RAPI_CLIENT_H

/*
 * OEM RAPI CLIENT Driver header file
 */

#include <linux/types.h>
#include <mach/msm_rpcrouter.h>

enum {
	OEM_RAPI_CLIENT_EVENT_NONE = 0,

	/*
	 * list of oem rapi client events
	 */

#if defined (CONFIG_LGE_SUPPORT_RAPI)
	/* LGE_CHANGES_S [khlee@lge.com] 2009-12-04, [VS740] use OEMRAPI */
	LG_FW_RAPI_START = 100,
	LG_FW_RAPI_CLIENT_EVENT_GET_LINE_TYPE = LG_FW_RAPI_START,
	LG_FW_TESTMODE_EVENT_FROM_ARM11 = LG_FW_RAPI_START + 1,
	LG_FW_A2M_BATT_INFO_GET = LG_FW_RAPI_START + 2,
	LG_FW_A2M_PSEUDO_BATT_INFO_SET = LG_FW_RAPI_START + 3,
	LG_FW_MEID_GET = LG_FW_RAPI_START + 4,
	/* LGE_CHANGE_S 
	 * SUPPORT TESTMODE FOR AIRPLAN MODE
	 * 2010-07-12 taehung.kim@lge.com
	 */
	LG_FW_SET_OPERATION_MODE = LG_FW_RAPI_START + 5,
	LG_FW_A2M_BLOCK_CHARGING_SET = LG_FW_RAPI_START + 6,
/* BEGIN: 0013860 jihoon.lee@lge.com 20110111 */
/* ADD 0013860: [FACTORY RESET] ERI file save */
#ifdef CONFIG_LGE_ERI_DOWNLOAD
	LGE_REQUEST_ERI_RPC = LG_FW_RAPI_START + 7,
#endif
/* END: 0013860 jihoon.lee@lge.com 20110111 */
#endif
	/* [yk.kim@lge.com] 2011-01-25, get manual test mode NV */
#ifdef CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB
	LG_FW_MANUAL_TEST_MODE = LG_FW_RAPI_START + 8,
#endif
/* BEGIN: 0016311 jihoon.lee@lge.com 20110217 */
/* ADD 0016311: [POWER OFF] CALL EFS_SYNC */
#ifdef CONFIG_LGE_SUPPORT_RAPI
	LGE_RPC_HANDLE_REQUEST = LG_FW_RAPI_START + 9,
#endif
/* END: 0016311 jihoon.lee@lge.com 20110217 */
#ifdef CONFIG_LGE_USB_GADGET_LLDM_DRIVER
    LG_FW_LLDM_SDIO_INFO_SET = LG_FW_RAPI_START + 10,
    LG_FW_LLDM_SDIO_INFO_GET = LG_FW_RAPI_START + 11,
#endif

	/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-10-02, charger logo notification to modem */
#ifdef CONFIG_LGE_CHARGING_MODE_INFO
	LG_FW_CHG_LOGO_MODE = LG_FW_RAPI_START + 13,
#endif
#ifdef CONFIG_LGE_DIAG_ICD
	// LGE_CHANGE [2011.02.08] [myeonggyu.son@lge.com] [gelato] add icd oem rapi function
	LG_FW_RAPI_ICD_DIAG_EVENT = LG_FW_RAPI_START + 14,
#endif
#ifdef CONFIG_LGE_DLOAD_RESET_BOOT_UP
	// LGE_CHANGE [2011.02.08] [myeonggyu.son@lge.com] [gelato] add icd oem rapi function
	LG_FW_FIRST_BOOT_COMPLETE_EVENT = LG_FW_RAPI_START + 15,
#endif
#ifdef CONFIG_LGE_DIAG_SRD
	LG_FW_REQUEST_SRD_RPC = LG_FW_RAPI_START+20, 
	LG_FW_OEM_RAPI_CLIENT_SRD_COMMAND = LG_FW_RAPI_START+21,  //send event 
#endif
//LGE_CHANGE_S [jinhwan.do][2012.03.09]USB Access Lock Porting [Start]
#ifdef CONFIG_LGE_DIAG_USB_ACCESS_LOCK
	LG_FW_SET_USB_LOCK_STATE =   LG_FW_RAPI_START + 25,
	LG_FW_GET_USB_LOCK_STATE =   LG_FW_RAPI_START + 26,
	LG_FW_GET_SPC_CODE =   LG_FW_RAPI_START + 28,
#endif
//LGE_CHANGE_S [jinhwan.do][2012.03.09]USB Access Lock Porting [End]
	//Start tao.jin@lge.com LG_FW_CAMERA_MODE added for camcorder current issue 2011-12-23
#if 1	
		LG_FW_CAMERA_MODE =   LG_FW_RAPI_START + 32,
#endif
	//End tao.jin@lge.com LG_FW_CAMERA_CODE added for camcorder current issue 2011-12-23
/* LGE_CHANGE_S [START] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */ 	
#ifdef CONFIG_LGE_DIAG_USB_ACCESS_LOCK_SHA1_ENCRYPTION
		LG_FW_GET_USB_LOCK_KEY	= LG_FW_RAPI_START + 59,
		LG_FW_SET_USB_LOCK_KEY	= LG_FW_RAPI_START + 60,
#endif
#ifdef CONFIG_LGE_DIAG_USB_PERMANENT_LOCK
		LG_FW_SET_USB_UNLOCK_FAIL_CNT	= LG_FW_RAPI_START + 61,
		LG_FW_GET_USB_UNLOCK_FAIL_CNT	= LG_FW_RAPI_START + 62,
#endif
#ifdef CONFIG_LGE_DIAG_USBLOCK_EFS_SYNC
		LG_FW_GET_USBLOCK_EFS_SYNC_RESULT	= LG_FW_RAPI_START + 63,
#endif
/* LGE_CHANGE_S [END] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */

	OEM_RAPI_CLIENT_EVENT_MAX,
};

/* BEGIN: 0016311 jihoon.lee@lge.com 20110217 */
/* ADD 0016311: [POWER OFF] CALL EFS_SYNC */
#ifdef CONFIG_LGE_SUPPORT_RAPI
enum {
	LGE_CLIENT_CMD_START = 0,
#ifdef CONFIG_LGE_SYNC_CMD
	LGE_SYNC_REQUEST = 1,
#endif
	LGE_SW_VERSION_INFO = 2,
	LGE_MIN_INFO = 3,
	LGE_TESTMODE_MANUAL_TEST_INFO = 4,
	LGE_MEID_INFO = 5,
	LGE_ESN_INFO = 6,
#ifdef CONFIG_LGE_SYNC_CMD
	LGE_DISABLE_EFS_SYNC = 7,
	LGE_ENABLE_EFS_SYNC = 8,
#endif
	LGE_REQUEST_MODEM_FACTORY_RESET = 0xA,
	LGE_REQUEST_ONLINE_MODE = 0xB,
	LGE_CLIENT_CMD_MAX = 0xF,
};
#endif
/* ADD: [FOTA] LGE_FOTA_MISC_INFO */
/* END: 0016311 jihoon.lee@lge.com 20110217 */

struct oem_rapi_client_streaming_func_cb_arg {
	uint32_t  event;
	void      *handle;
	uint32_t  in_len;
	char      *input;
	uint32_t out_len_valid;
	uint32_t output_valid;
	uint32_t output_size;
};

struct oem_rapi_client_streaming_func_cb_ret {
	uint32_t *out_len;
	char *output;
};

struct oem_rapi_client_streaming_func_arg {
	uint32_t event;
	int (*cb_func)(struct oem_rapi_client_streaming_func_cb_arg *,
		       struct oem_rapi_client_streaming_func_cb_ret *);
	void *handle;
	uint32_t in_len;
	char *input;
	uint32_t out_len_valid;
	uint32_t output_valid;
	uint32_t output_size;
};

struct oem_rapi_client_streaming_func_ret {
	uint32_t *out_len;
	char *output;
};

int oem_rapi_client_streaming_function(
	struct msm_rpc_client *client,
	struct oem_rapi_client_streaming_func_arg *arg,
	struct oem_rapi_client_streaming_func_ret *ret);

int oem_rapi_client_close(void);

struct msm_rpc_client *oem_rapi_client_init(void);

#endif
