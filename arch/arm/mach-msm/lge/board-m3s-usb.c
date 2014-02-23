/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/usb/android.h>
#include <linux/msm_ssbi.h>
#include <linux/regulator/gpio-regulator.h>
#ifdef CONFIG_USB_MSM_OTG_72K
#include <mach/msm_hsusb.h>
#else
#include <linux/usb/msm_hsusb.h>
#endif
#include <mach/usbdiag.h>
#include <mach/socinfo.h>
//#include <mach/rpm.h>
#include <mach/gpio.h>

#include "devices.h"
#include "devices-msm8x60.h"

#ifdef CONFIG_LGE_PM
#include <mach/board_lge.h>
#include <linux/platform_data/lge_android_usb.h>


#endif
/* LGE_CHANGE_S [START] 2012.2.26 jaeho.cho@lge.com support factory usb of the chipset below MSM8X60 */
#ifdef CONFIG_LGE_USB_FACTORY_BELOW_MSM8X60
#include "proc_comm.h"
#endif
/* LGE_CHANGE_S [END] 2012.2.26 jaeho.cho@lge.com support factory usb of the chipset below MSM8X60 */
/* LGE_CHANGE_S [START] 2012.6.14 jaeho.cho@lge.com keep LCD backlight brightness constant value with connected factory cable*/
#ifdef CONFIG_LGE_USB_DETECT_IMPROVEMENT
static int factory_cable = 0;
#endif
/* LGE_CHANGE_S [END] 2012.6.14 jaeho.cho@lge.com keep LCD backlight brightness constant value with connected factory cable*/

#if defined (CONFIG_USB_G_LGE_ANDROID) && defined (CONFIG_LGE_PM)
/* LGE_CHANGE_S [START] 2012.2.26 jaeho.cho@lge.com support factory usb of the chipset below MSM8X60 */
#ifdef CONFIG_LGE_USB_FACTORY_BELOW_MSM8X60
static int get_msm_cable_type(void)
{ 
	unsigned int cable_type;
	int fn_type = CUSTOMER_CMD1_GET_CABLE_TYPE;

	msm_proc_comm(PCOM_CUSTOMER_CMD1,  &cable_type,&fn_type);
	printk("[LGE_PWR] cable type detection from muic at modem side Cable=%d \n",cable_type);

	return cable_type;
}
#else
static int get_factory_cable(void)
{
	struct chg_cable_info info;
	int res;

	/* get cable infomation */
	res = lge_pm_get_cable_info(&info);
	if (res < 0) {
		pr_err("Error get cable information from PMIC %d\n", res);
		return 0;
	}

	switch(info.cable_type) {
	/* It is factory cable */
	case CABLE_56K:
		res = LGEUSB_FACTORY_56K;
		break;
	case CABLE_130K:
		res = LGEUSB_FACTORY_130K;
		break;
	case CABLE_910K:
		res = LGEUSB_FACTORY_910K;
		break;
	/* It is normal cable */
	default:
		res = 0;
		break;
	}

	return res;
}
#endif
/* LGE_CHANGE_S [END] 2012.2.26 jaeho.cho@lge.com support factory usb of the chipset below MSM8X60 */

struct lge_android_usb_platform_data lge_android_usb_pdata = {
	.vendor_id = 0x1004,
	.factory_pid = 0x6000,
	.iSerialNumber = 0,
	.product_name = "LGE Android Phone",
	.manufacturer_name = "LG Electronics Inc.",
	.factory_composition = "acm,diag",
/* LGE_CHANGE_S [START] 2012.2.26 jaeho.cho@lge.com support factory usb of the chipset below MSM8X60 */
#ifdef CONFIG_LGE_USB_FACTORY_BELOW_MSM8X60
	.get_factory_cable = get_msm_cable_type,
#else
	.get_factory_cable = get_factory_cable,
#endif
/* LGE_CHANGE_S [END] 2012.2.26 jaeho.cho@lge.com support factory usb of the chipset below MSM8X60 */
};

struct platform_device lge_android_usb_device = {
	.name = "lge_android_usb",
	.id = -1,
	.dev = {
		.platform_data = &lge_android_usb_pdata,
	},
};

static struct platform_device *usb_devices[] __initdata = {
#ifdef CONFIG_USB_G_LGE_ANDROID
	&lge_android_usb_device,
#endif
};

void __init lge_add_usb_devices(void)
{
	platform_add_devices(usb_devices, ARRAY_SIZE(usb_devices));
}
/* LGE_CHANGE_S [START] 2012.6.14 jaeho.cho@lge.com keep LCD backlight brightness constant value with connected factory cable*/
#ifdef CONFIG_LGE_USB_DETECT_IMPROVEMENT
int get_charger_id_type(void)
{
	return factory_cable;
}
EXPORT_SYMBOL(get_charger_id_type);
static int __init set_charger_id_type(char *charger_type)
{
	if (!strncmp("factory_cable", charger_type, strlen("factory_cable"))) {
		factory_cable = 1;
	} 

	return 1;
}

__setup("androidboot.charger_type=", set_charger_id_type);
#endif
/* LGE_CHANGE_S [END] 2012.6.14 jaeho.cho@lge.com keep LCD backlight brightness constant value with connected factory cable*/

#endif

