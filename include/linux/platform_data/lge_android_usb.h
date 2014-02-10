/* arch/arm/mach-msm/include/mach/lge/lge_android_usb.h
 *
 * Copyright (C) 2011, 2012 LG Electronics Inc.
 * Author : Hyeon H. Park <hyunhui.park@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LGE_ANDROID_USB_H__
#define __LGE_ANDROID_USB_H__

struct lge_android_usb_platform_data {
	__u16 vendor_id;
	__u16 factory_pid;
	__u8  iSerialNumber;
	char *product_name;
	char *manufacturer_name;
#if defined CONFIG_USB_G_LGE_ANDROID && defined CONFIG_LGE_PM
	char *factory_composition;
	int (*get_factory_cable)(void);
#endif
};

int lgeusb_get_pif_cable(void);

int lgeusb_get_vendor_id(void);
int lgeusb_get_factory_pid(void);
int lgeusb_get_serial_number(void);
int lgeusb_get_product_name(char *);
int lgeusb_get_manufacturer_name(char *);
#if defined CONFIG_USB_G_LGE_ANDROID && defined CONFIG_LGE_PM
int lgeusb_get_factory_composition(char *);
/* LGE_CHANGE_S [START] 2012.2.26 jaeho.cho@lge.com support factory usb of the chipset below MSM8X60 */
#ifdef CONFIG_LGE_USB_FACTORY_BELOW_MSM8X60
enum {
    NOINIT_CABLE = 0x00,
    RESERVED_CABLE1, /* 0x01 */
    RESERVED_CABLE2, /* 0x02 */
    LT_CABLE_56K, /* 0x03 */
	TA_CABLE_1A, /* 0x04 */ 		/* Travel Adaptor -MAX 1000mA	/USB */
	CRADLE_CABLE, /* 0x05 */	
	FORGED_CABLE, /* 0x06 */
    TA_CABLE_700MA, /* 0x07 */		 /* Travel Adaptor -MAX 700mA  /USB */
	RESERVED_CABLE3, /* 0x08 */
	RESERVED_CABLE4, /* 0x09 */
	LT_CABLE_130K, /* 0xA */
	LT_CABLE_910K, /* 0xB */
	RESERVED_CABLE5, /* 0x0C*/
    MAX_CABLE,
};
#else
#define LGEUSB_FACTORY_56K 1
#define LGEUSB_FACTORY_130K 2
#define LGEUSB_FACTORY_910K 3
#endif
/* LGE_CHANGE_S [END] 2012.2.26 jaeho.cho@lge.com support factory usb of the chipset below MSM8X60 */
#endif
#endif /* __LGE_ANDROID_USB_H__ */
