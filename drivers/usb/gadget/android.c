/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/android.h>

#include "gadget_chips.h"

#ifdef CONFIG_USB_G_LGE_ANDROID
#include <linux/platform_data/lge_android_usb.h>
#endif

#ifdef CONFIG_LGE_PM
#include <mach/board_lge.h>
#endif

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

#include "f_diag.c"
#include "f_rmnet_smd.c"
#include "f_rmnet_sdio.c"
#include "f_rmnet_smd_sdio.c"
#include "f_rmnet.c"
#include "f_mass_storage.c"
#include "u_serial.c"
#include "u_sdio.c"
#include "u_smd.c"
#include "u_bam.c"
#include "u_rmnet_ctrl_smd.c"
#include "u_ctrl_hsic.c"
#include "u_data_hsic.c"
#include "f_serial.c"
#include "f_acm.c"
#include "f_adb.c"
#include "f_ccid.c"
#include "f_mtp.c"
#include "f_accessory.c"

#ifdef CONFIG_USB_G_LGE_ANDROID
/* LGE_CHANGE
 * we must use ecm and rndis exclusively.
 * 2011-10-24, hyunhui.park@lge.com
 */
#include "f_ecm.c"
#else /* google original */
#define USB_ETH_RNDIS y
#include "f_rndis.c"
#include "rndis.c"
#endif
#include "u_ether.c"

#ifdef CONFIG_LGE_USB_CHARGE_ONLY
#include "f_charge_only.c"
#endif

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Android";

/* Default vendor and product IDs, overridden by userspace */
#define VENDOR_ID		0x18D1
#define PRODUCT_ID		0x0001

struct android_usb_function {
	char *name;
	void *config;

	struct device *dev;
	char *dev_name;
	struct device_attribute **attributes;

	/* for android_dev.enabled_functions */
	struct list_head enabled_list;

	/* Optional: initialization during gadget bind */
	int (*init)(struct android_usb_function *, struct usb_composite_dev *);
	/* Optional: cleanup during gadget unbind */
	void (*cleanup)(struct android_usb_function *);

	int (*bind_config)(struct android_usb_function *, struct usb_configuration *);

	/* Optional: called when the configuration is removed */
	void (*unbind_config)(struct android_usb_function *, struct usb_configuration *);
	/* Optional: handle ctrl requests before the device is configured */
	int (*ctrlrequest)(struct android_usb_function *,
					struct usb_composite_dev *,
					const struct usb_ctrlrequest *);
};

struct android_dev {
	struct android_usb_function **functions;
	struct list_head enabled_functions;
	struct usb_composite_dev *cdev;
	struct device *dev;
	struct android_usb_platform_data *pdata;

	bool enabled;
	bool connected;
	bool sw_connected;
	struct work_struct work;
#if defined CONFIG_USB_G_LGE_ANDROID && defined CONFIG_LGE_PM
	bool check_pif;
#endif
/* LGE_CHANGE_S [START] 2012.4.17 jaeho.cho@lge.com merge charge only feature from G1 TDR  */
#ifdef CONFIG_LGE_USB_CHARGE_ONLY
	bool check_charge_only;
#endif
/* LGE_CHANGE_S [END] 2012.4.17 jaeho.cho@lge.com merge charge only feature from G1 TDR */
#ifdef CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB
	struct mutex lock;
#endif
};

static struct class *android_class;
static struct android_dev *_android_dev;
static int android_bind_config(struct usb_configuration *c);
static void android_unbind_config(struct usb_configuration *c);

//LGE_CHANGE [jinhwan.do@lge.com][2012.04.13]get USB serial number, merge from LS696 [Start]
#ifdef CONFIG_LGE_USB_EXPORT_SERIAL_NUMBER
void android_get_serial_number(char *snum);
#define MAX_SERIAL_LEN 256
#endif
//LGE_CHANGE [jinhwan.do@lge.com][2012.04.13]get USB serial number, merge from LS696 [End]

/* string IDs are assigned dynamically */
#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

static char manufacturer_string[256];
static char product_string[256];
static char serial_string[256];
//LGE_CHANGE jinhwan.do 20120430 Slate command's concept is cahnged, USB Serial number is MEID's decimal value [Start]
#ifdef CONFIG_LGE_USB_EXPORT_SERIAL_NUMBER
static char hex_meid[256];
static char dec_meid[256];
#endif
//LGE_CHANGE jinhwan.do 20120430 Slate command's concept is cahnged, USB Serial number is MEID's decimal value [End]

/* LGE_CHANGE_S [START] 2012.4.17 jaeho.cho@lge.com merge charge only feature from G1 TDR  */
#ifdef CONFIG_LGE_USB_CHARGE_ONLY
#define CHARGE_ONLY_STRING_IDX		3
static char charge_only_string[256];
#endif
/* LGE_CHANGE_S [END] 2012.4.17 jaeho.cho@lge.com merge charge only feature from G1 TDR */

#ifdef CONFIG_USB_G_LGE_SERIALNO_REDIRECTION
#define STRING_SERIAL_REDI_IDX		4
static char serial_string_redi[256];
#endif

/* String Table */
static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer_string,
	[STRING_PRODUCT_IDX].s = product_string,
	[STRING_SERIAL_IDX].s = serial_string,
/* LGE_CHANGE_S [START] 2012.4.17 jaeho.cho@lge.com merge charge only feature from G1 TDR  */
#ifdef CONFIG_LGE_USB_CHARGE_ONLY
	[CHARGE_ONLY_STRING_IDX].s = charge_only_string,
#endif
/* LGE_CHANGE_S [END] 2012.4.17 jaeho.cho@lge.com merge charge only feature from G1 TDR */
#ifdef CONFIG_USB_G_LGE_SERIALNO_REDIRECTION
	[STRING_SERIAL_REDI_IDX].s = serial_string_redi,
#endif
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.unbind		= android_unbind_config,
	.bConfigurationValue = 1,
	.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower	= 0xFA, /* 500ma */
};

//LGE_CHANGE_S [START] 2012.3.28 yckim.kim@lge.com add Factorymode and LT Cable condition
#ifdef CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB
int lg_manual_test_mode = 0;
u8 manual_test_mode;
static int android_get_manual_test_mode(char *buffer, struct kernel_param *kp);
module_param_call(manual_test_mode, NULL, android_get_manual_test_mode,
					&manual_test_mode, 0444);
MODULE_PARM_DESC(manual_test_mode, "Manual Test Mode");
extern int msm_get_manual_test_mode(void);
static int android_get_manual_test_mode(char *buffer, struct kernel_param *kp)
{
	int ret;
	pr_debug("KYC : [%s] get manual test mode - %d\n", __func__, lg_manual_test_mode);
	mutex_lock(&_android_dev->lock);
	ret = sprintf(buffer, "%d", lg_manual_test_mode);
	mutex_unlock(&_android_dev->lock);
	return ret;
}
#endif
//LGE_CHANGE_S [END] 2012.3.28 yckim.kim@lge.com add Factorymode and LT Cable condition

/* LGE_CHANGE_S [START] 2012.2.26 jaeho.cho@lge.com support factory usb of the chipset below MSM8X60 */
#ifdef CONFIG_LGE_USB_FACTORY_BELOW_MSM8X60
int cable = 0;
int android_boot_cable_type(void)
{
    return cable;
}
EXPORT_SYMBOL(android_boot_cable_type);
#endif
/* LGE_CHANGE_S [END] 2012.2.26 jaeho.cho@lge.com support factory usb of the chipset below MSM8X60 */

enum android_device_state {
	USB_DISCONNECTED,
	USB_CONNECTED,
	USB_CONFIGURED,
};

static void android_work(struct work_struct *data)
{
	struct android_dev *dev = container_of(data, struct android_dev, work);
	struct usb_composite_dev *cdev = dev->cdev;
	char *disconnected[2] = { "USB_STATE=DISCONNECTED", NULL };
	char *connected[2]    = { "USB_STATE=CONNECTED", NULL };
	char *configured[2]   = { "USB_STATE=CONFIGURED", NULL };
	char **uevent_envp = NULL;
	static enum android_device_state last_uevent, next_state;
	unsigned long flags;

	spin_lock_irqsave(&cdev->lock, flags);
	if (cdev->config) {
		uevent_envp = configured;
		next_state = USB_CONFIGURED;
	} else if (dev->connected != dev->sw_connected) {
		uevent_envp = dev->connected ? connected : disconnected;
		next_state = dev->connected ? USB_CONNECTED : USB_DISCONNECTED;
	}
	dev->sw_connected = dev->connected;
	spin_unlock_irqrestore(&cdev->lock, flags);

	if (uevent_envp) {
		/*
		 * Some userspace modules, e.g. MTP, work correctly only if
		 * CONFIGURED uevent is preceded by DISCONNECT uevent.
		 * Check if we missed sending out a DISCONNECT uevent. This can
		 * happen if host PC resets and configures device really quick.
		 */
		if (((uevent_envp == connected) &&
		      (last_uevent != USB_DISCONNECTED)) ||
		    ((uevent_envp == configured) &&
		      (last_uevent == USB_CONFIGURED))) {
			pr_info("%s: sent missed DISCONNECT event\n", __func__);
			kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE,
								disconnected);
			msleep(20);
		}
		/*
		 * Before sending out CONFIGURED uevent give function drivers
		 * a chance to wakeup userspace threads and notify disconnect
		 */
		if (uevent_envp == configured)
			msleep(50);

		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, uevent_envp);
		last_uevent = next_state;
		pr_info("%s: sent uevent %s\n", __func__, uevent_envp[0]);
	} else {
		pr_info("%s: did not send uevent (%d %d %p)\n", __func__,
			 dev->connected, dev->sw_connected, cdev->config);
	}
}


/*-------------------------------------------------------------------------*/
/* Supported functions initialization */

/* RMNET_SMD */
static int rmnet_smd_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_smd_bind_config(c);
}

static struct android_usb_function rmnet_smd_function = {
	.name		= "rmnet_smd",
	.bind_config	= rmnet_smd_function_bind_config,
};

/* RMNET_SDIO */
static int rmnet_sdio_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_sdio_function_add(c);
}

static struct android_usb_function rmnet_sdio_function = {
	.name		= "rmnet_sdio",
	.bind_config	= rmnet_sdio_function_bind_config,
};

/* RMNET_SMD_SDIO */
static int rmnet_smd_sdio_function_init(struct android_usb_function *f,
				 struct usb_composite_dev *cdev)
{
	return rmnet_smd_sdio_init();
}

static void rmnet_smd_sdio_function_cleanup(struct android_usb_function *f)
{
	rmnet_smd_sdio_cleanup();
}

static int rmnet_smd_sdio_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_smd_sdio_function_add(c);
}

static struct device_attribute *rmnet_smd_sdio_attributes[] = {
					&dev_attr_transport, NULL };

static struct android_usb_function rmnet_smd_sdio_function = {
	.name		= "rmnet_smd_sdio",
	.init		= rmnet_smd_sdio_function_init,
	.cleanup	= rmnet_smd_sdio_function_cleanup,
	.bind_config	= rmnet_smd_sdio_bind_config,
	.attributes	= rmnet_smd_sdio_attributes,
};

/*rmnet transport string format(per port):"ctrl0,data0,ctrl1,data1..." */
#define MAX_XPORT_STR_LEN 50
static char rmnet_transports[MAX_XPORT_STR_LEN];

static void rmnet_function_cleanup(struct android_usb_function *f)
{
	frmnet_cleanup();
}

static int rmnet_function_bind_config(struct android_usb_function *f,
					 struct usb_configuration *c)
{
	int i;
	int err = 0;
	char *ctrl_name;
	char *data_name;
	char buf[MAX_XPORT_STR_LEN], *b;
	static int rmnet_initialized, ports;

	if (!rmnet_initialized) {
		rmnet_initialized = 1;
		strlcpy(buf, rmnet_transports, sizeof(buf));
		b = strim(buf);
		while (b) {
			ctrl_name = strsep(&b, ",");
			data_name = strsep(&b, ",");
			if (ctrl_name && data_name) {
				err = frmnet_init_port(ctrl_name, data_name);
				if (err) {
					pr_err("rmnet: Cannot open ctrl port:"
						"'%s' data port:'%s'\n",
						ctrl_name, data_name);
					goto out;
				}
				ports++;
			}
		}

		err = rmnet_gport_setup();
		if (err) {
			pr_err("rmnet: Cannot setup transports");
			goto out;
		}
	}

	for (i = 0; i < ports; i++) {
		err = frmnet_bind_config(c, i);
		if (err) {
			pr_err("Could not bind rmnet%u config\n", i);
			break;
		}
	}
out:
	return err;
}

static ssize_t rmnet_transports_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", rmnet_transports);
}

static ssize_t rmnet_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(rmnet_transports, buff, sizeof(rmnet_transports));

	return size;
}

static struct device_attribute dev_attr_rmnet_transports =
					__ATTR(transports, S_IRUGO | S_IWUSR,
						rmnet_transports_show,
						rmnet_transports_store);
static struct device_attribute *rmnet_function_attributes[] = {
					&dev_attr_rmnet_transports,
					NULL };

static struct android_usb_function rmnet_function = {
	.name		= "rmnet",
	.cleanup	= rmnet_function_cleanup,
	.bind_config	= rmnet_function_bind_config,
	.attributes	= rmnet_function_attributes,
};

/* DIAG */
static char diag_clients[32];	    /*enabled DIAG clients- "diag[,diag_mdm]" */
static ssize_t clients_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
#if defined CONFIG_USB_G_LGE_ANDROID && defined CONFIG_LGE_PM
	struct android_dev *dev = _android_dev;
	if (dev->check_pif) {
		dev_info(dev->dev, "pif cable is plugged, not permitted\n");
		return -EPERM;
	}
#endif

	strlcpy(diag_clients, buff, sizeof(diag_clients));

	return size;
}

static DEVICE_ATTR(clients, S_IWUSR, NULL, clients_store);
static struct device_attribute *diag_function_attributes[] =
					 { &dev_attr_clients, NULL };

static int diag_function_init(struct android_usb_function *f,
				 struct usb_composite_dev *cdev)
{
	return diag_setup();
}

static void diag_function_cleanup(struct android_usb_function *f)
{
	diag_cleanup();
}

static int diag_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	char *name;
	char buf[32], *b;
	int once = 0, err = -1;
	int (*notify)(uint32_t, const char *);

	strlcpy(buf, diag_clients, sizeof(buf));
	b = strim(buf);

	while (b) {
		notify = NULL;
		name = strsep(&b, ",");
		/* Allow only first diag channel to update pid and serial no */
		if (_android_dev->pdata && !once++)
			notify = _android_dev->pdata->update_pid_and_serial_num;

		if (name) {
			err = diag_function_add(c, name, notify);
			if (err)
				pr_err("diag: Cannot open channel '%s'", name);
		}
	}

	return err;
}

static struct android_usb_function diag_function = {
	.name		= "diag",
	.init		= diag_function_init,
	.cleanup	= diag_function_cleanup,
	.bind_config	= diag_function_bind_config,
	.attributes	= diag_function_attributes,
};

/* SERIAL */
static char serial_transports[32];	/*enabled FSERIAL ports - "tty[,sdio]"*/
static ssize_t serial_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(serial_transports, buff, sizeof(serial_transports));

	return size;
}

static DEVICE_ATTR(transports, S_IWUSR, NULL, serial_transports_store);
static struct device_attribute *serial_function_attributes[] =
					 { &dev_attr_transports, NULL };

static void serial_function_cleanup(struct android_usb_function *f)
{
	gserial_cleanup();
}

static int serial_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	char *name;
	char buf[32], *b;
	int err = -1, i;
	static int serial_initialized = 0, ports = 0;

	if (serial_initialized)
		goto bind_config;

	serial_initialized = 1;
	strlcpy(buf, serial_transports, sizeof(buf));
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");

		if (name) {
			err = gserial_init_port(ports, name);
			if (err) {
				pr_err("serial: Cannot open port '%s'", name);
				goto out;
			}
			ports++;
		}
	}
	err = gport_setup(c);
	if (err) {
		pr_err("serial: Cannot setup transports");
		goto out;
	}

bind_config:
	for (i = 0; i < ports; i++) { 
		err = gser_bind_config(c, i);
		if (err) {
			pr_err("serial: bind_config failed for port %d", i);
			goto out;
		}
	}

out:
	return err;
}

static struct android_usb_function serial_function = {
	.name		= "serial",
	.cleanup	= serial_function_cleanup,
	.bind_config	= serial_function_bind_config,
	.attributes	= serial_function_attributes,
};

/* ACM */
static char acm_transports[32];	/*enabled ACM ports - "tty[,sdio]"*/
static ssize_t acm_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
#if defined CONFIG_USB_G_LGE_ANDROID && defined CONFIG_LGE_PM
	struct android_dev *dev = _android_dev;
	if (dev->check_pif) {
		dev_info(dev->dev, "pif cable is plugged, not permitted\n");
		return -EPERM;
	}
#endif

	strlcpy(acm_transports, buff, sizeof(acm_transports));

	return size;
}

static DEVICE_ATTR(acm_transports, S_IWUSR, NULL, acm_transports_store);
static struct device_attribute *acm_function_attributes[] = {
		&dev_attr_acm_transports, NULL };

static void acm_function_cleanup(struct android_usb_function *f)
{
	gserial_cleanup();
}

static int acm_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	char *name;
	char buf[32], *b;
	int err = -1, i;
	static int acm_initialized, ports;

/* LGE_CHANGE_S [START] 2012.6.14 jaeho.cho@lge.com merge this condition from G1 TDR  */
#ifdef CONFIG_USB_G_LGE_ANDROID
	if (acm_initialized && ports)
#else
	if (acm_initialized)
#endif
/* LGE_CHANGE_S [END] 2012.6.14 jaeho.cho@lge.com merge this condition from G1 TDR  */
		goto bind_config;

	acm_initialized = 1;
	strlcpy(buf, acm_transports, sizeof(buf));
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");

		if (name) {
			err = acm_init_port(ports, name);
			if (err) {
				pr_err("acm: Cannot open port '%s'", name);
				goto out;
			}
			ports++;
		}
	}
	err = acm_port_setup(c);
	if (err) {
		pr_err("acm: Cannot setup transports");
		goto out;
	}

bind_config:
	for (i = 0; i < ports; i++) {
		err = acm_bind_config(c, i);
		if (err) {
			pr_err("acm: bind_config failed for port %d", i);
			goto out;
		}
	}

out:
	return err;
}
static struct android_usb_function acm_function = {
	.name		= "acm",
	.cleanup	= acm_function_cleanup,
	.bind_config	= acm_function_bind_config,
	.attributes	= acm_function_attributes,
};

/* ADB */
static int adb_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	return adb_setup();
}

static void adb_function_cleanup(struct android_usb_function *f)
{
	adb_cleanup();
}

static int adb_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return adb_bind_config(c);
}

static struct android_usb_function adb_function = {
	.name		= "adb",
	.init		= adb_function_init,
	.cleanup	= adb_function_cleanup,
	.bind_config	= adb_function_bind_config,
};

/* CCID */
static int ccid_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return ccid_setup();
}

static void ccid_function_cleanup(struct android_usb_function *f)
{
	ccid_cleanup();
}

static int ccid_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return ccid_bind_config(c);
}

static struct android_usb_function ccid_function = {
	.name		= "ccid",
	.init		= ccid_function_init,
	.cleanup	= ccid_function_cleanup,
	.bind_config	= ccid_function_bind_config,
};

static int mtp_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	return mtp_setup();
}

static void mtp_function_cleanup(struct android_usb_function *f)
{
	mtp_cleanup();
}

static int mtp_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return mtp_bind_config(c, false);
}

static int ptp_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	/* nothing to do - initialization is handled by mtp_function_init */
	return 0;
}

static void ptp_function_cleanup(struct android_usb_function *f)
{
	/* nothing to do - cleanup is handled by mtp_function_cleanup */
}

static int ptp_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return mtp_bind_config(c, true);
}

static int mtp_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return mtp_ctrlrequest(cdev, c);
}

static struct android_usb_function mtp_function = {
	.name		= "mtp",
	.init		= mtp_function_init,
	.cleanup	= mtp_function_cleanup,
	.bind_config	= mtp_function_bind_config,
	.ctrlrequest	= mtp_function_ctrlrequest,
};

/* PTP function is same as MTP with slightly different interface descriptor */
static struct android_usb_function ptp_function = {
	.name		= "ptp",
	.init		= ptp_function_init,
	.cleanup	= ptp_function_cleanup,
	.bind_config	= ptp_function_bind_config,
};

#ifdef CONFIG_USB_G_LGE_ANDROID
struct ecm_function_config {
	u8      ethaddr[ETH_ALEN];
	u32     vendorID;
	char	manufacturer[256];
};

static int ecm_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct ecm_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;
	return 0;
}

static void ecm_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int ecm_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int ret;
	struct ecm_function_config *ecm = f->config;

	if (!ecm) {
		pr_err("%s: ecm_function_config\n", __func__);
		return -1;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		ecm->ethaddr[0], ecm->ethaddr[1], ecm->ethaddr[2],
		ecm->ethaddr[3], ecm->ethaddr[4], ecm->ethaddr[5]);

	ret = gether_setup_name(c->cdev->gadget, ecm->ethaddr, "usb");
	if (ret) {
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}

	return ecm_bind_config(c, ecm->ethaddr);
}

static void ecm_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	gether_cleanup();
}

static ssize_t ecm_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *ecm = f->config;
	return sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		ecm->ethaddr[0], ecm->ethaddr[1], ecm->ethaddr[2],
		ecm->ethaddr[3], ecm->ethaddr[4], ecm->ethaddr[5]);
}

static ssize_t ecm_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *ecm = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&ecm->ethaddr[0], (int *)&ecm->ethaddr[1],
		    (int *)&ecm->ethaddr[2], (int *)&ecm->ethaddr[3],
		    (int *)&ecm->ethaddr[4], (int *)&ecm->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ethaddr, S_IRUGO | S_IWUSR, ecm_ethaddr_show,
					       ecm_ethaddr_store);

static struct device_attribute *ecm_function_attributes[] = {
	&dev_attr_ethaddr,
	NULL
};

static struct android_usb_function ecm_function = {
	.name		= "ecm",
	.init		= ecm_function_init,
	.cleanup	= ecm_function_cleanup,
	.bind_config	= ecm_function_bind_config,
	.unbind_config	= ecm_function_unbind_config,
	.attributes	= ecm_function_attributes,
};

#else /* google original: rndis */

struct rndis_function_config {
	u8      ethaddr[ETH_ALEN];
	u32     vendorID;
	char	manufacturer[256];
	bool	wceis;
};

static int rndis_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct rndis_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;
	return 0;
}

static void rndis_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int rndis_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int ret;
	struct rndis_function_config *rndis = f->config;

	if (!rndis) {
		pr_err("%s: rndis_pdata\n", __func__);
		return -1;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);

	ret = gether_setup_name(c->cdev->gadget, rndis->ethaddr, "rndis");
	if (ret) {
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}

	if (rndis->wceis) {
		/* "Wireless" RNDIS; auto-detected by Windows */
		rndis_iad_descriptor.bFunctionClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_iad_descriptor.bFunctionSubClass = 0x01;
		rndis_iad_descriptor.bFunctionProtocol = 0x03;
		rndis_control_intf.bInterfaceClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_control_intf.bInterfaceSubClass =	 0x01;
		rndis_control_intf.bInterfaceProtocol =	 0x03;
	}

	return rndis_bind_config(c, rndis->ethaddr, rndis->vendorID,
				    rndis->manufacturer);
}

static void rndis_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	gether_cleanup();
}

static ssize_t rndis_manufacturer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%s\n", config->manufacturer);
}

static ssize_t rndis_manufacturer_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	if (size >= sizeof(config->manufacturer))
		return -EINVAL;
	if (sscanf(buf, "%255s", config->manufacturer) == 1)
		return size;
	return -1;
}

static DEVICE_ATTR(manufacturer, S_IRUGO | S_IWUSR, rndis_manufacturer_show,
						    rndis_manufacturer_store);

static ssize_t rndis_wceis_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->wceis);
}

static ssize_t rndis_wceis_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%d", &value) == 1) {
		config->wceis = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(wceis, S_IRUGO | S_IWUSR, rndis_wceis_show,
					     rndis_wceis_store);

static ssize_t rndis_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;
	return snprintf(buf, PAGE_SIZE, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);
}

static ssize_t rndis_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&rndis->ethaddr[0], (int *)&rndis->ethaddr[1],
		    (int *)&rndis->ethaddr[2], (int *)&rndis->ethaddr[3],
		    (int *)&rndis->ethaddr[4], (int *)&rndis->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ethaddr, S_IRUGO | S_IWUSR, rndis_ethaddr_show,
					       rndis_ethaddr_store);

static ssize_t rndis_vendorID_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%04x\n", config->vendorID);
}

static ssize_t rndis_vendorID_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%04x", &value) == 1) {
		config->vendorID = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(vendorID, S_IRUGO | S_IWUSR, rndis_vendorID_show,
						rndis_vendorID_store);

static struct device_attribute *rndis_function_attributes[] = {
	&dev_attr_manufacturer,
	&dev_attr_wceis,
	&dev_attr_ethaddr,
	&dev_attr_vendorID,
	NULL
};

static struct android_usb_function rndis_function = {
	.name		= "rndis",
	.init		= rndis_function_init,
	.cleanup	= rndis_function_cleanup,
	.bind_config	= rndis_function_bind_config,
	.unbind_config	= rndis_function_unbind_config,
	.attributes	= rndis_function_attributes,
};
#endif /* CONFIG_USB_G_LGE_ANDROID */

#ifdef CONFIG_USB_G_LGE_ANDROID
static const char lge_vendor_name[] = "LGE";
static const char lge_product_name[] = CONFIG_USB_G_LGE_ANDROID_STORAGE_NAME;
#endif

struct mass_storage_function_config {
	struct fsg_config fsg;
	struct fsg_common *common;
};

static int mass_storage_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	struct mass_storage_function_config *config;
	struct fsg_common *common;
	int err;

	config = kzalloc(sizeof(struct mass_storage_function_config),
								GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	config->fsg.nluns = 1;
	config->fsg.luns[0].removable = 1;
#ifdef CONFIG_USB_G_LGE_ANDROID
	config->fsg.vendor_name = lge_vendor_name;
	config->fsg.product_name = lge_product_name;
#endif

	common = fsg_common_init(NULL, cdev, &config->fsg);
	if (IS_ERR(common)) {
		kfree(config);
		return PTR_ERR(common);
	}

	err = sysfs_create_link(&f->dev->kobj,
				&common->luns[0].dev.kobj,
				"lun");
	if (err) {
		fsg_common_release(&common->ref);
		kfree(config);
		return err;
	}

	config->common = common;
	f->config = config;
	return 0;
}

static void mass_storage_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int mass_storage_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct mass_storage_function_config *config = f->config;
	return fsg_bind_config(c->cdev, c, config->common);
}

static ssize_t mass_storage_inquiry_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%s\n", config->common->inquiry_string);
}

static ssize_t mass_storage_inquiry_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	if (size >= sizeof(config->common->inquiry_string))
		return -EINVAL;
	if (sscanf(buf, "%28s", config->common->inquiry_string) != 1)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR(inquiry_string, S_IRUGO | S_IWUSR,
					mass_storage_inquiry_show,
					mass_storage_inquiry_store);

static struct device_attribute *mass_storage_function_attributes[] = {
	&dev_attr_inquiry_string,
	NULL
};

static struct android_usb_function mass_storage_function = {
	.name		= "mass_storage",
	.init		= mass_storage_function_init,
	.cleanup	= mass_storage_function_cleanup,
	.bind_config	= mass_storage_function_bind_config,
	.attributes	= mass_storage_function_attributes,
};

#ifdef CONFIG_USB_G_LGE_ANDROID_AUTORUN
/* virtual cdrom usb gadget for autorun */
struct cdrom_storage_function_config {
	struct fsg_config fsg;
	struct fsg_common *common;
};

static const char cdrom_storage_kthread_name[] = "kcdrom-storaged";
static const char cdrom_lun_format[] = "clun%d";

static int cdrom_storage_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	struct cdrom_storage_function_config *config;
	struct fsg_common *common;
	int err;

	config = kzalloc(sizeof(struct cdrom_storage_function_config),
			GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	config->fsg.nluns = 1;
	config->fsg.luns[0].removable = 1;
	config->fsg.luns[0].cdrom = 1; /* cdrom(read only) flag */
	config->fsg.thread_name = cdrom_storage_kthread_name;
	config->fsg.lun_name_format = cdrom_lun_format;
	config->fsg.vendor_name = lge_vendor_name;
	config->fsg.product_name = lge_product_name;

	common = fsg_common_init(NULL, cdev, &config->fsg);
	if (IS_ERR(common)) {
		kfree(config);
		return PTR_ERR(common);
	}

	err = sysfs_create_link(&f->dev->kobj,
			&common->luns[0].dev.kobj,
			"lun");
	if (err) {
		fsg_common_release(&common->ref);
		kfree(config);
		return err;
	}

	config->common = common;
	f->config = config;
	return 0;
}

static int cdrom_storage_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	struct cdrom_storage_function_config *config = f->config;
	return csg_bind_config(c->cdev, c, config->common);
}

static void cdrom_storage_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static ssize_t cdrom_storage_inquiry_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct cdrom_storage_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%s\n", config->common->inquiry_string);
}

static ssize_t cdrom_storage_inquiry_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct cdrom_storage_function_config *config = f->config;
	if (size >= sizeof(config->common->inquiry_string))
		return -EINVAL;
	if (strlcpy(config->common->inquiry_string, buf,
				sizeof(config->common->inquiry_string)) == 0)
		return -EINVAL;

	return size;
}

static DEVICE_ATTR(cdrom_inquiry_string, S_IRUGO | S_IWUSR,
		cdrom_storage_inquiry_show,
		cdrom_storage_inquiry_store);

/* we borrow another parts from mass storage function driver */
static struct device_attribute *cdrom_storage_function_attributes[] = {
	&dev_attr_cdrom_inquiry_string,
	NULL
};

static struct android_usb_function cdrom_storage_function = {
	.name		= "cdrom_storage",
	.init		= cdrom_storage_function_init,
	.cleanup	= cdrom_storage_function_cleanup,
	.bind_config	= cdrom_storage_function_bind_config,
	.attributes	= cdrom_storage_function_attributes,
};
#endif /* CONFIG_USB_G_LGE_ANDROID_AUTORUN */
#ifdef CONFIG_LGE_USB_CHARGE_ONLY
/* charge only mode */
static int charge_only_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	return charge_only_setup();
}

static void charge_only_function_cleanup(struct android_usb_function *f)
{
	charge_only_cleanup();
}

static int charge_only_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return charge_only_bind_config(c);
}

static struct android_usb_function charge_only_function = {
	.name		= "charge_only",
	.init		= charge_only_function_init,
	.cleanup	= charge_only_function_cleanup,
	.bind_config	= charge_only_function_bind_config,
};
#endif
static int accessory_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return acc_setup();
}

static void accessory_function_cleanup(struct android_usb_function *f)
{
	acc_cleanup();
}

static int accessory_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return acc_bind_config(c);
}

static int accessory_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return acc_ctrlrequest(cdev, c);
}

static struct android_usb_function accessory_function = {
	.name		= "accessory",
	.init		= accessory_function_init,
	.cleanup	= accessory_function_cleanup,
	.bind_config	= accessory_function_bind_config,
	.ctrlrequest	= accessory_function_ctrlrequest,
};


static struct android_usb_function *supported_functions[] = {
	&rmnet_smd_function,
	&rmnet_sdio_function,
	&rmnet_smd_sdio_function,
	&rmnet_function,
	&diag_function,
	&serial_function,
	&adb_function,
	&ccid_function,
	&acm_function,
	&mtp_function,
	&ptp_function,
#ifdef CONFIG_USB_G_LGE_ANDROID
	&ecm_function,
#else /* google original: rndis */
	&rndis_function,
#endif
	&mass_storage_function,
#ifdef CONFIG_USB_G_LGE_ANDROID_AUTORUN
	&cdrom_storage_function,
#endif
#ifdef CONFIG_LGE_USB_CHARGE_ONLY
	&charge_only_function,
#endif
	&accessory_function,
	NULL
};


static int android_init_functions(struct android_usb_function **functions,
				  struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function *f;
	struct device_attribute **attrs;
	struct device_attribute *attr;
	int err = 0;
	int index = 0;

	for (; (f = *functions++); index++) {
		f->dev_name = kasprintf(GFP_KERNEL, "f_%s", f->name);
		f->dev = device_create(android_class, dev->dev,
				MKDEV(0, index), f, f->dev_name);
		if (IS_ERR(f->dev)) {
			pr_err("%s: Failed to create dev %s", __func__,
							f->dev_name);
			err = PTR_ERR(f->dev);
			goto err_create;
		}

		if (f->init) {
			err = f->init(f, cdev);
			if (err) {
				pr_err("%s: Failed to init %s", __func__,
								f->name);
				goto err_out;
			}
		}

		attrs = f->attributes;
		if (attrs) {
			while ((attr = *attrs++) && !err)
				err = device_create_file(f->dev, attr);
		}
		if (err) {
			pr_err("%s: Failed to create function %s attributes",
					__func__, f->name);
			goto err_out;
		}
	}
	return 0;

err_out:
	device_destroy(android_class, f->dev->devt);
err_create:
	kfree(f->dev_name);
	return err;
}

static void android_cleanup_functions(struct android_usb_function **functions)
{
	struct android_usb_function *f;

	while (*functions) {
		f = *functions++;

		if (f->dev) {
			device_destroy(android_class, f->dev->devt);
			kfree(f->dev_name);
		}

		if (f->cleanup)
			f->cleanup(f);
	}
}

static int
android_bind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;
	int ret;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		ret = f->bind_config(f, c);
		if (ret) {
			pr_err("%s: %s failed", __func__, f->name);
			return ret;
		}
	}
	return 0;
}

static void
android_unbind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->unbind_config)
			f->unbind_config(f, c);
	}
}

static int android_enable_function(struct android_dev *dev, char *name)
{
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;
	while ((f = *functions++)) {
		if (!strcmp(name, f->name)) {
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
			return 0;
		}
	}
	return -EINVAL;
}

/*-------------------------------------------------------------------------*/
/* /sys/class/android_usb/android%d/ interface */

static ssize_t remote_wakeup_show(struct device *pdev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",
			!!(android_config_driver.bmAttributes &
				USB_CONFIG_ATT_WAKEUP));
}

static ssize_t remote_wakeup_store(struct device *pdev,
		struct device_attribute *attr, const char *buff, size_t size)
{
	int enable = 0;

	sscanf(buff, "%d", &enable);

	pr_debug("android_usb: %s remote wakeup\n",
			enable ? "enabling" : "disabling");

	if (enable)
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	else
		android_config_driver.bmAttributes &= ~USB_CONFIG_ATT_WAKEUP;

	return size;
}

static ssize_t
functions_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct android_usb_function *f;
	char *buff = buf;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list)
		buff += snprintf(buff, PAGE_SIZE, "%s,", f->name);
	if (buff != buf)
		*(buff-1) = '\n';
	return buff - buf;
}

static ssize_t
functions_store(struct device *pdev, struct device_attribute *attr,
			       const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	char *name;
	char buf[256], *b;
	int err;

	dev_info(dev->dev, "request function list: %s\n", buff);
#if defined CONFIG_USB_G_LGE_ANDROID && defined CONFIG_LGE_PM
	if (dev->check_pif) {
		dev_info(dev->dev, "pif cable is plugged, not permitted\n");
		return -EPERM;
	}
#endif

	INIT_LIST_HEAD(&dev->enabled_functions);

	strlcpy(buf, buff, sizeof(buf));
	b = strim(buf);

/* LGE_CHANGE_S [START] 2012.4.17 jaeho.cho@lge.com merge charge only feature from G1 TDR  */
#ifdef CONFIG_LGE_USB_CHARGE_ONLY
	dev->check_charge_only = false;
	if (!strcmp(b, "charge_only"))
		dev->check_charge_only = true;
#endif
/* LGE_CHANGE_S [END] 2012.4.17 jaeho.cho@lge.com merge charge only feature from G1 TDR */

	while (b) {
		name = strsep(&b, ",");
		if (name) {
			err = android_enable_function(dev, name);
			if (err)
				pr_err("android_usb: Cannot enable '%s'", name);
		}
	}

	return size;
}

static ssize_t enable_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", dev->enabled);
}

static ssize_t enable_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	int enabled = 0;

	dev_info(dev->dev, "gadget enable(%s)\n", buff);
#if defined CONFIG_USB_G_LGE_ANDROID && defined CONFIG_LGE_PM
	if (dev->check_pif) {
		dev_info(dev->dev, "pif cable is plugged, not permitted\n");
		return -EPERM;
	}
#endif

	sscanf(buff, "%d", &enabled);
	if (enabled && !dev->enabled) {
		/* update values in composite driver's copy of device descriptor */
		cdev->desc.idVendor = device_desc.idVendor;
		cdev->desc.idProduct = device_desc.idProduct;
		cdev->desc.bcdDevice = device_desc.bcdDevice;
		cdev->desc.bDeviceClass = device_desc.bDeviceClass;
		cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
		cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
/* LGE_CHANGE_S [START] 2012.4.17 jaeho.cho@lge.com merge charge only feature from G1 TDR  */
#ifdef CONFIG_LGE_USB_CHARGE_ONLY
		if (dev->check_charge_only) {
			cdev->desc.iSerialNumber = 0;
			cdev->desc.iProduct = strings_dev[CHARGE_ONLY_STRING_IDX].id;
		} else {
			cdev->desc.iSerialNumber = strings_dev[STRING_SERIAL_IDX].id;
			cdev->desc.iProduct = strings_dev[STRING_PRODUCT_IDX].id;
		}
#endif
/* LGE_CHANGE_S [END] 2012.4.17 jaeho.cho@lge.com merge charge only feature from G1 TDR */
		if (usb_add_config(cdev, &android_config_driver,
							android_bind_config))
			return size;

		usb_gadget_connect(cdev->gadget);
		dev->enabled = true;
	} else if (!enabled && dev->enabled) {
		usb_gadget_disconnect(cdev->gadget);
		usb_remove_config(cdev, &android_config_driver);
		dev->enabled = false;
	} else {
		pr_err("android_usb: already %s\n",
				dev->enabled ? "enabled" : "disabled");
	}
	return size;
}

static ssize_t state_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	char *state = "DISCONNECTED";
	unsigned long flags;

	if (!cdev)
		goto out;

	spin_lock_irqsave(&cdev->lock, flags);
        if (cdev->config)
		state = "CONFIGURED";
	else if (dev->connected)
		state = "CONNECTED";
	spin_unlock_irqrestore(&cdev->lock, flags);
out:
	return snprintf(buf, PAGE_SIZE, "%s\n", state);
}

#if defined CONFIG_USB_G_LGE_ANDROID && defined CONFIG_LGE_PM
/* if pif cable is plugged, not allow request from user space */
#define DESCRIPTOR_ATTR(field, format_string)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE,					\
			format_string, device_desc.field);		\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)		       		\
{									\
	int value;					       		\
	struct android_dev *adev = dev_get_drvdata(dev); \
	if (adev->check_pif) \
		return -EPERM; \
	if (sscanf(buf, format_string, &value) == 1) {			\
		device_desc.field = value;				\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#define DESCRIPTOR_STRING_ATTR(field, buffer)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE, "%s", buffer);			\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)		       		\
{									\
	struct android_dev *adev = dev_get_drvdata(dev); \
	if (adev->check_pif) \
		return -EPERM; \
	if (size >= sizeof(buffer)) \
		return -EINVAL;			\
	if (sscanf(buf, "%255s", buffer) == 1) {			\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);
#else /* google original */
#define DESCRIPTOR_ATTR(field, format_string)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE,					\
			format_string, device_desc.field);		\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)		       		\
{									\
	int value;					       		\
	if (sscanf(buf, format_string, &value) == 1) {			\
		device_desc.field = value;				\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#define DESCRIPTOR_STRING_ATTR(field, buffer)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE, "%s", buffer);			\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)		       		\
{									\
	if (size >= sizeof(buffer)) return -EINVAL;			\
	if (sscanf(buf, "%255s", buffer) == 1) {			\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);
#endif /* CONFIG_USB_G_LGE_ANDROID */

DESCRIPTOR_ATTR(idVendor, "%04x\n")
DESCRIPTOR_ATTR(idProduct, "%04x\n")
DESCRIPTOR_ATTR(bcdDevice, "%04x\n")
DESCRIPTOR_ATTR(bDeviceClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceSubClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceProtocol, "%d\n")
DESCRIPTOR_STRING_ATTR(iManufacturer, manufacturer_string)
DESCRIPTOR_STRING_ATTR(iProduct, product_string)
DESCRIPTOR_STRING_ATTR(iSerial, serial_string)
#ifdef CONFIG_USB_G_LGE_ANDROID
DESCRIPTOR_ATTR(iSerialNumber, "%d\n")
#endif
#ifdef CONFIG_USB_G_LGE_SERIALNO_REDIRECTION
DESCRIPTOR_STRING_ATTR(iSerial_redi, serial_string_redi)
#endif

static DEVICE_ATTR(functions, S_IRUGO | S_IWUSR, functions_show, functions_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);
static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);
static DEVICE_ATTR(remote_wakeup, S_IRUGO | S_IWUSR,
		remote_wakeup_show, remote_wakeup_store);

static struct device_attribute *android_usb_attributes[] = {
	&dev_attr_idVendor,
	&dev_attr_idProduct,
	&dev_attr_bcdDevice,
	&dev_attr_bDeviceClass,
	&dev_attr_bDeviceSubClass,
	&dev_attr_bDeviceProtocol,
	&dev_attr_iManufacturer,
	&dev_attr_iProduct,
	&dev_attr_iSerial,
#ifdef CONFIG_USB_G_LGE_ANDROID
	&dev_attr_iSerialNumber,
#endif
#ifdef CONFIG_USB_G_LGE_SERIALNO_REDIRECTION
	&dev_attr_iSerial_redi,
#endif
	&dev_attr_functions,
	&dev_attr_enable,
	&dev_attr_state,
	&dev_attr_remote_wakeup,
	NULL
};

//LGE_CHANGE [jinhwan.do@lge.com][2012.04.13]get USB serial number, merge from LS696 [Start]
#ifdef CONFIG_LGE_USB_EXPORT_SERIAL_NUMBER
void android_get_serial_number(char *snum)
{
	snprintf(snum, MAX_SERIAL_LEN, iSerialNumber);
}
EXPORT_SYMBOL(android_get_serial_number);
#endif
//LGE_CHANGE [jinhwan.do@lge.com][2012.04.13]get USB serial number, merge from LS696 [End]

/* LGE_CHANGE_S [START] 2012.6.28 jaeho.cho@lge.com provide usb pid for external module  */
#if defined CONFIG_USB_G_LGE_ANDROID
unsigned short get_current_usb_pid(void)
{
    return device_desc.idProduct;
}
EXPORT_SYMBOL(get_current_usb_pid);
#endif
/* LGE_CHANGE_S [END] 2012.6.28 jaeho.cho@lge.com provide usb pid for external module  */

/*-------------------------------------------------------------------------*/
/* Composite driver */

#if defined CONFIG_USB_G_LGE_ANDROID && defined CONFIG_LGE_PM

#define LGE_PIF_VID 0x1004
#define LGE_PIF_PID 0x6000
#define LGE_PIF_SN 0

static void android_lge_factory_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	char lge_factory_composition[256];
	char *name, *b;
	int ret, err, value;

	/* update values in composite driver's copy of device descriptor */
	value = lgeusb_get_vendor_id();
	if (value < 0)
		value = LGE_PIF_VID;
	device_desc.idVendor = value;

	value = lgeusb_get_factory_pid();
	if (value < 0)
		value = LGE_PIF_PID;
	device_desc.idProduct = value;

	value = lgeusb_get_serial_number();
	if (value < 0)
		value = LGE_PIF_SN;
	device_desc.iSerialNumber = value;

	/*XXX: should we create sysfs with below 3 field? */
	device_desc.bDeviceClass = 2;
	device_desc.bDeviceSubClass = 0;
	device_desc.bDeviceProtocol = 0;

	cdev->desc.idVendor = device_desc.idVendor;
	cdev->desc.idProduct = device_desc.idProduct;
	cdev->desc.bcdDevice = device_desc.bcdDevice;
	cdev->desc.bDeviceClass = device_desc.bDeviceClass;
	cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
	cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;

	/*XXX: modem & diag specific configuration */
	strncpy(acm_transports, "tty", sizeof(acm_transports));
	strlcpy(diag_clients, "diag", sizeof(diag_clients));

	INIT_LIST_HEAD(&dev->enabled_functions);

	ret = lgeusb_get_factory_composition(lge_factory_composition);
	if (ret)
		strlcpy(lge_factory_composition, "acm,diag",
				sizeof(lge_factory_composition) - 1);

	b = strim(lge_factory_composition);
	while (b) {
		name = strsep(&b, ",");
		if (name) {
			err = android_enable_function(dev, name);
			if (err)
				pr_err("android_usb: Cannot enable '%s'", name);
		}
	}

	if (usb_add_config(cdev, &android_config_driver, android_bind_config))
		return;

	usb_gadget_connect(cdev->gadget);
	dev->enabled = true;
}
#endif /* CONFIG_USB_G_LGE_ANDROID && CONFIG_LGE_PM */

static int android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret = 0;

	ret = android_bind_enabled_functions(dev, c);
	if (ret)
		return ret;

	return 0;
}

static void android_unbind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	android_unbind_enabled_functions(dev, c);
}

//LGE_CHANGE jinhwan.do 20120430 Slate command's concept is cahnged, USB Serial number is MEID's decimal value [Start]
#ifdef CONFIG_LGE_USB_EXPORT_SERIAL_NUMBER
extern int do_get_usb_serial_number(char *serial_number);
extern void msm_get_MEID_type(char* sMeid, char* dec_sMeid);
#endif
//LGE_CHANGE jinhwan.do 20120430 Slate command's concept is cahnged, USB Serial number is MEID's decimal value [End]

/* LGE_CHANGE_S [START] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */ 
#ifdef CONFIG_LGE_DIAG_USB_PERMANENT_LOCK
int usb_lock_initialized = 0;
#endif
/* LGE_CHANGE_S [END] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */

static int android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum, id, ret;
#ifdef CONFIG_USB_G_LGE_ANDROID
	char lge_product[256];
	char lge_manufacturer[256];
#endif

	usb_gadget_disconnect(gadget);

	ret = android_init_functions(dev->functions, cdev);
	if (ret)
		return ret;

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

#ifdef CONFIG_USB_G_LGE_ANDROID
	/* Default string as LGE products */
	ret = lgeusb_get_manufacturer_name(lge_manufacturer);
	if (!ret)
		strlcpy(manufacturer_string, lge_manufacturer,
				sizeof(manufacturer_string) - 1);
	else
		strlcpy(manufacturer_string, "LG Electronics Inc.",
				sizeof(manufacturer_string) - 1);

	ret = lgeusb_get_product_name(lge_product);
	if (!ret)
		strlcpy(product_string, lge_product, sizeof(product_string) - 1);
	else
		strlcpy(product_string, "LGE Android Phone",
				sizeof(product_string) - 1);
#else
	/* Default strings - should be updated by userspace */
	strlcpy(manufacturer_string, "Android",
		sizeof(manufacturer_string) - 1);
	strlcpy(product_string, "Android", sizeof(product_string) - 1);
#endif
//LGE_CHANGE jinhwan.do 20120430 Slate command's concept is cahnged, USB Serial number is MEID's decimal value [Start]
#ifdef CONFIG_LGE_USB_EXPORT_SERIAL_NUMBER
	msm_get_MEID_type(hex_meid, dec_meid);
	//strlcpy(serial_string, dec_meid, sizeof(serial_string) - 1);	
	strlcpy(serial_string, hex_meid, sizeof(serial_string) - 1);	
#else
	strlcpy(serial_string, "0123456789ABCDEF", sizeof(serial_string) - 1);
#endif
//LGE_CHANGE jinhwan.do 20120430 Slate command's concept is cahnged, USB Serial number is MEID's decimal value [End]

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

/* LGE_CHANGE_S [START] 2012.4.17 jaeho.cho@lge.com merge charge only feature from G1 TDR  */
#ifdef CONFIG_LGE_USB_CHARGE_ONLY
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[CHARGE_ONLY_STRING_IDX].id = id;
	sprintf(charge_only_string, "USB Charge Only Interface");
#endif
/* LGE_CHANGE_S [END] 2012.4.17 jaeho.cho@lge.com merge charge only feature from G1 TDR */

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	usb_gadget_set_selfpowered(gadget);
	dev->cdev = cdev;

#if defined CONFIG_USB_G_LGE_ANDROID && defined CONFIG_LGE_PM
	/* XXX:
	 * Android ICS prohibits to change persist property
	 * before initialization of persist properties. Therefore,
	 * we cannot change to factory usb composition at userspace
	 * when pif cable is plugged.
	 */
/* LGE_CHANGE_S [START] 2012.2.26 jaeho.cho@lge.com support factory usb of the chipset below MSM8X60 */
#ifdef CONFIG_LGE_USB_FACTORY_BELOW_MSM8X60
	cable = lgeusb_get_pif_cable();

	lg_manual_test_mode = msm_get_manual_test_mode();
	
	if (cable == LT_CABLE_56K || cable == LT_CABLE_130K || cable == LT_CABLE_910K ||
		( cable == NOINIT_CABLE && lg_manual_test_mode)) {
#else
	if (lgeusb_get_pif_cable()) {
#endif
/* LGE_CHANGE_S [END] 2012.2.26 jaeho.cho@lge.com support factory usb of the chipset below MSM8X60 */

		dev_info(dev->dev, "pif cable is plugged, bind factory composition\n");
		dev->check_pif = true;
		android_lge_factory_bind(cdev);
	}
#endif

	return 0;
}

static int android_usb_unbind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;

	cancel_work_sync(&dev->work);
	android_cleanup_functions(dev->functions);
	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.unbind		= android_usb_unbind,
};

static int
android_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *c)
{
	struct android_dev		*dev = _android_dev;
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct usb_request		*req = cdev->req;
	struct android_usb_function	*f;
	int value = -EOPNOTSUPP;
	unsigned long flags;

	req->zero = 0;
	req->complete = composite_setup_complete;
	req->length = 0;
	gadget->ep0->driver_data = cdev;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->ctrlrequest) {
			value = f->ctrlrequest(f, cdev, c);
			if (value >= 0)
				break;
		}
	}

	/* Special case the accessory function.
	 * It needs to handle control requests before it is enabled.
	 */
	if (value < 0)
		value = acc_ctrlrequest(cdev, c);

	if (value < 0)
		value = composite_setup(gadget, c);

	spin_lock_irqsave(&cdev->lock, flags);
	if (!dev->connected) {
		dev->connected = 1;
		schedule_work(&dev->work);
	}
	else if (c->bRequest == USB_REQ_SET_CONFIGURATION && cdev->config) {
		schedule_work(&dev->work);
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	return value;
}

static void android_disconnect(struct usb_gadget *gadget)
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	unsigned long flags;

	composite_disconnect(gadget);

	spin_lock_irqsave(&cdev->lock, flags);
	dev->connected = 0;
	schedule_work(&dev->work);
	spin_unlock_irqrestore(&cdev->lock, flags);
}

static int android_create_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;
	int err;

	dev->dev = device_create(android_class, NULL,
					MKDEV(0, 0), NULL, "android0");
	if (IS_ERR(dev->dev))
		return PTR_ERR(dev->dev);

	dev_set_drvdata(dev->dev, dev);

	while ((attr = *attrs++)) {
		err = device_create_file(dev->dev, attr);
		if (err) {
			device_destroy(android_class, dev->dev->devt);
			return err;
		}
	}
	return 0;
}

static void android_destroy_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;

	while ((attr = *attrs++))
		device_remove_file(dev->dev, attr);
	device_destroy(android_class, dev->dev->devt);
}

static int __devinit android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;
	
//LGE_CHANGE_S [jinhwan.do][2012.03.09]USB Access Lock Porting [Start]
#ifdef CONFIG_LGE_DIAG_USB_ACCESS_LOCK
	extern int get_usb_lock(void);
	get_usb_lock();
#endif
//LGE_CHANGE_S [jinhwan.do][2012.03.09]USB Access Lock Porting [End]

/* LGE_CHANGE_S [START] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */   
#ifdef CONFIG_LGE_DIAG_USB_ACCESS_LOCK_SHA1_ENCRYPTION   
{
	extern void get_usb_lock_key(char* usb_lock_key);
	char tmp_key[20]={0,};
	get_usb_lock_key(tmp_key);
}	
#endif
#ifdef CONFIG_LGE_DIAG_USB_PERMANENT_LOCK
{
	extern int user_diag_unlock_fail_cnt;
	extern int get_usb_unlock_fail_cnt(void);
	user_diag_unlock_fail_cnt = get_usb_unlock_fail_cnt();
}	
#endif
/* LGE_CHANGE_S [END] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */

	dev->pdata = pdata;

	return 0;
}

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb"},
};

static int __init init(void)
{
	struct android_dev *dev;
	int ret;

	android_class = class_create(THIS_MODULE, "android_usb");
	if (IS_ERR(android_class))
		return PTR_ERR(android_class);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		pr_err("%s(): Failed to alloc memory for android_dev\n",
				__func__);
		class_destroy(android_class);
		return -ENOMEM;
	}
	dev->functions = supported_functions;
	INIT_LIST_HEAD(&dev->enabled_functions);
	INIT_WORK(&dev->work, android_work);

#if defined CONFIG_USB_G_LGE_ANDROID && defined CONFIG_LGE_PM
	dev->check_pif = false;
#endif

	ret = android_create_device(dev);
	if (ret) {
		pr_err("%s(): android_create_device failed\n", __func__);
		goto err_dev;
	}
	_android_dev = dev;

	/* Override composite driver functions */
	composite_driver.setup = android_setup;
	composite_driver.disconnect = android_disconnect;

	ret = platform_driver_probe(&android_platform_driver, android_probe);
	if (ret) {
		pr_err("%s(): Failed to register android"
				 "platform driver\n", __func__);
		goto err_probe;
	}
	ret = usb_composite_probe(&android_usb_driver, android_bind);
	if (ret) {
		pr_err("%s(): Failed to register android"
				 "composite driver\n", __func__);
		platform_driver_unregister(&android_platform_driver);
		goto err_probe;
	}
	return ret;

err_probe:
	android_destroy_device(dev);
err_dev:
	kfree(dev);
	class_destroy(android_class);
	return ret;
}
#ifdef CONFIG_LGE_PM
device_initcall_sync(init);
#else
module_init(init);
#endif

static void __exit cleanup(void)
{
	usb_composite_unregister(&android_usb_driver);
	class_destroy(android_class);
	kfree(_android_dev);
	_android_dev = NULL;
}
module_exit(cleanup);
