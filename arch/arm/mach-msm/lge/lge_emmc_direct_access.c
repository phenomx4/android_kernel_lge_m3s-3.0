/*
 * arch/arm/mach-msm/lge/lge_emmc_direct_access.c
 *
 * Copyright (C) 2010 LGE, Inc
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

#include <asm/div64.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/errno.h>
//#include <linux/mtd/mtd.h>
#include <linux/sched.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>

#include <linux/slab.h> // for kmalloc

/* ADD 0013860: [FACTORY RESET] ERI file save */
#ifdef CONFIG_LGE_ERI_DOWNLOAD
#include <linux/kmod.h>
#include <linux/workqueue.h>
#endif

#include <mach/lg_backup_items.h>

#ifdef CONFIG_LGE_DLOAD_FACTORY_RESET
#include <mach/oem_rapi_client.h>
#endif

/* Some useful define used to access the MBR/EBR table */
//#define BLOCK_SIZE                0x200
#define TABLE_ENTRY_0             0x1BE
#define TABLE_ENTRY_1             0x1CE
#define TABLE_ENTRY_2             0x1DE
#define TABLE_ENTRY_3             0x1EE
#define TABLE_SIGNATURE           0x1FE
#define TABLE_ENTRY_SIZE          0x010

#define OFFSET_STATUS             0x00
#define OFFSET_TYPE               0x04
#define OFFSET_FIRST_SEC          0x08
#define OFFSET_SIZE               0x0C
#define COPYBUFF_SIZE             (1024 * 16)
#define BINARY_IN_TABLE_SIZE      (16 * 512)
#define MAX_FILE_ENTRIES          20

#define MMC_RCA 2

#define MAX_PARTITIONS 64

#if defined (CONFIG_LGE_LCD_K_CAL)
#define LCD_K_CAL_SIZE 6
static unsigned char lcd_buf[LCD_K_CAL_SIZE]={255,};
#endif

#define GET_LWORD_FROM_BYTE(x)    ((unsigned)*(x) | \
        ((unsigned)*((x)+1) << 8) | \
        ((unsigned)*((x)+2) << 16) | \
        ((unsigned)*((x)+3) << 24))

#define PUT_LWORD_TO_BYTE(x, y)   do{*(x) = (y) & 0xff;     \
    *((x)+1) = ((y) >> 8) & 0xff;     \
    *((x)+2) = ((y) >> 16) & 0xff;     \
    *((x)+3) = ((y) >> 24) & 0xff; }while(0)

#define GET_PAR_NUM_FROM_POS(x) ((((x) & 0x0000FF00) >> 8) + ((x) & 0x000000FF))

#define MMC_BOOT_TYPE 			0x48
#define MMC_RECOVERY_TYPE		0x60
#define MMC_LG_FOTA_TYPE		0x6B
#define MMC_MISC_TYPE			0x77

#define HOTPLUG_PARTITION_ID_EXT4_TYPE	0x83
#define MMC_CACHE_TYPE						HOTPLUG_PARTITION_ID_EXT4_TYPE
#define MMC_USERDATA_TYPE					HOTPLUG_PARTITION_ID_EXT4_TYPE

typedef struct MmcPartition MmcPartition;

static unsigned ext4_count = 0;
static char *ext4_partitions[] = {"persist", "cache", "system", "userdata", "userdata_bkp", "mpt", "tombstones", "drm", "carrier"};

//static unsigned vfat_count = 0;
//static char *vfat_partitions[] = {"modem", "NONE"};

struct MmcPartition {
    char *device_index;
    char *filesystem;
    char *name;
    unsigned dstatus;
    unsigned dtype ;
    unsigned dfirstsec;
    unsigned dsize;
};

typedef struct {
    MmcPartition *partitions;
    int partitions_allocd;
    int partition_count;
} MmcState;

static MmcState g_mmc_state = {
    NULL,   // partitions
    0,      // partitions_allocd
    -1      // partition_count
};

typedef struct {
	char ret[32];
} testmode_rsp_from_diag_type;

/* ADD 0013860: [FACTORY RESET] ERI file save */
/* make work queue so that rpc for eri does not affect to the factory reset */
#ifdef CONFIG_LGE_ERI_DOWNLOAD
extern void remote_eri_rpc(void);

static struct workqueue_struct *eri_dload_wq;
struct __eri_data {
    unsigned long flag;
    struct work_struct work;
};
static struct __eri_data eri_dload_data;

static void eri_dload_func(struct work_struct *work);
#endif

#ifdef CONFIG_LGE_DLOAD_FACTORY_RESET
extern void request_sync_cmd_local(unsigned long cmd);
//extern bool get_lge_reset_mode_dload_upgrade(void);
#endif

#ifdef CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB
extern int msm_get_manual_test_mode(void);
#endif


int lge_erase_block(int secnum, size_t size);
int lge_write_block(unsigned int secnum, unsigned char *buf, size_t size);
int lge_read_block(unsigned int secnum, unsigned char *buf, size_t size);

static int dummy_arg;
int boot_complete_info = 0;

static int boot_info_write(const char *val, struct kernel_param *kp)
{
	unsigned long flag=0;
	/* LGE ,[US730], Charging Logo Initial Commit */
	// Merge from m3s code related DLOAD Reset Boot Up (informing Dload reset complete to modem)
#ifdef CONFIG_LGE_DLOAD_RESET_BOOT_UP
	extern void firstboot_complete_inform(int info);
#endif
	if(val == NULL)
	{
		printk(KERN_ERR "%s, NULL buf\n", __func__);
		return -1;
	}
	
	flag = simple_strtoul(val,NULL,10);
	boot_complete_info = (int)flag;
	printk(KERN_INFO "[BOOT_INFO] %s, flag : %d\n", __func__, boot_complete_info);
	/* LGE ,[US730], Charging Logo Initial Commit */
	// Merge from m3s code related DLOAD Reset Boot Up
#ifdef CONFIG_LGE_DLOAD_RESET_BOOT_UP
	firstboot_complete_inform(boot_complete_info);
#endif

	return 0;
}

module_param_call(boot_info, boot_info_write, param_get_int, &boot_complete_info, S_IWUSR | S_IRUGO);

#ifdef CONFIG_LGE_DIAG_MFT
int spk_vol_level = 0;
module_param(spk_vol_level, int, S_IWUSR | S_IRUGO);
#endif

//LGE_CHANGE [yunjeong.kang][2012.06.27] MFT of LCD [Start]
int mft_lcd_input_data_1 = 0;
module_param(mft_lcd_input_data_1, int, S_IWUSR | S_IRUGO);
int mft_lcd_input_data_2 = 0;
module_param(mft_lcd_input_data_2, int, S_IWUSR | S_IRUGO);
int mft_lcd_input_data_3 = 0;
module_param(mft_lcd_input_data_3, int, S_IWUSR | S_IRUGO);
//LGE_CHANGE [yunjeong.kang][2012.06.27] MFT of LCD [END]

//LGE_CHANGE [jinhwan.do][2012.06.02] MFT of Camera/Camcorder error is fixed  [Start]
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-21 In Testmode 9.0 of  MFT Camera/Camcorder [Start]
int mft_avr_record_time = 0;
module_param(mft_avr_record_time, int, S_IWUSR | S_IRUGO);

int mft_cam_led_status = 0;
module_param(mft_cam_led_status, int, S_IWUSR | S_IRUGO);

int mft_avr_led_status = 0;
module_param(mft_avr_led_status, int, S_IWUSR | S_IRUGO);

int mft_cam_af_status = 0;
module_param(mft_cam_af_status, int, S_IWUSR | S_IRUGO);

int mft_avr_play_status = 0;
module_param(mft_avr_play_status, int, S_IWUSR | S_IRUGO);
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-21 In Testmode 9.0 of  MFT Camera/Camcorder [End]
//LGE_CHANGE [jinhwan.do][2012.06.02] MFT of Camera/Camcorder error is fixed  [End]

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-16, TestMode 9.0, get Acoustic Status [Start]
int acoustic_status = 0;
module_param(acoustic_status, int, S_IWUSR | S_IRUGO);
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-16, TestMode 9.0, get Acoustic Status [End]

int db_integrity_ready = 0;
module_param(db_integrity_ready, int, S_IWUSR | S_IRUGO);

int fpri_crc_ready = 0;
module_param(fpri_crc_ready, int, S_IWUSR | S_IRUGO);

int file_crc_ready = 0;
module_param(file_crc_ready, int, S_IWUSR | S_IRUGO);

int code_partition_crc_ready = 0;
module_param(code_partition_crc_ready, int, S_IWUSR | S_IRUGO);

int total_crc_ready = 0;	
module_param(total_crc_ready, int, S_IWUSR | S_IRUGO);

int db_dump_ready = 0;	 //hojung7.kim@lge.com Add  (MS910)
module_param(db_dump_ready, int, S_IWUSR | S_IRUGO);

int db_copy_ready = 0;	//hojung7.kim@lge.com Add  (MS910)
module_param(db_copy_ready, int, S_IWUSR | S_IRUGO);

//LGE_CHANGE_S [US730] [TestMode] [jinhwan.do@lge.com] 2012-02-16, Test Mode Porting of FOTA ID CHECK [Start]
//LGE_FOTA_ID_CHECK
int fota_id_check = 0;
module_param(fota_id_check, int, S_IWUSR | S_IRUGO);

unsigned char fota_id_read[30] = "0";
module_param_string(fota_id_read, fota_id_read, 30, S_IWUSR | S_IRUGO);
//LGE_CHANGE_S [US730] [TestMode] [jinhwan.do@lge.com] 2012-02-16, Test Mode Porting of FOTA ID CHECK [END]

testmode_rsp_from_diag_type integrity_ret;
static int integrity_ret_write(const char *val, struct kernel_param *kp)
{
	memcpy(integrity_ret.ret, val, 32);
	return 0;
}
static int integrity_ret_read(char *buf, struct kernel_param *kp)
{
	memcpy(buf, integrity_ret.ret, 32);
	return 0;
}

module_param_call(integrity_ret, integrity_ret_write, integrity_ret_read, &dummy_arg, S_IWUSR | S_IRUGO);

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-03-14, SWVersion [Start]
testmode_rsp_from_diag_type sp_ver;
static int sp_ver_write(const char *val, struct kernel_param *kp)
{
	memcpy(sp_ver.ret, val, 32);
	return 0;
}
static int sp_ver_read(char *buf, struct kernel_param *kp)
{
	memcpy(buf, sp_ver.ret, 32);
	return 0;
}

module_param_call(sp_ver, sp_ver_write, sp_ver_read, &dummy_arg, S_IWUSR | S_IRUGO);
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-03-14, SWVersion [End]

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-02, get Provisioned for test mode [Start]
testmode_rsp_from_diag_type test_get_provisioned;
static int test_get_provisioned_write(const char *val, struct kernel_param *kp)
{
	memcpy(test_get_provisioned.ret, val, 32);
	return 0;
}
static int test_get_provisioned_read(char *buf, struct kernel_param *kp)
{
	memcpy(buf, test_get_provisioned.ret, 32);
	return 0;
}

module_param_call(test_get_provisioned, test_get_provisioned_write, test_get_provisioned_read, &dummy_arg, S_IWUSR | S_IRUGO);
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-02, get Provisioned for test mode [End]

//LGE_FOTA_ID_CHECK
int fota_cmd_ret_ready = 0;
module_param(fota_cmd_ret_ready, int, S_IWUSR | S_IRUGO);

char fota_cmd_ret[100];
static int fota_cmd_ret_write(const char *val, struct kernel_param *kp)
{
	memcpy(fota_cmd_ret, val, strlen(val));
	return 0;
}
static int fota_cmd_ret_read(char *buf, struct kernel_param *kp)
{
	memcpy(buf, fota_cmd_ret, sizeof(fota_cmd_ret));
	return 0;
}

module_param_call(fota_cmd_ret, fota_cmd_ret_write, fota_cmd_ret_read, &dummy_arg, S_IWUSR | S_IRUGO);

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-04, Manual Mode value backup [Start]
int manual_mode_flag = 0;
//module_param(manual_mode_on, int, S_IWUSR | S_IRUGO);
static int manual_mode_flag_read(char *buf, struct kernel_param *kp)
{
#ifdef CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB
	manual_mode_flag = msm_get_manual_test_mode();
#endif
	sprintf(buf,"%d\n",manual_mode_flag);
	return sizeof(manual_mode_flag);
}
module_param_call(manual_mode_on, NULL, manual_mode_flag_read, &manual_mode_flag, S_IWUSR | S_IRUGO);
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-04, Manual Mode value backup [End]

// MOD 0009484: [FactoryReset] Enable FactoryReset
#define FACTORY_RESET_STR_SIZE 11
#define FACTORY_RESET_STR "FACT_RESET_"
// END: 0009484 
#define MMC_DEVICENAME "/dev/block/mmcblk0"

static char *lge_strdup(const char *str)
{
	size_t len;
	char *copy;
	
	len = strlen(str) + 1;
	copy = kmalloc(len, GFP_KERNEL);
	if (copy == NULL)
		return NULL;
	memcpy(copy, str, len);
	return copy;
}

int lge_erase_block(int bytes_pos, size_t erase_size)
{
	unsigned char *erasebuf;
	unsigned written = 0;
	erasebuf = kmalloc(erase_size, GFP_KERNEL);
	// allocation exception handling
	if(!erasebuf)
	{
		printk("%s, allocation failed, expected size : %d\n", __func__, (int)erase_size);
		return 0;
	}
	memset(erasebuf, 0xff, erase_size);
	written += lge_write_block(bytes_pos, erasebuf, erase_size);

	kfree(erasebuf);

	return written;
}
EXPORT_SYMBOL(lge_erase_block);

/* MOD 0014570: [FACTORY RESET] change system call to filp function for handling the flag */
int lge_write_block(unsigned int bytes_pos, unsigned char *buf, size_t size)
{
	struct file *phMscd_Filp = NULL;
	mm_segment_t old_fs;
	unsigned int write_bytes = 0;

	// exception handling
	if((buf == NULL) || size <= 0)
	{
		printk(KERN_ERR "%s, NULL buffer or NULL size : %d\n", __func__, (int)size);
		return 0;
	}
		
	old_fs=get_fs();
	set_fs(get_ds());

	// change from sys operation to flip operation, do not use system call since this routine is also system call service.
	// set O_SYNC for synchronous file io
	phMscd_Filp = filp_open(MMC_DEVICENAME, O_RDWR | O_SYNC, 0);
	if(IS_ERR(phMscd_Filp))
	{
		printk(KERN_ERR "%s, Can not access 0x%x bytes postition\n", __func__, (int)bytes_pos );
		goto write_fail;
	}

	phMscd_Filp->f_pos = (loff_t)bytes_pos;
	write_bytes = phMscd_Filp->f_op->write(phMscd_Filp, buf, size, &phMscd_Filp->f_pos);

	if(write_bytes <= 0)
	{
		printk(KERN_ERR "%s, Can not write 0x%x bytes postition %d size \n", __func__, (int)bytes_pos, (int)size);
		goto write_fail;
	}

write_fail:
	if(!IS_ERR(phMscd_Filp))
		filp_close(phMscd_Filp,NULL);
	set_fs(old_fs); 
	return write_bytes;
	
}

EXPORT_SYMBOL(lge_write_block);

/* MOD 0014570: [FACTORY RESET] change system call to filp function for handling the flag */
int lge_read_block(unsigned int bytes_pos, unsigned char *buf, size_t size)
{
	struct file *phMscd_Filp = NULL;
	mm_segment_t old_fs;
	unsigned int read_bytes = 0;

	// exception handling
	if((buf == NULL) || size <= 0)
	{
		printk(KERN_ERR "%s, NULL buffer or NULL size : %d\n", __func__, (int)size);
		return 0;
	}
		
	old_fs=get_fs();
	set_fs(get_ds());

	// change from sys operation to flip operation, do not use system call since this routine is also system call service.
	phMscd_Filp = filp_open(MMC_DEVICENAME, O_RDONLY, 0);
	if(IS_ERR(phMscd_Filp))
	{
		printk(KERN_ERR "%s, Can not access 0x%x bytes postition\n", __func__, (int)bytes_pos );
		goto read_fail;
	}

	phMscd_Filp->f_pos = (loff_t)bytes_pos;
	read_bytes = phMscd_Filp->f_op->read(phMscd_Filp, buf, size, &phMscd_Filp->f_pos);

	if(read_bytes <= 0)
	{
		printk(KERN_ERR "%s, Can not read 0x%x bytes postition %d size \n", __func__, (int)bytes_pos, (int)size);
		goto read_fail;
	}

read_fail:
	if(!IS_ERR(phMscd_Filp))
		filp_close(phMscd_Filp,NULL);
	set_fs(old_fs); 
	return read_bytes;
}

EXPORT_SYMBOL(lge_read_block);

const MmcPartition *lge_mmc_find_partition_by_name(const char *name)
{
    if (g_mmc_state.partitions != NULL) {
        int i;
        for (i = 0; i < g_mmc_state.partitions_allocd; i++) {
            MmcPartition *p = &g_mmc_state.partitions[i];
            if (p->device_index !=NULL && p->name != NULL) {
                if (strcmp(p->name, name) == 0) {
                    return p;
                }
            }
        }
    }
    return NULL;
}

void lge_mmc_print_partition_status(void)
{
    if (g_mmc_state.partitions != NULL) 
    {
        int i;
        for (i = 0; i < g_mmc_state.partitions_allocd; i++) 
        {
            MmcPartition *p = &g_mmc_state.partitions[i];
            if (p->device_index !=NULL && p->name != NULL) {
                printk(KERN_INFO"Partition Name: %s\n",p->name);
                printk(KERN_INFO"Partition Name: %s\n",p->device_index);
            }
        }
    }
    return;
}


EXPORT_SYMBOL(lge_mmc_find_partition_by_name);
EXPORT_SYMBOL(lge_mmc_print_partition_status);

static void lge_mmc_partition_name (MmcPartition *mbr, unsigned int type) {
	char *name;
	name = kmalloc(64, GFP_KERNEL);
	switch(type)
	{
		case MMC_MISC_TYPE:
			sprintf(name,"misc");
			mbr->name = lge_strdup(name);
			break;
		case MMC_RECOVERY_TYPE:
			sprintf(name,"recovery");
			mbr->name = lge_strdup(name);
			break;
		case MMC_BOOT_TYPE:
			sprintf(name,"boot");
			mbr->name = lge_strdup(name);
			break;

		case HOTPLUG_PARTITION_ID_EXT4_TYPE:
			if (strcmp("NONE", ext4_partitions[ext4_count])) {
				strcpy((char *)name,(const char *)ext4_partitions[ext4_count]);
				mbr->name = lge_strdup(name);
				ext4_count++;
			}
			mbr->filesystem = lge_strdup("ext4");
			break;
	};
	kfree(name);
}


//static int lge_mmc_read_mbr (MmcPartition *mbr) {
/* MOD 0014570: [FACTORY RESET] change system call to filp function for handling the flag */
int lge_mmc_read_mbr (MmcPartition *mbr) {
	//int fd;
	unsigned char *buffer = NULL;
	char *device_index = NULL;
	int idx, i;
	unsigned mmc_partition_count = 0;
	unsigned int dtype;
	unsigned int dfirstsec;
	unsigned int EBR_first_sec;
	unsigned int EBR_current_sec;
	int ret = -1;

	struct file *phMscd_Filp = NULL;
	mm_segment_t old_fs;

	old_fs=get_fs();
	set_fs(get_ds());

	buffer = kmalloc(512, GFP_KERNEL);
	device_index = kmalloc(128, GFP_KERNEL);
	if((buffer == NULL) || (device_index == NULL))
	{
		printk("%s, allocation failed\n", __func__);
		goto ERROR2;
	}

	// change from sys operation to flip operation, do not use system call since this routine is also system call service.
	phMscd_Filp = filp_open(MMC_DEVICENAME, O_RDONLY, 0);
	if(IS_ERR(phMscd_Filp))
	{
		printk(KERN_ERR "%s, Can't open device\n", __func__ );
		goto ERROR2;
	}

	phMscd_Filp->f_pos = (loff_t)0;
	if (phMscd_Filp->f_op->read(phMscd_Filp, buffer, 512, &phMscd_Filp->f_pos) != 512)
	{
		printk(KERN_ERR "%s, Can't read device: \"%s\"\n", __func__, MMC_DEVICENAME);
		goto ERROR1;
	}

	/* Check to see if signature exists */
	if ((buffer[TABLE_SIGNATURE] != 0x55) || \
		(buffer[TABLE_SIGNATURE + 1] != 0xAA))
	{
		printk(KERN_ERR "Incorrect mbr signatures!\n");
		goto ERROR1;
	}
	idx = TABLE_ENTRY_0;
	for (i = 0; i < 4; i++)
	{
		//char device_index[128];

		mbr[mmc_partition_count].dstatus = \
		            buffer[idx + i * TABLE_ENTRY_SIZE + OFFSET_STATUS];
		mbr[mmc_partition_count].dtype   = \
		            buffer[idx + i * TABLE_ENTRY_SIZE + OFFSET_TYPE];
		mbr[mmc_partition_count].dfirstsec = \
		            GET_LWORD_FROM_BYTE(&buffer[idx + \
		                                i * TABLE_ENTRY_SIZE + \
		                                OFFSET_FIRST_SEC]);
		mbr[mmc_partition_count].dsize  = \
		            GET_LWORD_FROM_BYTE(&buffer[idx + \
		                                i * TABLE_ENTRY_SIZE + \
		                                OFFSET_SIZE]);
		dtype  = mbr[mmc_partition_count].dtype;
		dfirstsec = mbr[mmc_partition_count].dfirstsec;
		lge_mmc_partition_name(&mbr[mmc_partition_count], \
		                mbr[mmc_partition_count].dtype);

		sprintf(device_index, "%sp%d", MMC_DEVICENAME, (mmc_partition_count+1));
		mbr[mmc_partition_count].device_index = lge_strdup(device_index);

		mmc_partition_count++;
		if (mmc_partition_count == MAX_PARTITIONS)
			goto SUCCESS;
	}

	/* See if the last partition is EBR, if not, parsing is done */
	if (dtype != 0x05)
	{
		goto SUCCESS;
	}

	EBR_first_sec = dfirstsec;
	EBR_current_sec = dfirstsec;

	phMscd_Filp->f_pos = (loff_t)(EBR_first_sec * 512);
	if (phMscd_Filp->f_op->read(phMscd_Filp, buffer, 512, &phMscd_Filp->f_pos) != 512)
	{
		printk(KERN_ERR "%s, Can't read device: \"%s\"\n", __func__, MMC_DEVICENAME);
		goto ERROR1;
	}

	/* Loop to parse the EBR */
	for (i = 0;; i++)
	{

		if ((buffer[TABLE_SIGNATURE] != 0x55) || (buffer[TABLE_SIGNATURE + 1] != 0xAA))
		{
		break;
		}
		mbr[mmc_partition_count].dstatus = \
                    buffer[TABLE_ENTRY_0 + OFFSET_STATUS];
		mbr[mmc_partition_count].dtype   = \
                    buffer[TABLE_ENTRY_0 + OFFSET_TYPE];
		mbr[mmc_partition_count].dfirstsec = \
                    GET_LWORD_FROM_BYTE(&buffer[TABLE_ENTRY_0 + \
                                        OFFSET_FIRST_SEC])    + \
                                        EBR_current_sec;
		mbr[mmc_partition_count].dsize = \
                    GET_LWORD_FROM_BYTE(&buffer[TABLE_ENTRY_0 + \
                                        OFFSET_SIZE]);
		lge_mmc_partition_name(&mbr[mmc_partition_count], \
                        mbr[mmc_partition_count].dtype);

		sprintf(device_index, "%sp%d", MMC_DEVICENAME, (mmc_partition_count+1));
		mbr[mmc_partition_count].device_index = lge_strdup(device_index);

		mmc_partition_count++;
		if (mmc_partition_count == MAX_PARTITIONS)
		goto SUCCESS;

		dfirstsec = GET_LWORD_FROM_BYTE(&buffer[TABLE_ENTRY_1 + OFFSET_FIRST_SEC]);
		if(dfirstsec == 0)
		{
			/* Getting to the end of the EBR tables */
			break;
		}
		
		 /* More EBR to follow - read in the next EBR sector */
		 phMscd_Filp->f_pos = (loff_t)((EBR_first_sec + dfirstsec) * 512);
		 if (phMscd_Filp->f_op->read(phMscd_Filp, buffer, 512, &phMscd_Filp->f_pos) != 512)
		 {
			 printk(KERN_ERR "%s, Can't read device: \"%s\"\n", __func__, MMC_DEVICENAME);
			 goto ERROR1;
		 }

		EBR_current_sec = EBR_first_sec + dfirstsec;
	}

SUCCESS:
    ret = mmc_partition_count;
ERROR1:
    if(phMscd_Filp != NULL)
		filp_close(phMscd_Filp,NULL);
ERROR2:
	set_fs(old_fs);
	if(buffer != NULL)
		kfree(buffer);
	if(device_index != NULL)
		kfree(device_index);
    return ret;
}

static int lge_mmc_partition_initialied = 0;
int lge_mmc_scan_partitions(void) {
    int i;
    //ssize_t nbytes;

	if ( lge_mmc_partition_initialied )
		return g_mmc_state.partition_count;
	
    if (g_mmc_state.partitions == NULL) {
        const int nump = MAX_PARTITIONS;
        MmcPartition *partitions = kmalloc(nump * sizeof(*partitions), GFP_KERNEL);
        if (partitions == NULL) {
            return -1;
        }
        g_mmc_state.partitions = partitions;
        g_mmc_state.partitions_allocd = nump;
        memset(partitions, 0, nump * sizeof(*partitions));
    }
    g_mmc_state.partition_count = 0;
    ext4_count = 0;
    //vfat_count = 0;

    /* Initialize all of the entries to make things easier later.
     * (Lets us handle sparsely-numbered partitions, which
     * may not even be possible.)
     */
    for (i = 0; i < g_mmc_state.partitions_allocd; i++) {
        MmcPartition *p = &g_mmc_state.partitions[i];
        if (p->device_index != NULL) {
            kfree(p->device_index);
            p->device_index = NULL;
        }
        if (p->name != NULL) {
            kfree(p->name);
            p->name = NULL;
        }
        if (p->filesystem != NULL) {
            kfree(p->filesystem);
            p->filesystem = NULL;
        }
    }

    g_mmc_state.partition_count = lge_mmc_read_mbr(g_mmc_state.partitions);
    if(g_mmc_state.partition_count == -1)
    {
        printk(KERN_ERR"Error in reading mbr!\n");
        // keep "partitions" around so we can free the names on a rescan.
        g_mmc_state.partition_count = -1;
    }
	if ( g_mmc_state.partition_count != -1 )
		lge_mmc_partition_initialied = 1;
    return g_mmc_state.partition_count;
}

EXPORT_SYMBOL(lge_mmc_scan_partitions);

#ifdef CONFIG_LGE_DLOAD_FACTORY_RESET
static int fact_reset_flag_cp_set(const char *val, struct kernel_param *kp)
{
	int mtd_op_result = 0;
	const MmcPartition *pMisc_part; 
	unsigned long factoryreset_bytes_pos_in_emmc = 0;
	unsigned long flag=0;

	unsigned char *test_string = NULL;

	test_string = kmalloc(FACTORY_RESET_STR_SIZE+2, GFP_KERNEL);
	if(!test_string)
	{
		printk(KERN_ERR "allocation failed, return\n");
		return 0;
	}

	flag = simple_strtoul(val,NULL,10);
	sprintf(test_string,"FACT_RESET_%d\n", (char)flag);

	mtd_op_result = lge_mmc_scan_partitions();
	if(mtd_op_result < 0)
	{
		printk(KERN_ERR "NOT READY! unable to scan partitions\n");
		return 0;
	}
	pMisc_part = lge_mmc_find_partition_by_name("misc");
	if ( pMisc_part == NULL )
	{
		printk(KERN_ERR "NO MISC\n");
		return 0;
	}
	
	factoryreset_bytes_pos_in_emmc = (pMisc_part->dfirstsec*512)+PTN_DLOAD_FRST_PERSIST_POSITION_IN_MISC_PARTITION;

	printk(KERN_INFO "%s, writing block, %s\n", __func__, test_string);

	mtd_op_result = lge_write_block(factoryreset_bytes_pos_in_emmc, test_string, FACTORY_RESET_STR_SIZE+2);
	if ( mtd_op_result != (FACTORY_RESET_STR_SIZE+2) ) {
		printk(KERN_ERR"%s: write 0x%lX (%d sector) fail\n", __func__, factoryreset_bytes_pos_in_emmc, (int)(factoryreset_bytes_pos_in_emmc/512));
		kfree(test_string);		
		return -1;
	}

	printk(KERN_INFO " %s, write 0x%lX (%d sector)\n", __func__, factoryreset_bytes_pos_in_emmc, (int)(factoryreset_bytes_pos_in_emmc/512));
	kfree(test_string);
	return 0;
}

static int fact_reset_flag_cp_get(char *buf, struct kernel_param *kp)
{
	int flag = 0;
	int mtd_op_result = 0;
	
	const MmcPartition *pMisc_part; 
	unsigned long factoryreset_bytes_pos_in_emmc = 0;

	unsigned char *test_string = NULL;
	
	test_string = kmalloc(FACTORY_RESET_STR_SIZE+2, GFP_KERNEL);
	if(!test_string)
	{
		printk(KERN_ERR "allocation failed, return\n");
		return 0;
	}
	
	mtd_op_result = lge_mmc_scan_partitions();
	if(mtd_op_result < 0)
	{
		printk(KERN_ERR "NOT READY! unable to scan partitions\n");
		return 0;
	}
	pMisc_part = lge_mmc_find_partition_by_name("misc");
	if ( pMisc_part == NULL )
	{
		printk(KERN_ERR "NO MISC\n");
		return 0;
	}

	factoryreset_bytes_pos_in_emmc = (pMisc_part->dfirstsec*512)+PTN_DLOAD_FRST_PERSIST_POSITION_IN_MISC_PARTITION;

	memset(test_string, 0 , FACTORY_RESET_STR_SIZE+2);
	mtd_op_result = lge_read_block(factoryreset_bytes_pos_in_emmc, test_string, FACTORY_RESET_STR_SIZE+2);
	if (mtd_op_result != (FACTORY_RESET_STR_SIZE+2) ) {
		printk(KERN_ERR "read 0x%lX (%d sector) failed\n", factoryreset_bytes_pos_in_emmc, (int)(factoryreset_bytes_pos_in_emmc/512));
		return 0;
	}

	printk(KERN_INFO "%s, read block : %s\n", __func__, test_string);
	printk(KERN_INFO "%s, read 0x%lX (%d sector)\n", __func__, factoryreset_bytes_pos_in_emmc, (int)(factoryreset_bytes_pos_in_emmc/512));

	if(memcmp(test_string, FACTORY_RESET_STR, FACTORY_RESET_STR_SIZE)==0){
		flag = *(test_string+FACTORY_RESET_STR_SIZE) - '0';
		sprintf(buf,"%d",flag);
		printk(KERN_INFO "modem factory reset backup status = %d \n", (int)flag);
	}
	else{
		sprintf(buf,"0");
		flag = 0;
		printk(KERN_ERR "modem factory reset tag read failed :  %s \n", test_string);
	}

	kfree(test_string);
	return flag;
}

module_param_call(fact_reset_flag_cp, fact_reset_flag_cp_set, fact_reset_flag_cp_get, &dummy_arg, S_IWUSR | S_IRUGO);
#endif

/* MOD 0013861: [FACTORY RESET] emmc_direct_access factory reset flag access */
/* add carriage return and change flag size in each functions for the platform access */

static int test_write_block(const char *val, struct kernel_param *kp)
{

	//int i;
	//int err;
	//int normal_block_seq = 0;
	int mtd_op_result = 0;
	const MmcPartition *pMisc_part; 
	unsigned long factoryreset_bytes_pos_in_emmc = 0;
	unsigned long flag=0;

	unsigned char *test_string = NULL;

	test_string = kmalloc(FACTORY_RESET_STR_SIZE+2, GFP_KERNEL);
	// allocation exception handling
	if(!test_string)
	{
		printk(KERN_ERR "allocation failed, return\n");
		return 0;
	}
	
	printk(KERN_INFO"write block1\n");
	
	flag = simple_strtoul(val,NULL,10);
//	if (flag == 5 || flag == 6 )
//	{
/* MOD 0014076: [FACTORY RESET] Android factory reset flag bug fix */
/* make sure to store only 13 bytes, string 11, flag 1, carriage 1 */
		//sprintf(test_string,"FACT_RESET_%d\n", flag);
		sprintf(test_string,"FACT_RESET_%d\n", (char)flag);
//	} else {
//		kfree(test_string);
//		return -1;
//	}
	
	mtd_op_result = lge_mmc_scan_partitions();
	if(mtd_op_result < 0)
	{
		printk(KERN_ERR "NOT READY! unable to scan partitions\n");
		return 0;
	}
	pMisc_part = lge_mmc_find_partition_by_name("misc");
	if ( pMisc_part == NULL )
	{
		printk(KERN_ERR "NO MISC\n");
		return 0;
	}
	
	factoryreset_bytes_pos_in_emmc = (pMisc_part->dfirstsec*512)+PTN_FRST_PERSIST_POSITION_IN_MISC_PARTITION;


	printk(KERN_INFO"writing block, %s\n", test_string);


	mtd_op_result = lge_write_block(factoryreset_bytes_pos_in_emmc, test_string, FACTORY_RESET_STR_SIZE+2);
	if ( mtd_op_result != (FACTORY_RESET_STR_SIZE+2) ) {
		printk(KERN_ERR"%s: write 0x%lX (%d sector) fail\n", __func__, factoryreset_bytes_pos_in_emmc, (int)(factoryreset_bytes_pos_in_emmc/512));
		kfree(test_string);		
		return -1;
	}

/* ADD 0013860: [FACTORY RESET] ERI file save */
/* request rpc for eri file when the factory reset completes */
#ifdef CONFIG_LGE_ERI_DOWNLOAD
	if (flag == 5)
	{
		printk(KERN_INFO "%s, received flag : %ld, activate work queue\n", __func__, flag);
		eri_dload_data.flag = flag;
		queue_work(eri_dload_wq, &eri_dload_data.work);
	}
#endif

#ifdef CONFIG_LGE_DLOAD_FACTORY_RESET
	if(flag == 5)
	{
		memset(test_string, 0, FACTORY_RESET_STR_SIZE+2);
		flag = (unsigned long)fact_reset_flag_cp_get(test_string, NULL);
		
		if(flag == 1) // FACTORY_RESET_INITIAL
		{
			printk(KERN_INFO "Request efs_sync for dload upgrade\n");
			request_sync_cmd_local(LGE_SYNC_REQUEST);

			flag = 2; // FACTORY_RESET_ARM9_END
			memset(test_string, 0, FACTORY_RESET_STR_SIZE+2);
			sprintf(test_string,"%d\n", (char)flag);
			fact_reset_flag_cp_set(test_string, NULL);

			printk(KERN_INFO "Request ONLINE mode for dload upgrade\n");
			request_sync_cmd_local(LGE_REQUEST_ONLINE_MODE);
		}
	}
#endif

	kfree(test_string);
	return 0;
}
module_param_call(write_block, test_write_block, NULL, &dummy_arg, S_IWUSR | S_IRUGO);

static unsigned char global_buf[FACTORY_RESET_STR_SIZE+2];

static int test_read_block(char *buf, struct kernel_param *kp)
{
	//int i;
	int err = -1;
	int mtd_op_result = 0;
	
	const MmcPartition *pMisc_part; 
	unsigned long factoryreset_bytes_pos_in_emmc = 0;
	
	printk(KERN_INFO"read block1\n");
	
	mtd_op_result = lge_mmc_scan_partitions();
	if(mtd_op_result < 0)
	{
		printk(KERN_ERR "NOT READY! unable to scan partitions\n");
		return 0;
	}
//	lge_mmc_partition_initialied = 4;
	
	pMisc_part = lge_mmc_find_partition_by_name("misc");
	if ( pMisc_part == NULL )
	{
		printk(KERN_ERR "NO MISC\n");
		return 0;
	}
//	lge_mmc_partition_initialied = 5;
	factoryreset_bytes_pos_in_emmc = (pMisc_part->dfirstsec*512)+PTN_FRST_PERSIST_POSITION_IN_MISC_PARTITION;

	printk(KERN_INFO"read block\n");
//	lge_mmc_partition_initialied = 6;
	memset(global_buf, 0 , FACTORY_RESET_STR_SIZE+2);

	mtd_op_result = lge_read_block(factoryreset_bytes_pos_in_emmc, global_buf, FACTORY_RESET_STR_SIZE+2);
//	lge_mmc_partition_initialied = 7;
	
	if (mtd_op_result != (FACTORY_RESET_STR_SIZE+2) ) {
		printk(KERN_ERR "read 0x%lX (%d sector) failed\n", factoryreset_bytes_pos_in_emmc, (int)(factoryreset_bytes_pos_in_emmc/512));
		return err;
	}
//	lge_mmc_partition_initialied = 8;

	printk(KERN_ERR "read 0x%lX (%d sector)\n", factoryreset_bytes_pos_in_emmc, (int)(factoryreset_bytes_pos_in_emmc/512));
//	printk(KERN_INFO"%s\n", __func__, global_buf);

// MOD 0009484: [FactoryReset] Enable FactoryReset
	if(memcmp(global_buf, FACTORY_RESET_STR, FACTORY_RESET_STR_SIZE)==0){
		err = sprintf(buf,"%s",global_buf+FACTORY_RESET_STR_SIZE);
//		err = sprintf(buf,"123456789");
		return err;
	}
	else{
		err = sprintf(buf,"1");
		return err;
	}
}

module_param_call(read_block, NULL, test_read_block, &dummy_arg, S_IWUSR | S_IRUGO);

/* ADD 0013860: [FACTORY RESET] ERI file save */
#ifdef CONFIG_LGE_ERI_DOWNLOAD
static void
eri_dload_func(struct work_struct *work)
{
	printk(KERN_INFO "%s, flag : %ld\n", __func__, eri_dload_data.flag);	
	//printk("%s [WQ] pressed: %d, keycode: %d\n", __func__, eta_gpio_matrix_data.pressed, eta_gpio_matrix_data.keycode);
#ifdef CONFIG_LGE_SUPPORT_RAPI
	remote_eri_rpc();
#endif
	return;
}
#endif

#if defined (CONFIG_LGE_LCD_K_CAL)
static unsigned char lcdtest_ascii_to_int(unsigned char ascii)
{
	return (ascii - 48);
}

static int write_lcd_k_cal(const char *val, struct kernel_param *kp)
{

	int i = 0;
	int err;
	int mtd_op_result = 0;
	const MmcPartition *pMisc_part;
	unsigned long lcdkcal_bytes_pos_in_emmc = 0;

	unsigned char buf[10]={255,};
	memcpy(buf,val,10);
	
	lcd_buf[0] = lcdtest_ascii_to_int(buf[0]) * 100 + lcdtest_ascii_to_int(buf[1])*10 + lcdtest_ascii_to_int(buf[2]);
	lcd_buf[1] = lcdtest_ascii_to_int(buf[3]) * 100 + lcdtest_ascii_to_int(buf[4])*10 + lcdtest_ascii_to_int(buf[5]);
	lcd_buf[2] = lcdtest_ascii_to_int(buf[6]) * 100 + lcdtest_ascii_to_int(buf[7])*10 + lcdtest_ascii_to_int(buf[8]);
	lcd_buf[3]=0;
	lcd_buf[4]=111;
	lcd_buf[5]=222;
	
#if 1
		for(i=0;i<LCD_K_CAL_SIZE;i++)
		{
			printk("write_lcd_k_cal,  lcd_buf[%d] :%d:\n",i,lcd_buf[i]);
		}
#endif

	mtd_op_result = lge_mmc_scan_partitions();
	if(mtd_op_result < 0)
	{
		printk(KERN_ERR "NOT READY! unable to scan partitions\n");
		return 0;
	}
	pMisc_part = lge_mmc_find_partition_by_name("misc");
	if ( pMisc_part == NULL )
	{
		printk(KERN_ERR "NO MISC\n");
		return 0;
	}

	lcdkcal_bytes_pos_in_emmc = (pMisc_part->dfirstsec*512)+PTN_LCD_K_CAL_POSITION_IN_MISC_PARTITION;

	printk("write_lcd_k_cal %ld block\n", lcdkcal_bytes_pos_in_emmc);

	mtd_op_result = lge_write_block(lcdkcal_bytes_pos_in_emmc, lcd_buf, LCD_K_CAL_SIZE);

	if ( mtd_op_result != LCD_K_CAL_SIZE ) {
		printk("%s: write %u block fail\n", __func__, i);
		return err;
	}
	printk("write %d block\n", i);
	return 0;
}
EXPORT_SYMBOL(write_lcd_k_cal);


int read_lcd_k_cal( char *buf)
{
	int err=0;
	int mtd_op_result = 0;
//	int i;

	const MmcPartition *pMisc_part;
	unsigned long lcdkcal_bytes_pos_in_emmc = 0;

	printk(KERN_INFO"read read_lcd_k_cal\n");

	mtd_op_result = lge_mmc_scan_partitions();
	if(mtd_op_result < 0)
	{
		printk(KERN_ERR "NOT READY! unable to scan partitions\n");
		return 0;
	}
	pMisc_part = lge_mmc_find_partition_by_name("misc");

	if ( pMisc_part == NULL )
	{
		printk(KERN_ERR "NO MISC\n");
		return 0;
	}

	lcdkcal_bytes_pos_in_emmc = (pMisc_part->dfirstsec*512)+PTN_LCD_K_CAL_POSITION_IN_MISC_PARTITION;

	memset(lcd_buf, 0 ,LCD_K_CAL_SIZE);
	mtd_op_result = lge_read_block(lcdkcal_bytes_pos_in_emmc, &lcd_buf[0], LCD_K_CAL_SIZE);

	if (mtd_op_result != LCD_K_CAL_SIZE ) {
		printk(KERN_INFO" read %ld block fail\n", lcdkcal_bytes_pos_in_emmc);
		return err;
	}

	printk(KERN_INFO"read %ld block\n", lcdkcal_bytes_pos_in_emmc);
	memcpy(&buf[0],&lcd_buf[0],LCD_K_CAL_SIZE);

	return LCD_K_CAL_SIZE;
}
EXPORT_SYMBOL(read_lcd_k_cal);
//heebae.song for kcal
module_param_call(lcd_k_cal, write_lcd_k_cal, NULL, NULL,S_IWUSR|S_IRUSR|S_IRGRP|S_IWGRP);	//heebae.song for kcal
#endif /* CONFIG_LGE_LCD_K_CAL */

static int __init lge_emmc_direct_access_init(void)
{
	printk(KERN_INFO"%s: finished\n", __func__);

/* ADD 0013860: [FACTORY RESET] ERI file save */
#ifdef CONFIG_LGE_ERI_DOWNLOAD
	eri_dload_wq = create_singlethread_workqueue("eri_dload_wq");
	INIT_WORK(&eri_dload_data.work, eri_dload_func);
#endif

	return 0;
}

static void __exit lge_emmc_direct_access_exit(void)
{
	return;
}

module_init(lge_emmc_direct_access_init);
module_exit(lge_emmc_direct_access_exit);

MODULE_DESCRIPTION("LGE emmc direct access apis");
MODULE_AUTHOR("SeHyun Kim <sehyuny.kim@lge.com>");
MODULE_LICENSE("GPL");
