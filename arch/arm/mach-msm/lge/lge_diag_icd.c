/* arch/arm/mach-msm/lge/lg_fw_diag_icd.c
 *
 * Copyright (C) 2009,2010 LGE, Inc.
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

#include <linux/module.h>
#include <mach/lge_diagcmd.h>
#include <mach/lge_diag_icd.h>
#include <mach/lge_base64.h>
#include <mach/lge_pcb_version.h>
#include <mach/lge_diag_communication.h>

#include <linux/unistd.h>	/*for open/close */
#include <linux/fcntl.h>	/*for O_RDWR */

#include <linux/fb.h>		/* to handle framebuffer ioctls */
#include <linux/ioctl.h>
#include <linux/uaccess.h>

#include <linux/syscalls.h>	//for sys operations

#include <linux/input.h>	// for input_event
#include <linux/fs.h>		// for file struct
#include <linux/types.h>	// for ssize_t
#include <linux/input.h>	// for event parameters
#include <linux/jiffies.h>
#include <linux/delay.h>

//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
#include <mach/lge_diag_screen_shot.h>
#include <linux/vmalloc.h>

#define LCD_BUFFER_SIZE LCD_MAIN_WIDTH * LCD_MAIN_HEIGHT * 4
#define CONVERT565(r, g, b) ( ((r >> 3) << 11) | (g >> 2) << 5 | (b >> 3) )
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]

#include <mach/lge_diag_mtc.h>

/*
 * EXTERNAL FUNCTION AND VARIABLE DEFINITIONS
 */
extern PACK(void *) diagpkt_alloc(diagpkt_cmd_code_type code,unsigned int length);
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
extern PACK(void *) diagpkt_alloc2 (diagpkt_cmd_code_type code, unsigned int length, unsigned int packet_length);
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]
extern PACK(void *) diagpkt_free(PACK(void *)pkt);

extern void icd_send_to_arm9(void* pReq, void* pRsp, unsigned int output_length);

extern icd_user_table_entry_type icd_mstr_tbl[ICD_MSTR_TBL_SIZE];

//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
unsigned char g_diag_slate_capture_rsp_num = 0;
unsigned char g_slate_status = 0x0;//Slate_ADB
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]

extern int lge_bd_rev;

extern unsigned char lge_img_capture_32bpp[MTC_SCRN_BUF_32BPP_SIZE];
extern unsigned char lge_img_capture_16bpp[MTC_SCRN_BUF_SIZE_MAX];

/*
 * LOCAL DEFINITIONS AND DECLARATIONS FOR MODULE
 *
 * This section contains local definitions for constants, macros, types,
 * variables and other items needed by this module.
 */
static struct diagcmd_dev *diagpdev;

char process_status[10];
EXPORT_SYMBOL(process_status);

char process_value[100];
EXPORT_SYMBOL(process_value);

// LGE_CHANGE_S [myeonggyu.son@lge.com] [2011.02.25] [GELATO] enable or disable key logging status of slate [START]
#ifdef CONFIG_LGE_DIAG
int key_touch_logging_status = 0;
EXPORT_SYMBOL(key_touch_logging_status);
#endif
// LGE_CHANGE_E [myeonggyu.son@lge.com] [2011.02.25] [GELATO] enable or disable key logging status of slate [END]

/*
 * INTERNAL FUNCTION DEFINITIONS
 */
PACK(void *) LGE_ICDProcess(PACK(void *)req_pkt_ptr,	/* pointer to request packet  */
			    		unsigned short pkt_len			/* length of request packet   */)
{
	DIAG_ICD_F_req_type *req_ptr = (DIAG_ICD_F_req_type *) req_pkt_ptr;
	DIAG_ICD_F_rsp_type *rsp_ptr = NULL;
	icd_func_type func_ptr = NULL;
	int is_valid_arm9_command = 1;
	unsigned int rsp_ptr_len;

	int nIndex = 0;
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
	if(!g_slate_status)//Slate_ADB
		g_slate_status = 0x1;
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]

	diagpdev = diagcmd_get_dev();

	for (nIndex = 0; nIndex < ICD_MSTR_TBL_SIZE; nIndex++) 
	{
		if(icd_mstr_tbl[nIndex].cmd_code == req_ptr->hdr.sub_cmd) 
		{
			if (icd_mstr_tbl[nIndex].which_procesor == ICD_ARM11_PROCESSOR)
			{
				func_ptr = icd_mstr_tbl[nIndex].func_ptr;
			}
			
			break;
		} 
		else if (icd_mstr_tbl[nIndex].cmd_code == ICD_MAX_REQ_CMD)
		{
			break;
		}
		else
		{
			continue;
		}
	}
	
	sys_chmod("/data/img", 0664); // change directory permission to make adb commands non-available on "data/img", prevent rooting

	if (func_ptr != NULL) 
	{
		printk(KERN_INFO "[ICD] cmd_code : [0x%X], sub_cmd : [0x%X]\n",req_ptr->hdr.cmd_code, req_ptr->hdr.sub_cmd);
		rsp_ptr = func_ptr((DIAG_ICD_F_req_type *) req_ptr);
	} 
	else
	{
		switch(req_ptr->hdr.sub_cmd) {
			case ICD_GETDEVICEINFO_REQ_CMD:
				rsp_ptr_len = sizeof(icd_device_info_rsp_type);
				break;
			case ICD_GETGPSSTATUS_REQ_CMD:
				rsp_ptr_len = sizeof(icd_get_gps_status_rsp_type);
				break;
			case ICD_SETGPSSTATUS_REQ_CMD:
				rsp_ptr_len = sizeof(icd_set_gps_status_rsp_type);
				break;
			case ICD_GETROAMINGMODE_REQ_CMD:
				rsp_ptr_len = sizeof(icd_get_roamingmode_rsp_type);
				break;
			case ICD_GETSTATEANDCONNECTIONATTEMPTS_REQ_CMD:
				rsp_ptr_len = sizeof(icd_get_state_connect_rsp_type);
				break;
			case ICD_GETBATTERYCHARGINGSTATE_REQ_CMD:
				rsp_ptr_len = sizeof(icd_set_battery_charging_state_rsp_type);
				break;
//[dugyung.ahn@lge.com] 2012-05-01, [Slate] GETBATTERYLEVEL [s]				
//			case ICD_GETBATTERYLEVEL_REQ_CMD:
//				rsp_ptr_len = sizeof(icd_get_battery_level_rsp_type);
//				break;
//[dugyung.ahn@lge.com] 2012-05-01, [Slate] GETBATTERYLEVEL [e]				
//[dugyung.ahn@lge.com] 2012-06-27, [Slate] GETRSSI [s] 
//			case ICD_GETRSSI_REQ_CMD:
//				rsp_ptr_len = sizeof(icd_get_rssi_rsp_type);
//				break;
//[dugyung.ahn@lge.com] 2012-06-27, [Slate] GETRSSI [e]					
			case ICD_SETDISCHARGING_REQ_CMD:
				rsp_ptr_len = sizeof(icd_set_discharger_rsp_type);
				break;
			case ICD_GETUSBDEBUGSTATUSSTATUS_REQ_CMD:
				rsp_ptr_len = sizeof(icd_get_usbdebug_status_rsp_type);				
				break;
				
			case ICD_SETUSBDEBUGSTATUSSTATUS_REQ_CMD:
				rsp_ptr_len = sizeof(icd_set_usbdebug_status_rsp_type);				
				break;
				
			case ICD_GETLATITUDELONGITUDEVALUES_REQ_CMD:
				rsp_ptr_len = sizeof(icd_get_latitude_longitude_values_rsp_type);
				break;
			case ICD_GETSCREENLOCKSTATUS_REQ_CMD:
				rsp_ptr_len = sizeof(icd_get_screenlock_status_rsp_type);				
				break;
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
			case ICD_SETSCREENLOCKSTATUS_REQ_CMD:
				rsp_ptr_len = sizeof(icd_set_screenlock_status_rsp_type);				
				break;
			case ICD_SETSCREENORIENTATIONLOCK_REQ_CMD:
				rsp_ptr_len = sizeof(icd_set_screenorientationlock_rsp_type);
				break;
			case ICD_GETANDROIDIDENTIFIER_REQ_CMD:
				rsp_ptr_len = sizeof(icd_get_android_identifier_rsp_type);				
				break;
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]
			default:
				is_valid_arm9_command = 0;
				printk(KERN_INFO "[ICD] %s : invalid sub command : 0x%x\n",__func__,req_ptr->hdr.sub_cmd);
				break;
		}
		
		if(is_valid_arm9_command == 1)
		{
			rsp_ptr = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_ptr_len);
			if (rsp_ptr == NULL) {
				printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
				return rsp_ptr;
			}
				
			printk(KERN_INFO "[ICD] cmd_code : [0x%X], sub_cmd : [0x%X] --> goto MODEM through oem rapi\n",req_ptr->hdr.cmd_code, req_ptr->hdr.sub_cmd);
			icd_send_to_arm9((void *)req_ptr, (void *)rsp_ptr, rsp_ptr_len);
		}
	}

	return (rsp_ptr);
}
EXPORT_SYMBOL(LGE_ICDProcess);

DIAG_ICD_F_rsp_type *icd_app_handler(DIAG_ICD_F_req_type*pReq)
{
	return NULL;
}

/** SAR : Sprint Automation Requirement - START **/
DIAG_ICD_F_rsp_type *icd_info_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;

	rsp_len = sizeof(icd_device_info_rsp_type);

	printk(KERN_INFO "[ICD] icd_info_req_proc\n");
	printk(KERN_INFO "[ICD] icd_info_req_proc rsp_len :(%d)\n", rsp_len);

	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}

	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_GETDEVICEINFO_REQ_CMD;

	// get manufacture info
	
	// get model name info

	// get hw version info
	
	// get sw version info

	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_extended_info_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;

	rsp_len = sizeof(icd_extended_info_rsp_type);

	printk(KERN_INFO "[ICD] icd_extended_info_req_proc\n");
	printk(KERN_INFO "[ICD] icd_extended_info_req_proc rsp_len :(%d)\n", rsp_len);

	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}

	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_EXTENDEDVERSIONINFO_REQ_CMD;

	// get extended version info
	memset(pRsp->icd_rsp.extended_rsp_info.ver_string, 0x00, sizeof(pRsp->icd_rsp.extended_rsp_info.ver_string));
	if(diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n",__func__);
		update_diagcmd_state(diagpdev, "ICD_GETEXTENDEDVERSION", 1);
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
		mdelay(100);
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]
		if(strcmp(process_status,"COMPLETED") == 0)
		{
			strcpy(pRsp->icd_rsp.extended_rsp_info.ver_string,process_value);
			printk(KERN_INFO "[ICD] %s was successful\n",__func__);
		}
		else
		{
			strcpy(pRsp->icd_rsp.extended_rsp_info.ver_string,"UNKNOWN");
			printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
		}
	}
	else
	{
		strcpy(pRsp->icd_rsp.extended_rsp_info.ver_string,"UNKNOWN");
		printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
	}
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_handset_disp_text_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;

	rsp_len = sizeof(icd_handset_disp_text_rsp_type);

	printk(KERN_INFO "[ICD] icd_handset_disp_text_req_proc\n");
	printk(KERN_INFO "[ICD] icd_handset_disp_text_req_proc rsp_len :(%d)\n", rsp_len);

	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}

	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_HANDSETDISPLAYTEXT_REQ_CMD;

	// get handset display text
	
	return pRsp;
}
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
typedef struct
{
	short 	bfType;
	long 	bfSize;
	short 	bfReserved1;
	short 	bfReserved2;
	long 	bfOffBits;
	long 	biSize;
	long 	biWidth;
	long 	biHeight;
	short 	biPlanes;
	short 	biBitCount;
	long 	biCompression;
	long 	biSizeImage;
	long 	biXPelsPerMeter;
	long 	biYPelsPerMeter;
	long 	biClrUsed;
	long 	biClrImportant;
} PACKED BMPHEAD;


typedef struct tagBITMAPFILEHEADER {
	short bfType;    
	long bfSize;      
	short bfReserved1;
	short bfReserved2;
	long bfOffBits;  
} PACKED BITMAPFILEHEADER;

typedef struct tagBITMAPINFOHEADER{
	long biSize;
	long biWidth;
	long biHeight;
	short biPlanes;
	short biBitCount;
	long biCompression;
	long biSizeImage;
	long biXPelsPerMeter;
	long biYPelsPerMeter;
	long biClrUsed;
	long biClrImportant;
} PACKED BITMAPINFOHEADER;

int removefile( char const *filename )
{
             char *argv[4] = { NULL, NULL, NULL, NULL };
             char *envp[3] = { NULL, NULL, NULL };

             if ( !filename )
                          return -EINVAL;
 
             argv[0] = "/system/bin/rm";
             argv[1] = (char *)filename;
 
             envp[0] = "HOME=/";
             envp[1] = "TERM=linux";
 
             return call_usermodehelper( argv[0], argv, envp, UMH_WAIT_PROC);
}
EXPORT_SYMBOL(removefile);

// careful of slate_screencap, added -s for slate capture and the path
int makepix(void)
{
             char *argv[4] = { NULL, NULL, NULL, NULL };
             char *envp[3] = { NULL, NULL, NULL };
 
             argv[0] = "/system/bin/slate_screencap";
			 argv[1] = "-s";
			 argv[2] = "/data/img/img.raw";
 
             envp[0] = "HOME=/";
             envp[1] = "TERM=linux";

             return call_usermodehelper( argv[0], argv, envp, UMH_WAIT_PROC );
}

static byte save_buffer[LCD_BUFFER_SIZE];

static void read_Framebuffer(int x_start, int y_start, int x_end, int y_end)
{
	//byte *fb_buffer;
	//byte *save_buffer;

	int fbfd, i, j;
	mm_segment_t old_fs=get_fs();
	int point = 0;
	BITMAPFILEHEADER map_header;
	BITMAPINFOHEADER info_header;
	byte header[12] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00};

	set_fs(get_ds());
	
	//fb_buffer = vmalloc(LCD_BUFFER_SIZE);
	memset(lge_img_capture_32bpp, 0x00, sizeof(lge_img_capture_32bpp));

	makepix();

	if( (fbfd = sys_open("/data/img/img.raw", O_RDONLY, 0)) == -1 )
	{
		printk(KERN_ERR "%s, Can't open %s\n",__func__,"/data/img/img.raw");
		return;
	}
	
	sys_read(fbfd, lge_img_capture_32bpp, sizeof(lge_img_capture_32bpp)) ;

	sys_close(fbfd);

	removefile("/data/img/img.raw");
	
	if( (fbfd = sys_open("/data/img/image.bmp", O_CREAT | O_LARGEFILE | O_WRONLY, 0)) < 0)
	{
		printk(KERN_ERR "%s, Can't open %s\n",__func__,"/data/img/image.bmp");
		return;
	}
	
	//save_buffer = vmalloc(LCD_BUFFER_SIZE);	
	memset(save_buffer, 0x00, LCD_BUFFER_SIZE);

	for(j=0;j<LCD_MAIN_HEIGHT;j++)
	{
		for(i=0;i<LCD_MAIN_WIDTH;i++)
		{
			if(((i>=(x_start)) && (i<(x_end+1))) && ((j>=(y_start)) && (j<(y_end+1))))
			{
				memcpy(&save_buffer[point], &lge_img_capture_32bpp[(j*LCD_MAIN_WIDTH*4)+i*4], sizeof(byte) * 4);
				point += 4;
			}
		}
	}

	map_header.bfType = 0x4d42;
	map_header.bfSize = point + 66;
	map_header.bfReserved1 = 0x0;
	map_header.bfReserved2 = 0x0;
	map_header.bfOffBits = 0x42;

	info_header.biSize = 0x28;
	info_header.biWidth = x_end - x_start + 1; 
	info_header.biHeight =  (y_end - y_start + 1) * -1;

	info_header.biPlanes = 0x1;
	info_header.biBitCount = 0x20;
	info_header.biCompression = 0x3;
	info_header.biSizeImage = point;
	info_header.biXPelsPerMeter = 0xec4;
	info_header.biYPelsPerMeter = 0xec4;
	info_header.biClrUsed = 0x0;
	info_header.biClrImportant = 0x0;
#if 0	
	printk(KERN_INFO "%s(), sizeof(BITMAPFILEHEADER):%d\n",__func__,sizeof(BITMAPFILEHEADER));
	printk(KERN_INFO "%s(), map_header.bfType:0x%X\n", __func__,(int)map_header.bfType);
	printk(KERN_INFO "%s(), map_header.bfSize:0x%lX\n", __func__,map_header.bfSize);	
	printk(KERN_INFO "%s(), map_header.bfReserved1:0x%X\n", __func__,(int)map_header.bfReserved1);	
	printk(KERN_INFO "%s(), map_header.bfReserved2:0x%X\n", __func__,(int)map_header.bfReserved2);
	printk(KERN_INFO "%s(), map_header.bfOffBits:0x%lX\n", __func__,map_header.bfOffBits);
	
	printk(KERN_INFO "%s(), sizeof(BITMAPINFOHEADER):%d\n",__func__,sizeof(BITMAPINFOHEADER));
	printk(KERN_INFO "%s(), info_header.biSize:0x%lX \n",__func__,info_header.biSize);
	printk(KERN_INFO "%s(), info_header.biWidth:0x%lX\n",__func__,info_header.biWidth);
	printk(KERN_INFO "%s(), info_header.biHeight:0x%lX\n",__func__,info_header.biHeight);
	printk(KERN_INFO "%s(), info_header.biPlanes:0x%X\n",__func__,(int)info_header.biPlanes);
	printk(KERN_INFO "%s(), info_header.biBitCount:0x%X\n",__func__,(int)info_header.biBitCount);
	printk(KERN_INFO "%s(), info_header.biCompression:0x%lX\n",__func__,info_header.biCompression);
	printk(KERN_INFO "%s(), info_header.biSizeImage:0x%lX\n",__func__,info_header.biSizeImage);
	printk(KERN_INFO "%s(), info_header.biXPelsPerMeter:0x%lX\n",__func__,info_header.biXPelsPerMeter);
	printk(KERN_INFO "%s(), info_header.biYPelsPerMeter:0x%lX\n",__func__,info_header.biYPelsPerMeter);
	printk(KERN_INFO "%s(), info_header.biClrUsed:0x%lX\n",__func__,info_header.biClrUsed);
	printk(KERN_INFO "%s(), info_header.biClrImportant:0x%lX\n",__func__,info_header.biClrImportant);
#endif

	sys_write(fbfd, (const char __user *)&map_header, sizeof(BITMAPFILEHEADER));
	sys_write(fbfd, (const char __user *)&info_header, sizeof(BITMAPINFOHEADER));
	sys_write(fbfd, (const char __user *)&header, sizeof(byte)*12);
	
	sys_write(fbfd, save_buffer, point);
	sys_close(fbfd);
	
	set_fs(old_fs);	

	//if(save_buffer != NULL)
	//	vfree(save_buffer);
	
	//if(fb_buffer != NULL)
	//	vfree(fb_buffer);
	
}

#ifdef CONFIG_LGE_DIAG_TESTMODE //2012.05.14 yunjeong.kang Testmode 250-116-6
int read_Framebuffer_Testmode(char *path, unsigned int x, unsigned int y, unsigned int w, unsigned int h)
{
    char *argv[4] = { NULL, NULL, NULL, NULL };
    char *envp[3] = { NULL, NULL, NULL };
    char buf[20] = {0,};
	int ret = -1;

    /* use modified screencap */
    argv[0] = "/system/bin/screencap";
    sprintf(buf, "-c %dx%d+%d+%d", x, y, w, h);
    argv[1] = buf;
	if(path == NULL)
	    argv[2] = "/data/img/mft_lcd_img";
	else
		argv[2] = path;

    envp[0] = "HOME=/";
    envp[1] = "TERM=linux";

    ret = call_usermodehelper( argv[0], argv, envp, UMH_WAIT_PROC );
    return ret;
}
EXPORT_SYMBOL(read_Framebuffer_Testmode);
#endif

#define CPU_PERFORMANCE_PATH 		"/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
#define ONDEMAND 			0
#define PERFORMANCE 		1
int write_cpu_performanc(char * path, int on)
{
	int write;
	int err;
	char str[20];
	mm_segment_t oldfs=get_fs();

	memset(str, 0x0, sizeof(char) * 20);

	set_fs(KERNEL_DS);

	write = sys_open((const char __user *) path, O_WRONLY | O_CREAT | O_TRUNC , 0);

	if(write < 0) {
		printk(KERN_ERR "%s, cpu performance file Open Fail\n",__func__);
		return -1;
	}else {
		printk(KERN_INFO "%s, cpu performance file Open Success\n",__func__);
	}
	if(on)
	{
		sprintf(str, "performance");
	}
	else
	{
		sprintf(str, "ondemand");		
	}
	
	err = sys_write(write, str, strlen(str));

	if(err < 0){
		printk(KERN_ERR "%s, cpu performance file write Fail\n",__func__);
	}
	else {
		printk(KERN_INFO "%s, cpu performance file write Success\n",__func__);		
	}
	sys_close(write);
	set_fs(oldfs);	
	return -1;
}

EXPORT_SYMBOL(write_cpu_performanc);

int read_cpu_performanc(const char *path)
{
	int read;
	int read_size;
	char buf[20];
	mm_segment_t oldfs=get_fs();
	
	set_fs(KERNEL_DS);

	read = sys_open((const char __user *)path, O_RDONLY , 0);

	if(read < 0) {
		printk(KERN_ERR "%s, cpu performance file Open Fail\n",__func__);
		return -1;
	}else {
		printk(KERN_INFO "%s, cpu performance file Open Success\n",__func__);
	}

	memset(buf, 0x0, sizeof(char)*20);	
	read_size = 0;

	while(sys_read(read, &buf[read_size++], 1) == 1){}
		
	if(read_size <= 0){
		printk(KERN_ERR "%s, cpu performance file Read Fail\n",__func__);
		return -1;
	} else {
		printk(KERN_INFO "%s, cpu performance file Read Success\n",__func__);
	}
	set_fs(oldfs);
	sys_close(read);

	if(!strncmp(buf, "ondemand", 8))
	{
		printk(KERN_INFO "%s(), return ondemand\n",__func__);
		return ONDEMAND;
	}
	else
	{
		printk(KERN_INFO "%s(), return performance\n",__func__);
		return PERFORMANCE;
	}
	
	return -1;
}
EXPORT_SYMBOL(read_cpu_performanc);

DIAG_ICD_F_rsp_type *icd_capture_img_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len, packet_len;	
	static short x_start=0, y_start=0, x_end=0, y_end=0;
	static short seq_num = 0;
	DIAG_ICD_F_rsp_type *pRsp;	

	g_diag_slate_capture_rsp_num = 0x01;

	if(pReq->icd_req.capture_req_info.seq_num == 0)
	{
		x_start = pReq->icd_req.capture_req_info.upper_left_x;
		y_start = pReq->icd_req.capture_req_info.upper_left_y;
		x_end = pReq->icd_req.capture_req_info.lower_right_x;
		y_end = pReq->icd_req.capture_req_info.lower_right_y;
		seq_num = pReq->icd_req.capture_req_info.seq_num;
	}
	
	rsp_len = sizeof(icd_screen_capture_rsp_type);
	packet_len = rsp_len;

	printk(KERN_INFO "%s(), x_start:%d, y_start:%d, x_end:%d, y_end:%d, seq_num:%d \n",
		__func__,x_start ,y_start ,x_end ,y_end , seq_num);

	pRsp = (DIAG_ICD_F_rsp_type *)diagpkt_alloc2(DIAG_ICD_F, rsp_len, packet_len);
  	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return NULL;
  	}
	memset(pRsp, 0, rsp_len);


	if(pReq->icd_req.capture_req_info.seq_num == 0)
	{
		read_Framebuffer( x_start,y_start,x_end, y_end);
	}

	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_CAPTUREIMAGE_REQ_CMD;
	pRsp->icd_rsp.capture_rsp_info.screen_id = 0;
	pRsp->icd_rsp.capture_rsp_info.upper_left_x = x_start;
	pRsp->icd_rsp.capture_rsp_info.upper_left_y = y_start;		
	pRsp->icd_rsp.capture_rsp_info.lower_right_x = x_end;
	pRsp->icd_rsp.capture_rsp_info.lower_right_y = y_end;
	pRsp->icd_rsp.capture_rsp_info.bit_per_pixel = 16;
	pRsp->icd_rsp.capture_rsp_info.actual_width = x_end - x_start + 1;
	pRsp->icd_rsp.capture_rsp_info.actual_height = y_end - y_start + 1;
	pRsp->icd_rsp.capture_rsp_info.seq_num = 0;
	pRsp->icd_rsp.capture_rsp_info.flags = LastImageNoHeader;
	memset(pRsp->icd_rsp.capture_rsp_info.image_data_block, 0x0, sizeof(char) * ICD_SEND_SAVE_IMG_PATH_LEN);
	sprintf(pRsp->icd_rsp.capture_rsp_info.image_data_block, "/data/img/image.bmp");

    sys_chmod("/data/img", 0755); // change directory permission to make adb commands available on "data/img"
	sys_chmod("/data/img/image.bmp", 0644);

	return pRsp;
}
/** SAR : Sprint Automation Requirement - END **/
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]

/** ICDR : ICD Implementation Recommendation  - START **/
DIAG_ICD_F_rsp_type *icd_get_airplanemode_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
	
	rsp_len = sizeof(icd_get_airplane_mode_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_get_airplanemode_req_proc\n");
	printk(KERN_INFO "[ICD] icd_get_airplanemode_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_GETAIRPLANEMODE_REQ_CMD;
	
	// get airplane mode info
	if(diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n",__func__);
		update_diagcmd_state(diagpdev, "ICD_GETAIRPLANEMODE", 1);
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
		mdelay(100);
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]
		if(strcmp(process_status,"COMPLETED") == 0)
		{
			if(strcmp(process_value,"GETAIRPLANE_0") == 0)
			{
				pRsp->icd_rsp.get_airplane_mode_rsp_info.airplane_mode = 0;
				printk(KERN_INFO "[ICD] %s was successful : airplan mode is on\n",__func__);
			}
			else if(strcmp(process_value,"GETAIRPLANE_1") == 0)
			{
				pRsp->icd_rsp.get_airplane_mode_rsp_info.airplane_mode = 1;
				printk(KERN_INFO "[ICD] %s was successful : airplan mode is off\n",__func__);
			}
			else
			{
				pRsp->icd_rsp.get_airplane_mode_rsp_info.airplane_mode = 0xFF;
				printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
			}
		}
		else
		{
			pRsp->icd_rsp.get_airplane_mode_rsp_info.airplane_mode = 0xFF;
			printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
		}
	}
	else
	{
		pRsp->icd_rsp.get_airplane_mode_rsp_info.airplane_mode = 0xFF;
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather : Error cannot open diagpdev\n",__func__);
	}
		
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_set_airplanemode_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
	
	rsp_len = sizeof(icd_set_airplane_mode_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_set_airplanemode_req_proc : req mode = %d\n",pReq->icd_req.set_aiplane_mode_req_info.airplane_mode);
	printk(KERN_INFO "[ICD] icd_set_airplanemode_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_SETAIRPLANEMODE_REQ_CMD;
	
	// set airplane mode info
	if(diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n",__func__);
		update_diagcmd_state(diagpdev, "ICD_SETAIRPLANEMODE", pReq->icd_req.set_aiplane_mode_req_info.airplane_mode);
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
		mdelay(300);
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]

		if(strcmp(process_status,"COMPLETED") == 0)
		{
			if(strcmp(process_value,"SETAIRPLANE_0") == 0)
			{
				pRsp->icd_rsp.set_airplane_mode_rsp_info.cmd_status = 0;
				printk(KERN_INFO "[ICD] %s was successful\n",__func__);
			}
			else if(strcmp(process_value,"SETAIRPLANE_1") == 0)
			{
				pRsp->icd_rsp.set_airplane_mode_rsp_info.cmd_status = 1;
				printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
			}
			else
			{
				pRsp->icd_rsp.set_airplane_mode_rsp_info.cmd_status = 0xFF;
				printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
			}
		}
		else
		{
			pRsp->icd_rsp.set_airplane_mode_rsp_info.cmd_status = 0xFF;
			printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
		}
	}
	else
	{
		pRsp->icd_rsp.set_airplane_mode_rsp_info.cmd_status = 0xFF;
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather : Error cannot open diagpdev\n",__func__);
	}
	
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_get_backlight_setting_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
	
	rsp_len = sizeof(icd_get_backlight_setting_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_get_backlight_setting_req_proc\n");
	printk(KERN_INFO "[ICD] icd_get_backlight_setting_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_GETBACKLIGHTSETTING_REQ_CMD;
	
	// get backlight setting info
	if(diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n",__func__);
		update_diagcmd_state(diagpdev, "ICD_GETBACKLIGHTSETTING", 1);
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
		mdelay(100);
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]

		if(strcmp(process_status,"COMPLETED") == 0)
		{
			if(strcmp(process_value,"GETBACKLIGHT_15") == 0)
			{
				pRsp->icd_rsp.get_backlight_setting_rsp_info.item_data = 15;
				printk(KERN_INFO "[ICD] %s was successful : %dsec\n",__func__,pRsp->icd_rsp.get_backlight_setting_rsp_info.item_data);
			}
			else if(strcmp(process_value,"GETBACKLIGHT_30") == 0)
			{
				pRsp->icd_rsp.get_backlight_setting_rsp_info.item_data = 30;
				printk(KERN_INFO "[ICD] %s was successful : %dsec\n",__func__,pRsp->icd_rsp.get_backlight_setting_rsp_info.item_data);
			}
			else if(strcmp(process_value,"GETBACKLIGHT_60") == 0)
			{
				pRsp->icd_rsp.get_backlight_setting_rsp_info.item_data = 60;
				printk(KERN_INFO "[ICD] %s was successful : %dsec\n",__func__,pRsp->icd_rsp.get_backlight_setting_rsp_info.item_data);
			}
			else if(strcmp(process_value,"GETBACKLIGHT_120") == 0)
			{
				pRsp->icd_rsp.get_backlight_setting_rsp_info.item_data = 120;
				printk(KERN_INFO "[ICD] %s was successful : %dsec\n",__func__,pRsp->icd_rsp.get_backlight_setting_rsp_info.item_data);
			}
			else if(strcmp(process_value,"GETBACKLIGHT_600") == 0)
			{
				pRsp->icd_rsp.get_backlight_setting_rsp_info.item_data = 600;
				printk(KERN_INFO "[ICD] %s was successful : %dsec\n",__func__,pRsp->icd_rsp.get_backlight_setting_rsp_info.item_data);
			}
			else if(strcmp(process_value,"GETBACKLIGHT_1800") == 0)
			{
				pRsp->icd_rsp.get_backlight_setting_rsp_info.item_data = 1800;
				printk(KERN_INFO "[ICD] %s was successful : %dsec\n",__func__,pRsp->icd_rsp.get_backlight_setting_rsp_info.item_data);
			}
			else if(strcmp(process_value,"GETBACKLIGHT_ALWAY") == 0)
			{
				pRsp->icd_rsp.get_backlight_setting_rsp_info.item_data = 100;
				printk(KERN_INFO "[ICD] %s was successful : %dsec\n",__func__,pRsp->icd_rsp.get_backlight_setting_rsp_info.item_data);
			}
			else
			{
				pRsp->icd_rsp.get_backlight_setting_rsp_info.item_data = 0x00;
				printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
			}
		}
		else
		{
			pRsp->icd_rsp.get_backlight_setting_rsp_info.item_data = 0x00;
			printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
		}
	}
	else
	{
		pRsp->icd_rsp.get_backlight_setting_rsp_info.item_data = 0x00;
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather : Error cannot open diagpdev\n",__func__);
	}
	
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_set_backlight_setting_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
	int retry_num = 0;
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]
	
	rsp_len = sizeof(icd_set_backlight_setting_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_set_backlight_setting_req_proc, req = %d\n",pReq->icd_req.set_backlight_setting_req_info.item_data);
	printk(KERN_INFO "[ICD] icd_set_backlight_setting_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_SETBACKLIGHTSETTING_REQ_CMD;
	
	// set backlight setting info
	if(diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n",__func__);
		update_diagcmd_state(diagpdev, "ICD_SETBACKLIGHTSETTING", pReq->icd_req.set_backlight_setting_req_info.item_data);
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
		mdelay(200);

		while(strcmp(process_status,"START") == 0)
		{
			mdelay(100);			
			printk(KERN_INFO "[ICD_BRIGHTNESS] %s wait again, Retry_num =%d \n",__func__,retry_num);
			retry_num++;
			
			if(retry_num==3)	
				break;
		}
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]
		
		if(strcmp(process_status,"COMPLETED") == 0)
		{
			if(strcmp(process_value,"SETBACKLIGHT_0") == 0)
			{
				pRsp->icd_rsp.set_backlight_setting_rsp_info.cmd_status = 0;
				printk(KERN_INFO "[ICD] %s was successful\n",__func__);
			}
			else if(strcmp(process_value,"SETBACKLIGHT_1") == 0)
			{
				pRsp->icd_rsp.set_backlight_setting_rsp_info.cmd_status = 1;
				printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
			}
			else
			{
				pRsp->icd_rsp.set_backlight_setting_rsp_info.cmd_status = 0xFF;
				printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
			}
		}
		else
		{
			pRsp->icd_rsp.set_backlight_setting_rsp_info.cmd_status = 0xFF;
			printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
		}
	}
	else
	{
		pRsp->icd_rsp.set_backlight_setting_rsp_info.cmd_status = 0xFF;
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather : Error cannot open diagpdev\n",__func__);
	}
	
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_get_batterycharging_state_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
	
	rsp_len = sizeof(icd_get_battery_charging_state_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_get_batterycharging_state_req_proc\n");
	printk(KERN_INFO "[ICD] icd_get_batterycharging_state_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_GETBATTERYCHARGINGSTATE_REQ_CMD;
	
	// get battery charging info
	
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_set_batterycharging_state_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
	
	rsp_len = sizeof(icd_set_battery_charging_state_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_set_batterycharging_state_req_proc\n");
	printk(KERN_INFO "[ICD] icd_set_batterycharging_state_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_SETBATTERYCHARGINGSTATE_REQ_CMD;
	
	// set battery charging info
	
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_get_battery_level_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	unsigned int crc_val =0;
	DIAG_ICD_F_rsp_type *pRsp;
	
	rsp_len = sizeof(icd_get_battery_level_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_get_batterylevel_req_proc req \n");
	printk(KERN_INFO "[ICD] icd_info_req_proc rsp_len :(%d)\n", rsp_len);
//[dugyung.ahn@lge.com] 2012-05-01, [Slate] GETBATTERYLEVEL 

	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_GETBATTERYLEVEL_REQ_CMD;
//[dugyung.ahn@lge.com] 2012-05-01, [Slate] GETBATTERYLEVEL [s]
	if(diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n",__func__);
		update_diagcmd_state(diagpdev, "ICD_GETBATTERYLEVEL",  1); 
	}
	mdelay(100);
	//memset(pRsp->icd_rsp.get_battery_level_rsp_info.battery_level, 0x0, ICD_MAX_STRING);
	if(strcmp(process_value,"100") == 0)
	{
		pRsp->icd_rsp.get_battery_level_rsp_info.battery_level = 100;
	}
	else
	{
		crc_val = simple_strtoul(process_value,NULL,10);
		pRsp->icd_rsp.get_battery_level_rsp_info.battery_level = crc_val;
		//strcpy(pRsp->icd_rsp.get_battery_level_rsp_info.battery_level, process_value);
		//pRsp->icd_rsp.get_battery_level_rsp_info.battery_level = process_value
	}
//[dugyung.ahn@lge.com] 2012-05-01, [Slate] GETBATTERYLEVEL [e]	
	printk(KERN_INFO "[ICD]  crc_val :   %d    battery_level :  %d    process_value  : %s  \n",crc_val , pRsp->icd_rsp.get_battery_level_rsp_info.battery_level, process_value);
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_get_bluetooth_status_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
	
	rsp_len = sizeof(icd_get_bluetooth_status_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_get_bluetooth_status_req_proc\n");
	printk(KERN_INFO "[ICD] icd_get_bluetooth_status_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_GETBLUETOOTHSTATUS_REQ_CMD;
	
	// get bluetooth status info
	if (diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n", __func__);
		update_diagcmd_state(diagpdev, "ICD_GETBLUETOOTH", 1);
		mdelay(50);
//LG_BTUI_S
// new
        if (strcmp(process_status, "COMPLETED") == 0)
        {		
            if (strcmp(process_value, "GETBLUETOOTH_0x11") == 0)
            {
                pRsp->icd_rsp.get_bluetooth_status_rsp_info.bluetooth_status = 3;
                printk(KERN_INFO "[ICD] %s was successful : status = %d\n", __func__, pRsp->icd_rsp.get_bluetooth_status_rsp_info.bluetooth_status);
            }
            else if (strcmp(process_value, "GETBLUETOOTH_0x01") == 0)
            {
                pRsp->icd_rsp.get_bluetooth_status_rsp_info.bluetooth_status = 1;
                printk(KERN_INFO "[ICD] %s was successful : status = %d\n", __func__, pRsp->icd_rsp.get_bluetooth_status_rsp_info.bluetooth_status);
            }
            else if (strcmp(process_value, "GETBLUETOOTH_0x00") == 0)
            {
                pRsp->icd_rsp.get_bluetooth_status_rsp_info.bluetooth_status = 0;
                printk(KERN_INFO "[ICD] %s was successful : status = %d\n", __func__, pRsp->icd_rsp.get_bluetooth_status_rsp_info.bluetooth_status);
            }			
            else
            {
                pRsp->icd_rsp.get_bluetooth_status_rsp_info.bluetooth_status = 0xFF;
                printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n", __func__);
            }
        }
        else
        {
            pRsp->icd_rsp.get_bluetooth_status_rsp_info.bluetooth_status = 0xFF;
            printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n", __func__);
        }
// old
/*
		if (strcmp(process_status, "COMPLETED") == 0)
		{		
			if (strcmp(process_value, "GETBLUETOOTH_0") == 0)
			{
				pRsp->icd_rsp.get_bluetooth_status_rsp_info.bluetooth_status = 0;
				printk(KERN_INFO "[ICD] %s was successful : status = %d\n",__func__,pRsp->icd_rsp.get_bluetooth_status_rsp_info.bluetooth_status);
			}
			else if (strcmp(process_value, "GETBLUETOOTH_1") == 0)
			{
				pRsp->icd_rsp.get_bluetooth_status_rsp_info.bluetooth_status = 1;
				printk(KERN_INFO "[ICD] %s was successful : status = %d\n",__func__,pRsp->icd_rsp.get_bluetooth_status_rsp_info.bluetooth_status);
			}
			else
			{
				pRsp->icd_rsp.get_bluetooth_status_rsp_info.bluetooth_status = 0xFF;
				printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
			}
		}
		else
		{
			pRsp->icd_rsp.get_bluetooth_status_rsp_info.bluetooth_status = 0xFF;
			printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
		}
*/
//LG_BTUI_E
	}
	else
	{
		pRsp->icd_rsp.get_bluetooth_status_rsp_info.bluetooth_status = 0xFF;
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather : Error cannot open diagpdev\n",__func__);
	}
	
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_set_bluetooth_status_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
//LG_BTUI_S
    unsigned int i = 0;
//LG_BTUI_E
	rsp_len = sizeof(icd_set_bluetooth_status_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_set_bluetooth_status_req_proc\n");
	printk(KERN_INFO "[ICD] icd_set_bluetooth_status_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_SETBLUETOOTHSTATUS_REQ_CMD;
	
	// set bluetooth status info
	if (diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n", __func__);
		update_diagcmd_state(diagpdev, "ICD_SETBLUETOOTH", pReq->icd_req.set_bluetooth_status_req_info.bluetooth_status);
		mdelay(50);
//LG_BTUI_S
// new
        while (i < 100)
		{
			if (strcmp(process_status, "COMPLETED") == 0)
			{
			    printk(KERN_INFO "[ICD] %s COMPLETED!!!\n", __func__);
			    break;
            }
			mdelay(50);
			i++;
        }

        if (strcmp(process_status, "COMPLETED") == 0)
        {
            if (strcmp(process_value, "SETBLUETOOTH_0") == 0)
            {
                pRsp->icd_rsp.set_bluetooth_status_rsp_info.cmd_status = 0;
                printk(KERN_INFO "[ICD] %s was successful \n", __func__);
            }
            else if (strcmp(process_value, "SETBLUETOOTH_1") == 0)
            {
                pRsp->icd_rsp.set_bluetooth_status_rsp_info.cmd_status = 1;
                printk(KERN_INFO "[ICD] %s was unsuccessful\n", __func__);
            }
            else
            {
                pRsp->icd_rsp.set_bluetooth_status_rsp_info.cmd_status = 0xFF;
                printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n", __func__);
            }
        }
        else
        {
            pRsp->icd_rsp.set_bluetooth_status_rsp_info.cmd_status = 0xFF;
            printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n", __func__);
        }
// old
/*
		mdelay(50);

		if (strcmp(process_status,"COMPLETED") == 0)
		{		
			if (strcmp(process_value,"SETBLUETOOTH_0") == 0)
			{
				pRsp->icd_rsp.set_bluetooth_status_rsp_info.cmd_status = 0;
				printk(KERN_INFO "[ICD] %s was successful \n", __func__);
			}
			else if (strcmp(process_value,"SETBLUETOOTH_1") == 0)
			{
				pRsp->icd_rsp.set_bluetooth_status_rsp_info.cmd_status = 1;
				printk(KERN_INFO "[ICD] %s was unsuccessful\n", __func__);
			}
			else
			{
				pRsp->icd_rsp.set_bluetooth_status_rsp_info.cmd_status = 0xFF;
				printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n", __func__);
			}
		}
		else
		{
			pRsp->icd_rsp.set_bluetooth_status_rsp_info.cmd_status = 0xFF;
			printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n", __func__);
		}
*/
//LG_BTUI_E
	}
	else
	{
		pRsp->icd_rsp.set_bluetooth_status_rsp_info.cmd_status = 0xFF;
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather : Error cannot open diagpdev\n", __func__);
	}
	
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_get_gps_status_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
	
	rsp_len = sizeof(icd_get_gps_status_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_get_gps_status_req_proc\n");
	printk(KERN_INFO "[ICD] icd_get_gps_status_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_GETGPSSTATUS_REQ_CMD;
	
	// get gps status info
	
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_set_gps_status_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
	
	rsp_len = sizeof(icd_set_gps_status_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_set_gps_status_req_proc\n");
	printk(KERN_INFO "[ICD] icd_set_gps_status_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_SETGPSSTATUS_REQ_CMD;
	
	// set gps status info
	
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_get_keypadbacklight_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
	
	rsp_len = sizeof(icd_get_keypadbacklight_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_get_keypadbacklight_req_proc\n");
	printk(KERN_INFO "[ICD] icd_get_keypadbacklight_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_GETKEYPADBACKLIGHT_REQ_CMD;
	
	// get keypadbacklight status info
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
	if(diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n",__func__);
		update_diagcmd_state(diagpdev, "ICD_GETKEYPADBACKLIGHT", 1);
		mdelay(200);

		if(strcmp(process_status,"COMPLETED") == 0)
		{
			if(strcmp(process_value,"GETKEYPADBACKLIGHT_15") == 0)
			{
				pRsp->icd_rsp.get_keypadbacklight_rsp_info.keypad_backlight = 15;
				printk(KERN_INFO "[ICD] %s was successful : %dsec\n",__func__,pRsp->icd_rsp.get_keypadbacklight_rsp_info.keypad_backlight);
			}
//[dugyung.ahn@lge.com] 2012-05-01, [Slate] GETKEYPADBACKLIGHT  [s]
			else if(strcmp(process_value,"GETKEYPADBACKLIGHT_30") == 0)
			{
				pRsp->icd_rsp.get_keypadbacklight_rsp_info.keypad_backlight = 30;
				printk(KERN_INFO "[ICD] %s was successful : %dsec\n",__func__,pRsp->icd_rsp.get_keypadbacklight_rsp_info.keypad_backlight);
			}
//[dugyung.ahn@lge.com] 2012-05-01, [Slate] GETKEYPADBACKLIGHT [e]
			else if(strcmp(process_value,"GETKEYPADBACKLIGHT_50") == 0)
			{
				pRsp->icd_rsp.get_keypadbacklight_rsp_info.keypad_backlight = 50;
				printk(KERN_INFO "[ICD] %s was successful : %dsec\n",__func__,pRsp->icd_rsp.get_keypadbacklight_rsp_info.keypad_backlight);
			}
//[dugyung.ahn@lge.com] 2012-05-01, [Slate] GETKEYPADBACKLIGHT [s]			
//			else if(strcmp(process_value,"GETKEYPADBACKLIGHT_NEVER") == 0)
//			{
//				pRsp->icd_rsp.get_keypadbacklight_rsp_info.keypad_backlight = 0;
//				printk(KERN_INFO "[ICD] %s was successful : %dsec\n",__func__,pRsp->icd_rsp.get_keypadbacklight_rsp_info.keypad_backlight);
//			}
//[dugyung.ahn@lge.com] 2012-05-01, [Slate] GETKEYPADBACKLIGHT [e]			
			else if(strcmp(process_value,"GETKEYPADBACKLIGHT_ALWAYS") == 0)
			{
				pRsp->icd_rsp.get_keypadbacklight_rsp_info.keypad_backlight = 100;
				printk(KERN_INFO "[ICD] %s was successful : %dsec\n",__func__,pRsp->icd_rsp.get_keypadbacklight_rsp_info.keypad_backlight);
			}
			else
			{
				pRsp->icd_rsp.get_keypadbacklight_rsp_info.keypad_backlight = 0x00;
				printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
			}
		}
		else
		{
			pRsp->icd_rsp.get_keypadbacklight_rsp_info.keypad_backlight = 0x00;
			printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
		}
	}
	else
	{
		pRsp->icd_rsp.get_keypadbacklight_rsp_info.keypad_backlight = 0x00;
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather : Error cannot open diagpdev\n",__func__);
	}
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]
	
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_set_keypadbacklight_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
	
	rsp_len = sizeof(icd_set_keypadbacklight_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_set_keypadbacklight_req_proc\n");
	printk(KERN_INFO "[ICD] icd_set_keypadbacklight_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_SETKEYPADBACKLIGHT_REQ_CMD;
	
	// set keypadbacklight status info
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
	if(diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n",__func__);
		update_diagcmd_state(diagpdev, "ICD_SETKEYPADBACKLIGHT", pReq->icd_req.set_keypadbacklight_req_info.keypad_backlight);
		mdelay(200);

		if(strcmp(process_status,"COMPLETED") == 0)
		{
			if(strcmp(process_value,"SETKEYPADBACKLIGHT_0") == 0)
			{
				pRsp->icd_rsp.set_keypadbacklight_rsp_info.cmd_status = 0;
				printk(KERN_INFO "[ICD] %s was successful\n",__func__);
			}
			else if(strcmp(process_value,"SETKEYPADBACKLIGHT_1") == 0)
			{
				pRsp->icd_rsp.set_keypadbacklight_rsp_info.cmd_status = 1;
				printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
			}
			else
			{
				pRsp->icd_rsp.set_keypadbacklight_rsp_info.cmd_status = 0xFF;
				printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
			}
		}
		else
		{
			pRsp->icd_rsp.set_keypadbacklight_rsp_info.cmd_status = 0xFF;
			printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
		}
	}
	else
	{
		pRsp->icd_rsp.set_keypadbacklight_rsp_info.cmd_status = 0xFF;
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather : Error cannot open diagpdev\n",__func__);
	}
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]
	
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_get_roamingmode_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
	
	rsp_len = sizeof(icd_get_roamingmode_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_get_roamingmode_req_proc\n");
	printk(KERN_INFO "[ICD] icd_get_roamingmode_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_GETROAMINGMODE_REQ_CMD;
	
	// get roaming mode info
	
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_get_rssi_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
	int rssi, ecio, no_of_bar, status;  	/* [dugyung.ahn@lge.com] 2012-06-27, [Slate] GETRSSI  */ 
	
	rsp_len = sizeof(icd_get_rssi_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_get_rssi_req_proc\n");
	printk(KERN_INFO "[ICD] icd_get_rssi_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_GETRSSI_REQ_CMD;
	
	// get RSSI info
//[dugyung.ahn@lge.com] 2012-06-27, [Slate] GETRSSI [s] 	
	if(diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n",__func__);
		update_diagcmd_state(diagpdev, "ICD_GETRSSI", 0);
		mdelay(100);
	
		if(strcmp(process_status,"COMPLETED") == 0)
		{
			// printk(KERN_INFO "[ICD] %s process_value = %s\n",__func__, process_value);

			sscanf(process_value, "%d %d %d %d", &rssi, &ecio, &no_of_bar, &status);
			pRsp->icd_rsp.get_rssi_rsp_info.rx_power = rssi*100; /* multiplied by 100 due to req. */
			pRsp->icd_rsp.get_rssi_rsp_info.rx_ec_io = ecio*10; /* multiplied by 100 due to req. & already multiplied by 10 */
			pRsp->icd_rsp.get_rssi_rsp_info.numbar = no_of_bar;
			pRsp->icd_rsp.get_rssi_rsp_info.status = status;
	
			printk(KERN_INFO "[ICD] %s rssi = %d(%d)\n",__func__, pRsp->icd_rsp.get_rssi_rsp_info.rx_power, rssi);
			printk(KERN_INFO "[ICD] %s ecio = %d(%d)\n",__func__, pRsp->icd_rsp.get_rssi_rsp_info.rx_ec_io, ecio);
			printk(KERN_INFO "[ICD] %s bars = %d(%d)\n",__func__, pRsp->icd_rsp.get_rssi_rsp_info.numbar, no_of_bar);
			printk(KERN_INFO "[ICD] %s status = %d(%d)\n",__func__, pRsp->icd_rsp.get_rssi_rsp_info.status, status);
		}
		else
		{			
			printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
		}
	}
	else
	{		
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather : Error cannot open diagpdev\n",__func__);
	}	
//[dugyung.ahn@lge.com] 2012-06-27, [Slate] GETRSSI [e] 	
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_get_latitude_longitude_values_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
	
	rsp_len = sizeof(icd_get_latitude_longitude_values_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_get_latitude_longitude_values_rsp_type\n");
	printk(KERN_INFO "[ICD] icd_get_latitude_longitude_values_rsp_type rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_GETLATITUDELONGITUDEVALUES_REQ_CMD;
	
	// get RSSI info
	
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_get_state_connect_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
	
	rsp_len = sizeof(icd_get_state_connect_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_get_state_connect_req_proc\n");
	printk(KERN_INFO "[ICD] icd_get_state_connect_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_GETSTATEANDCONNECTIONATTEMPTS_REQ_CMD;
	
	// get state and connection attempts info
	
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_get_ui_screen_id_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
	
	rsp_len = sizeof(icd_get_ui_screen_id_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_get_ui_screen_id_req_proc\n");
	printk(KERN_INFO "[ICD] icd_get_ui_screen_id_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_GETUISCREENID_REQ_CMD;
	
	// get ui screen id 
	if(diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n",__func__);
		update_diagcmd_state(diagpdev, "ICD_GETUISCREENID", pReq->icd_req.get_ui_srceen_id_req_info.physical_screen);
		mdelay(50);

		if(strcmp(process_status,"COMPLETED") == 0)
		{
			if(strcmp(process_value,"GETUISCREENID_1") == 0)
			{
				pRsp->icd_rsp.get_ui_screen_id_rsp_info.ui_screen_id = 1;
				printk(KERN_INFO "[ICD] %s was successful\n",__func__);
			}
			else if(strcmp(process_value,"GETUISCREENID_2") == 0)
			{
				pRsp->icd_rsp.get_ui_screen_id_rsp_info.ui_screen_id = 2;
				printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
			}
			else if(strcmp(process_value,"GETUISCREENID_3") == 0)
			{
				pRsp->icd_rsp.get_ui_screen_id_rsp_info.ui_screen_id = 3;
				printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
			}
			else if(strcmp(process_value,"GETUISCREENID_4") == 0)
			{
				pRsp->icd_rsp.get_ui_screen_id_rsp_info.ui_screen_id = 4;
				printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
			}
			else if(strcmp(process_value,"GETUISCREENID_5") == 0)
			{
				pRsp->icd_rsp.get_ui_screen_id_rsp_info.ui_screen_id = 5;
				printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
			}
			else if(strcmp(process_value,"GETUISCREENID_6") == 0)
			{
				pRsp->icd_rsp.get_ui_screen_id_rsp_info.ui_screen_id = 6;
				printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
			}
			else if(strcmp(process_value,"GETUISCREENID_7") == 0)
			{
				pRsp->icd_rsp.get_ui_screen_id_rsp_info.ui_screen_id = 7;
				printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
			}
			else
			{
				pRsp->icd_rsp.get_ui_screen_id_rsp_info.ui_screen_id = 0xFF;
				printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
			}
		}
		else
		{
			pRsp->icd_rsp.get_ui_screen_id_rsp_info.ui_screen_id = 0xFF;
			printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
		}
	}
	else
	{
		pRsp->icd_rsp.get_ui_screen_id_rsp_info.ui_screen_id = 0xFF;
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather : Error cannot open diagpdev\n",__func__);
	}
	pRsp->icd_rsp.get_ui_screen_id_rsp_info.physical_screen = 0;
		
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_get_wifi_status_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
	
	rsp_len = sizeof(icd_get_wifi_status_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_get_wifi_status_req_proc\n");
	printk(KERN_INFO "[ICD] icd_get_wifi_status_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_GETWIFISTATUS_REQ_CMD;

	// get wifi status
	if(diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n",__func__);
		update_diagcmd_state(diagpdev, "ICD_GETWIFISTATUS", 1);
		mdelay(50);

		if(strcmp(process_status,"COMPLETED") == 0)
		{
			if(strcmp(process_value,"GETWIFISTATUS_0") == 0)
			{
				pRsp->icd_rsp.get_wifi_status_rsp_info.wifi_status = 0;
				printk(KERN_INFO "[ICD] %s was successful : wifi status = %d\n",__func__,pRsp->icd_rsp.get_wifi_status_rsp_info.wifi_status);
			}
			else if(strcmp(process_value,"GETWIFISTATUS_1") == 0)
			{
				pRsp->icd_rsp.get_wifi_status_rsp_info.wifi_status = 1;
				printk(KERN_INFO "[ICD] %s was successful : wifi status = %d\n",__func__,pRsp->icd_rsp.get_wifi_status_rsp_info.wifi_status);
			}
			else
			{
				pRsp->icd_rsp.get_wifi_status_rsp_info.wifi_status = 0xFF;
				printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
			}
		}
		else
		{
			pRsp->icd_rsp.get_wifi_status_rsp_info.wifi_status = 0xFF;
			printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
		}
	}
	else
	{
		pRsp->icd_rsp.get_wifi_status_rsp_info.wifi_status = 0xFF;
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather : Error cannot open diagpdev\n",__func__);
	}
	
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_set_wifi_status_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;
	
	rsp_len = sizeof(icd_set_wifi_status_rsp_type);
	
	printk(KERN_INFO "[ICD] icd_set_wifi_status_req_proc, req = %d\n", pReq->icd_req.set_wifi_status_req_info.wifi_status);
	printk(KERN_INFO "[ICD] icd_set_wifi_status_req_proc rsp_len :(%d)\n", rsp_len);
	
	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}
	
	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_SETWIFISTATUS_REQ_CMD;
	
	// set wifi status
	if(diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n",__func__);
		update_diagcmd_state(diagpdev, "ICD_SETWIFISTATUS", pReq->icd_req.set_wifi_status_req_info.wifi_status);
		mdelay(50);

		if(strcmp(process_status,"COMPLETED") == 0)
		{
			if(strcmp(process_value,"SETWIFISTATUS_0") == 0)
			{
				pRsp->icd_rsp.set_wifi_status_rsp_info.cmd_status = 0;
				printk(KERN_INFO "[ICD] %s was successful \n",__func__);
			}
			else if(strcmp(process_value,"SETWIFISTATUS_1") == 0)
			{
				pRsp->icd_rsp.set_wifi_status_rsp_info.cmd_status = 1;
				printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
			}
			else
			{
				pRsp->icd_rsp.set_wifi_status_rsp_info.cmd_status = 0xFF;
				printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
			}
		}
		else
		{
			pRsp->icd_rsp.set_wifi_status_rsp_info.cmd_status = 0xFF;
			printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
		}
	}
	else
	{
		pRsp->icd_rsp.set_wifi_status_rsp_info.cmd_status = 0xFF;
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather : Error cannot open diagpdev\n",__func__);
	}
	
	return pRsp;
}

DIAG_ICD_F_rsp_type *icd_set_screenorientationlock_req_proc(DIAG_ICD_F_req_type * pReq)	
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;

	rsp_len = sizeof(icd_set_screenorientationlock_rsp_type);
		
	printk(KERN_INFO "[ICD] %s req = %d\n", __func__,pReq->icd_req.set_screenorientationlock_req_info.orientation_mode);
	printk(KERN_INFO "[ICD] %s rsp_len :(%d)\n", __func__, rsp_len);

	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}

	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_SETSCREENORIENTATIONLOCK_REQ_CMD;
		
	// set screen orientation lock
	
	if(diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n",__func__);
		update_diagcmd_state(diagpdev, "ICD_SETORIENTATIONLOCK", pReq->icd_req.set_screenorientationlock_req_info.orientation_mode);
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
		mdelay(200);
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]
		if(strcmp(process_status,"COMPLETED") == 0)
		{
			if(strcmp(process_value,"SETORIENTATION_0") == 0)
			{
				pRsp->icd_rsp.set_screenorientation_rsp_info.cmd_status = 0;
				printk(KERN_INFO "[ICD] %s was successful\n",__func__);
			}
			else if(strcmp(process_value,"SETORIENTATION_1") == 0)
			{
				pRsp->icd_rsp.set_screenorientation_rsp_info.cmd_status = 1;
				printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
			}
			else
			{
				pRsp->icd_rsp.set_screenorientation_rsp_info.cmd_status = 0xFF;
				printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
			}
		}
		else
		{
			pRsp->icd_rsp.set_screenorientation_rsp_info.cmd_status = 0xFF;
			printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
		}
	}
	else
	{
		pRsp->icd_rsp.set_screenorientation_rsp_info.cmd_status = 0xFF;
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather : Error cannot open diagpdev\n",__func__);
	}
	
	return pRsp;
}

//  20111125 [begin] suhyun.lee@lge.com SLATE ICD 231,232,238 Merge From LS840 
DIAG_ICD_F_rsp_type* icd_get_usbdebug_status_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;	
	DIAG_ICD_F_rsp_type *pRsp;

	rsp_len = sizeof(icd_get_usbdebug_status_rsp_type);

	printk(KERN_INFO "[ICD] icd_get_usbdebug_status_req_proc req \n");
	printk(KERN_INFO "[ICD] icd_info_req_proc rsp_len :(%d)\n", rsp_len);

	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}	

	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_GETUSBDEBUGSTATUSSTATUS_REQ_CMD;

	// get usbdebug status
	
	if(diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n",__func__);
		update_diagcmd_state(diagpdev, "ICD_GETUSBDEBUGSTATUS", 1); 
		mdelay(50);
		if(strcmp(process_status,"COMPLETED") == 0)
		{
			if(strcmp(process_value,"GETUSBDEBUGSTATUS_0") == 0)
			{
				pRsp->icd_rsp.get_usbdebug_status_rsp_info.usbdebug_status = 0;
				printk(KERN_INFO "[ICD] %s was successful\n",__func__);
			}
			else if(strcmp(process_value,"GETUSBDEBUGSTATUS_1") == 0)
			{
				pRsp->icd_rsp.get_usbdebug_status_rsp_info.usbdebug_status = 1;
				printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
			}
			else
			{
				pRsp->icd_rsp.get_usbdebug_status_rsp_info.usbdebug_status = 0xFF;
				printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
			}
		}
		else
		{
			pRsp->icd_rsp.get_usbdebug_status_rsp_info.usbdebug_status = 0xFF;
			printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
		}
	}
	else
	{
		pRsp->icd_rsp.get_usbdebug_status_rsp_info.usbdebug_status = 0xFF;
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather : Error cannot open diagpdev\n",__func__);
	}
		
	return pRsp;
}
DIAG_ICD_F_rsp_type* icd_set_usbdebug_status_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;	
	DIAG_ICD_F_rsp_type *pRsp;

	rsp_len = sizeof(icd_set_usbdebug_status_rsp_type);

	printk(KERN_INFO "[ICD] icd_set_usbdebug_status_req_proc : req mode = %d\n",pReq->icd_req.set_usbdebug_status_req_info.usbdebug_status);
	printk(KERN_INFO "[ICD] icd_info_req_proc rsp_len :(%d)\n", rsp_len);

	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}	

	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_SETUSBDEBUGSTATUSSTATUS_REQ_CMD;

	// set usbdebug status
	
	if(diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n",__func__);
		update_diagcmd_state(diagpdev, "ICD_SETUSBDEBUGSTATUS",  pReq->icd_req.set_usbdebug_status_req_info.usbdebug_status); 
		mdelay(50);
		if(strcmp(process_status,"COMPLETED") == 0)
		{
			if(strcmp(process_value,"SETUSBDEBUGSTATUS_0") == 0)
			{
				pRsp->icd_rsp.set_usbdebug_status_rsp_info.cmd_status = 0;
				printk(KERN_INFO "[ICD] %s was successful\n",__func__);
			}
			else if(strcmp(process_value,"SETUSBDEBUGSTATUS_1") == 0)
			{
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
				pRsp->icd_rsp.set_usbdebug_status_rsp_info.cmd_status = 0;
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]
				printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
			}
			else
			{
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
				pRsp->icd_rsp.set_usbdebug_status_rsp_info.cmd_status = 1;
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]
				printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid 1\n",__func__);
			}
		}
		else
		{
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
			pRsp->icd_rsp.set_usbdebug_status_rsp_info.cmd_status = 1;
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]
			printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid 2\n",__func__);
		}
	}
	else
	{
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
		pRsp->icd_rsp.set_usbdebug_status_rsp_info.cmd_status = 1;
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather : Error cannot open diagpdev\n",__func__);
	}
	
	return pRsp;
}

DIAG_ICD_F_rsp_type* icd_get_screenlock_status_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;	
	DIAG_ICD_F_rsp_type *pRsp;

	rsp_len = sizeof(icd_get_screenlock_status_rsp_type);

	printk(KERN_INFO "[ICD] icd_get_screenlock_status_req_proc req \n");
	printk(KERN_INFO "[ICD] icd_info_req_proc rsp_len :(%d)\n", rsp_len);

	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}	

	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_GETSCREENLOCKSTATUS_REQ_CMD;

	// get screenlock status
	
	if(diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n",__func__);
		update_diagcmd_state(diagpdev, "ICD_GETSCREENLOCKSTATUS", 1); 
		mdelay(50);
		if(strcmp(process_status,"COMPLETED") == 0)
		{
			if(strcmp(process_value,"GETSCREENLOCKSTATUS_0") == 0)
			{
				pRsp->icd_rsp.get_screenlock_status_rsp_info.screenlock_status = 0;
				printk(KERN_INFO "[ICD] %s was successful\n",__func__);
			}
			else if(strcmp(process_value,"GETSCREENLOCKSTATUS_1") == 0)
			{
				pRsp->icd_rsp.get_screenlock_status_rsp_info.screenlock_status = 1;
				printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
			}
			else
			{
				pRsp->icd_rsp.get_screenlock_status_rsp_info.screenlock_status = 0xFF;
				printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
			}
		}
		else
		{
			pRsp->icd_rsp.get_screenlock_status_rsp_info.screenlock_status = 0xFF;
			printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
		}
	}
	else
	{
		pRsp->icd_rsp.get_screenlock_status_rsp_info.screenlock_status = 0xFF;
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather : Error cannot open diagpdev\n",__func__);
	}
		
	return pRsp;
}
DIAG_ICD_F_rsp_type* icd_set_screenlock_status_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;	
	DIAG_ICD_F_rsp_type *pRsp;

	rsp_len = sizeof(icd_set_screenlock_status_rsp_type);

	printk(KERN_INFO "[ICD] icd_set_screenlock_status_req_proc req \n");
	printk(KERN_INFO "[ICD] icd_set_screenlock_status_req_proc : req mode = %d\n",pReq->icd_req.set_screenlock_status_req_info.screenlock_status);
	printk(KERN_INFO "[ICD] icd_info_req_proc rsp_len :(%d)\n", rsp_len);

	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}	

	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_SETSCREENLOCKSTATUS_REQ_CMD;

	// set screenlock status
	
	if(diagpdev != NULL)
	{
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather\n",__func__);
		update_diagcmd_state(diagpdev, "ICD_SETSCREENLOCKSTATUS",  pReq->icd_req.set_screenlock_status_req_info.screenlock_status); 
		mdelay(50);
		if(strcmp(process_status,"COMPLETED") == 0)
		{
			if(strcmp(process_value,"SETSCREENLOCKSTATUS_0") == 0)
			{
				pRsp->icd_rsp.set_screenlock_status_rsp_info.cmd_status = 0;
				printk(KERN_INFO "[ICD] %s was successful\n",__func__);
			}
			else if(strcmp(process_value,"SETSCREENLOCKSTATUS_1") == 0)
			{
				pRsp->icd_rsp.set_screenlock_status_rsp_info.cmd_status = 1;
				printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
			}
			else if(strcmp(process_value,"SETSCREENLOCKSTATUS_2") == 0)
			{
				pRsp->icd_rsp.set_screenlock_status_rsp_info.cmd_status = 2;
				printk(KERN_INFO "[ICD] %s was unsuccessful\n",__func__);
			}
			else
			{
				pRsp->icd_rsp.set_screenlock_status_rsp_info.cmd_status = 0xFF;
				printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
			}
		}
		else
		{
			pRsp->icd_rsp.set_screenlock_status_rsp_info.cmd_status = 0xFF;
			printk(KERN_INFO "[ICD] %s return value from DiagCommandDispather is invalid\n",__func__);
		}
	}
	else
	{
		pRsp->icd_rsp.set_screenlock_status_rsp_info.cmd_status = 0xFF;
		printk(KERN_INFO "[ICD] %s goto DiagCommandDispather : Error cannot open diagpdev\n",__func__);
	}
			
	return pRsp;
}

//LGE_CHANGE jinhwan.do 20120430 Slate command's concept is cahnged, USB Serial number is MEID's decimal value [Start]
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
DIAG_ICD_F_rsp_type* icd_get_android_identifier_req_proc(DIAG_ICD_F_req_type * pReq)
{
	unsigned int rsp_len;
	DIAG_ICD_F_rsp_type *pRsp;

	//yckim.kim@lge.com 2012.04.05 : GetAndroidIdentifier cmd add [START]
	int id_fd;
	int read_size=0;
	char id_buf[34];

	memset(id_buf,0,sizeof(char) * 34);  //yckim.kim@lge.com 2012.05.03: garbage charactor remove
	//yckim.kim@lge.com 2012.04.05 : GetAndroidIdentifier cmd add [END]

	rsp_len = sizeof(icd_get_android_identifier_rsp_type);


	//printk( "[ICD] get android identifier req \n");  //yckim.kim@lge.com 2012.05.03: garbage charactor remove
	//printk( "[ICD] get android identifier rsp_len :(%d)\n", rsp_len);  //yckim.kim@lge.com 2012.05.03: garbage charactor remove

	pRsp = (DIAG_ICD_F_rsp_type *) diagpkt_alloc(DIAG_ICD_F, rsp_len);
	if (pRsp == NULL) {
			 printk("[ICD] diagpkt_alloc failed\n");
			 return pRsp;
	}

	pRsp->hdr.cmd_code = DIAG_ICD_F;
	pRsp->hdr.sub_cmd = ICD_GETANDROIDIDENTIFIER_REQ_CMD;

	//char * returnStr = pRsp->icd_rsp.get_android_identifier_rsp_info.android_id_string;

	//yckim.kim@lge.com 2012.04.05 : GetAndroidIdentifier cmd add [START]
	id_fd = sys_open("/sys/class/android_usb/android0/iSerial", 0, 0);

	if(id_fd < 0) {
			 printk("%s, STATUS File Open Fail\n",__func__);
			 return pRsp;
	}

	memset(id_buf,0,sizeof(char) * 34);  //yckim.kim@lge.com 2012.05.03: garbage charactor remove
	while(sys_read(id_fd, &id_buf[read_size++], 1) == 1){}
	sys_close(id_fd);

	memset(pRsp->icd_rsp.get_android_identifier_rsp_info.android_id_string, 0, VARIABLE);
	sprintf(pRsp->icd_rsp.get_android_identifier_rsp_info.android_id_string,"%s", id_buf);
	printk( "[ICD] get android identifier android_id_string = %s   id_buf = %s \n",pRsp->icd_rsp.get_android_identifier_rsp_info.android_id_string, id_buf);
	//yckim.kim@lge.com 2012.04.05 : GetAndroidIdentifier cmd add [END]

	return pRsp;
}
//LGE_CHANGE jinhwan.do 20120430 Slate command's concept is cahnged, USB Serial number is MEID's decimal value [End]

//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]

//  20111125 [end] suhyun.lee@lge.com SLATE ICD 231,232,238 Merge From LS840 
/** ICDR : ICD Implementation Recommendation  - END **/

/*  USAGE (same as testmode
 *    1. If you want to handle at ARM9 side, 
 *       you have to insert fun_ptr as NULL and mark ARM9_PROCESSOR
 *    2. If you want to handle at ARM11 side , 
 *       you have to insert fun_ptr as you want and mark AMR11_PROCESSOR.
 */
icd_user_table_entry_type icd_mstr_tbl[ICD_MSTR_TBL_SIZE] = {
	/*sub_command								fun_ptr									which procesor*/
	/** SAR : Sprint Automation Requirement - START **/
	{ICD_GETDEVICEINFO_REQ_CMD, 				NULL, 									ICD_ARM9_PROCESSOR},
	{ICD_EXTENDEDVERSIONINFO_REQ_CMD,			icd_extended_info_req_proc, 			ICD_ARM11_PROCESSOR},
	{ICD_HANDSETDISPLAYTEXT_REQ_CMD,			icd_handset_disp_text_req_proc,		 	ICD_ARM11_PROCESSOR},
	{ICD_CAPTUREIMAGE_REQ_CMD,					icd_capture_img_req_proc,				ICD_ARM11_PROCESSOR},
	/** SAR : Sprint Automation Requirement - END **/

	/** ICDR : ICD Implementation Recommendation  - START **/
	{ICD_GETAIRPLANEMODE_REQ_CMD,				icd_get_airplanemode_req_proc,			ICD_ARM11_PROCESSOR},
	{ICD_SETAIRPLANEMODE_REQ_CMD,				icd_set_airplanemode_req_proc,			ICD_ARM11_PROCESSOR},
	{ICD_GETBACKLIGHTSETTING_REQ_CMD,			icd_get_backlight_setting_req_proc,		ICD_ARM11_PROCESSOR},
	{ICD_SETBACKLIGHTSETTING_REQ_CMD,			icd_set_backlight_setting_req_proc,		ICD_ARM11_PROCESSOR},
	{ICD_GETBATTERYCHARGINGSTATE_REQ_CMD,			NULL,						ICD_ARM9_PROCESSOR},
	{ICD_SETBATTERYCHARGINGSTATE_REQ_CMD,			NULL,						ICD_ARM9_PROCESSOR},
//[dugyung.ahn@lge.com] 2012-05-01, [Slate] GETBATTERYLEVEL [s]
	{ICD_GETBATTERYLEVEL_REQ_CMD,				icd_get_battery_level_req_proc,			ICD_ARM11_PROCESSOR},
	{ICD_GETBLUETOOTHSTATUS_REQ_CMD,			icd_get_bluetooth_status_req_proc,		ICD_ARM11_PROCESSOR},
	{ICD_SETBLUETOOTHSTATUS_REQ_CMD,			icd_set_bluetooth_status_req_proc,		ICD_ARM11_PROCESSOR},
	{ICD_GETGPSSTATUS_REQ_CMD,					NULL,									ICD_ARM9_PROCESSOR},
	{ICD_SETGPSSTATUS_REQ_CMD,					NULL,									ICD_ARM9_PROCESSOR},
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
	{ICD_GETKEYPADBACKLIGHT_REQ_CMD,			icd_get_keypadbacklight_req_proc,		ICD_ARM11_PROCESSOR},
	{ICD_SETKEYPADBACKLIGHT_REQ_CMD,			icd_set_keypadbacklight_req_proc,		ICD_ARM11_PROCESSOR},
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]
	{ICD_GETROAMINGMODE_REQ_CMD,				NULL,									ICD_ARM9_PROCESSOR},
	{ICD_GETSTATEANDCONNECTIONATTEMPTS_REQ_CMD,	NULL,									ICD_ARM9_PROCESSOR},
	{ICD_GETUISCREENID_REQ_CMD,					icd_get_ui_screen_id_req_proc,			ICD_ARM11_PROCESSOR},
	{ICD_GETWIFISTATUS_REQ_CMD,					icd_get_wifi_status_req_proc,			ICD_ARM11_PROCESSOR},
	{ICD_SETWIFISTATUS_REQ_CMD,					icd_set_wifi_status_req_proc,			ICD_ARM11_PROCESSOR},
	{ICD_SETSCREENORIENTATIONLOCK_REQ_CMD,		icd_set_screenorientationlock_req_proc,	ICD_ARM11_PROCESSOR},
//[dugyung.ahn@lge.com] 2012-06-27, [Slate] GETRSSI [s]	
//	{ICD_GETRSSI_REQ_CMD,						NULL,					ICD_ARM9_PROCESSOR},
	{ICD_GETRSSI_REQ_CMD,						icd_get_rssi_req_proc,					ICD_ARM11_PROCESSOR},	
//[dugyung.ahn@lge.com] 2012-06-27, [Slate] GETRSSI [e]
	{ICD_GETUSBDEBUGSTATUSSTATUS_REQ_CMD,		icd_get_usbdebug_status_req_proc,		ICD_ARM11_PROCESSOR},
	{ICD_SETUSBDEBUGSTATUSSTATUS_REQ_CMD,		icd_set_usbdebug_status_req_proc,		ICD_ARM11_PROCESSOR},
	{ICD_GETLATITUDELONGITUDEVALUES_REQ_CMD,	NULL,					ICD_ARM9_PROCESSOR},
	{ICD_GETSCREENLOCKSTATUS_REQ_CMD,			icd_get_screenlock_status_req_proc,		ICD_ARM11_PROCESSOR},
	{ICD_SETSCREENLOCKSTATUS_REQ_CMD,			icd_set_screenlock_status_req_proc,		ICD_ARM11_PROCESSOR},
//LGE_CHANGE jinhwan.do 20120430 Slate command's concept is cahnged, USB Serial number is MEID's decimal value [Start]
	{ICD_GETANDROIDIDENTIFIER_REQ_CMD,			icd_get_android_identifier_req_proc,	ICD_ARM11_PROCESSOR},
//LGE_CHANGE jinhwan.do 20120430 Slate command's concept is cahnged, USB Serial number is MEID's decimal value [End]
	/** ICDR : ICD Implementation Recommendation  - END **/
};
