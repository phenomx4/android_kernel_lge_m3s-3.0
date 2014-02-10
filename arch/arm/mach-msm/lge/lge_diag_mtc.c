/*===========================================================================

                     INCLUDE FILES FOR MODULE

===========================================================================*/
#include <linux/module.h>
#include <mach/lge_diagcmd.h>
#include <mach/lge_diag_mtc.h>

#include <linux/unistd.h> /*for open/close*/
#include <linux/fcntl.h> /*for O_RDWR*/

#include <linux/fb.h> /* to handle framebuffer ioctls */
#include <linux/ioctl.h>
#include <linux/uaccess.h>

#include <linux/syscalls.h> //for sys operations

#include <linux/input.h> // for input_event
#include <linux/fs.h> // for file struct
#include <linux/types.h> // for ssize_t
#include <linux/input.h> // for event parameters
#include <linux/jiffies.h>
#include <linux/delay.h>

#if 1 //SPRINT_SLATE_KEYPRESS_TEST
#include <linux/crc-ccitt.h>
#include <linux/delay.h>

#define ESC_CHAR     0x7D
#define CONTROL_CHAR 0x7E
#define ESC_MASK     0x20

#define CRC_16_L_SEED           0xFFFF

#define CRC_16_L_STEP(xx_crc, xx_c) \
	crc_ccitt_byte(xx_crc, xx_c)

void *lg_diag_mtc_req_pkt_ptr;
unsigned short lg_diag_mtc_req_pkt_length;
#endif //SPRINT_SLATE_KEYPRESS_TEST

// matthew.choi@lge.com 111003 prevent reset when try to start recording key input
static bool mtc_key_log_started = FALSE;

#ifndef LG_FW_DUAL_TOUCH
#define LG_FW_DUAL_TOUCH
#endif
///////////
typedef struct {
  unsigned int handset_7k_key_code;
  unsigned int Android_key_code;
}keycode_trans_type;

#define HANDSET_7K_KEY_TRANS_MAP_SIZE 60
keycode_trans_type handset_7k_keytrans_table[HANDSET_7K_KEY_TRANS_MAP_SIZE]=
{
   {KEY_BACK, '^'}, {KEY_1,'1'}, {KEY_2,'2'}, {KEY_3,'3'}, {KEY_4,'4'}, {KEY_5,'5'}, {KEY_6,'6'}, 
	{KEY_7,'7'}, {KEY_8,'8'}, {KEY_9,'9'}, {KEY_0,'0'}, {KEY_BACKSPACE,'Y'}, {KEY_HOME,'!'}, {KEY_MENU,'O'},
   {KEY_SEARCH, '+'}, {KEY_Q,'q'}, {KEY_W,'w'}, {KEY_E,'e'}, {KEY_R,'r'}, {KEY_T,'t'}, {KEY_Y,'y'}, 
	{KEY_U,'u'}, {KEY_I,'i'}, {KEY_O,'o'}, {KEY_P,'p'}, {KEY_LEFT,'/'},
	{KEY_LEFTALT,'$' }, {KEY_A,'a'}, {KEY_S,'s'}, {KEY_D,'d'}, {KEY_F,'f'}, {KEY_G,'g'}, {KEY_H,'h'}, 
	{KEY_J,'j'}, {KEY_K,'k'}, {KEY_L,'l'}, {KEY_ENTER,'='}, {KEY_UP,'L'}, {KEY_REPLY,32}, {KEY_DOWN,'R'},
	{KEY_LEFTSHIFT, '~'}, {KEY_Z,'z'}, {KEY_X,'x'}, {KEY_C,'c'}, {KEY_V,'v'}, {KEY_B,'b'}, {KEY_N,'n'}, 
	{KEY_M,'m'}, {KEY_DOT, 32}, {KEY_RIGHT,'V'},
	{KEY_MENU,'O'}, {KEY_HOME, '+'}, {KEY_BACK, '^'}, {KEY_SEARCH, '+'}, 
	{KEY_SEND,'S'}, {KEY_END,'E'},
	{KEY_VOLUMEUP,'U'}, {KEY_VOLUMEDOWN,'D'}, {KEY_VIDEO_PREV,'Z'}, {KEY_CAMERA,'A'}
};
///////////
/*===========================================================================

                      EXTERNAL FUNCTION AND VARIABLE DEFINITIONS

===========================================================================*/
extern PACK(void *) diagpkt_alloc (diagpkt_cmd_code_type code, unsigned int length);
extern PACK(void *) diagpkt_alloc2 (diagpkt_cmd_code_type code, unsigned int length, unsigned int packet_length);
extern PACK(void *) diagpkt_free (PACK(void *)pkt);
extern void send_to_arm9( void*	pReq, void	*pRsp);

extern void* lg_diag_req_pkt_ptr;
extern uint16 lg_diag_req_pkt_length;

#ifdef CONFIG_LGE_DIAG_ATS_ETA_MTC_KEY_LOGGING
extern unsigned long int ats_mtc_log_mask;
extern void diagpkt_commit (PACK(void *)pkt);
extern int event_log_start(void);
extern int event_log_end(void);
#endif

extern int base64_encode(char *text, int numBytes, char *encodedText);
/*===========================================================================

            LOCAL DEFINITIONS AND DECLARATIONS FOR MODULE

  This section contains local definitions for constants, macros, types,
  variables and other items needed by this module.

===========================================================================*/
#ifdef CONFIG_LGE_DIAG_ATS_ETA_MTC_KEY_LOGGING
#define JIFFIES_TO_MS(t) ((t) * 1000 / HZ)
#endif 

extern mtc_user_table_entry_type mtc_mstr_tbl[MTC_MSTR_TBL_SIZE];

unsigned char g_diag_mtc_check = 0;
unsigned char g_diag_mtc_capture_rsp_num = 0;

#if 1 //SPRINT_SLATE_KEYPRESS_TEST
static char mtc_running = 0;
extern int diagchar_ioctl(unsigned int iocmd, unsigned long ioarg);
#endif //SPRINT_SLATE_KEYPRESS_TEST

static mtc_lcd_info_type lcd_info;

static int ats_mtc_set_lcd_info (mtc_scrn_id_type ScreenType);
static ssize_t read_framebuffer(byte* pBuf);

//static char bmp_data_array[MTC_SCRN_BUF_SIZE_MAX];

unsigned char lge_img_capture_32bpp[MTC_SCRN_BUF_32BPP_SIZE];
unsigned char lge_img_capture_16bpp[MTC_SCRN_BUF_SIZE_MAX];

/*===========================================================================

                      INTERNAL FUNCTION DEFINITIONS

===========================================================================*/
PACK (void *)LGF_MTCProcess (
        PACK (void	*)req_pkt_ptr,	/* pointer to request packet  */
        unsigned short		pkt_len )		      /* length of request packet   */
{
	DIAG_MTC_F_req_type *req_ptr = (DIAG_MTC_F_req_type *) req_pkt_ptr;
  	DIAG_MTC_F_rsp_type *rsp_ptr = NULL;
  	mtc_func_type func_ptr= NULL;
  	int nIndex = 0;
  
  	g_diag_mtc_check = 1;
#if 1 //SPRINT_SLATE_KEYPRESS_TEST
  	mtc_running = 1;
#endif //SPRINT_SLATE_KEYPRESS_TEST

	lg_diag_req_pkt_ptr = req_pkt_ptr;
	lg_diag_req_pkt_length = pkt_len;

  	for( nIndex = 0 ; nIndex < MTC_MSTR_TBL_SIZE  ; nIndex++)
  	{
    	if( mtc_mstr_tbl[nIndex].cmd_code == req_ptr->hdr.sub_cmd)
    	{
      		if( mtc_mstr_tbl[nIndex].which_procesor == MTC_ARM11_PROCESSOR)
        		func_ptr = mtc_mstr_tbl[nIndex].func_ptr;
      		break;
    	}
    	else if (mtc_mstr_tbl[nIndex].cmd_code == MTC_MAX_CMD)
      		break;
    	else
      		continue;
  	}
#ifdef MTC_DBG
  	printk(KERN_INFO "[MTC]cmd_code : [0x%X], sub_cmd : [0x%X]\n",req_ptr->hdr.cmd_code, req_ptr->hdr.sub_cmd);
#endif

  	if( func_ptr != NULL)
  	{
#ifdef MTC_DBG  
    	printk(KERN_INFO "[MTC]cmd_code : [0x%X], sub_cmd : [0x%X]\n",req_ptr->hdr.cmd_code, req_ptr->hdr.sub_cmd);
#endif
    	rsp_ptr = func_ptr((DIAG_MTC_F_req_type*)req_ptr);
  	}
//	else
//		send_to_arm9((void*)req_ptr, (void*)rsp_ptr);
//	diagpkt_free(rsp_ptr);

#if 1 //SPRINT_SLATE_KEYPRESS_TEST
	mtc_running = 0;
#endif //SPRINT_SLATE_KEYPRESS_TEST

	return (rsp_ptr);
}

EXPORT_SYMBOL(LGF_MTCProcess);

DIAG_MTC_F_rsp_type* mtc_info_req_proc(DIAG_MTC_F_req_type *pReq)
{
	unsigned int rsp_len;
	DIAG_MTC_F_rsp_type *pRsp;

	printk(KERN_INFO "[MTC]mtc_info_req_proc\n");

	rsp_len = sizeof(mtc_info_rsp_type);
  	printk(KERN_INFO "[MTC] mtc_info_req_proc rsp_len :(%d)\n", rsp_len);
   
  	pRsp = (DIAG_MTC_F_rsp_type *)diagpkt_alloc(DIAG_MTC_F, rsp_len);
  	if (pRsp == NULL) {
     	printk(KERN_ERR "[MTC] diagpkt_alloc failed\n");
  	}

  	pRsp->hdr.cmd_code = DIAG_MTC_F;
  	pRsp->hdr.sub_cmd = MTC_INFO_REQ_CMD;
  
  	if(pReq->mtc_req.info.screen_id == MTC_SUB_LCD) // N/A
  	{
    	printk(KERN_ERR "[MTC]mtc_info_req_proc, sub lcd is not supported\n");
    	return pRsp;
  	}

  	if(pReq->mtc_req.info.screen_id == MTC_MAIN_LCD)
  	{
    	ats_mtc_set_lcd_info(MTC_MAIN_LCD);
    	pRsp->mtc_rsp.info.scrn_id = MTC_MAIN_LCD;
  	}
#ifdef LGE_USES_SUBLCD
  	else if(pReq->mtc_req.info.screen_id == MTC_SUB_LCD)
  	{
    	ats_mtc_set_lcd_info(MTC_SUB_LCD);
    	pRsp->mtc_rsp.info.scrn_id = MTC_SUB_LCD;
  	}
#endif
  	else
  	{
    	printk(KERN_ERR "[MTC]mtc_info_req_proc, unknown screen_id type : %d\n", pRsp->mtc_rsp.info.scrn_id);
    	return pRsp;
  	}

  	pRsp->mtc_rsp.info.scrn_width = lcd_info.width_max;
  	pRsp->mtc_rsp.info.scrn_height = lcd_info.height_max;
  	pRsp->mtc_rsp.info.bits_pixel = 16;//lcd_info.bits_pixel;

  	return pRsp;
}

static int ats_mtc_set_lcd_info (mtc_scrn_id_type ScreenType)
{
  	struct fb_var_screeninfo fb_varinfo;
  	int fbfd;
  
  	if ((fbfd = sys_open ("/dev/graphics/fb0", O_RDWR, 0)) == -1)
  	{
    	printk(KERN_ERR "[MTC]ats_mtc_set_lcd_info, Can't open %s\n", "/dev/graphics/fb0");
    	return 0;
  	}

  	memset((void *)&fb_varinfo, 0, sizeof(struct fb_var_screeninfo));

  	if (sys_ioctl (fbfd, FBIOGET_VSCREENINFO, (long unsigned int)&fb_varinfo) < 0)
  	{
    	printk(KERN_ERR "[MTC]ats_mtc_set_lcd_info, ioctl failed\n");
    	return 0;
  	}

  	printk(KERN_INFO "[MTC]ats_mtc_set_lcd_info, fbvar.xres= %d, fbvar.yres= %d, fbvar.pixel= %d\n", fb_varinfo.xres, fb_varinfo.yres, fb_varinfo.bits_per_pixel);
  
  	sys_close(fbfd);
  
  	if (ScreenType == MTC_MAIN_LCD){
  		lcd_info.id = MTC_MAIN_LCD;
  		lcd_info.width_max = fb_varinfo.xres;
  		lcd_info.height_max =fb_varinfo.yres;
  	}
#if defined (LG_SUBLCD_INCLUDE)  
  	else if (ScreenType == MTC_SUB_LCD){
  		lcd_info.id = MTC_SUB_LCD;
  		lcd_info.width_max = fb_varinfo.xres;
  		lcd_info.height_max = fb_varinfo.yres;
  	}
#endif

  	//To Get the Bits Depth
  	lcd_info.bits_pixel = fb_varinfo.bits_per_pixel;

#if 0  
  	if( lcd_info.bits_pixel == MTC_BIT_65K )
  	{
  		lcd_info.mask.blue = MTC_65K_CMASK_BLUE;
  		lcd_info.mask.green = MTC_65K_CMASK_GREEN;
  		lcd_info.mask.red = MTC_65K_CMASK_RED;
  	}
  	else if( lcd_info.bits_pixel == MTC_BIT_262K )
  	{
  		lcd_info.mask.blue = MTC_262K_CMASK_BLUE;
  		lcd_info.mask.green = MTC_262K_CMASK_GREEN;
  		lcd_info.mask.red = MTC_262K_CMASK_RED;
  	}
  	else // default 16 bit
  	{
  		lcd_info.bits_pixel = MTC_BIT_65K;
  		lcd_info.mask.blue = MTC_65K_CMASK_BLUE;
  		lcd_info.mask.green = MTC_65K_CMASK_GREEN;
  		lcd_info.mask.red = MTC_65K_CMASK_RED;
  	}
#else
   lcd_info.bits_pixel = 16;
   lcd_info.mask.blue = MTC_65K_CMASK_BLUE;
   lcd_info.mask.green = MTC_65K_CMASK_GREEN;
   lcd_info.mask.red = MTC_65K_CMASK_RED;
#endif
  	lcd_info.degrees = 0; //No rotation .. manual Data	

  	return 1;
}

#if 1 //SLATE_CROPPED_CAPTURE
unsigned char tmp_img_block[MTC_SCRN_BUF_SIZE_MAX+16];
EXPORT_SYMBOL(tmp_img_block);
#endif //SLATE_CROPPED_CAPTURE

#define	FB_DEV	"/dev/graphics/fb0"

unsigned long convert_32_to_16_bpp(byte* target_buf, byte* src_buf)
{
  int x, y, start_pos;
  byte* src_buf_ptr;
  unsigned long dst_cnt = 0;
  uint8 red, green, blue;

  printk(KERN_INFO "%s", __func__);

  //memset((void *)target_buf, 0, LCD_BUFFER_SIZE);

  for (y = 0; y < 480; y++) 
  {
    start_pos = y*320*4;
    src_buf_ptr = (byte *)src_buf + start_pos;

    for (x = 0; x < (320*4); x += 4) 
    {
      red = ((src_buf_ptr[x]>>3)&0x1F);
      green = ((src_buf_ptr[x+1]>>2)&0x3F);
      blue = ((src_buf_ptr[x+2]>>3)&0x1F);
      // ignore alpa (every 4th bit)
      target_buf[dst_cnt++] = ((green&0x07)<<5)|blue;
      target_buf[dst_cnt++] = ((red<<3)&0xF8)|((green>>3)&0x07);					
    }
  }
  printk(KERN_INFO "%s, translated count : %ld", __func__, dst_cnt);
  
  return dst_cnt;
}

DEFINE_SPINLOCK( mtc_spinlock );

static ssize_t read_framebuffer(byte* pBuf)
{
  	struct file *phMscd_Filp = NULL;
  	ssize_t read_size = 0;

	unsigned long flags;

  	mm_segment_t old_fs=get_fs();

  	set_fs(get_ds());

  	phMscd_Filp = filp_open("/dev/graphics/fb0", O_RDONLY |O_LARGEFILE, 0);

  	if( !phMscd_Filp)
    	printk("open fail screen capture \n" );

   spin_lock_irqsave( &mtc_spinlock, flags );
  	read_size = phMscd_Filp->f_op->read(phMscd_Filp, pBuf, MTC_SCRN_BUF_SIZE_MAX, &phMscd_Filp->f_pos);
   spin_unlock_irqrestore( &mtc_spinlock, flags );
   
  	filp_close(phMscd_Filp,NULL);

  	set_fs(old_fs);

  	return read_size;
}

#ifdef CONFIG_LGE_DIAG_ATS_ETA_MTC_KEY_LOGGING
DIAG_MTC_F_rsp_type* mtc_logging_mask_req_proc(DIAG_MTC_F_req_type *pReq)
{
  	unsigned int rsp_len;
  	DIAG_MTC_F_rsp_type *pRsp;

  	rsp_len = sizeof(mtc_log_req_type);
  	printk(KERN_INFO "[MTC] mtc_logging_mask_req_proc rsp_len :(%d)\n", rsp_len);

	pRsp = (DIAG_MTC_F_rsp_type *)diagpkt_alloc(DIAG_MTC_F, rsp_len);
  	if (pRsp == NULL) {
  		printk(KERN_ERR "[MTC] diagpkt_alloc failed\n");
    	return pRsp;
  	}

  	switch(pReq->mtc_req.log.mask)
  	{
    	case 0x00000000://ETA_LOGMASK_DISABLE_ALL:
			// matthew.choi@lge.com 111003 prevent reset when try to start recording key input
    		if (mtc_key_log_started == FALSE)
      		{
				pRsp->mtc_rsp.log.mask = ats_mtc_log_mask = 0xFFFFFFFF;
				mtc_key_log_started = TRUE;
    		}
			else
			{
	      		ats_mtc_log_mask = pReq->mtc_req.log.mask;
	      		pRsp->mtc_rsp.log.mask = ats_mtc_log_mask;
			}
			break;
    	case 0xFFFFFFFF://ETA_LOGMASK_ENABLE_ALL:
    	case 0x00000001://ETA_LOGITEM_KEY:
    	case 0x00000002://ETA_LOGITEM_TOUCHPAD:
    	case 0x00000003://ETA_LOGITME_KEYTOUCH:
      		ats_mtc_log_mask = pReq->mtc_req.log.mask;
      		pRsp->mtc_rsp.log.mask = ats_mtc_log_mask;
			// matthew.choi@lge.com 111003 prevent reset when try to start recording key input
			mtc_key_log_started = TRUE;
      		break;
    	default:
      		ats_mtc_log_mask = 0x00000000; // //ETA_LOGMASK_DISABLE_ALL
      		pRsp->mtc_rsp.log.mask = ats_mtc_log_mask;
			// matthew.choi@lge.com 111003 prevent reset when try to start recording key input
			mtc_key_log_started = FALSE;
      		break;
  	}

  	/* LGE_CHANGE
   	* support mtc using diag port
   	* 2010 07-11 taehung.kim@lge.com
   	*/
   	if(ats_mtc_log_mask & 0xFFFFFFFF)
   		event_log_start();
   	else
   	{
		// matthew.choi@lge.com 111003 prevent reset when try to start recording key input
		if (mtc_key_log_started == TRUE)
	   	{
			event_log_end();
			mtc_key_log_started = FALSE;
		}
   	}

  	return pRsp;
}

/////////////////////
unsigned int Handset_7k_KeycodeTrans(word input)
{
  	int index = 0;
  	unsigned int ret = (unsigned int)input;  // if we can not find, return the org value. 
 
  	for( index = 0; index < HANDSET_7K_KEY_TRANS_MAP_SIZE  ; index++)
  	{
	 	if( handset_7k_keytrans_table[index].handset_7k_key_code == input)
	 	{
			ret = handset_7k_keytrans_table[index].Android_key_code;
			printk(KERN_ERR "[MTC] input keycode %d, mapped keycode %d\n", input, ret);
			break;
	 	}
  	}  

  	return ret;
}
///////////////////

void mtc_send_key_log_data(struct ats_mtc_key_log_type* p_ats_mtc_key_log)
{
  	unsigned int rsp_len;
  	DIAG_MTC_F_rsp_type *pRsp;
  
  	rsp_len = sizeof(mtc_log_data_rsp_type);
  	printk(KERN_INFO "[MTC] mtc_send_key_log_data rsp_len :(%d)\n", rsp_len);

	pRsp = (DIAG_MTC_F_rsp_type *)diagpkt_alloc(DIAG_MTC_F, rsp_len);
  	if (pRsp == NULL) {
  		printk(KERN_ERR "[MTC] diagpkt_alloc failed\n");
  		diagpkt_commit (pRsp);
  	}

  	pRsp->hdr.cmd_code = DIAG_MTC_F;
  	pRsp->hdr.sub_cmd = MTC_LOG_REQ_CMD;

  	pRsp->mtc_rsp.log_data.log_id = p_ats_mtc_key_log->log_id; //LOG_ID, 1 key, 2 touch
  	pRsp->mtc_rsp.log_data.log_len = p_ats_mtc_key_log->log_len; //LOG_LEN
  
  	if(p_ats_mtc_key_log->log_id == 1) // key
  	{
    	pRsp->mtc_rsp.log_data.log_type.log_data_key.time = (unsigned long long)JIFFIES_TO_MS(jiffies);
    	pRsp->mtc_rsp.log_data.log_type.log_data_key.hold = (unsigned char)((p_ats_mtc_key_log->x_hold)&0xFF);// hold
		//pRsp->mtc_rsp.log_data.log_type.log_data_key.keycode = ((p_ats_mtc_key_log->y_code)&0xFF);//key code
    	pRsp->mtc_rsp.log_data.log_type.log_data_key.keycode = Handset_7k_KeycodeTrans((p_ats_mtc_key_log->y_code)&0xFF);//key code
    	pRsp->mtc_rsp.log_data.log_type.log_data_key.active_uiid = 0;
  	}
  	else // touch
  	{
    	pRsp->mtc_rsp.log_data.log_type.log_data_touch.time = (unsigned long long)JIFFIES_TO_MS(jiffies);
    	pRsp->mtc_rsp.log_data.log_type.log_data_touch.screen_id = (unsigned char)1; // MAIN LCD
    	pRsp->mtc_rsp.log_data.log_type.log_data_touch.action = (unsigned char)p_ats_mtc_key_log->action;
		pRsp->mtc_rsp.log_data.log_type.log_data_touch.x = (unsigned short)p_ats_mtc_key_log->x_hold;
    	pRsp->mtc_rsp.log_data.log_type.log_data_touch.y = (unsigned short)p_ats_mtc_key_log->y_code;
    	pRsp->mtc_rsp.log_data.log_type.log_data_touch.active_uiid = 0;
  	}

  	diagpkt_commit (pRsp);
}
EXPORT_SYMBOL(mtc_send_key_log_data);
#endif 

DIAG_MTC_F_rsp_type* mtc_serialized_data_req_proc(DIAG_MTC_F_req_type *pReq)
{
  unsigned int rsp_len;
  DIAG_MTC_F_rsp_type *pRsp;
  static unsigned long bmp_sent_cnt = 0; // save PMP data sent count
// BEGIN: 0010599 alan.park@lge.com 2010.11.07 
// ADD	0010599: [ETA/MTC] MTC capture ¹× touch ±â´É
  static ssize_t read_size = 0; // save BMP data read count
  unsigned int left_size = 0;
  
  struct file *phMscd_Filp = NULL;
  
  mm_segment_t old_fs;

  if(bmp_sent_cnt == 0) // read only once
  {
	old_fs=get_fs();
	set_fs(get_ds());

	phMscd_Filp = filp_open(ETA_MTC_BMP_DATA, O_RDONLY |O_LARGEFILE, 0);
	if( !phMscd_Filp)
	  printk(KERN_ERR "%s, open fail screen capture \n", __func__);

	printk(KERN_INFO "%s, %s offset : %d\n", __func__, ETA_MTC_BMP_DATA, (int)phMscd_Filp->f_pos);
	phMscd_Filp->f_pos = ETA_MTC_BMP_HEADER_CNT;
	printk(KERN_INFO "%s, %s changed offset : %d for bmp header\n", __func__, ETA_MTC_BMP_DATA, (int)phMscd_Filp->f_pos);

	read_size = phMscd_Filp->f_op->read(phMscd_Filp, lge_img_capture_16bpp, MTC_SCRN_BUF_SIZE_MAX, &phMscd_Filp->f_pos);
	printk(KERN_INFO "%s, read size : %d(0x%X) \n", __func__, read_size, read_size);

	filp_close(phMscd_Filp,NULL);
	set_fs(old_fs);
  }

  rsp_len = sizeof(mtc_serialized_data_rsp_type);
#ifdef MTC_DBG
  printk(KERN_INFO "%s, rsp_len :(%d)\n", __func__, rsp_len);
#endif

  pRsp = (DIAG_MTC_F_rsp_type *)diagpkt_alloc(DIAG_MTC_F, rsp_len);
  if (pRsp == NULL) {
	printk(KERN_ERR "%s, diagpkt_alloc failed\n", __func__);
	return pRsp;
  }

  pRsp->hdr.cmd_code = DIAG_MTC_F;
  pRsp->hdr.sub_cmd = MTC_SERIALIZED_DATA_REQ_CMD;
  
  if((bmp_sent_cnt+1)*MTC_SCRN_BUF_SIZE < read_size) // more than data pkt size
  {
	pRsp->mtc_rsp.serial_data.seqeunce = bmp_sent_cnt;
	pRsp->mtc_rsp.serial_data.length = MTC_SCRN_BUF_SIZE;

#ifdef MTC_DBG
	memset((void*)pRsp->mtc_rsp.serial_data.bmp_data, 0, MTC_SCRN_BUF_SIZE);  
#endif
	memcpy((void*)pRsp->mtc_rsp.serial_data.bmp_data, (void*)(lge_img_capture_16bpp+bmp_sent_cnt*MTC_SCRN_BUF_SIZE), MTC_SCRN_BUF_SIZE);

	bmp_sent_cnt++;

	left_size = read_size - bmp_sent_cnt*MTC_SCRN_BUF_SIZE;
#ifdef MTC_DBG
	printk(KERN_INFO "%s, sent_cnt : %d(0x%X), left : %d(0x%X)\n", __func__, bmp_sent_cnt, bmp_sent_cnt, left_size, left_size);
#endif
  }
  else // less than data pkt size
  {
	pRsp->mtc_rsp.serial_data.seqeunce = 0xFFFF;

	pRsp->mtc_rsp.serial_data.length = read_size - bmp_sent_cnt*MTC_SCRN_BUF_SIZE;
	memset((void*)pRsp->mtc_rsp.serial_data.bmp_data, 0, MTC_SCRN_BUF_SIZE);
	memcpy((void*)pRsp->mtc_rsp.serial_data.bmp_data, (void*)(lge_img_capture_16bpp+bmp_sent_cnt*MTC_SCRN_BUF_SIZE), pRsp->mtc_rsp.serial_data.length);
  
	bmp_sent_cnt = 0;

	printk(KERN_INFO "%s, last_pkt len : %d(0x%X)\n", __func__, (int)pRsp->mtc_rsp.serial_data.length, (int)pRsp->mtc_rsp.serial_data.length);
  }
  
  return pRsp;
}

DIAG_MTC_F_rsp_type* mtc_serialized_capture_req_proc(DIAG_MTC_F_req_type *pReq)
{
  	unsigned int rsp_len;
  	DIAG_MTC_F_rsp_type *pRsp;
  	ssize_t bmp_size;
  
  	rsp_len = sizeof(mtc_serialized_capture_rsp_type);
  	printk(KERN_INFO "[MTC] mtc_serialized_data_req_proc rsp_len :(%d)\n", rsp_len);

  	pRsp = (DIAG_MTC_F_rsp_type *)diagpkt_alloc(DIAG_MTC_F, rsp_len);
  	if (pRsp == NULL) {
  		printk(KERN_ERR "[MTC] diagpkt_alloc failed\n");
    	return pRsp;
  	}

  	pRsp->hdr.cmd_code = DIAG_MTC_F;
  	pRsp->hdr.sub_cmd = MTC_SERIALIZED_CAPTURE_REQ_CMD;

  	pRsp->mtc_rsp.serial_capture.screen_id = lcd_info.id;
  	pRsp->mtc_rsp.serial_capture.bmp_size = lcd_info.width_max*lcd_info.height_max*2;
  	pRsp->mtc_rsp.serial_capture.bmp_width = lcd_info.width_max;
  	pRsp->mtc_rsp.serial_capture.bmp_height = lcd_info.height_max;
  	pRsp->mtc_rsp.serial_capture.bits_pixel = lcd_info.bits_pixel;
  	pRsp->mtc_rsp.serial_capture.mask.blue = lcd_info.mask.blue;
  	pRsp->mtc_rsp.serial_capture.mask.green = lcd_info.mask.green;
  	pRsp->mtc_rsp.serial_capture.mask.red = lcd_info.mask.red;

  	memset(lge_img_capture_16bpp, 0, MTC_SCRN_BUF_SIZE_MAX);

  	bmp_size = read_framebuffer((byte*)lge_img_capture_16bpp);
  	printk(KERN_INFO "[MTC]mtc_serialized_capture_req_proc, Read framebuffer & Bmp convert complete.. %d\n", (int)bmp_size);

  	return pRsp;
}

DIAG_MTC_F_rsp_type* mtc_null_rsp(DIAG_MTC_F_req_type *pReq)
{
  	unsigned int rsp_len;
  	DIAG_MTC_F_rsp_type *pRsp;

  	rsp_len = sizeof(mtc_req_hdr_type);
  	printk(KERN_INFO "[MTC] mtc_null_rsp rsp_len :(%d)\n", rsp_len);
   
  	pRsp = (DIAG_MTC_F_rsp_type *)diagpkt_alloc(DIAG_MTC_F, rsp_len);
  	if (pRsp == NULL) {
     	printk(KERN_ERR "[MTC] diagpkt_alloc failed\n");
  	}

  	pRsp->hdr.cmd_code = pReq->hdr.cmd_code;
  	pRsp->hdr.sub_cmd = pReq->hdr.sub_cmd;

  	return pRsp;
}

DIAG_MTC_F_rsp_type* mtc_execute(DIAG_MTC_F_req_type *pReq)
{
  	int ret;
  	char cmdstr[100];
	int fd;

  
  	char *envp[] = {
  		"HOME=/",
  		"TERM=linux",
  		NULL,
  	};
  
  	char *argv[] = {
  		"sh",
  		"-c",
  		cmdstr,
  		NULL,
  	};

	printk(KERN_INFO "%s, %d, %d\n", __func__, (int)pReq->hdr.cmd_code, (int)pReq->hdr.sub_cmd);

	if ( (fd = sys_open((const char __user *)MTC_MODULE, O_RDONLY ,0) ) < 0 )
  	{
		printk(KERN_ERR "\n%s, can not open %s\n", __func__, MTC_MODULE);
		sprintf(cmdstr, "%s", MTC_MODULE);
  	}
  	else
  	{
		sprintf(cmdstr, "%s", MTC_MODULE);
  		sys_close(fd);
  	}
  
	printk(KERN_INFO "%s, execute mtc : data - %s\n\n", __func__, cmdstr);
	if ((ret =
		call_usermodehelper("/system/bin/sh", argv, envp, UMH_WAIT_PROC)) != 0) {
		printk(KERN_ERR "%s, MTC failed to run : %i\n", __func__, ret);
	}
	else
		printk(KERN_INFO "%s, execute ok, ret = %d\n", __func__, ret);
  
  return NULL;
}
EXPORT_SYMBOL(mtc_execute);

#if 1 //ifdef SPRINT_SLATE_KEYPRESS_TEST
static void add_hdlc_packet(struct mtc_data_buffer *mb, char data)
{
	mb->data[mb->data_length++] = data;

	//if (mb->data_length == BUFFER_MAX_SIZE) {
	if (mb->data_length >= BUFFER_MAX_SIZE) {
		mb->data_length = BUFFER_MAX_SIZE;

		msleep(10);

		if (diagchar_ioctl (DIAG_IOCTL_BULK_DATA, (unsigned long)mb)) {
			printk(KERN_ERR "[MTC] %s: diagchar_ioctl error\n", __func__);
		} 

		mb->data_length = 0;
	}

	return;
}

/*
 * FUNCTION	add_hdlc_esc_packet.
 */
static void add_hdlc_esc_packet(struct mtc_data_buffer *mb, char data)
{
	if (data == ESC_CHAR || data == CONTROL_CHAR) {
		add_hdlc_packet(mb, ESC_CHAR);
		add_hdlc_packet(mb, (data ^ ESC_MASK));
	} 
	else {
		add_hdlc_packet(mb, data);
	}

	return;
}

/*
 * FUNCTION	mtc_send_hdlc_packet.
 */
static void mtc_send_hdlc_packet(byte * pBuf, int len)
{
	int i;
	struct mtc_data_buffer *mb;
	word crc = CRC_16_L_SEED;

	mb = kzalloc(sizeof(struct mtc_data_buffer), GFP_ATOMIC);
	if (mb == NULL) {
		printk(KERN_ERR "[MTC] %s: failed to alloc memory\n", __func__);
		return;
	}

	//Generate crc data.
	for (i = 0; i < len; i++) {
		add_hdlc_esc_packet(mb, pBuf[i]);
		crc = CRC_16_L_STEP(crc, (word) pBuf[i]);
	}

	crc ^= CRC_16_L_SEED;
	add_hdlc_esc_packet(mb, ((unsigned char)crc));
	add_hdlc_esc_packet(mb, ((unsigned char)((crc >> 8) & 0xFF)));
	add_hdlc_packet(mb, CONTROL_CHAR);

	if (diagchar_ioctl(DIAG_IOCTL_BULK_DATA, (unsigned long)mb)) {
		printk(KERN_ERR "[MTC] %s: ioctl ignored\n", __func__);
	}

	kfree(mb);
	mb = NULL;

	return;
}

/*
 * FUNCTION	translate_key_code.
 */
dword translate_key_code(dword keycode)
{
	if (keycode == KERNELFOCUSKEY)
		return KERNELCAMERAKEY;
	else
		return keycode;
}

/*
 * FUNCTION	mtc_send_key_log_packet.
 */
void mtc_send_key_log_packet(unsigned long keycode, unsigned long state)
{
	key_msg_type msg;
	dword sendKeyValue = 0;

	/* LGE_CHANGE [dojip.kim@lge.com] 2010-06-04 [LS670]
	 * don't send a raw diag packet in running MTC
	 */
	if (mtc_running)
		return;

	memset(&msg, 0, sizeof(key_msg_type));

	sendKeyValue = translate_key_code(keycode);

	msg.cmd_code = 121;
	msg.ts_type = 0;	//2;
	msg.num_args = 2;
	msg.drop_cnt = 0;
	//ts_get(&msg.time[0]); 
	msg.time[0] = 0;
	msg.time[1] = 0;
	msg.line_number = 261;
	msg.ss_id = LGE_DIAG_ICD_LOGGING_SSID;
	msg.ss_mask = LGE_DIAG_ICD_LOGGING_SSID_MASK;
	msg.args[0] = sendKeyValue;
	msg.args[1] = state;

	memcpy(&msg.code[0], "Debounced %d", sizeof("Debounced %d"));
	//msg.code[12] = '\0';

	memcpy(&msg.file_name[0], "DiagDaemon.c", sizeof("DiagDaemon.c"));
	//msg.fle_name[13] = '\0';

	mtc_send_hdlc_packet((byte *) & msg, sizeof(key_msg_type));

	return;
}
EXPORT_SYMBOL(mtc_send_key_log_packet);

/*
 * FUNCTION	mtc_send_touch_log_packet.
 */
void mtc_send_touch_log_packet(unsigned long touch_x, unsigned long touch_y, unsigned long status)
{
	touch_msg_type msg;
	
	/* LGE_CHANGE [dojip.kim@lge.com] 2010-06-04 [LS670]
	 * don't send a raw diag packet in running MTC
	 */
	if (mtc_running)
		return;

	memset(&msg, 0, sizeof(touch_msg_type));

	msg.cmd_code = 121;
	msg.ts_type = 0;	//2;
	msg.num_args = 3;
	msg.drop_cnt = 0;
	//ts_get(&msg.time[0]); 
	msg.time[0] = 0;
	msg.time[1] = 0;
	msg.line_number = 261;
	msg.ss_id = LGE_DIAG_ICD_LOGGING_SSID;
	msg.ss_mask = LGE_DIAG_ICD_LOGGING_SSID_MASK;
	if(status == 1) // push - "DWN"
		msg.args[0] = 0x004E5744;
	else	// release - "UP"
		msg.args[0] = 0x00005055;
#if 0 // temp blocked - myeonggyu.son@lge.com
	msg.args[1] = touch_x*320/max_x;
	msg.args[2] = touch_y*480/max_y;
#else
	msg.args[1] = touch_x*320;
	msg.args[2] = touch_y*480;
#endif
	memcpy(&msg.code[0], "PenEvent %d,%d", sizeof("PenEvent %d,%d"));
	//msg.code[12] = '\0';

	memcpy(&msg.file_name[0], "DiagDaemon.c", sizeof("DiagDaemon.c"));
	//msg.fle_name[13] = '\0';

	mtc_send_hdlc_packet((byte *) & msg, sizeof(touch_msg_type));
	
	return;
}
EXPORT_SYMBOL(mtc_send_touch_log_packet);
#endif /*SPRINT_SLATE_KEYPRESS_TEST*/

/*  USAGE (same as testmode
  *    1. If you want to handle at ARM9 side, you have to insert fun_ptr as NULL and mark ARM9_PROCESSOR
  *    2. If you want to handle at ARM11 side , you have to insert fun_ptr as you want and mark AMR11_PROCESSOR.
  */
mtc_user_table_entry_type mtc_mstr_tbl[MTC_MSTR_TBL_SIZE] =
{ 
/*	sub_command							fun_ptr								which procesor              */
	{ MTC_INFO_REQ_CMD					,mtc_execute						, MTC_ARM11_PROCESSOR},
	{ MTC_CAPTURE_REQ_CMD				,mtc_null_rsp						, MTC_ARM11_PROCESSOR},
	{ MTC_KEY_EVENT_REQ_CMD				,mtc_execute						, MTC_ARM11_PROCESSOR},
	{ MTC_TOUCH_REQ_CMD					,mtc_execute						, MTC_ARM11_PROCESSOR},
#ifdef CONFIG_LGE_DIAG_ATS_ETA_MTC_KEY_LOGGING
	{ MTC_LOGGING_MASK_REQ_CMD			,mtc_logging_mask_req_proc			, MTC_ARM11_PROCESSOR},
	{ MTC_LOG_REQ_CMD					,mtc_null_rsp						, MTC_ARM11_PROCESSOR}, /*mtc_send_key_log_data*/
#endif 
	{ MTC_SERIALIZED_DATA_REQ_CMD		,mtc_serialized_data_req_proc		, MTC_ARM11_PROCESSOR},
	/*{ MTC_SERIALIZED_CAPTURE_REQ_CMD 	,mtc_serialized_capture_req_proc	, MTC_ARM11_PROCESSOR},*/
	{ MTC_SERIALIZED_CAPTURE_REQ_CMD	,mtc_execute						, MTC_ARM11_PROCESSOR},
	{ MTC_PHONE_RESTART_REQ_CMD			,mtc_null_rsp						, MTC_ARM9_PROCESSOR},
	{ MTC_FACTORY_RESET					,mtc_null_rsp						, MTC_ARM9_ARM11_BOTH},
	{ MTC_PHONE_REPORT					,mtc_null_rsp						, MTC_ARM9_PROCESSOR},
	{ MTC_PHONE_STATE					,mtc_null_rsp						, MTC_NOT_SUPPORTED},
	{ MTC_CAPTURE_PROP					,mtc_null_rsp						, MTC_NOT_SUPPORTED},
	{ MTC_NOTIFICATION_REQUEST			,mtc_null_rsp						, MTC_NOT_SUPPORTED},
	{ MTC_CUR_PROC_NAME_REQ_CMD			,mtc_null_rsp						, MTC_NOT_SUPPORTED},
	{ MTC_KEY_EVENT_UNIV_REQ_CMD		,mtc_null_rsp						, MTC_NOT_SUPPORTED}, /*ETA command*/
	{ MTC_MEMORY_DUMP					,mtc_null_rsp						, MTC_NOT_SUPPORTED},
	{ MTC_BATTERY_POWER					,mtc_null_rsp						, MTC_ARM9_PROCESSOR},
	{ MTC_BACKLIGHT_INFO				,mtc_null_rsp						, MTC_NOT_SUPPORTED},
	{ MTC_FLASH_MODE					,mtc_null_rsp						, MTC_NOT_SUPPORTED},
	{ MTC_MODEM_MODE					,mtc_null_rsp						, MTC_NOT_SUPPORTED},
	{ MTC_CELL_INFORMATION				,mtc_null_rsp						, MTC_NOT_SUPPORTED},
	{ MTC_HANDOVER						,mtc_null_rsp						, MTC_NOT_SUPPORTED},
	{ MTC_ERROR_CMD						,mtc_null_rsp						, MTC_NOT_SUPPORTED},
	{ MTC_MAX_CMD						,mtc_null_rsp						, MTC_NOT_SUPPORTED},
};
