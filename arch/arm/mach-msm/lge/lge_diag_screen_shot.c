/*
  Program : Screen Shot

  Author : khlee

  Date : 2010.01.26
*/
/* ==========================================================================*/
#include <linux/module.h>
#include <mach/lge_diag_screen_shot.h>
#include <linux/fcntl.h> 
#include <linux/fs.h>
#include <mach/lge_diagcmd.h>
#include <linux/uaccess.h>
#include <mach/lge_diag_mtc.h>

/* ==========================================================================
===========================================================================*/
#undef SCREEN_DBG
#define LCD_BUFFER_SIZE LCD_MAIN_WIDTH * LCD_MAIN_HEIGHT * 4
//extern unsigned short tmp_img_block[320*480*4];
extern unsigned char lge_img_capture_32bpp[MTC_SCRN_BUF_32BPP_SIZE];
extern unsigned char lge_img_capture_16bpp[MTC_SCRN_BUF_SIZE_MAX];
/* ==========================================================================
===========================================================================*/
extern PACK(void *) diagpkt_alloc (diagpkt_cmd_code_type code, unsigned int length);
/*==========================================================================*/
/*==========================================================================*/

lcd_buf_info_type lcd_buf_info;

/*==========================================================================*/

#ifdef SCREEN_DBG
struct bmp_handle_t {
	unsigned long file_size;
	unsigned long reserved;
	unsigned long data_offset;
	unsigned long header_size;
	unsigned long width;
	unsigned long height;
	unsigned short planes;
	unsigned short bpp;
	unsigned long compression;
	unsigned long data_size;
	unsigned long hresolution;
	unsigned long vresolution;
	unsigned long colors;
	unsigned long important_colors;
	void *palette;
	char *data;
};

static void save_bmp_img(byte* name, byte* pBuf, unsigned long len)
{
	struct bmp_handle_t bmp;
	int count;
	struct file *phMscd_Filp = NULL;
	mm_segment_t old_fs=get_fs();

	set_fs(get_ds());

	phMscd_Filp = filp_open(name, O_CREAT | O_RDWR | O_LARGEFILE, 0777);

	if(IS_ERR(phMscd_Filp)) 
	{
		printk("open fail %s \n", name);
		return;
	}

	// make bmp header
	bmp.data_offset = 54;
	bmp.bpp = 16;
	bmp.data_size = LCD_MAIN_WIDTH * LCD_MAIN_HEIGHT * bmp.bpp / 8;
	bmp.file_size = bmp.data_size + bmp.data_offset;
	bmp.reserved = 0;
	bmp.header_size = 0x28;	// windows 3.1x, 95, NT...
	bmp.width = LCD_MAIN_WIDTH;
	bmp.height = LCD_MAIN_HEIGHT;
	bmp.planes = 1;
	bmp.compression = 0;	// none
	bmp.hresolution = 0x1711;
	bmp.vresolution = 0x1711;
	bmp.colors = 0;
	bmp.important_colors = 0;

	phMscd_Filp->f_pos = 0;

	count = 0;
	//count += write (fout, "BM", 2);
	count += phMscd_Filp->f_op->write(phMscd_Filp, "BM", 2, &phMscd_Filp->f_pos);
	//count += write (fout, &bmp.file_size, 52);
	count += phMscd_Filp->f_op->write(phMscd_Filp, (const char __user *)&bmp.file_size, 52, &phMscd_Filp->f_pos);
	//printf("bmp header written, count = %d\n", count);
	//count += write (fout, buf_p, input);
	count += phMscd_Filp->f_op->write(phMscd_Filp, pBuf, len, &phMscd_Filp->f_pos);
	printk(KERN_INFO "[ETA] bmp written, count = %d bytes\n", count);

	filp_close(phMscd_Filp,NULL);
		
	set_fs(old_fs);

}

static void save_buf_img(byte* name, byte* pBuf, unsigned long len)
{
  struct file *phMscd_Filp = NULL;
  mm_segment_t old_fs=get_fs();

  set_fs(get_ds());

  phMscd_Filp = filp_open(name, O_CREAT | O_RDWR | O_LARGEFILE, 0777);

  if(IS_ERR(phMscd_Filp)) 
  {
    printk("open fail screen capture \n" );
    return;
  }

   printk(KERN_INFO "path : %s, len : %ld\n", name, len);
   
   phMscd_Filp->f_pos = 0;

   phMscd_Filp->f_op->write(phMscd_Filp, pBuf, len, &phMscd_Filp->f_pos);
   filp_close(phMscd_Filp,NULL);

	
   set_fs(old_fs);
}
#endif

static void read_framebuffer(byte* pBuf)
{
  struct file *phMscd_Filp = NULL;
  mm_segment_t old_fs=get_fs();

  set_fs(get_ds());

  phMscd_Filp = filp_open("/dev/graphics/fb0", O_RDONLY |O_LARGEFILE, 0);

  if(IS_ERR(phMscd_Filp)) 
  {
    printk("open fail screen capture \n" );
    return;
  }

  phMscd_Filp->f_op->read(phMscd_Filp, pBuf, LCD_BUFFER_SIZE, &phMscd_Filp->f_pos);
  filp_close(phMscd_Filp,NULL);

  set_fs(old_fs);

}


//static void to_rgb565(byte* from, u16* to)
static void to_rgb565(byte* from, byte* to)
{
	int i;
	int r,g,b;
	//u16 h;

	int index = 0;

	for(i=0; i<LCD_BUFFER_SIZE; i+=4){
		/*
		r=from[i];
		g=from[i+1];
		b=from[i+2];
		
		h=(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3));

		to[i/4]=h;
		*/
		r=((from[i]>>3)&0x1F);
		g=((from[i+1]>>2)&0x3F);
		b=((from[i+2]>>3)&0x1F);

		// ignore alpa (every 4th bit)
		to[index ++] = ((g&0x07)<<5)|b;
		to[index ++] = ((r<<3)&0xF8)|((g>>3)&0x07);
		
		//h= ((((g&0x07)<<5)|b)&0x00FF) | (((((r<<3)&0xF8)|((g>>3)&0x07))<<8)&0xFF00);
		//to[i/4]=h;
		
	
	}
}


/*
int removefile( char const *filename )
{
  char *argv[4] = { NULL, NULL, NULL, NULL };
  char *envp[] = {
		"HOME=/",
		"TERM=linux",
		NULL,
	};
  if ( !filename )
  return -EINVAL;

  argv[0] = "/system/bin/rm";
  argv[1] = "-f";
  argv[2] = (char *)filename;

  return call_usermodehelper( argv[0], argv, envp, 0 );
}
*/

PACK (void *)LGF_ScreenShot (
        PACK (void	*)req_pkt_ptr,	/* pointer to request packet  */
        uint16		pkt_len )		      /* length of request packet   */
{
	diag_screen_shot_type *req_ptr = (diag_screen_shot_type *)req_pkt_ptr;
	diag_screen_shot_type *rsp_ptr = 0;
	int rsp_len=0;

//LGE_CHANGE_S [US730] [TestMode] [jinhwan.do@lge.com] 2012-02-07, Temporary blocking source for Test mode porting.
#if 0 
	uint8 red, green, blue;
	int x_start=0,x_end=0;
	int input=0;
	byte *pImgBlock;
	int x=0, y=0, i=0;
#endif
//LGE_CHANGE_S [US730] [TestMode] [jinhwan.do@lge.com] 2012-02-07, Temporary blocking source for Test mode porting.
   
printk("[UTS] %s, %d : cmd = %d, sub_cmd = %d\n",__func__,__LINE__,req_ptr->lcd_bk_ctrl.sub_cmd_code,req_ptr->lcd_buf.seq_flow);  

  	switch(req_ptr->lcd_bk_ctrl.sub_cmd_code)
  	{
    	case SCREEN_SHOT_BK_CTRL:
	  		break;
    	case SCREEN_SHOT_LCD_BUF:
      		switch(req_ptr->lcd_buf.seq_flow)
      		{
        		case SEQ_START:
					memset(lge_img_capture_32bpp, 0, sizeof(lge_img_capture_32bpp));
					lcd_buf_info.buftmp = lge_img_capture_32bpp;
					memset(lge_img_capture_16bpp, 0, sizeof(lge_img_capture_16bpp));
					lcd_buf_info.buf = lge_img_capture_16bpp;
					
          			rsp_len = sizeof(diag_lcd_get_buf_req_type);
          			rsp_ptr = (diag_screen_shot_type *)diagpkt_alloc(DIAG_LGF_SCREEN_SHOT_F, rsp_len - SCREEN_SHOT_PACK_LEN);

		  			if (!rsp_ptr)
		  			{
		  				printk(KERN_ERR "%s rsp_ptr allocation failed!!!\n", __func__);
		  				return 0;
		  			}

		          	lcd_buf_info.is_fast_mode = req_ptr->lcd_buf.is_fast_mode;
		          	lcd_buf_info.full_draw = req_ptr->lcd_buf.full_draw;
		          	lcd_buf_info.update = TRUE;
		          	lcd_buf_info.updated = FALSE;
		          	lcd_buf_info.width = LCD_MAIN_WIDTH;
		          	lcd_buf_info.height = LCD_MAIN_HEIGHT;
		          	lcd_buf_info.total_bytes = lcd_buf_info.width * lcd_buf_info.height * 2;
		          	lcd_buf_info.sended_bytes = 0;
		          	lcd_buf_info.update = FALSE;
		          	lcd_buf_info.updated = TRUE;
                  #if 1
		          	read_framebuffer(lcd_buf_info.buftmp );    // read file

		          	//to_rgb565(lcd_buf_info.buftmp, (u16 *)lcd_buf_info.buf);
		            to_rgb565(lcd_buf_info.buftmp, lcd_buf_info.buf);

#ifdef SCREEN_DBG
		            save_buf_img("/data/buftmp.img", lcd_buf_info.buftmp, (unsigned long)sizeof(lge_img_capture_32bpp));
		            save_buf_img("/data/buf.img", lcd_buf_info.buf, (unsigned long)sizeof(lge_img_capture_16bpp));	
		            save_bmp_img("/data/buf.bmp", lcd_buf_info.buf, (unsigned long)sizeof(lge_img_capture_16bpp));
#endif
                  #else
                  read_framebuffer((byte*)tmp_img_block);
                  pImgBlock = lcd_buf_info.buf;
                  for (y = 0; y < 480; y++) 
                  {
                     printk(" \n y = %d ",y);
                  	x_start = y*320*4;
                  	x_end = (y*320*4) + (0+320)*4;
                     printk("\n x_start = %d, x_end=%d \n",x_start,x_end );
                  	for ( i = x_start; i < x_end; i += 4) 
                  	{
                        printk(" \n i = %d ",i);
                  		red = ((tmp_img_block[i]>>3)&0x1F);  
                  		green = ((tmp_img_block[i+1]>>2)&0x3F);
                  		blue = ((tmp_img_block[i+2]>>3)&0x1F);
                  		// ignore alpa (every 4th bit)
                  		pImgBlock[input++] = ((green&0x07)<<5)|blue;
                  		pImgBlock[input++] = ((red<<3)&0xF8)|((green>>3)&0x07);
                  	}
                     printk("\n input = %d \n",input);
                  }
                  #endif
                  printk(" \n exit from for \n");
					rsp_ptr->lcd_buf.cmd_code = DIAG_LGF_SCREEN_SHOT_F;
		          	rsp_ptr->lcd_buf.sub_cmd_code = SCREEN_SHOT_LCD_BUF;
		          	rsp_ptr->lcd_buf.ok = TRUE;
					rsp_ptr->lcd_buf.is_main_lcd = FALSE;
					rsp_ptr->lcd_buf.x = 0;
					rsp_ptr->lcd_buf.y = 0;
					rsp_ptr->lcd_buf.width = lcd_buf_info.width;
					rsp_ptr->lcd_buf.height = lcd_buf_info.height;
		          	rsp_ptr->lcd_buf.seq_flow = SEQ_START;

		          	break;
        		case SEQ_REGET_BUF:
          			lcd_buf_info.sended_bytes = 0;
        		case SEQ_GET_BUF:
          			if(lcd_buf_info.updated == TRUE)
          			{
              			rsp_len = sizeof(diag_lcd_get_buf_req_type);
              			rsp_ptr = (diag_screen_shot_type *)diagpkt_alloc(DIAG_LGF_SCREEN_SHOT_F, rsp_len);

						rsp_ptr->lcd_buf.cmd_code = DIAG_LGF_SCREEN_SHOT_F;
						rsp_ptr->lcd_buf.sub_cmd_code = SCREEN_SHOT_LCD_BUF;
						rsp_ptr->lcd_buf.ok = TRUE;
              			rsp_ptr->lcd_buf.is_main_lcd = TRUE;
              			rsp_ptr->lcd_buf.x = lcd_buf_info.x;
              			rsp_ptr->lcd_buf.y = lcd_buf_info.y;
              			rsp_ptr->lcd_buf.width = lcd_buf_info.width;
              			rsp_ptr->lcd_buf.height = lcd_buf_info.height;
              			rsp_ptr->lcd_buf.total_bytes = lcd_buf_info.total_bytes;
              			rsp_ptr->lcd_buf.sended_bytes = lcd_buf_info.sended_bytes;
              			rsp_ptr->lcd_buf.packed = FALSE;
              			rsp_ptr->lcd_buf.is_fast_mode = lcd_buf_info.is_fast_mode;
              			rsp_ptr->lcd_buf.full_draw = lcd_buf_info.full_draw;

              			if(lcd_buf_info.total_bytes < SCREEN_SHOT_PACK_LEN)
              			{
                        	// completed
                  			lcd_buf_info.sended_bytes = 0;
                  			rsp_ptr->lcd_buf.sended_bytes = lcd_buf_info.sended_bytes;

                  			memcpy((void *)&(rsp_ptr->lcd_buf.buf[0]), (void *)&(lcd_buf_info.buf[lcd_buf_info.sended_bytes]), lcd_buf_info.total_bytes);
                  			rsp_ptr->lcd_buf.seq_flow = SEQ_GET_BUF_COMPLETED;
                  			lcd_buf_info.update = TRUE;
                  			lcd_buf_info.updated = FALSE;
                		}
              			else if(lcd_buf_info.total_bytes <= lcd_buf_info.sended_bytes)
              			{
                    		// completed
                			lcd_buf_info.sended_bytes -= SCREEN_SHOT_PACK_LEN;
                			rsp_ptr->lcd_buf.sended_bytes = lcd_buf_info.sended_bytes;

                			memcpy((void *)&(rsp_ptr->lcd_buf.buf[0]), (void *)&(lcd_buf_info.buf[lcd_buf_info.sended_bytes]), lcd_buf_info.total_bytes - lcd_buf_info.sended_bytes);

                			rsp_ptr->lcd_buf.seq_flow = SEQ_GET_BUF_COMPLETED;
                			lcd_buf_info.update = TRUE;
                			lcd_buf_info.updated = FALSE;
              			}
              			else
              			{
                    		// getting
                			memcpy((void *)&(rsp_ptr->lcd_buf.buf[0]), (void *)&(lcd_buf_info.buf[lcd_buf_info.sended_bytes]), SCREEN_SHOT_PACK_LEN);
                			lcd_buf_info.sended_bytes += SCREEN_SHOT_PACK_LEN;
                			rsp_ptr->lcd_buf.seq_flow = SEQ_GET_BUF;
              			}
            		}
            		else
            		{
              			rsp_len = sizeof(diag_lcd_get_buf_req_type);
              			rsp_ptr = (diag_screen_shot_type *)diagpkt_alloc(DIAG_LGF_SCREEN_SHOT_F, rsp_len - SCREEN_SHOT_PACK_LEN);

						rsp_ptr->lcd_buf.cmd_code = DIAG_LGF_SCREEN_SHOT_F;
						rsp_ptr->lcd_buf.sub_cmd_code = SCREEN_SHOT_LCD_BUF;
              			rsp_ptr->lcd_buf.is_main_lcd = TRUE;
              			rsp_ptr->lcd_buf.x = lcd_buf_info.x;
              			rsp_ptr->lcd_buf.y = lcd_buf_info.y;
              			rsp_ptr->lcd_buf.width = lcd_buf_info.width;
              			rsp_ptr->lcd_buf.height = lcd_buf_info.height;
              			rsp_ptr->lcd_buf.total_bytes = lcd_buf_info.total_bytes;
              			rsp_ptr->lcd_buf.sended_bytes = lcd_buf_info.sended_bytes;
              			rsp_ptr->lcd_buf.packed = FALSE;
              			rsp_ptr->lcd_buf.is_fast_mode = lcd_buf_info.is_fast_mode;
              			rsp_ptr->lcd_buf.full_draw = lcd_buf_info.full_draw;
              			rsp_ptr->lcd_buf.seq_flow = SEQ_GET_BUF_SUSPEND;
            		}
 					break;
        		case SEQ_STOP:
           			rsp_len = sizeof(diag_lcd_get_buf_req_type);
           			rsp_ptr = (diag_screen_shot_type *)diagpkt_alloc(DIAG_LGF_SCREEN_SHOT_F, rsp_len - SCREEN_SHOT_PACK_LEN);

					rsp_ptr->lcd_buf.cmd_code = DIAG_LGF_SCREEN_SHOT_F;
					rsp_ptr->lcd_buf.sub_cmd_code = SCREEN_SHOT_LCD_BUF;
           			rsp_ptr->lcd_buf.ok = TRUE;
           			rsp_ptr->lcd_buf.seq_flow = SEQ_STOP;

           			lcd_buf_info.update = FALSE;
           			lcd_buf_info.updated = FALSE;
        			break;		
      		}
      		break;  		
	}
	return (rsp_ptr);	
}        

EXPORT_SYMBOL(LGF_ScreenShot);        
