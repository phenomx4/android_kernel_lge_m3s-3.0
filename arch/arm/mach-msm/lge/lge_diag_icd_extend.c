/*
  Program : ICD KEY 

  Author : khlee

  Date : 2010.01.26
*/
/* ==========================================================================
===========================================================================*/
#include <linux/module.h>
#include <mach/lge_diag_icd_extend.h>
#include <mach/lge_diagcmd.h>
#include <mach/lge_diag_testmode.h>
#include <mach/lge_diag_communication.h>
#include <mach/lge_diag_icd.h>
#include <linux/jiffies.h>
#include <mach/lge_diag_mtc.h>

/* ==========================================================================
===========================================================================*/

#ifndef JIFFIES_TO_MS
#define JIFFIES_TO_MS(t) ((t) * 1000 / HZ)
#endif

PACK (void *)LGF_Icd_Key(PACK (void*)req_pkt_ptr, uint16 pkt_len);
//static struct diagcmd_dev *diagpdev;
unsigned char g_diag_icd_check = 0;
key_msg_type slatekey;
touch_msg_type slatetouch;

extern PACK(void *) diagpkt_alloc (diagpkt_cmd_code_type code, unsigned int length);
extern int diag_event_log_start(void);
extern int diag_event_log_end(void);
extern char key_buf[MAX_KEY_BUFF_SIZE];
extern int count_key_buf;
extern icd_key_user_table_entry_type icd_key_mstr_tbl[256];
extern void diagpkt_commit (PACK(void *)pkt);

#ifdef CONFIG_LGE_DIAG_ATS_ETA_MTC_KEY_LOGGING
extern int event_log_start(void);
extern int event_log_end(void);
extern unsigned long int ats_mtc_log_mask;
#endif

/*==========================================================================*/
static key_msg_type make_key_log_packet(unsigned long keycode, unsigned long state)
{
	dword sendKeyValue = 0;
    key_msg_type msg;
	/* LGE_CHANGE [dojip.kim@lge.com] 2010-06-04 [LS670]
	 * don't send a raw diag packet in running MTC
	 */
	printk(KERN_INFO "make_key_log_packet\n");
	memset(&msg, 0, sizeof(key_msg_type));

	sendKeyValue = keycode;

	msg.cmd_code = 121;
	msg.ts_type = 0;	//2;
	msg.num_args = 2;
	msg.drop_cnt = 0;
	//ts_get(&msg.time[0]);
	msg.time[0] = 0;
	msg.time[1] = 0;
	msg.line_number = 261;
	msg.ss_id = 4604;
	msg.ss_mask = 1;


	msg.args[0] = sendKeyValue;
	msg.args[1] = state;


    /* ACCORDING TO SLATE ENGINEER'S COMMENT,
     * ARG1 : PRESS - KEYCODE, RELEASE - FF
     * ARG2 : PRESS - 00, RELEASE - KEYCODE

    if(state == 1) {
        msg.args[0] = sendKeyValue;
        msg.args[1] = 0;
    }
    else if (state == 0){
        msg.args[0] = 0xFFFF,
        msg.args[1] = sendKeyValue;
    }
     */
     
	memcpy(&msg.code[0], "Debounced %d", sizeof("Debounced %d"));
	//msg.code[12] = '\0';

	memcpy(&msg.file_name[0], "DiagDaemon.c", sizeof("DiagDaemon.c"));
	//msg.fle_name[13] = '\0';

	return msg;
}

static touch_msg_type make_touch_log_packet(unsigned long touch_x, unsigned long touch_y, unsigned long status)
{
	touch_msg_type msg;
	printk(KERN_INFO "make_touch_log_packet\n");
	memset(&msg, 0, sizeof(touch_msg_type));

	msg.cmd_code = 121;
	msg.ts_type = 0;	//2;
	msg.num_args = 3;
	msg.drop_cnt = 0;
	//ts_get(&msg.time[0]);
	msg.time[0] = 0;
	msg.time[1] = 0;
	msg.line_number = 261;
	msg.ss_id = 4604;
	msg.ss_mask = 1;

    /* KEEP THE ORIGINAL CODE
	if(status == 0) // push - "DWN"
		msg.args[0] = 0x004E5744;
	else	// release - "UP"
		msg.args[0] = 0x00005055;
	msg.args[1] = touch_x;//320/max_x;
	msg.args[2] = touch_y;//480/max_y;
	*/

    msg.args[1] = touch_x;
    msg.args[2] = touch_y;
    //msg.args[0] = status;

    switch(status) {
        case 4: // ETA_TOUCH_DOWN
        msg.args[0] = 0x004E5744;
        break;
        case 5: // ETA_TOUCH_UP
        msg.args[0] = 0x00005055;
        break;
        case 0: //ETA_TOUCH_MOVETO
        msg.args[0] = 0x00475244;
        break;
	}

	memcpy(&msg.code[0], "PenEvent %d,%d", sizeof("PenEvent %d,%d"));
	//msg.code[12] = '\0';

	memcpy(&msg.file_name[0], "DiagDaemon.c", sizeof("DiagDaemon.c"));
	//msg.fle_name[13] = '\0';

	return msg;
}

int slate_send_log_data(struct ats_mtc_key_log_type* p_ats_mtc_key_log)
{
	unsigned short log_len;
	key_msg_type * pRsp = NULL;
    touch_msg_type * pRsp1 = NULL;

	//switch (log->id) {
	switch (p_ats_mtc_key_log->log_id) {
		case ICD_LOG_ID_KEY:
			printk(KERN_INFO "MTC_ETA_LOG_ID_KEY\n");
			log_len = sizeof(key_msg_type);
     	    pRsp = (key_msg_type *)diagpkt_alloc(DIAG_EXT_MSG_F, log_len);
            if(!pRsp)
            {
                printk(KERN_ERR "[SLATE] packet for key logging response failed.\n");
                return 0;
            }
            //slatekey=make_key_log_packet((unsigned long)log->data.key.key_code,(unsigned long)log->data.key.hold);
            slatekey=make_key_log_packet((unsigned long)p_ats_mtc_key_log->y_code,(unsigned long)p_ats_mtc_key_log->x_hold);
            printk(KERN_INFO "[SLATE] got keycode .\n");
            memcpy(pRsp,&slatekey,log_len);
			printk(KERN_INFO "[SLATE] key.cmd_cod =%d,  func <%s>\n",slatekey.cmd_code, __func__);
			diagpkt_commit(pRsp);
            printk(KERN_INFO "[SLATE] diagbuf_encode_and_send_pkt returned! SEND finish.\n");
            printk(KERN_INFO "[SLATE] diagpkt_free end.\n");
			break;
		
		case ICD_LOG_ID_TOUCH:
			printk(KERN_INFO "MTC_ETA_LOG_ID_TOUCH\n");
			log_len = sizeof(touch_msg_type);
     	    pRsp1 = (touch_msg_type *)diagpkt_alloc(DIAG_EXT_MSG_F, log_len);
			//slatetouch=make_touch_log_packet(log->data.touch.x,log->data.touch.y,log->data.touch.action);
			slatetouch=make_touch_log_packet((unsigned long)p_ats_mtc_key_log->x_hold,
							(unsigned long)p_ats_mtc_key_log->y_code, (unsigned long)p_ats_mtc_key_log->action);
            memcpy(pRsp1,&slatetouch,log_len);
            diagpkt_commit(pRsp1);
            printk(KERN_INFO "[SLATE] touch msg commit complete.\n");
			break;
		
		default:
			break;
	}
    printk(KERN_INFO "[SLATE] slate_send_log_data end.\n");
	return 0;
}

/*
void LGT_Icd_send_key_log_data(struct ats_mtc_key_log_type* p_ats_mtc_key_log)
{
	struct icd_log_type icd_log;
	
	icd_log.id = (int)p_ats_mtc_key_log->log_id;
	icd_log.time = (unsigned int)JIFFIES_TO_MS(jiffies);

	if(icd_log.id == ICD_LOG_ID_KEY)
	{
		icd_log.data.key.hold = (int)p_ats_mtc_key_log->x_hold;
		icd_log.data.key.key_code = (unsigned int)p_ats_mtc_key_log->y_code;
	}
	else if(icd_log.id == ICD_LOG_ID_TOUCH)
	{
		icd_log.data.touch.action = (unsigned int)p_ats_mtc_key_log->action;
		icd_log.data.touch.x = (int)p_ats_mtc_key_log->x_hold;
		icd_log.data.touch.y = (int)p_ats_mtc_key_log->y_code;
	}
	else
	{
		printk(KERN_ERR "%s, invalid event type : %d\n", icd_log.id);
		return;
	}

	
	slate_send_log_data(&icd_log);

}
*/

PACK(void *) LGF_Icd_Key(PACK(void *)req_pkt_ptr,	/* pointer to request packet  */
			    		unsigned short pkt_len			/* length of request packet   */)
{
	unsigned char *req_ptr = (unsigned char *) req_pkt_ptr;
	int maskFieldOffset;
	unsigned short *startssid, *endssid;

	unsigned int rsp_len;
	DIAG_ICD_KEY_F_rsp_type *pRsp = NULL;

	rsp_len = sizeof(icd_key_get_rsq_type);

	printk(KERN_INFO "[SLATE] pkt dump. len : %d\n", pkt_len);

	// handle only key / touch logging command, other commands will be handled in CP
	if(req_ptr[1] == 0x4)
	{
		printk(KERN_INFO "[ICD_KEY] LGF_Icd_Key\n");
		printk(KERN_INFO "[ICD_KEY] LGF_Icd_Key rsp_len :(%d)\n", rsp_len);
	
		pRsp = (DIAG_ICD_KEY_F_rsp_type *) diagpkt_alloc(0x7d, rsp_len);
		if (pRsp == NULL) {
			printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
			return pRsp;
		}
		pRsp->hdr.cmd_code = req_ptr[0];
		pRsp->hdr.sub_cmd = req_ptr[1];
	
		g_diag_icd_check = 1; // set this flag to receive key / touch logging packet.
		
		startssid = (unsigned short *)(req_ptr+2);
		endssid = (unsigned short *)(req_ptr+4);

		printk(KERN_INFO " startssid : 0x%x=%d, endSSID : 0x%x=%d\n", *startssid,*startssid,*endssid, *endssid);

		//check 0x11FC mask value for slate.
		if((*startssid > 0x11FC) || (*endssid < 0x11FC))
			return 0;

		maskFieldOffset = (0xFC - req_ptr[2]) *4 + 8;
		printk(KERN_INFO "[SLATE]maskfieldoffset : %d\n", maskFieldOffset);

		if(req_ptr[maskFieldOffset]==1)
		{
			printk(KERN_INFO "[SLATE] %s : enable hard key press & pan tap event logging\n",__func__);
#ifdef CONFIG_LGE_DIAG_ATS_ETA_MTC_KEY_LOGGING
			event_log_start();
			ats_mtc_log_mask = 0xFFFFFFFF; // //ETA_LOGMASK_ENABLE_ALL
#endif
		}
		else
		{
			printk(KERN_INFO "mask is not 1:: %d\n", req_ptr[maskFieldOffset]);
#ifdef CONFIG_LGE_DIAG_ATS_ETA_MTC_KEY_LOGGING
			event_log_end();
			ats_mtc_log_mask = 0x00000000; // //ETA_LOGMASK_DISABLE_ALL
#endif
			printk(KERN_INFO "[SLATE] %s : disable hard key press & pan tap event logging\n",__func__);
		}
		
	}

	return (pRsp);
}
EXPORT_SYMBOL(LGF_Icd_Key); 

/*
DIAG_ICD_KEY_F_rsp_type* LGT_Icd_KeyTest(DIAG_ICD_KEY_F_req_type * pReq)
{

	unsigned int rsp_len;
	DIAG_ICD_KEY_F_rsp_type *pRsp;

	rsp_len = sizeof(icd_key_get_rsq_type);

	printk(KERN_INFO "[ICD_KEY] LGT_Icd_KeyTest\n");
	printk(KERN_INFO "[ICD_KEY] LGT_Icd_KeyTest rsp_len :(%d)\n", rsp_len);

	pRsp = (DIAG_ICD_KEY_F_rsp_type *) diagpkt_alloc(0x7d, rsp_len);
	if (pRsp == NULL) {
		printk(KERN_ERR "[ICD] diagpkt_alloc failed\n");
		return pRsp;
	}

	g_diag_icd_check = 1;

	pRsp->hdr.cmd_code = 0x7d;
	pRsp->hdr.sub_cmd = 0x5d;
	
	memset(pRsp->icd_key_rsp.get_key_data_rsq_info.key_data, 0x00, MAX_KEY_BUFF_SIZE);
	
	switch( pReq->icd_key_req.get_key_data_req_info.icd_key_read )
	{
		case 1:
#ifdef CONFIG_LGE_DIAG_ATS_ETA_MTC_KEY_LOGGING
			event_log_start();
			ats_mtc_log_mask = 0xFFFFFFFF; // //ETA_LOGMASK_ENABLE_ALL
#endif
			break;
		case 5:
#ifdef CONFIG_LGE_DIAG_ATS_ETA_MTC_KEY_LOGGING
			event_log_end();
			ats_mtc_log_mask = 0x00000000; // //ETA_LOGMASK_DISABLE_ALL
#endif
			break;
		default:
#ifdef CONFIG_LGE_DIAG_ATS_ETA_MTC_KEY_LOGGING
			event_log_end();
			ats_mtc_log_mask = 0x00000000; // //ETA_LOGMASK_DISABLE_ALL
#endif
			break;
	}  
	printk(KERN_ERR "[ICD_KEY] MainCmd = <%d>     SubCmd=<%d>   subsubCmd=<%d>\n",pReq->hdr.cmd_code, pReq->hdr.sub_cmd,  pReq->icd_key_req.get_key_data_req_info.icd_key_read);
	return (pRsp);	
}

icd_key_user_table_entry_type icd_key_mstr_tbl[256] = {
	// sub_command								fun_ptr									which procesor
	// SAR : Sprint Automation Requirement - START
	{0x5d,								LGT_Icd_KeyTest, 							1},
};
*/

