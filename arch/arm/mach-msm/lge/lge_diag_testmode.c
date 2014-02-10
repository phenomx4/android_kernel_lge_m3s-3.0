#include <linux/module.h>
#include <linux/input.h>
#include <linux/syscalls.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fcntl.h> 
#include <linux/fs.h>
#include <linux/uaccess.h>

#include <mach/lge_diag_communication.h>
#include <mach/lge_diag_testmode.h>
#include <mach/lge_diagcmd.h>
#include <mach/lg_backup_items.h>

//choongnam.kim 2012.04.27 to enable USBLOCK hidden menu.
#include <linux/crypto.h>
#include <crypto/hash.h>
#include <linux/gpio.h>
#include <mach/board_lge.h>
/* 2011-10-13, dongseok.ok@lge.com, Add Wi-Fi Testmode [START] */
#include <linux/parser.h>
#define WL_IS_WITHIN(min,max,expr)         (((min)<=(expr))&&((max)>(expr)))
/* 2011-10-13, dongseok.ok@lge.com, Add Wi-Fi Testmode [END] */
//#define SYS_GPIO_SD_DET
#ifdef CONFIG_TSPDRV
extern int Immersion_vib_isON(void);
extern uint32_t get_lge_hw_revision(void);
#endif
#define FACTORY_RESET_STR       "FACT_RESET_"
#define FACTORY_RESET_STR_SIZE	11
#define FACTORY_RESET_BLK 1 // read / write on the first block

//[20101010] NFC, addy.kim@lge.com [START]
#define NFC_RESULT_PATH 	"/sys/class/lg_fw_diagclass/lg_fw_diagcmd/nfc_testmode_result"
//[20101010] NFC, addy.kim@lge.com [END]

#define MSLEEP_CNT 100

//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [Start]
#define MOTOR_STATUS_PATH 	"/sys/class/timed_output/vibrator/enable"
//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [End]

//LGE_CHANGE [jinhwan.do][2012.05.15] Merge from VS930 MFT command's source [Start]
#ifdef CONFIG_LGE_DIAG_MFT
#define LGE_CAMERA_FILE "/mnt/sdcard/auto_test_picture.jpg"
#define LGE_CAMCORDER_FILE "/mnt/sdcard/auto_test_picture.3gp"

unsigned int position = 0;
unsigned int total_size = 0;
char mft_key_buf[MAX_MFT_KEY_BUFF_SIZE];
int count_mft_key_buf = 0;
#endif
//LGE_CHANGE [jinhwan.do][2012.05.15] Merge from VS930 MFT command's source [End]

//typedef struct MmcPartition MmcPartition;
typedef struct MmcPartition {
    char *device_index;
    char *filesystem;
    char *name;
    unsigned dstatus;
    unsigned dtype ;
    unsigned dfirstsec;
    unsigned dsize;
} MmcPartition;

struct statfs_local {
 	__u32 f_type;
 	__u32 f_bsize;
 	__u32 f_blocks;
 	__u32 f_bfree;
 	__u32 f_bavail;
 	__u32 f_files;
 	__u32 f_ffree;
 	__kernel_fsid_t f_fsid;
 	__u32 f_namelen;
 	__u32 f_frsize;
 	__u32 f_spare[5];
};

typedef struct {
	char ret[32];
} testmode_rsp_from_diag_type;

uint8_t if_condition_is_on_air_plain_mode = 0;

char key_buf[MAX_KEY_BUFF_SIZE];
boolean if_condition_is_on_key_buffering = FALSE;
int count_key_buf = 0;


static struct diagcmd_dev *diagpdev;
static unsigned char test_mode_factory_reset_status = FACTORY_RESET_START;
//LGE_CHANGE [Testmode][jy0127.jang@lge.com]2012-06-06,Testmode 9.0 Acoustic dealy
test_mode_req_acoustic_type loopback_state=ACOUSTIC_OFF;

extern int db_integrity_ready;
extern int fpri_crc_ready;
extern int file_crc_ready;
extern int code_partition_crc_ready;
extern int total_crc_ready;
extern int db_dump_ready;	
extern int db_copy_ready;
extern testmode_rsp_from_diag_type integrity_ret;
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-03-14, SWVersion [Start]
extern testmode_rsp_from_diag_type sp_ver;
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-03-14, SWVersion [End]
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-02, get Provisioned for test mode [Start]
extern testmode_rsp_from_diag_type test_get_provisioned;
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-02, get Provisioned for test mode [End]
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-04, Manual Mode value backup [Start]
extern int manual_mode_on;
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-04, Manual Mode value backup [End]
#ifdef CONFIG_LGE_DIAG_MFT
extern int spk_vol_level;
extern int mft_avr_record_time;  //LGE_CHANGE [jinhwan.do][2012.04.21] Test Mode 9.0 MFT Camera/Camcordor
extern int mft_cam_led_status;  //LGE_CHANGE [jinhwan.do][2012.04.21] Test Mode 9.0 MFT Camera/Camcordor
extern int mft_cam_af_status;  //LGE_CHANGE [jinhwan.do][2012.06.02] Test Mode 9.0 MFT Camera/Camcorder
extern int mft_avr_led_status;  //LGE_CHANGE [jinhwan.do][2012.06.02] Test Mode 9.0 MFT Camera/Camcorder
extern int mft_avr_play_status; //LGE_CHANGE [jinhwan.do][2012.06.02] Test Mode 9.0 MFT Camera/Camcorder
#endif
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-16, TestMode 9.0, get Acoustic Status [Start]
extern int acoustic_status;
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-16, TestMode 9.0, get Acoustic Status [End]

extern PACK(void *) diagpkt_alloc (diagpkt_cmd_code_type code, unsigned int length);
extern PACK(void *) diagpkt_free (PACK(void *)pkt);
extern void send_to_arm9( void*	pReq, void	*pRsp, unsigned int rsp_len);
extern testmode_user_table_entry_type testmode_mstr_tbl[TESTMODE_MSTR_TBL_SIZE];
extern int diag_event_log_start(void);
extern int diag_event_log_end(void);
extern unsigned int LGF_KeycodeTrans(word input);
extern void LGF_SendKey(word keycode);
extern void set_operation_mode(boolean isOnline);
extern struct input_dev* get_ats_input_dev(void);
extern int boot_complete_info;
extern void remote_set_operation_mode(int info);
extern void remote_set_ftm_boot(int info);
extern void remote_set_ftmboot_reset(uint32 info);

extern int lge_erase_block(int secnum, size_t size);
extern int lge_write_block(int secnum, unsigned char *buf, size_t size);
extern int lge_read_block(int secnum, unsigned char *buf, size_t size);
extern int lge_mmc_scan_partitions(void);
extern const MmcPartition *lge_mmc_find_partition_by_name(const char *name);
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-04, Manual Mode value backup [Start]
//extern int msm_get_manual_test_mode(void);
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-04, Manual Mode value backup [End]

//LGE_CHANGE [jinhwan.do][2012.06.11] When LCD turn off, push home key for LCD turn on [Start]
extern int lm3530_get_state(void);
//LGE_CHANGE [jinhwan.do][2012.06.11] When LCD turn off, push home key for LCD turn on [End]

#ifdef CONFIG_LGE_DIAG_MFT
extern int read_Framebuffer_Testmode(char *path, unsigned int x, unsigned int y, unsigned int w, unsigned int h);
#endif

PACK (void *)LGF_TestMode(PACK (void*)req_pkt_ptr, uint16 pkt_len);
void* LGF_TestModeFactoryReset(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp);
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-03-14, SWVersion [Start]
void* LGF_TestModeVer(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp);
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-03-14, SWVersion [End]
void* LGF_TestMotor(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type *pRsp);
void* LGF_TestAcoustic(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp);
void* LGF_TestCam(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp);
void* LGT_TestModeKeyTest(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type *pRsp);
void* LGF_ExternalSocketMemory(	test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type	*pRsp);
#ifndef LG_BTUI_TEST_MODE
void* LGF_TestModeBlueTooth(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp);
#endif
void* LGF_TestModeMP3(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp);
void* LGF_MemoryFormatTest(test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type *pRsp);
void* LGF_TestModeKeyData(test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type *pRsp);
void* LGF_MemoryVolumeCheck(test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type *pRsp);
void* LGF_PowerSaveMode(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp);
void* LGF_PowerSaveMode(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp);
void* LGF_TestModeSpeakerPhone(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp);
void* LGF_TestScriptItemSet(test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type	*pRsp);
void* LGT_TestModeVolumeLevel(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp);
void* LGF_TestModeFboot(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp);
void* LGF_TestModeDBIntegrityCheck(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp);
void* LGF_TestModeFOTAIDCheck(test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type* pRsp);
void* LGF_TestModePowerReset(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp);
void* LGF_TestModeKEYLOCK(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp);
#if defined (CONFIG_LGE_LCD_K_CAL)
void* LGF_TestLCD_Cal(test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type	*pRsp);
#endif
void* LGF_TestModeWiFiMACRW(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp);
//[20111006] addy.kim@lge.com [START]
void* LGF_TestModeNFC( test_mode_req_type*	pReq, DIAG_TEST_MODE_F_rsp_type	*pRsp);
//[20111006] addy.kim@lge.com [END]		
void* linux_app_handler(test_mode_req_type*	pReq, DIAG_TEST_MODE_F_rsp_type	*pRsp);
#ifdef CONFIG_LGE_DIAG_MFT
void* LGF_TestMode_NotSupported(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp);
void* LGF_TestMode_MFT_Camera(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp);
void* LGF_TestMode_MFT_Camcorder(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp);
void* LGF_TestMode_MFT_Key(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp);
void* LGF_TestMode_MFT_Volume(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp);
void* LGF_TestMode_MFT_TouchDraw(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp);
void* LGF_TestMode_Mft_Lcd(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp);
#endif /* CONFIG_LGE_DIAG_MFT */
//LGE_CHANGE_S[TestMode][hongsic.kim@lge.com] 2012-04-10, TESTMODE CMD ADD Earjack_check porting [Start]
void* LGF_TestModeEarJackCheck(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp);
//LGE_CHANGE_S[TestMode][hongsic.kim@lge.com] 2012-04-10, TESTMODE CMD ADD Earjack_check porting [End]
// +s AAT apk manager cwgf.lee@lge.com
void* LGF_TestMode_AAT_Transfer(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp);
// +e AAT apk manager cwgf.lee@lge.com

/* 2011-10-13, dongseok.ok@lge.com, Add Wi-Fi Testmode function [START] */
typedef struct _rx_packet_info 
{
	int goodpacket;
	int badpacket;
} rx_packet_info_t;

enum {
	Param_none = -1,
	Param_goodpacket,
	Param_badpacket,
	Param_end,
	Param_err,
};

static const match_table_t param_tokens = {
	{Param_goodpacket, "good=%d"},
	{Param_badpacket, "bad=%d"},
	{Param_end,	"END"},
	{Param_err, NULL}
};

void* LGF_TestModeWLAN(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type *pRsp);
/* 2011-10-13, dongseok.ok@lge.com, Add Wi-Fi Testmode function [END] */

testmode_user_table_entry_type testmode_mstr_tbl[TESTMODE_MSTR_TBL_SIZE] =
{
	/* sub command							func_ptr						processor type */
	/* 0 ~ 5   */
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-03-14, SWVersion [Start]
//	{TEST_MODE_VERSION,						NULL,							ARM9_PROCESSOR},
	{TEST_MODE_VERSION,						LGF_TestModeVer,							ARM11_PROCESSOR},
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-03-14, SWVersion [End]
	{TEST_MODE_LCD,							linux_app_handler,				ARM11_PROCESSOR},
#if defined (CONFIG_LGE_LCD_K_CAL)
	{TEST_MODE_LCD_CAL, 	  				LGF_TestLCD_Cal,				ARM11_PROCESSOR },
#endif
	{TEST_MODE_MOTOR,						LGF_TestMotor,					ARM11_PROCESSOR},
	{TEST_MODE_ACOUSTIC,					LGF_TestAcoustic,				ARM11_PROCESSOR},
	/* 6 ~ 10  */
	{TEST_MODE_CAM,							LGF_TestCam,					ARM11_PROCESSOR},
	/* 11 ~ 15 */
	/* 16 ~ 20 */
	/* 21 ~ 25 */
	{TEST_MODE_KEY_TEST,					LGT_TestModeKeyTest,			ARM11_PROCESSOR},	
//LGE_CHANGE_S[TestMode][hongsic.kim@lge.com] 2012-06-06, SD Card  Porting for TestMode CMD [Start]
	{TEST_MODE_EXT_SOCKET_TEST,          	linux_app_handler, 				ARM11_PROCESSOR},
//LGE_CHANGE_S[TestMode][hongsic.kim@lge.com] 2012-06-06, SD Card  Porting for TestMode CMD [Start]	
#ifndef LG_BTUI_TEST_MODE
	{TEST_MODE_BLUETOOTH_TEST,           	LGF_TestModeBlueTooth,    		ARM11_PROCESSOR},
#endif
	/* 26 ~ 30 */
	{TEST_MODE_MP3_TEST,                 	LGF_TestModeMP3,          		ARM11_PROCESSOR},
	/* 31 ~ 35 */
	{TEST_MODE_ACCEL_SENSOR_TEST,        	linux_app_handler,        		ARM11_PROCESSOR},
//LGE_CHANGE jinhwan.do 20120326 NFC is not supported [Start]
	//{ TEST_MODE_ALCOHOL_SENSOR_TEST,		LGF_TestModeNFC,				ARM11_PROCESSOR},
	{ TEST_MODE_ALCOHOL_SENSOR_TEST,		LGF_TestMode_NotSupported,				ARM11_PROCESSOR},	
//LGE_CHANGE jinhwan.do 20120326 NFC is not supported [End]]
	/* 2011-10-13, dongseok.ok@lge.com, Add Wi-Fi Testmode function [START] */
	{TEST_MODE_WIFI_TEST,                	LGF_TestModeWLAN,        		ARM11_PROCESSOR},
	/* 2011-10-13, dongseok.ok@lge.com, Add Wi-Fi Testmode function [END] */
	/* 36 ~ 40 */
	{TEST_MODE_KEY_DATA_TEST,            	LGF_TestModeKeyData,        	ARM11_PROCESSOR},
	{TEST_MODE_FACTORY_RESET_CHECK_TEST,	LGF_TestModeFactoryReset,		ARM11_PROCESSOR},
	/* 41 ~ 45 */
	{TEST_MODE_MEMORY_CAPA_TEST,         	LGF_MemoryVolumeCheck,    		ARM11_PROCESSOR},
	{TEST_MODE_SLEEP_MODE_TEST,          	LGF_PowerSaveMode,        		ARM11_PROCESSOR},
	{TEST_MODE_SPEAKER_PHONE_TEST,       	LGF_TestModeSpeakerPhone, 		ARM11_PROCESSOR},
	/* 46 ~ 50 */
    	{TEST_MODE_VCO_SELF_TUNNING_TEST,       NULL,                     		ARM9_PROCESSOR},
	{TEST_MODE_MRD_USB_TEST,             	NULL,                     		ARM9_PROCESSOR},
	{TEST_MODE_TEST_SCRIPT_MODE,         	LGF_TestScriptItemSet,    		ARM11_PROCESSOR},

	{TEST_MODE_PROXIMITY_SENSOR_TEST,    	linux_app_handler,        		ARM11_PROCESSOR},
	{TEST_MODE_FORMAT_MEMORY_TEST,		  	LGF_MemoryFormatTest,			ARM11_PROCESSOR},
	/* 51 ~ 55 */
	{TEST_MODE_VOLUME_TEST,              	LGT_TestModeVolumeLevel,  		ARM11_PROCESSOR},
	{TEST_MODE_CGPS_MEASURE_CNO,			NULL,							ARM9_PROCESSOR},
	/* 56 ~ 60 */
	{TEST_MODE_FIRST_BOOT_COMPLETE_TEST,  	LGF_TestModeFboot,        		ARM11_PROCESSOR},
	{TEST_MODE_LED_TEST, 					linux_app_handler,				ARM11_PROCESSOR},
	/* 61 ~ 65 */
	/* 66 ~ 70 */
	{TEST_MODE_PID_TEST,             	 	NULL,  							ARM9_PROCESSOR},
	/* 71 ~ 75 */
	{TEST_MODE_SW_VERSION, 					NULL, 							ARM9_PROCESSOR},
//LGE_CHANGE [jinhwan.do][2012.07.06] U0_CDMA project is not IMEI type [start]
	//{TEST_MODE_IME_TEST, 					NULL, 							ARM9_PROCESSOR},
	{TEST_MODE_IME_TEST, 				LGF_TestMode_NotSupported,		ARM11_PROCESSOR},
//LGE_CHANGE [jinhwan.do][2012.07.06] U0_CDMA project is not IMEI type [End]
	{TEST_MODE_IMPL_TEST, 					NULL, 							ARM9_PROCESSOR},
	{TEST_MODE_UNLOCK_CODE_TEST, 			NULL, 							ARM9_PROCESSOR},
	/* 76 ~ 80 */
	{TEST_MODE_IDDE_TEST, 					NULL, 							ARM9_PROCESSOR},
	{TEST_MODE_FULL_SIGNATURE_TEST, 		NULL, 							ARM9_PROCESSOR},
	{TEST_MODE_NT_CODE_TEST, 				NULL, 							ARM9_PROCESSOR},
	/* 81 ~ 85 */
	{TEST_MODE_CAL_CHECK, 					NULL,							ARM9_PROCESSOR},
#ifndef LG_BTUI_TEST_MODE
	{TEST_MODE_BLUETOOTH_TEST_RW,			NULL,							ARM9_PROCESSOR},
#endif
	/* 86 ~ 90 */
	{TEST_MODE_SKIP_WELCOM_TEST, 			NULL,							ARM9_PROCESSOR},
	{TEST_MODE_MAC_READ_WRITE,    			linux_app_handler,        		ARM11_PROCESSOR},
	/* 91 ~ 95 */
	{TEST_MODE_DB_INTEGRITY_CHECK,         	LGF_TestModeDBIntegrityCheck,	ARM11_PROCESSOR},
	{TEST_MODE_NVCRC_CHECK, 				NULL,							ARM9_PROCESSOR},
	{TEST_MODE_RELEASE_CURRENT_LIMIT,		NULL,							ARM9_PROCESSOR},
	/* 96 ~ 100*/
	{TEST_MODE_RESET_PRODUCTION, 			NULL,							ARM9_PROCESSOR},
//LGE_CHANGE jinhwan.do 20120326 FOTA ID CHECK is not supported [Start]
	{TEST_MODE_FOTA_ID_CHECK, 				LGF_TestMode_NotSupported,		ARM11_PROCESSOR},
	//{TEST_MODE_FOTA_ID_CHECK, 				LGF_TestModeFOTAIDCheck,		ARM11_PROCESSOR},
//LGE_CHANGE jinhwan.do 20120326 FOTA ID CHECK is not supported [End]
	{TEST_MODE_KEY_LOCK,					LGF_TestModeKEYLOCK,			ARM11_PROCESSOR},
	{TEST_MODE_SENSOR_CALIBRATION_TEST,     linux_app_handler,              ARM11_PROCESSOR},
	{TEST_MODE_ACCEL_SENSOR_ONOFF_TEST,	linux_app_handler,			ARM11_PROCESSOR},
	{TEST_MODE_COMPASS_SENSOR_TEST,		linux_app_handler,			ARM11_PROCESSOR},
	{TEST_MODE_GNSS_MEASURE_CNO, 			NULL,							ARM9_PROCESSOR},
	{TEST_MODE_POWER_RESET, 				LGF_TestModePowerReset,	ARM11_PROCESSOR},	
	//LGE_CHANGE_S mschoi 20120531 add JTAG disable cmd (LG_FW_JTAG_DISABLE)[Start]
	{TEST_MODE_JTAG_DISABLE,			NULL,							ARM9_PROCESSOR},
	//LGE_CHANGE jini1711 20120409	add JTAG disable cmd (LG_FW_JTAG_DISABLE)[End]
#ifdef CONFIG_LGE_DIAG_MFT	//LGE_CHANGE jinhwan.do 20120326 test mode MFT featrue porting
	//{TEST_MODE_MFT_LCD_TEST, 				linux_app_handler,				ARM11_PROCESSOR},
	{TEST_MODE_MFT_LCD_TEST, 				LGF_TestMode_Mft_Lcd,			ARM11_PROCESSOR},
	{TEST_MODE_MFT_LED_TEST, 				linux_app_handler,				ARM11_PROCESSOR},
	{TEST_MODE_MFT_CAMERA_TEST, 				LGF_TestMode_MFT_Camera,			ARM11_PROCESSOR},
	{TEST_MODE_MFT_CAM_TEST, 				LGF_TestMode_MFT_Camcorder,			ARM11_PROCESSOR},
	{TEST_MODE_MFT_KEY_TEST, 				LGF_TestMode_MFT_Key,			ARM11_PROCESSOR},
	{TEST_MODE_MFT_PROXIMITY_SENSOR_TEST, 				linux_app_handler,		ARM11_PROCESSOR},
	{TEST_MODE_MFT_ILLUMINATION_SENSOR_TEST, 				LGF_TestMode_NotSupported,		ARM11_PROCESSOR},
	{TEST_MODE_MFT_VOLUME_TEST, 				LGF_TestMode_MFT_Volume,		ARM11_PROCESSOR},
	{TEST_MODE_MFT_TOUCH_DRAW_TEST, 				LGF_TestMode_MFT_TouchDraw,			ARM11_PROCESSOR},
#endif /* CONFIG_LGE_DIAG_MFT */
//LGE_CHANGE jini1711 20120502 add AAT SET test list nv add and change AAT SET diag command  [Start]
    {TEST_MODE_MFT_AAT_SET, 					NULL, 							ARM9_PROCESSOR},
//LGE_CHANGE jini1711 20120502 add AAT SET test list nv add and change AAT SET diag command  [End]    
//LGE_CHANGE_S [ihyunju.kim@lge.com] 2012-04-19 US730 TestMode 
//LG_SYS_MISC_TESTMODE_LOOPBACK_CALL
       {TEST_MODE_LOOPBACK_CALL, 					NULL, 							ARM9_PROCESSOR},  
//LGE_CHANGE_E [ihyunju.kim@lge.com] 2012-04-19       
//LGE_CHANGE_S[TestMode][hongsic.kim@lge.com] 2012-04-10, TESTMODE CMD ADD Earjack_check porting [Start]
	{TEST_MODE_EARJACK_CHECK,				LGF_TestModeEarJackCheck,		ARM11_PROCESSOR},
//LGE_CHANGE_S[TestMode][hongsic.kim@lge.com] 2012-04-10, TESTMODE CMD ADD Earjack_check porting [End]	
	{TESTMODE_DLOAD_MEID, 					NULL,							ARM9_PROCESSOR},
// +s AAT apk manager cwgf.lee@lge.com
	{TEST_MODE_MFT_AAT_TRANSFER,       LGF_TestMode_AAT_Transfer,			ARM11_PROCESSOR},
// +e AAT apk manager cwgf.lee@lge.com

};
/* LGE_CHANGE_S [START] 2012.6.29 jaeho.cho@lge.com change mode by usbmode-manager */  
#ifdef CONFIG_LGE_USB_CHANGE_MODE_USERSPACE
#define LG_FACTORY_USB_PID 0x6000
extern void usbmode_change_delayed_work(void);
extern unsigned short get_current_usb_pid(void);
#endif
/* LGE_CHANGE_S [END] 2012.6.29 jaeho.cho@lge.com change mode by usbmode-manager */  

//LGE_CHANGE_S [jinhwan.do][2012.03.09]USB Access Lock Porting [Start]
#ifdef CONFIG_LGE_DIAG_USB_ACCESS_LOCK
#define SECURITY_BINARY_CODE	86  //LGE_CHANG [jinhwan.do][2012.07.02] for LGL86C DLL comand secode pkt 696 -> 75 -> 75 & 86 ->only 86

/* LGE_CHANGE_S [START] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */
#ifdef CONFIG_LGE_USB_ACCESS_LOCK_INODE   
static char usb_lock_key_enc[20] = {0x0, };
static int diag_set_usb_lock_key_string(const char *val, struct kernel_param *kp);
static int diag_get_usb_lock_key_string(char *buffer, struct kernel_param *kp);
module_param_call(usb_lock_key_enc, diag_set_usb_lock_key_string, diag_get_usb_lock_key_string,
					&usb_lock_key_enc, 0660);
MODULE_PARM_DESC(usb_lock_key_enc, "USB lock key encryption");
#endif
/* LGE_CHANGE_S [END] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */

/* LGE_CHANGE_S [START] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */
#ifdef CONFIG_LGE_DIAG_USB_ACCESS_LOCK_SHA1_ENCRYPTION   
int SHA1(const char * key, char * sha1_key);
char nv_usb_lock_key[20] = {0xb0, 0x21, 0x32, 0x08, 0x18, 0x08, 0xb4, 0x93, 0xc6, 0x1e, 0x86, 0x62, 0x6e, 0xe6, 0xc2, 0xe2, 0x93, 0x26, 0xa6, 0x62};
#endif
#if defined(CONFIG_LGE_DIAG_USB_ACCESS_LOCK_SHA1_ENCRYPTION) && defined(CONFIG_LGE_USB_ACCESS_LOCK_INODE)
static int diag_set_usb_lock_key_string(const char *val, struct kernel_param *kp)
{
	if (!val)
		return 0;

//	printk(KERN_ERR "[CHOONG] input=%x_%x_%x_%x_%x_%x_%x_%x_%x_%x_%x_%x_%x_%x_%x_%x_ \n"
//		, val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7]
//		, val[8], val[9], val[10], val[11], val[12], val[13], val[14], val[15]);
	if (strlen(val)!=16)
	{
		if( !(strlen(val)==17 && val[16]=='\n') )
			return -EINVAL;
	}

	SHA1(val, usb_lock_key_enc);

	return 0;
}
static int diag_get_usb_lock_key_string(char *buffer, struct kernel_param *kp)
{
//	byte nv_lock_key[20] = {0,};
//	extern void get_usb_lock_key(char* usb_lock_key);
//	extern void set_usb_lock(int lock);

	if (!buffer)
		return 0;
	
	if (!memcmp(usb_lock_key_enc,nv_usb_lock_key, 20 ))
	{
		strcpy(buffer, "correct");
	}
	else
		strcpy(buffer, "discorrect");

//	printk(KERN_ERR "[CHOONG] %s ret=%s\n", __func__, buffer);
	
	return (strlen(buffer));
}

static char usb_lock_state=0;

static int diag_set_usb_lock_state(const char *val, struct kernel_param *kp);
static int diag_get_usb_lock_state(char *buffer, struct kernel_param *kp);
module_param_call(usb_lock_state, diag_set_usb_lock_state, diag_get_usb_lock_state,
					&usb_lock_state, 0660);
MODULE_PARM_DESC(usb_lock_state, "USB lock state");

static int diag_set_usb_lock_state(const char *val, struct kernel_param *kp)
{
	extern int user_diag_enable;

	if (!val)
		return 0;
	
//	printk(KERN_ERR "[CHOONG] %s val=%s\n", __func__, val);
	
	if (!strncmp(val, "1", 1))
	{
		user_diag_enable = 0;
//		printk(KERN_ERR "[SoN] %s user_diag_enable set to disable\n", __func__);
	}
	else if(!strncmp(val, "0", 1))
	{
		user_diag_enable = 1;
//		printk(KERN_ERR "[SoN] %s user_diag_enable set to enable\n", __func__);
	}

	return 0;
}
static int diag_get_usb_lock_state(char *buffer, struct kernel_param *kp)
{
	extern int user_diag_enable;

	if (!buffer)
		return 0;
	
	if (user_diag_enable)
	{
		memcpy(buffer, "0", 1);
//		printk(KERN_ERR "[SoN] %s user_diag_enable is enable state\n", __func__);
	}
	else
	{
		memcpy(buffer, "1", 1);
//		printk(KERN_ERR "[SoN] %s user_diag_enable is disable state\n", __func__);

	}
	return 1;
} 
#endif 
/* LGE_CHANGE_S [END] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */

PACK (void *)LGF_TFSBProcess (
			PACK (void	*)req_pkt_ptr,	/* pointer to request packet  */
			uint16		pkt_len )			  /* length of request packet	*/
{
	DIAG_TF_SB_F_req_type *req_ptr = (DIAG_TF_SB_F_req_type *) req_pkt_ptr;
	DIAG_TF_SB_F_rsp_type *rsp_ptr = NULL;
	unsigned int rsp_len;

	extern void set_usb_lock(int lock);

	//LGE_CHANG [jinhwan.do][2012.07.02] for LGL86C DLL comand secode pkt 696 -> 75 -> 75 & 86 ->only 86
	if(req_ptr->seccode != SECURITY_BINARY_CODE) 
	{
		set_usb_lock(1);
	}

	rsp_len = sizeof(DIAG_TF_SB_F_rsp_type);
	rsp_ptr = (DIAG_TF_SB_F_rsp_type *)diagpkt_alloc(DIAG_TF_SB_CMD_F, rsp_len);

	rsp_ptr->cmd_code = req_ptr->cmd_code;	
	return (rsp_ptr);
}
EXPORT_SYMBOL(LGF_TFSBProcess);   

/* LGE_CHANGE_S [START] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */
#ifdef CONFIG_LGE_DIAG_USB_ACCESS_LOCK_SHA1_ENCRYPTION  
int SHA1(const char * key, char * sha1_key)
{
	struct crypto_shash *tfm = crypto_alloc_shash("sha1-generic", 0, 0);
	extern int crypto_shash_update(struct shash_desc *desc, const u8 *data, unsigned int len);
	extern int crypto_shash_final(struct shash_desc *desc, u8 *out);
	extern void crypto_destroy_tfm(void *mem, struct crypto_tfm *tfm);
	extern struct crypto_shash *crypto_alloc_shash(const char *alg_name, u32 type,u32 mask);

	struct {
		struct shash_desc shash;
		char ctx[tfm->descsize];
	} desc;

	if (key == NULL || sha1_key == NULL)
	{
		printk(KERN_ERR "%s : bad parameters\n", __func__);
		return -1;
	}

//	tfm = crypto_alloc_shash("sha1-generic", 0, 0);
		

	desc.shash.tfm = tfm;
	desc.shash.flags = 0;

//	printk(KERN_ERR "[CHOONG] input=%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x \n"
//		, key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7]
//		, key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);

	crypto_shash_init(&desc.shash);	
	crypto_shash_update(&desc.shash, key, USB_LOCK_KEY_LENGTH);
	crypto_shash_final(&desc.shash,sha1_key);

//	printk(KERN_ERR "[CHOONG] sha1_key=%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n"
//		, sha1_key[0], sha1_key[1], sha1_key[2], sha1_key[3], sha1_key[4], sha1_key[5], sha1_key[6], sha1_key[7]
//		, sha1_key[8], sha1_key[9], sha1_key[10], sha1_key[11], sha1_key[12], sha1_key[13], sha1_key[14], sha1_key[15]
//		, sha1_key[16], sha1_key[17], sha1_key[18], sha1_key[19]);

	crypto_destroy_tfm(tfm, &(tfm->base));

	return 0;			
	
}

PACK (void *)LGF_USBLOCKKEYProcess (
			PACK (void	*)req_pkt_ptr,	/* pointer to request packet  */
			uint16		pkt_len )			  /* length of request packet	*/
{
	DIAG_USB_LOCK_KEY_F_req_type *req_ptr = (DIAG_USB_LOCK_KEY_F_req_type *) req_pkt_ptr;
	DIAG_USB_LOCK_KEY_F_rsp_type *rsp_ptr;
	unsigned int rsp_len;
	extern void set_usb_lock_key(char* usb_lock_key, int length);

	char usb_lock_key[20]={0,};

	rsp_len = sizeof(DIAG_USB_LOCK_KEY_F_rsp_type);
	rsp_ptr = (DIAG_USB_LOCK_KEY_F_rsp_type *)diagpkt_alloc(DIAG_USB_LOCK_KEY_CMD_F, rsp_len);

	rsp_ptr->cmd_code = req_ptr->cmd_code;

	if (SHA1(req_ptr->key, usb_lock_key)== -1)
	{
		rsp_ptr->result = 4;	// NV_FAIL_S
	}
	else
	{
		set_usb_lock_key((char *)usb_lock_key, 20);
		rsp_ptr->result = 0;
	}

	return (rsp_ptr);
}
EXPORT_SYMBOL(LGF_USBLOCKKEYProcess);
#endif   
/* LGE_CHANGE_S [END] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */
PACK (void *)LGF_TFProcess (
			PACK (void	*)req_pkt_ptr,	/* pointer to request packet  */
			uint16		pkt_len )			  /* length of request packet	*/
{
	DIAG_TF_F_req_type *req_ptr = (DIAG_TF_F_req_type *) req_pkt_ptr;
	DIAG_TF_F_rsp_type *rsp_ptr = NULL;
	unsigned int rsp_len;
	/* LGE_CHANGE_S [START] 2012.6.29 jaeho.cho@lge.com change mode by usbmode-manager */  
#ifdef CONFIG_LGE_USB_CHANGE_MODE_USERSPACE
	unsigned short usb_pid = 0;
#endif
/* LGE_CHANGE_S [END] 2012.6.29 jaeho.cho@lge.com change mode by usbmode-manager */  

	extern int get_usb_lock(void);
	extern void set_usb_lock(int lock);
#ifdef CONFIG_LGE_DIAG_USB_ACCESS_LOCK_SHA1_ENCRYPTION
		extern void get_usb_lock_key(char* usb_lock_key);
#else
		extern void get_spc_code(char * spc_code);
#endif
#ifdef CONFIG_LGE_DIAG_USB_PERMANENT_LOCK
//	extern int user_diag_unlock_fail_cnt;
	extern void set_usb_unlock_fail_cnt(int cnt);
	extern int get_usb_unlock_fail_cnt(void);
#endif
#ifdef CONFIG_LGE_DIAG_USBLOCK_EFS_SYNC
	extern int get_usblock_efs_sync_result(void);
#endif

	rsp_len = sizeof(DIAG_TF_F_rsp_type);
	rsp_ptr = (DIAG_TF_F_rsp_type *)diagpkt_alloc(DIAG_TF_CMD_F, rsp_len);
	
	switch(req_ptr->sub_cmd)
	{
		case TF_SUB_CHECK_PORTLOCK:
			if (get_usb_lock())
				rsp_ptr->result = TF_STATUS_PORT_UNLOCK;
			else
				rsp_ptr->result = TF_STATUS_PORT_LOCK;
			break;

		case TF_SUB_LOCK_PORT:
/* LGE_CHANGE_S [START] 2012.6.29 jaeho.cho@lge.com change mode by usbmode-manager */  
#ifdef CONFIG_LGE_USB_CHANGE_MODE_USERSPACE
#ifdef CONFIG_USB_G_LGE_ANDROID
			usb_pid = get_current_usb_pid();
#endif
			if(usb_pid != LG_FACTORY_USB_PID)
			{
				usbmode_change_delayed_work();
			}
#endif
/* LGE_CHANGE_S [END] 2012.6.29 jaeho.cho@lge.com change mode by usbmode-manager */  
			set_usb_lock(1);
			rsp_ptr->result = TF_STATUS_SUCCESS;
			break;

		case TF_SUB_UNLOCK_PORT:
		{
/* LGE_CHANGE_S [START] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */  
#ifdef CONFIG_LGE_DIAG_USB_ACCESS_LOCK_SHA1_ENCRYPTION
			char nv_lock_key[20]={0,};
			char diag_lock_key[20] = {0,};

			if (sizeof(req_ptr->buf.keybuf) != USB_LOCK_KEY_LENGTH)
			{
#if 0	// 	LG_FW_USB_PERMANENT_LOCK
				if(user_diag_unlock_fail_cnt <5)
					user_diag_unlock_fail_cnt++;
#endif
				rsp_ptr->result = TF_STATUS_FAIL;
				break;
			}

			get_usb_lock_key((char*) nv_lock_key);

			SHA1(req_ptr->buf.keybuf, diag_lock_key);

			if (!memcmp(diag_lock_key,nv_lock_key, 20 ))
			{
				set_usb_lock(0);
				rsp_ptr->result = TF_STATUS_SUCCESS;
			}
			else
			{
#if 0	// 	LG_FW_USB_PERMANENT_LOCK
				if(user_diag_unlock_fail_cnt <5)
					user_diag_unlock_fail_cnt++;
#endif
				rsp_ptr->result = TF_STATUS_FAIL;
			}
#else
			char spc_code[6];
			
			get_spc_code(spc_code);

			if (memcmp((byte *)spc_code,req_ptr->buf.keybuf, PPE_UART_KEY_LENGTH )==0)
			{
				set_usb_lock(0);
				rsp_ptr->result = TF_STATUS_SUCCESS;
			}
			else
				rsp_ptr->result = TF_STATUS_FAIL;
#endif
/* LGE_CHANGE_S [END] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */

			break;
		}

/* LGE_CHANGE_S [START] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */   
#ifdef CONFIG_LGE_DIAG_USB_ACCESS_LOCK_SHA1_ENCRYPTION 
		case TF_SUB_KEY_VERIFY:
		{
			char nv_lock_key[20]={0,};
			char diag_lock_key[20] = {0,};

			if (sizeof(req_ptr->buf.keybuf) != USB_LOCK_KEY_LENGTH)
			{
				rsp_ptr->result = TF_STATUS_FAIL;
				break;
			}

			get_usb_lock_key((char*) nv_lock_key);
			SHA1(req_ptr->buf.keybuf, diag_lock_key);

			if (!memcmp(diag_lock_key,nv_lock_key, 20 ))
			{
				rsp_ptr->result = TF_STATUS_VER_KEY_OK;
			}
			else
			{
				rsp_ptr->result = TF_STATUS_VER_KEY_NG;
			}
			break;
		}	
#endif

#ifdef CONFIG_LGE_DIAG_USB_PERMANENT_LOCK
		case TF_SUB_CLEAR_USB_UNLOCK_FAIL_CNT:
		{
			set_usb_unlock_fail_cnt(0);

			if (get_usb_unlock_fail_cnt()) 
				rsp_ptr->result = TF_STATUS_FAIL;
			else
				rsp_ptr->result = TF_STATUS_SUCCESS;
			
			break;
		}

		case TF_SUB_GET_USB_UNLOCK_FAIL_CNT:
		{
			rsp_ptr->result = get_usb_unlock_fail_cnt();
			break;
		}
		
#endif

#ifdef CONFIG_LGE_DIAG_USBLOCK_EFS_SYNC		
		case TF_SUB_CHECK_EFS_SYNC:
			if (get_usblock_efs_sync_result())
				rsp_ptr->result = TF_STATUS_SUCCESS;
			else
				rsp_ptr->result = TF_STATUS_FAIL;
			break;
#endif 
/* LGE_CHANGE_S [END] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */

	}

	rsp_ptr->sub_cmd = req_ptr->sub_cmd;
	
	return (rsp_ptr);
}
EXPORT_SYMBOL(LGF_TFProcess);
#endif
//LGE_CHANGE_S [jinhwan.do][2012.03.09]USB Access Lock Porting [End]

PACK (void *)LGF_TestMode(PACK (void*)req_pkt_ptr, uint16 pkt_len)
{
	DIAG_TEST_MODE_F_req_type *req_ptr = (DIAG_TEST_MODE_F_req_type *) req_pkt_ptr;
  	DIAG_TEST_MODE_F_rsp_type *rsp_ptr = NULL;
	unsigned int rsp_len=0;
  	testmode_func_type func_ptr= NULL;
  	int nIndex = 0;
	int is_valid_arm9_command = 1;

  	diagpdev = diagcmd_get_dev();

	for(nIndex = 0 ; nIndex < TESTMODE_MSTR_TBL_SIZE  ; nIndex++)
  	{
		if(testmode_mstr_tbl[nIndex].cmd_code == req_ptr->sub_cmd_code)
		{
			if( testmode_mstr_tbl[nIndex].which_procesor == ARM11_PROCESSOR)
				func_ptr = testmode_mstr_tbl[nIndex].func_ptr;
			break;
		}
		else if(testmode_mstr_tbl[nIndex].cmd_code == MAX_TEST_MODE_SUBCMD)
		{
			break;
		}
		else
		{
			continue;
		}
  	}

	if( func_ptr != NULL)
	{
		switch(req_ptr->sub_cmd_code) {
			case TEST_MODE_FACTORY_RESET_CHECK_TEST:
			case TEST_MODE_FIRST_BOOT_COMPLETE_TEST:
//LGE_CHANGE [jinhwan.do][2012.06.28] response packet data filter [Start]
			case TEST_MODE_MP3_TEST:
			case TEST_MODE_VOLUME_TEST:
			case TEST_MODE_KEY_DATA_TEST:
			case TEST_MODE_SPEAKER_PHONE_TEST:
//LGE_CHANGE [jinhwan.do][2012.06.28] response packet data filter [End]
				rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type);
				break;
				
//LGE_CHANGE [jinhwan.do][2012.06.28] response packet data filter [Start]
			case TEST_MODE_KEY_TEST:
      				rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type) + sizeof(key_buf);
				break;
				
			case TEST_MODE_ACOUSTIC:
			case TEST_MODE_MOTOR:
			case TEST_MODE_SLEEP_MODE_TEST:
			case TEST_MODE_KEY_LOCK:
      				rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type) + sizeof(int);
				break;
//LGE_CHANGE [jinhwan.do][2012.06.28] response packet data filter [End]

			case TEST_MODE_TEST_SCRIPT_MODE:
      			rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type) + sizeof(test_mode_req_test_script_mode_type);
      			break;	

//LGE_CHANGE [jinhwan.do][2012.06.11] When LCD turn off, push home key for LCD turn on [Start]
			case TEST_MODE_MFT_LCD_TEST:
				if(lm3530_get_state() == 2)
				{
					LGF_SendKey(KEY_HOME);
					msleep(500);
				}
				rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type);			
				break; //TEST_MODE_MFT_LCD_TEST will be handled in Kernel first.
//LGE_CHANGE [jinhwan.do][2012.06.11] When LCD turn off, push home key for LCD turn on [End]

//LGE_CHANGE [myungsu.choi][2012.07.04] Version string size definition for fixing webdload failure [Start]
			case TEST_MODE_VERSION:
				rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type) + STR_BUF_SIZE;
				break;
//LGE_CHANGE [myungsu.choi][2012.07.04] Version string size definition for fixing webdload failure [End]

			default:
				rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type);
				break;
		}

		rsp_ptr = (DIAG_TEST_MODE_F_rsp_type *)diagpkt_alloc(DIAG_TEST_MODE_F, rsp_len);
		if(rsp_ptr == NULL)
		{
			printk(KERN_ERR "%s, allocation failed ! len : %d", __func__, rsp_len);
        	return rsp_ptr;
		}

		rsp_ptr->sub_cmd_code = req_ptr->sub_cmd_code;
  		rsp_ptr->ret_stat_code = TEST_OK_S; // test ok	
	
		return func_ptr( &(req_ptr->test_mode_req), rsp_ptr);
	}
	else
	{
		switch(req_ptr->sub_cmd_code) {

			case TEST_MODE_MANUAL_MODE_TEST:
				if(req_ptr->test_mode_req.test_manual_mode == MANUAL_MODE_CHECK)
					rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type) + sizeof(int); // add manual_test type
				else
					rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type);
				break;
			
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-03-14, SWVersion [Start]
//			case TEST_MODE_VERSION:
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-03-14, SWVersion [End]
			case TEST_MODE_VCO_SELF_TUNNING_TEST:	
			case TEST_MODE_MRD_USB_TEST:	
			case TEST_MODE_RESET_PRODUCTION: 
			case TEST_MODE_BATT_TEST:
			case TEST_MODE_CAL_CHECK:
			case TEST_MODE_PID_TEST:
			case TEST_MODE_RELEASE_CURRENT_LIMIT:
			case TEST_MODE_CGPS_MEASURE_CNO:
			case TEST_MODE_GNSS_MEASURE_CNO:
			case TEST_MODE_SW_VERSION:
			case TEST_MODE_BLUETOOTH_TEST_RW:
			case TEST_MODE_MAC_READ_WRITE:
//LGE_CHANGE_S [US730] [TestMode] [hongsic.kim@lge.com] 2012-02-14, Add Function for TestMode porting.					
			case TEST_MODE_UNLOCK_CODE_TEST:
			case TEST_MODE_FULL_SIGNATURE_TEST:
			case TEST_MODE_NVCRC_CHECK:
//LGE_CHANGE [jinhwan.do][2012.07.06] U0_CDMA project is not IMEI type [start]
//			case TEST_MODE_IME_TEST:
//LGE_CHANGE [jinhwan.do][2012.07.06] U0_CDMA project is not IMEI type [End]
			case TEST_MODE_SKIP_WELCOM_TEST:
			case TEST_MODE_IMPL_TEST:
			case TEST_MODE_IDDE_TEST:
			case TEST_MODE_NT_CODE_TEST:
//LGE_CHANGE_S mschoi 20120531 add JTAG disable cmd (LG_FW_JTAG_DISABLE)[Start]
			case TEST_MODE_JTAG_DISABLE:
//LGE_CHANGE jini1711 20120409	add JTAG disable cmd (LG_FW_JTAG_DISABLE)[End]	
//LGE_CHANGE_S [US730] [TestMode] [hongsic.kim@lge.com] 2012-02-14, Add Function for TestMode porting.			
//LGE_CHANGE jini1711 20120409 add AAT SET diag command [Start]
            case TEST_MODE_MFT_AAT_SET: 
//LGE_CHANGE jini1711 20120409 add AAT SET diag command [End]
//LGE_CHANGE_S [ihyunju.kim@lge.com] 2012-04-19 US730 TestMode 
//LG_SYS_MISC_TESTMODE_LOOPBACK_CALL
			case TEST_MODE_LOOPBACK_CALL: 
//LGE_CHANGE_E [ihyunju.kim@lge.com] 2012-04-19 
			case TESTMODE_DLOAD_MEID:
				rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type);
				break;

			default:
				is_valid_arm9_command=0;
				break;
		}
		
		if(is_valid_arm9_command == 1)
		{
			rsp_ptr = (DIAG_TEST_MODE_F_rsp_type *)diagpkt_alloc(DIAG_TEST_MODE_F, rsp_len);
			if(rsp_ptr == NULL)
				return rsp_ptr;

			rsp_ptr->sub_cmd_code = req_ptr->sub_cmd_code;
  			rsp_ptr->ret_stat_code = TEST_OK_S; // test ok	
			
			send_to_arm9((void*)req_ptr, (void*)rsp_ptr, rsp_len);
		}
	}
	
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-04, Manual Mode value backup [Start]
//	manual_mode_on = msm_get_manual_test_mode();
//	printk("[LGF_TestMode] manual_mode_on  =  %d\n", manual_mode_on );
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-04, Manual Mode value backup [End]

	return (rsp_ptr);
}
EXPORT_SYMBOL(LGF_TestMode);

void* linux_app_handler(test_mode_req_type*	pReq, DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
	diagpkt_free(pRsp);
  	return 0;
}

/* LCD QTEST */
PACK (void *)LGF_LcdQTest (
        PACK (void	*)req_pkt_ptr,	/* pointer to request packet  */
        uint16		pkt_len )		      /* length of request packet   */
{
	printk("[%s] LGF_LcdQTest\n", __func__ );

	/* Returns 0 for executing lg_diag_app */
	return 0;
}
EXPORT_SYMBOL(LGF_LcdQTest);

#if defined (CONFIG_LGE_LCD_K_CAL)
void* LGF_TestLCD_Cal(
		test_mode_req_type* pReq ,
		DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
	char ptr[30];
	
	pRsp->ret_stat_code = TEST_OK_S;
	printk("<6>" "pReq->lcd_cal: (%d)\n", pReq->lcd_cal.MaxRGB[0]);

	if (diagpdev != NULL){
		if (pReq->lcd_cal.MaxRGB[0] != 5)
			update_diagcmd_state(diagpdev, "LCD_Cal", pReq->lcd_cal.MaxRGB[0]);
		else {
			printk("<6>" "pReq->MaxRGB string type : %s\n",pReq->lcd_cal.MaxRGB);
        	sprintf(ptr,"LCD_Cal,%s",&pReq->lcd_cal.MaxRGB[1]);
			printk("<6>" "%s \n", ptr);
			update_diagcmd_state(diagpdev, ptr, pReq->lcd_cal.MaxRGB[0]);
		}
	}
	else
	{
		printk("\n[%s] error LCD_cal", __func__ );
		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}
	return pRsp;
}
#endif /*CONFIG_LGE_LCD_K_CAL*/

//#ifdef CONFIG_LGE_PCB_VERSION
byte CheckHWRev(void)
{
	// request packet of send_to_arm9 should be DIAG_TEST_MODE_F_req_type
	//test_mode_req_type Req;
	DIAG_TEST_MODE_F_req_type Req;
	DIAG_TEST_MODE_F_rsp_type *Rsp = NULL;
	unsigned int rsp_len=0;
	byte result = 0;

	/*
	char *pReq = (char *) &Req;
	char *pRsp = (char *) &Rsp;

	pReq[0] = 250;
	pReq[1] = 0;
	pReq[2] = 0;
	pReq[3] = 8;

	send_to_arm9(pReq , pRsp);
	printk("CheckHWRev> 0x%x 0x%x 0x%x 0x%x 0x%x\n", pRsp[0],pRsp[1],pRsp[2],pRsp[3],pRsp[4]);
	*/

	Req.sub_cmd_code = TEST_MODE_VERSION;
	Req.test_mode_req.version = VER_HW;

	/*
	 * build error
	 * warning: the frame size of 2176 bytes is larger than 1024 bytes; see http://go/big_stack_frame
	 * change the local variable to buffer allocation
	*/
	rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type);
	Rsp = (DIAG_TEST_MODE_F_rsp_type *)diagpkt_alloc(DIAG_TEST_MODE_F, rsp_len);
	if(Rsp == NULL)
	{
		printk("%s allocation failed !\n", __func__);
		return result;
	}

	send_to_arm9((void*)&Req, (void*)Rsp, sizeof(DIAG_TEST_MODE_F_rsp_type));
	/*
	 * previous kmsg : CheckHWRev> 0xfa 0x0 0x0 0x0 0x44
	 * current kmsg : CheckHWRev> 0xfa 0x0 0x0 0x44
	 * last packet matches to the previous, so this modification seems to be working fine
	*/
	printk("CheckHWRev> 0x%x 0x%x 0x%x 0x%x\n", Rsp->xx_header.opaque_header, \
		Rsp->sub_cmd_code, Rsp->ret_stat_code, Rsp->test_mode_rsp.str_buf[0]);
	
	result = Rsp->test_mode_rsp.str_buf[0];
	diagpkt_free(Rsp);

	return result;
/* END: 0014656 jihoon.lee@lge.com 2011024 */	
}
EXPORT_SYMBOL(CheckHWRev);

void CheckHWRevStr(char *buf, int str_size)
{
	DIAG_TEST_MODE_F_req_type Req;
	DIAG_TEST_MODE_F_rsp_type *Rsp = NULL;
	unsigned int rsp_len=0;

	/*
	 * build error
	 * warning: the frame size of 2176 bytes is larger than 1024 bytes; see http://go/big_stack_frame
	 * change the local variable to buffer allocation
	*/
	rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type);
	Rsp = (DIAG_TEST_MODE_F_rsp_type *)diagpkt_alloc(DIAG_TEST_MODE_F, rsp_len);
	if(Rsp == NULL)
	{
		printk("%s allocation failed !\n", __func__);
		return;
	}

	Req.sub_cmd_code = TEST_MODE_VERSION;
	Req.test_mode_req.version = VER_HW;

	send_to_arm9((void*)&Req, (void*)Rsp, sizeof(DIAG_TEST_MODE_F_rsp_type));
    memcpy(buf, Rsp->test_mode_rsp.str_buf, (str_size <= sizeof(Rsp->test_mode_rsp.str_buf) ? str_size: sizeof(Rsp->test_mode_rsp.str_buf)));
	diagpkt_free(Rsp);
}
//#endif

boolean lgf_factor_key_test_rsp (char key_code)
{
    /* sanity check */
    if (count_key_buf>=MAX_KEY_BUFF_SIZE)
        return FALSE;
		
//LGE_CHANGE [jinhwan.do][2012.05.15] Merge from VS930 MFT command's source [Start]
#ifdef CONFIG_LGE_DIAG_MFT
    mft_key_buf[count_mft_key_buf++] = key_code;
#endif
//LGE_CHANGE [jinhwan.do][2012.05.15] Merge from VS930 MFT command's source [End]

    key_buf[count_key_buf++] = key_code;
    return TRUE;
}
EXPORT_SYMBOL(lgf_factor_key_test_rsp);

void* LGF_TestModeFactoryReset(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp)
{
	unsigned char pbuf[50]; //no need to have huge size, this is only for the flag
  	const MmcPartition *pMisc_part; 
  	unsigned char startStatus = FACTORY_RESET_NA; 
  	int mtd_op_result = 0;
  	unsigned long factoryreset_bytes_pos_in_emmc = 0;

	/* MOD 0014656: [LG RAPI] OEM RAPI PACKET MISMATCH KERNEL CRASH FIX */
  	DIAG_TEST_MODE_F_req_type req_ptr;

  	req_ptr.sub_cmd_code = TEST_MODE_FACTORY_RESET_CHECK_TEST;
  	req_ptr.test_mode_req.factory_reset = pReq->factory_reset;
  
	/* handle operation or rpc failure as well */
  	pRsp->ret_stat_code = TEST_FAIL_S;
  
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
  
  	printk("LGF_TestModeFactoryReset> mmc info sec : 0x%x, size : 0x%x type : 0x%x frst sec: 0x%lx\n", 
													pMisc_part->dfirstsec, pMisc_part->dsize, pMisc_part->dtype, factoryreset_bytes_pos_in_emmc);

	/* MOD 0013861: [FACTORY RESET] emmc_direct_access factory reset flag access */
	/* add carriage return and change flag size for the platform access */
  	switch(pReq->factory_reset)
  	{
    	case FACTORY_RESET_CHECK :
			/* MOD 0014110: [FACTORY RESET] stability */
			/* handle operation or rpc failure as well */
      		memset((void*)pbuf, 0, sizeof(pbuf));
      		mtd_op_result = lge_read_block(factoryreset_bytes_pos_in_emmc, pbuf, FACTORY_RESET_STR_SIZE+2);

      		if( mtd_op_result != (FACTORY_RESET_STR_SIZE+2) )
      		{
        		printk(KERN_ERR "[Testmode]lge_read_block, read data  = %d \n", mtd_op_result);
        		pRsp->ret_stat_code = TEST_FAIL_S;
        		break;
      		}
      		else
      		{
        		//printk(KERN_INFO "\n[Testmode]factory reset memcmp\n");
        		if(memcmp(pbuf, FACTORY_RESET_STR, FACTORY_RESET_STR_SIZE) == 0) // tag read sucess
        		{
          			startStatus = pbuf[FACTORY_RESET_STR_SIZE] - '0';
          			printk(KERN_INFO "[Testmode]factory reset backup status = %d \n", startStatus);
        		}
        		else
        		{
          			// if the flag storage is erased this will be called, start from the initial state
          			printk(KERN_ERR "[Testmode] tag read failed :  %s \n", pbuf);
        		}
      		}  

	      	test_mode_factory_reset_status = FACTORY_RESET_INITIAL;
      		memset((void *)pbuf, 0, sizeof(pbuf));
      		sprintf(pbuf, "%s%d\n",FACTORY_RESET_STR, test_mode_factory_reset_status);
      		printk(KERN_INFO "[Testmode]factory reset status = %d\n", test_mode_factory_reset_status);

      		mtd_op_result = lge_erase_block(factoryreset_bytes_pos_in_emmc, FACTORY_RESET_STR_SIZE+2);	

			/* handle operation or rpc failure as well */
      		if(mtd_op_result!= (FACTORY_RESET_STR_SIZE+2))
      		{
        		printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
        		pRsp->ret_stat_code = TEST_FAIL_S;
        		break;
      		}
      		else
      		{	
        		mtd_op_result = lge_write_block(factoryreset_bytes_pos_in_emmc, pbuf, FACTORY_RESET_STR_SIZE+2);
        		if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
        		{
          			printk(KERN_ERR "[Testmode]lge_write_block, error num = %d \n", mtd_op_result);
          			pRsp->ret_stat_code = TEST_FAIL_S;
          			break;
        		}
      		}

#if 1 
			/* MOD 0014656: [LG RAPI] OEM RAPI PACKET MISMATCH KERNEL CRASH FIX */
      		//send_to_arm9((void*)(((byte*)pReq) -sizeof(diagpkt_header_type) - sizeof(word)) , pRsp);
      		send_to_arm9((void*)&req_ptr, (void*)pRsp, sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type));

			/* handle operation or rpc failure as well */
      		if(pRsp->ret_stat_code != TEST_OK_S)
      		{
        		printk(KERN_ERR "[Testmode]send_to_arm9 response : %d\n", pRsp->ret_stat_code);
        		pRsp->ret_stat_code = TEST_FAIL_S;
        		break;
      		}
#endif

      		/*LG_FW khlee 2010.03.04 -If we start at 5, we have to go to APP reset state(3) directly */
		if((startStatus == FACTORY_RESET_COLD_BOOT_END) || (startStatus == FACTORY_RESET_HOME_SCREEN_END))
        		test_mode_factory_reset_status = FACTORY_RESET_COLD_BOOT_START;
      		else
        		test_mode_factory_reset_status = FACTORY_RESET_ARM9_END;

      		memset((void *)pbuf, 0, sizeof(pbuf));
      		sprintf(pbuf, "%s%d\n",FACTORY_RESET_STR, test_mode_factory_reset_status);
      		printk(KERN_INFO "[Testmode]factory reset status = %d\n", test_mode_factory_reset_status);

      		mtd_op_result = lge_erase_block(factoryreset_bytes_pos_in_emmc, FACTORY_RESET_STR_SIZE+2);
			/* handle operation or rpc failure as well */
      		if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
      		{
        		printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
        		pRsp->ret_stat_code = TEST_FAIL_S;
        		break;
      		}
      		else
      		{
         		mtd_op_result = lge_write_block(factoryreset_bytes_pos_in_emmc, pbuf, FACTORY_RESET_STR_SIZE+2);
         		if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
         		{
          			printk(KERN_ERR "[Testmode]lge_write_block, error num = %d \n", mtd_op_result);
          			pRsp->ret_stat_code = TEST_FAIL_S;
          			break;
         		}
      		}

      		printk(KERN_INFO "%s, factory reset check completed \n", __func__);
      		pRsp->ret_stat_code = TEST_OK_S;
      		break;

		case FACTORY_RESET_COMPLETE_CHECK:
      		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
      		printk(KERN_ERR "[Testmode]not supported\n");
      		break;

    	case FACTORY_RESET_STATUS_CHECK:
      		memset((void*)pbuf, 0, sizeof(pbuf));
      		mtd_op_result = lge_read_block(factoryreset_bytes_pos_in_emmc, pbuf, FACTORY_RESET_STR_SIZE+2 );
			/* handle operation or rpc failure as well */
      		if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
      		{
      	 		printk(KERN_ERR "[Testmode]lge_read_block, error num = %d \n", mtd_op_result);
      	 		pRsp->ret_stat_code = TEST_FAIL_S;
      	 		break;
      		}
      		else
      		{
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-02, get Provisioned for test mode [Start]
      	 		if(memcmp(pbuf, FACTORY_RESET_STR, FACTORY_RESET_STR_SIZE) == 0) // tag read sucess
      	 		{
				test_mode_factory_reset_status = pbuf[FACTORY_RESET_STR_SIZE] - '0';
				if(test_mode_factory_reset_status == FACTORY_RESET_COLD_BOOT_END )
				{
					if (diagpdev != NULL){
						update_diagcmd_state(diagpdev, "TEST_GET_PROVISIONED", 0);
					}
//LGE_CHANGE [TestMode][jinhwan.do] 2012-05-23, add wait times for factory reset status 5->6 error is fixed [Start]
					msleep(300);
//LGE_CHANGE [TestMode][jinhwan.do] 2012-05-23, add wait times for factory reset status 5->6 error is fixed [End]
					if(memcmp(test_get_provisioned.ret, "1" , 1) == 0)
					{
						printk(KERN_INFO "LGF_TestModeFactoryReset     1   \n");
						test_mode_factory_reset_status = FACTORY_RESET_HOME_SCREEN_END;
						mtd_op_result = lge_erase_block(factoryreset_bytes_pos_in_emmc, FACTORY_RESET_STR_SIZE+2);
				      		if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
				      		{
				      	 		printk(KERN_ERR "[Testmode]lge_read_block, error num = %d \n", mtd_op_result);
				      	 		pRsp->ret_stat_code = TEST_FAIL_S;
				      	 		break;
				      		}
						else
						{
							memset((void *)pbuf, 0, sizeof(pbuf));
							sprintf(pbuf, "%s%d\n",FACTORY_RESET_STR, test_mode_factory_reset_status);
							mtd_op_result = lge_write_block(factoryreset_bytes_pos_in_emmc, pbuf, FACTORY_RESET_STR_SIZE+2);								
				         		if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
				         		{
				          			printk(KERN_ERR "[Testmode]lge_write_block, error num = %d \n", mtd_op_result);
				          			pRsp->ret_stat_code = TEST_FAIL_S;
				          			break;
				         		}
						}
					}
				}
				else
				{
					printk(KERN_INFO "[Testmode]factory reset status = %d \n", test_mode_factory_reset_status);
				}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-02, get Provisioned for test mode [End]
      	   			pRsp->ret_stat_code = test_mode_factory_reset_status;
      	 		}
      	 		else
      	 		{
      	   			printk(KERN_ERR "[Testmode]factory reset tag fail, set initial state\n");
      	   			test_mode_factory_reset_status = FACTORY_RESET_START;
      	   			pRsp->ret_stat_code = test_mode_factory_reset_status;
      	   			break;
      	 		}
      		}
      		break;

    	case FACTORY_RESET_COLD_BOOT:
			// remove requesting sync to CP as all sync will be guaranteed on their own.
      		test_mode_factory_reset_status = FACTORY_RESET_COLD_BOOT_START;
      		memset((void *)pbuf, 0, sizeof(pbuf));
      		sprintf(pbuf, "%s%d",FACTORY_RESET_STR, test_mode_factory_reset_status);
      		printk(KERN_INFO "[Testmode]factory reset status = %d\n", test_mode_factory_reset_status);
      		mtd_op_result = lge_erase_block(factoryreset_bytes_pos_in_emmc,  FACTORY_RESET_STR_SIZE+2);
			/* handle operation or rpc failure as well */
      		if(mtd_op_result!=( FACTORY_RESET_STR_SIZE+2))
      		{
        		printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
        		pRsp->ret_stat_code = TEST_FAIL_S;
        		break;
      		}
      		else
      		{
        		mtd_op_result = lge_write_block(factoryreset_bytes_pos_in_emmc, pbuf,  FACTORY_RESET_STR_SIZE+2);
        		if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
        		{
          			printk(KERN_ERR "[Testmode]lge_write_block, error num = %d \n", mtd_op_result);
          			pRsp->ret_stat_code = TEST_FAIL_S;
        		}
      		}
      		pRsp->ret_stat_code = TEST_OK_S;
      		break;

    	case FACTORY_RESET_ERASE_USERDATA:
      		test_mode_factory_reset_status = FACTORY_RESET_COLD_BOOT_START;
      		memset((void *)pbuf, 0, sizeof(pbuf));
      		sprintf(pbuf, "%s%d",FACTORY_RESET_STR, test_mode_factory_reset_status);
      		printk(KERN_INFO "[Testmode-erase userdata]factory reset status = %d\n", test_mode_factory_reset_status);
      		mtd_op_result = lge_erase_block(factoryreset_bytes_pos_in_emmc , FACTORY_RESET_STR_SIZE+2);
			/* handle operation or rpc failure as well */
      		if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
      		{
        		printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
        		pRsp->ret_stat_code = TEST_FAIL_S;
        		break;
      		}
      		else
      		{
        		mtd_op_result = lge_write_block(factoryreset_bytes_pos_in_emmc, pbuf, FACTORY_RESET_STR_SIZE+2);
        		if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
        		{
          			printk(KERN_ERR "[Testmode]lge_write_block, error num = %d \n", mtd_op_result);
          			pRsp->ret_stat_code = TEST_FAIL_S;
          			break;
        		}
      		}
    		pRsp->ret_stat_code = TEST_OK_S;
    		break;

     	default:
        	pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
        	break;
	}

  	return pRsp;
}

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-17, TestMode Porting of NFC [Start]
//20110930, addy.kim@lge.com,  [START]
static int lg_diag_nfc_result_file_read(int i_testcode ,int *ptrRenCode, char *sz_extra_buff )
{
	int read;
	int read_size;
	char buf[32] = {0,};

	
	mm_segment_t oldfs;
	

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	printk("[NFC] HELLO START FILE READ");
	read = sys_open((const char __user *)NFC_RESULT_PATH, O_RDONLY , 0);

	if(read < 0) {
		printk(KERN_ERR "%s, NFC Result File Open Fail\n",__func__);
		goto nfc_read_err;
		
	}else {
		printk(KERN_ERR "%s, NFC Result File Open Success\n",__func__);
		 
	}

	read_size = 0;
	printk("[_NFC_] copy read to buf variable From read\n");
	while( sys_read(read, &buf[read_size], 1) == 1){
		printk("[_NFC_] READ  buf[%d]:%c \n",read_size,buf[read_size]);
		read_size++;
	}	

	printk("[_NFC_] READ char %d\n",buf[0]-48);
	
	*ptrRenCode = buf[0]-48; //change ASCII Code to int Number
	
	printk("[_NFC_] lg_diag_nfc_result_file_read : i_result_status == %d\n",*ptrRenCode);

	if((strlen(buf) > 1) && (i_testcode == 5 || i_testcode == 7))
	{
		if(buf == NULL){
			printk(KERN_ERR "ADDY.KIM@lge.com : [_NFC_] BUFF is NULL\n");
			goto nfc_read_err;
			
		}
		printk("[_NFC_] lg_diag_nfc_result_file_read : Start Copy From szExtraData -> buf buf\n");
		strcpy( sz_extra_buff,(char*)(&buf[1]) );
	}	
		
	//sscanf(buf,"%d",&result);
	
	set_fs(oldfs);
	sys_close(read);

	return 1;

	nfc_read_err:
		set_fs(oldfs);
		sys_close(read);
		return 0;
		
}

void* LGF_TestModeNFC(
		test_mode_req_type*	pReq,
		DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
	int nfc_result = 0;
	
	char szExtraData[16] = {0,};
//	char szExtra
	printk(KERN_ERR "ADDY.KIM@lge.com : [_NFC_] [%s:%d] SubCmd=<%d>\n", __func__, __LINE__, pReq->nfc);

	if (diagpdev != NULL && (pReq->nfc != 2)){ 
		update_diagcmd_state(diagpdev, "NFC_TEST_MODE", pReq->nfc);
		
		if(pReq->nfc == 0)
			msleep(3000);
		else if(pReq->nfc == 1)
			msleep(6000);
		else if(pReq->nfc == 2)
		{
			//nothing do
		}
		else if(pReq->nfc == 3)
			msleep(4000);
		else if(pReq->nfc == 4)
			msleep(7000);
		else if(pReq->nfc == 5)
			msleep(5000);
		else if(pReq->nfc == 6)
			msleep(15000);
		else if(pReq->nfc == 7)
			msleep(5000);
		else
			msleep(7000);		
	
		if( (lg_diag_nfc_result_file_read(pReq->nfc,&nfc_result, szExtraData)) == 0 ){
			pRsp->ret_stat_code = TEST_FAIL_S;
			printk(KERN_ERR "addy.kim@lge.com , [NFC] FIle Open Error");
			goto nfc_test_err;
		}

		printk(KERN_ERR "ADDY.KIM@lge.com : [_NFC_] [%s:%d] Result Value=<%d>\n", __func__, __LINE__, nfc_result);

		if( nfc_result == 0 || nfc_result == 9 ){
			pRsp->ret_stat_code = TEST_OK_S;
			printk(KERN_ERR "ADDY.KIM@lge.com : [_NFC_] [%s:%d] DAIG RETURN VALUE=<%s>\n", __func__, __LINE__, "TEST_OK");
		}else if(nfc_result == 1){
			pRsp->ret_stat_code = TEST_FAIL_S;
			printk(KERN_ERR "ADDY.KIM@lge.com : [_NFC_] [%s:%d] DAIG RETURN VALUE=<%s>\n", __func__, __LINE__, "TEST_FAIL");
			goto nfc_test_err;
		}else{
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
			printk(KERN_ERR "ADDY.KIM@lge.com : [_NFC_] [%s:%d] DAIG RETURN VALUE=<%s>\n", __func__, __LINE__, "NOT_SUPPORT");
			goto nfc_test_err;
		}
			
		if( pReq->nfc == 5 || pReq->nfc == 7 )
		{
			if(szExtraData == NULL ){
				printk(KERN_ERR "[_NFC_] [%s:%d] response Data is NULL \n", __func__, __LINE__);
				pRsp->ret_stat_code = TEST_FAIL_S;
				goto nfc_test_err;
			}

			printk("[_NFC_] Start Copy From szExtraData : [%s] -> test_mode_rsp buf\n",szExtraData);


			//save data to response to Diag
			//strcat(szExtraData,"0");
			strcpy(pRsp->test_mode_rsp.str_buf,(byte*)szExtraData);
	
			printk(KERN_INFO "%s\n", pRsp->test_mode_rsp.str_buf);
			pRsp->ret_stat_code = TEST_OK_S;
		}
	}
	else
	{
		printk(KERN_ERR "[_NFC_] [%s:%d] SubCmd=<%d> ERROR\n", __func__, __LINE__, pReq->nfc);
		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}
	return pRsp;

	nfc_test_err:
		return pRsp;	
}
//20110930, addy.kim@lge.com,  [END]
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-17, TestMode Porting of NFC [END]
	
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-03-14, SWVersion [Start]
byte CheckTestVer(char *buf, test_mode_req_version_type version)
{
	DIAG_TEST_MODE_F_req_type Req;
	DIAG_TEST_MODE_F_rsp_type *Rsp = NULL;
	unsigned int rsp_len=0;
	byte result = 0;

	Req.sub_cmd_code = TEST_MODE_VERSION;
	Req.test_mode_req.version = version;

	/*
	 * build error
	 * warning: the frame size of 2176 bytes is larger than 1024 bytes; see http://go/big_stack_frame
	 * change the local variable to buffer allocation
	*/
	rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type);
	Rsp = (DIAG_TEST_MODE_F_rsp_type *)diagpkt_alloc(DIAG_TEST_MODE_F, rsp_len);
	if(Rsp == NULL)
	{
		printk("%s allocation failed !\n", __func__);
		return result;
	}
	
	send_to_arm9((void*)&Req, (void*)Rsp, sizeof(DIAG_TEST_MODE_F_rsp_type));

	memcpy(buf, Rsp->test_mode_rsp.str_buf, sizeof(Rsp->test_mode_rsp.str_buf));	   
	printk("CheckTestVer> 0x%x 0x%x 0x%x 0x%x\n", Rsp->xx_header.opaque_header, Rsp->sub_cmd_code, Rsp->ret_stat_code, Rsp->test_mode_rsp.str_buf[0]);
	result = Rsp->test_mode_rsp.str_buf[0];
	diagpkt_free(Rsp);
	
	return result;
}
void* LGF_TestModeVer(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type *pRsp)
{	
	char ret_buf[32] = {0};
	pRsp->ret_stat_code = TEST_OK_S;
	
	memset(sp_ver.ret, 0, 32);
	if (diagpdev != NULL){
		update_diagcmd_state(diagpdev, "TEST_SP_VER", pReq->version);
	}
	switch(pReq->version)
	{
		case VER_SP_VER:	
			msleep(100);
			strncpy(pRsp->test_mode_rsp.str_buf,(byte*)sp_ver.ret, sizeof(pRsp->test_mode_rsp.str_buf));		
			printk(KERN_INFO "LGF_TestModeVer      sp_ver.ret :   %s\n", sp_ver.ret);
			printk(KERN_INFO "LGF_TestModeVer      test_mode_rsp.str_buf :   %s\n", pRsp->test_mode_rsp.str_buf);			
			break;
			
	        case VER_SW :
		case VER_MODEL :
		case VER_HW :	
		case VER_PRL :
		case VER_MMS :
		case VER_LCD_REVISION:			
			CheckTestVer(ret_buf, pReq->version);
			printk(KERN_INFO "LGF_TestModeVer     buffer   : %s \n", ret_buf);
			strcpy(pRsp->test_mode_rsp.str_buf,(byte*)ret_buf);
			break;
   
		case VER_ERI :
		case VER_BREW :
		case VER_DSP : 
		case REV_DSP :
		case VER_CONTENTS :
		case CAMERA_REV:
		case CONTENTS_SIZE:
		case JAVA_FILE_CNT:
		case JAVA_FILE_SIZE:
		case VER_JAVA:
		case BANK_ON_CNT:
		case BANK_ON_SIZE:
		case MODULE_FILE_CNT:
		case MODULE_FILE_SIZE:
		case MP3_DSP_OS_VER:
		case TOUCH_MODULE_REV:
		case TCC_OS_VER_MODULE:
		case VER_GSM_SW:
		default :
			printk(KERN_INFO "LGF_TestModeVer     Not Supported \n");
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
			break;
	}
	return pRsp;
}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-03-14, SWVersion [End]

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, TestMode Motor [Start]

//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [Start]
static int lg_diag_motor_status_read(void)
{
	int read;
	int read_size;
	char buf[32] = {0,};
	
	mm_segment_t oldfs;	

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	printk("[motor] HELLO START FILE READ");
	read = sys_open((const char __user *)MOTOR_STATUS_PATH, O_RDONLY , 0);

	if(read < 0) {
		printk(KERN_ERR "%s, motor Result File Open Fail\n",__func__);
		goto motor_read_err;
		
	}else {
		printk(KERN_ERR "%s, motor Result File Open Success\n",__func__);
		 
	}

	read_size = 0;
	printk("[motor] copy read to buf variable From read\n");
	while( sys_read(read, &buf[read_size], 1) == 1){
		printk("[motor] READ  buf[%d]:%c \n",read_size,buf[read_size]);
		read_size++;
	}	

	printk("[motor] READ char %d\n",buf[0]-48);
	
	
	set_fs(oldfs);
	sys_close(read);
	
	if(memcmp(buf, "0" , 1) == 0)
		return 0;
	else
		return 1;

	motor_read_err:
		set_fs(oldfs);
		sys_close(read);
		return 2;
		
}

void* LGF_TestMotor(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
	pRsp->ret_stat_code = TEST_OK_S;
	

	switch(pReq->motor)
	{
		case MOTOR_ON :
		case MOTOR_OFF :
			if (diagpdev != NULL)
			{
				update_diagcmd_state(diagpdev, "MOTOR", pReq->motor);
			}
			else
			{
				printk("\n[%s] error MOTOR", __func__ );
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
			}
			break;
		case MOTOR_STATUS :
		#ifdef CONFIG_TSPDRV
			if(get_lge_hw_revision() >= HW_REV_D)
				pRsp->test_mode_rsp.motor_status =  Immersion_vib_isON();
			else
		#endif
			pRsp->test_mode_rsp.motor_status =  lg_diag_motor_status_read();
			break;
	}
	printk(KERN_ERR "[LGF_TestMotor] pReq->motor=%d     motor_status =%d  \n",pReq->motor, pRsp->test_mode_rsp.motor_status);
  	return pRsp;
}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, TestMode Motor [END]
//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [End]

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-16, TestMode 9.0, get Acoustic Status [Start]
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-17, Test Mode Porting of Acoustic [Start]
void* LGF_TestAcoustic(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp)
{
    pRsp->ret_stat_code = TEST_OK_S;

	if (diagpdev != NULL){
		if((pReq->acoustic > ACOUSTIC_LOOPBACK_OFF) && (pReq->acoustic != ACOUSTIC_LOOPBACK_STATUS))
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;

#if 1
//LGE_CHANGE_S [Testmode][ jy0127.jang@lge.com] 2012-06-06,Testmode 9.0 Acoustic dealy[START]
		if((loopback_state == ACOUSTIC_LOOPBACK_ON)&&((pReq->acoustic==ACOUSTIC_ON)||(pReq->acoustic==ACOUSTIC_LOOPBACK_ON)))
		{
			pRsp->ret_stat_code = TEST_FAIL_S;
		}
		else if((loopback_state == ACOUSTIC_ON)&&(pReq->acoustic==ACOUSTIC_LOOPBACK_ON))
		{
			pRsp->ret_stat_code = TEST_FAIL_S;
		}
		else
//LGE_CHANGE_E[Testmode][ jy0127.jang@lge.com] 2012-06-06,Testmode 9.0 Acoustic dealy[END] 
#endif
		{
			loopback_state = pReq->acoustic;
			update_diagcmd_state(diagpdev, "ACOUSTIC", pReq->acoustic);
		}
	}
	else
	{
		printk("\n[%s] error ACOUSTIC", __func__ );
		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}
	printk(KERN_INFO "LGF_TestAcoustic      acoustic_status :   %d\n", acoustic_status);
  	if (pReq->acoustic == ACOUSTIC_LOOPBACK_STATUS)
	{
		msleep(200);
		pRsp->test_mode_rsp.acoustic_status = acoustic_status;
		printk(KERN_INFO "LGF_TestAcoustic      test_mode_rsp.acoustic_status :   %d\n", pRsp->test_mode_rsp.acoustic_status);	
	}
	return pRsp;
}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-17, Test Mode Porting of Acoustic [END]
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-16, TestMode 9.0, get Acoustic Status [End]

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-17, Test Mode Porting of Camera [Start]
void* LGF_TestCam(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp)
{
	pRsp->ret_stat_code = TEST_OK_S;

	switch(pReq->camera)
	{
// Start LGE_BSP_CAMERA::tao.jin@lge.com 2011-11-08  enable CAM_TEST command for Testmode v 8.7
#if 0
		case CAM_TEST_SAVE_IMAGE:
		case CAM_TEST_FLASH_ON:
		case CAM_TEST_FLASH_OFF:
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
			break;
#endif
// End LGE_BSP_CAMERA::tao.jin@lge.com 2011-11-08  enable CAM_TEST command for Testmode v 8.7
		default:
			if (diagpdev != NULL)
			{
				update_diagcmd_state(diagpdev, "CAMERA", pReq->camera);
			}
			else
			{
				printk("\n[%s] error CAMERA", __func__ );
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
			}
			break;
	}
	
	return pRsp;
}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-17, Test Mode Porting of Camera [END]

//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [Start]
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of KeyTest [Start]
extern int diag_log_status;
void* LGT_TestModeKeyTest(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type *pRsp)
{
  	pRsp->ret_stat_code = TEST_OK_S;

  	switch(pReq->key_test_start)
	{
		case KEY_DATA_END:
			memcpy((void *)((DIAG_TEST_MODE_KEY_F_rsp_type *)pRsp)->key_pressed_buf, (void *)key_buf, MAX_KEY_BUFF_SIZE);
			memset((void *)key_buf,0x00,MAX_KEY_BUFF_SIZE);
			diag_event_log_end();
			break;

		case KEY_DATA_START:
			memset((void *)key_buf,0x00,MAX_KEY_BUFF_SIZE);
			count_key_buf=0;
			diag_event_log_start();
			break;
			
		case KEY_DATA_STATUS:	
			pRsp->test_mode_rsp.key_data_status = diag_log_status;
			break;
			
		default:	
		  	pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
			break;
  	}  
	printk(KERN_ERR "[LGT_TestModeKeyTest] key_test_start =%d     diag_log_status = %d      key_buf = %s \n",pReq->key_test_start, diag_log_status, key_buf);
	return pRsp;
}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of KeyTest [END]
//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [End]

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, SD Card Patch from LS696 for TestMode [Start]
char external_memory_copy_test(void)
{
	char return_value = 1;
	char *src = (void *)0;
	char *dest = (void *)0;
	off_t fd_offset;
	int fd;
	mm_segment_t old_fs=get_fs();
    set_fs(get_ds());

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, SD Card Path is changed
// BEGIN : munho.lee@lge.com 2010-12-30
// MOD: 0013315: [SD-card] SD-card drectory path is changed in the testmode
	if ( (fd = sys_open((const char __user *) "/sdcard/SDTest.txt", O_CREAT | O_RDWR, 0) ) < 0 )
/*
	if ( (fd = sys_open((const char __user *) "/sdcard/_ExternalSD/SDTest.txt", O_CREAT | O_RDWR, 0) ) < 0 )
*/	
// END : munho.lee@lge.com 2010-12-30
	{
		printk(KERN_ERR "[ATCMD_EMT] Can not access SD card\n");
		goto file_fail;
	}

	if ( (src = kmalloc(10, GFP_KERNEL)) )
	{
		sprintf(src,"TEST");
		if ((sys_write(fd, (const char __user *) src, 5)) < 0)
		{
			printk(KERN_ERR "[ATCMD_EMT] Can not write SD card \n");
			goto file_fail;
		}
		fd_offset = sys_lseek(fd, 0, 0);
	}
	if ( (dest = kmalloc(10, GFP_KERNEL)) )
	{
		if ((sys_read(fd, (char __user *) dest, 5)) < 0)
		{
			printk(KERN_ERR "[ATCMD_EMT]Can not read SD card \n");
			goto file_fail;
		}
		if ((memcmp(src, dest, 4)) == 0)
			return_value = 0;
		else
			return_value = 1;
	}

	kfree(src);
	kfree(dest);
file_fail:
	sys_close(fd);
    set_fs(old_fs);
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, SD Card Path is changed
// BEGIN : munho.lee@lge.com 2010-12-30
// MOD: 0013315: [SD-card] SD-card drectory path is changed in the testmode
	sys_unlink((const char __user *)"/sdcard/SDTest.txt");
/*
	sys_unlink((const char __user *)"/sdcard/_ExternalSD/SDTest.txt");
*/
// END : munho.lee@lge.com 2010-12-30
	return return_value;
}

void* LGF_ExternalSocketMemory(	test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
	struct statfs_local  sf;
    pRsp->ret_stat_code = TEST_OK_S;

    switch( pReq->esm){
	case EXTERNAL_SOCKET_MEMORY_CHECK:
// BEGIN : munho.lee@lge.com 2011-01-15
// ADD: 0013541: 0014142: [Test_Mode] To remove Internal memory information in External memory test when SD-card is not exist 	
		if(gpio_get_value(SYS_GPIO_SD_DET))
		{
			pRsp->test_mode_rsp.memory_check = TEST_FAIL_S;
			break;
		}		
// END : munho.lee@lge.com 2011-01-15		
        pRsp->test_mode_rsp.memory_check = external_memory_copy_test();
        break;

	case EXTERNAL_FLASH_MEMORY_SIZE:
// BEGIN : munho.lee@lge.com 2010-12-30
// MOD: 0013315: [SD-card] SD-card drectory path is changed in the testmode
// BEGIN : munho.lee@lge.com 2011-01-15
// ADD: 0013541: 0014142: [Test_Mode] To remove Internal memory information in External memory test when SD-card is not exist 
		if(gpio_get_value(SYS_GPIO_SD_DET))
		{
			pRsp->test_mode_rsp.socket_memory_size = 0;
			break;
		}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, SD Card Path is changed
// END : munho.lee@lge.com 2011-01-15		
	        if (sys_statfs("/sdcard", (struct statfs *)&sf) != 0)
/*
   		if (sys_statfs("/sdcard/_ExternalSD", (struct statfs *)&sf) != 0)		
*/       
// END : munho.lee@lge.com 2010-12-30
        {
			printk(KERN_ERR "[Testmode]can not get sdcard infomation \n");
			pRsp->ret_stat_code = TEST_FAIL_S;
			break;
        }

// BEGIN : munho.lee@lge.com 2011-01-15
// MOD: 0013541: 0014142: [Test_Mode] To remove Internal memory information in External memory test when SD-card is not exist 
//		pRsp->test_mode_rsp.socket_memory_size = ((long long)sf.f_blocks * (long long)sf.f_bsize);  // needs byte
		pRsp->test_mode_rsp.socket_memory_size = ((long long)sf.f_blocks * (long long)sf.f_bsize) >> 20; // needs Mb.
// END : munho.lee@lge.com 2011-01-15
        break;

	case EXTERNAL_SOCKET_ERASE:

        if (diagpdev != NULL){
// BEGIN : munho.lee@lge.com 2010-11-27
// MOD : 0011477: [SD-card] Diag test mode			
// BEGIN : munho.lee@lge.com 2011-01-15
// ADD: 0013541: 0014142: [Test_Mode] To remove Internal memory information in External memory test when SD-card is not exist 	

//LGE_CHANGE_S [U0 CDMA] hongsic.kim 2012-05-07 Without external memory internal memory can be formatted via diag command. [Start]
			if(gpio_get_value(SYS_GPIO_SD_DET))
			{
				pRsp->ret_stat_code = TEST_FAIL_S;
				break;
			}
//LGE_CHANGE_S [U0 CDMA] hongsic.kim 2012-05-07 Without external memory internal memory can be formatted via diag command. [End]

// END : munho.lee@lge.com 2011-01-15
			update_diagcmd_state(diagpdev, "MMCFORMAT", 1);
//			update_diagcmd_state(diagpdev, "FACTORY_RESET", 3);
// END : munho.lee@lge.com 2010-11-27			
			msleep(5000);
			pRsp->ret_stat_code = TEST_OK_S;
        }
        else 
        {
			printk("\n[%s] error FACTORY_RESET", __func__ );
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
        }
        break;

	case EXTERNAL_FLASH_MEMORY_USED_SIZE:
// BEGIN : munho.lee@lge.com 2010-12-30
// MOD: 0013315: [SD-card] SD-card drectory path is changed in the testmode
// BEGIN : munho.lee@lge.com 2011-01-15
// ADD: 0013541: 0014142: [Test_Mode] To remove Internal memory information in External memory test when SD-card is not exist	
		
		if(gpio_get_value(SYS_GPIO_SD_DET))
		{
			pRsp->test_mode_rsp.socket_memory_usedsize = 0;
			break;
		}		

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, SD Card Path is changed
// END : munho.lee@lge.com 2011-01-15
		if (sys_statfs("/sdcard", (struct statfs *)&sf) != 0)
/*
		if (sys_statfs("/sdcard/_ExternalSD", (struct statfs *)&sf) != 0)		
*/			
// END : munho.lee@lge.com 2010-12-30
		{
			printk(KERN_ERR "[Testmode]can not get sdcard information \n");
			pRsp->ret_stat_code = TEST_FAIL_S;
			break;
		}
// BEGIN : munho.lee@lge.com 2011-01-15
// MOD: 0013541: 0014142: [Test_Mode] To remove Internal memory information in External memory test when SD-card is not exist	
		pRsp->test_mode_rsp.socket_memory_usedsize = ((long long)(sf.f_blocks - (long long)sf.f_bfree) * sf.f_bsize); // needs byte		

// END : munho.lee@lge.com 2011-01-15
//		pRsp->test_mode_rsp.socket_memory_usedsize = ((long long)(sf.f_blocks - (long long)sf.f_bfree) * sf.f_bsize) >> 20;  // needs mega byte
		
		break;

	case EXTERNAL_SOCKET_ERASE_SDCARD_ONLY: /*0xE*/
		if (diagpdev != NULL){
			update_diagcmd_state(diagpdev, "MMCFORMAT", EXTERNAL_SOCKET_ERASE_SDCARD_ONLY);		
			msleep(5000);
			pRsp->ret_stat_code = TEST_OK_S;
		}
		else 
		{
			printk("\n[%s] error EXTERNAL_SOCKET_ERASE_SDCARD_ONLY", __func__ );
			pRsp->ret_stat_code = TEST_FAIL_S;
		}
		break;
	case EXTERNAL_SOCKET_ERASE_FAT_ONLY: /*0xF*/		//not used internal_FAT for US730

		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
		/*if (diagpdev != NULL)
		{
			update_diagcmd_state(diagpdev, "MMCFORMAT", EXTERNAL_SOCKET_ERASE_FAT_ONLY);		
			msleep(5000);
			pRsp->ret_stat_code = TEST_OK_S;
		}
		else 
		{
			printk("\n[%s] error EXTERNAL_SOCKET_ERASE_FAT_ONLY", __func__ );
			pRsp->ret_stat_code = TEST_FAIL_S;
		}*/
		break;

	default:
        pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
        break;
	}

    return pRsp;
}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, SD Card Patch from LS696 for TestMode [END]

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-17, Test Mode Porting of BT [Start]
#ifndef LG_BTUI_TEST_MODE
void* LGF_TestModeBlueTooth(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp)
{
	// 2011.04.30 sunhee.kang@lge.com BT TestMode merge from Gelato [START]
	if (diagpdev != NULL){
		update_diagcmd_state(diagpdev, "BT_TEST_MODE", pReq->bt);
		//if(pReq->bt==1) msleep(4900); //6sec timeout
		//else if(pReq->bt==2) msleep(4900);
		//else msleep(4900);
		printk(KERN_ERR "[_BTUI_] [%s:%d] BTSubCmd=<%d>\n", __func__, __LINE__, pReq->bt);
		msleep(6000);
		printk(KERN_ERR "[_BTUI_] [%s:%d] BTSubCmd=<%d>\n", __func__, __LINE__, pReq->bt);
	
		pRsp->ret_stat_code = TEST_OK_S;
	}
	else
	{
		printk(KERN_ERR "[_BTUI_] [%s:%d] BTSubCmd=<%d> ERROR\n", __func__, __LINE__, pReq->bt);
		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}
	return pRsp;
	// 2011.04.30 sunhee.kang@lge.com BT TestMode merge from Gelato [END]
}
#endif
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-17, Test Mode Porting of BT [END]

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-17, Test Mode Porting of MP3  [Start]
void* LGF_TestModeMP3 (test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp)
{
	pRsp->ret_stat_code = TEST_OK_S;
//	printk("\n[%s] diagpdev 0x%x", __func__, diagpdev );

	if (diagpdev != NULL){
		if(pReq->mp3_play == MP3_SAMPLE_FILE)
		{
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
		}
		else
		{
			update_diagcmd_state(diagpdev, "MP3", pReq->mp3_play);
		}
	}
	else
	{
		printk("\n[%s] error MP3", __func__ );
		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}
  
	return pRsp;
}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-17, Test Mode Porting of MP3  [END]

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of MemoryFormat [Start]
void* LGF_MemoryFormatTest(	test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
  	struct statfs_local  sf;
//  	unsigned int remained = 0;
  	
	pRsp->ret_stat_code = TEST_OK_S;
  	
	if (sys_statfs("/data", (struct statfs *)&sf) != 0)
  	{
    	printk(KERN_ERR "[Testmode]can not get sdcard infomation \n");
    	pRsp->ret_stat_code = TEST_FAIL_S;
  	}
  	else
  	{	
		switch(pReq->memory_format)
		{
			case MEMORY_TOTAL_SIZE_TEST:
			break;	

			case MEMORY_FORMAT_MEMORY_TEST:
			/*
			For code of format memory
			*/		  
			pRsp->ret_stat_code = TEST_OK_S;
			break;

			default :
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
			break;
		}
  	}

  	return pRsp;
}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of MemoryFormat [END]

void* LGF_TestModeKeyData(	test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
	pRsp->ret_stat_code = TEST_OK_S;

	LGF_SendKey(LGF_KeycodeTrans(pReq->key_data));

	return pRsp;
}

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of MemoryVolume [Start]
void* LGF_MemoryVolumeCheck(	test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
  	struct statfs_local  sf;
 	unsigned int total = 0;
  	unsigned int used = 0;
  	unsigned int remained = 0;
  	
	pRsp->ret_stat_code = TEST_OK_S;

  	if (sys_statfs("/data", (struct statfs *)&sf) != 0)
  	{
	    	printk(KERN_ERR "[Testmode]can not get sdcard infomation \n");
	    	pRsp->ret_stat_code = TEST_FAIL_S;
  	}
  	else
  	{
		total = (sf.f_blocks * sf.f_bsize) >> 20;
		remained = (sf.f_bavail * sf.f_bsize) >> 20;
		used = total - remained;

		switch(pReq->mem_capa)
		{
			case MEMORY_TOTAL_CAPA_TEST:
				pRsp->test_mode_rsp.mem_capa = total;
				break;

			case MEMORY_USED_CAPA_TEST:
				pRsp->test_mode_rsp.mem_capa = used;
				break;

			case MEMORY_REMAIN_CAPA_TEST:
				pRsp->test_mode_rsp.mem_capa = remained;
				break;

			default :
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
				break;
		}
  	}
  	return pRsp;
}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of MemoryVolume [END]

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of SpeakerPhone [Start]
void* LGF_TestModeSpeakerPhone(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp)
{
	pRsp->ret_stat_code = TEST_OK_S;

	if (diagpdev != NULL)
	{
		if((pReq->speaker_phone == NOMAL_Mic1) || (pReq->speaker_phone == NC_MODE_ON)
			|| (pReq->speaker_phone == ONLY_MIC2_ON_NC_ON) || (pReq->speaker_phone == ONLY_MIC1_ON_NC_ON))
		{
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
		}
		else
		{
			update_diagcmd_state(diagpdev, "SPEAKERPHONE", pReq->speaker_phone);
		}
	}
	else
	{
		printk("\n[%s] error SPEAKERPHONE", __func__ );
		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}
  	
	return pRsp;
}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of SpeakerPhone [End]


//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-16, Test Mode Porting of PowerSaveMode [Start]
void* LGF_PowerSaveMode(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp)
{ 
	pRsp->ret_stat_code = TEST_OK_S;
	pReq->sleep_mode = (pReq->sleep_mode & 0x00FF);     // 2011.06.21 biglake for power test after cal

	switch(pReq->sleep_mode){
		case SLEEP_MODE_ON:
//LGE_CHANGE_S [jinhwan.do][2012.03.07]TestMode patch source merge from LS696 Model [Start]
			if(lm3530_get_state() ==1)
//LGE_CHANGE_S [jinhwan.do][2012.03.07]TestMode patch source merge from LS696 Model [End]
			LGF_SendKey(KEY_END);
			break;

		case AIR_PLAIN_MODE_ON:
//LGE_CHANGE_S [jinhwan.do][2012.03.07]TestMode patch source merge from LS696 Model [Start]
			if (diagpdev != NULL){
				update_diagcmd_state(diagpdev, "LCD_KEEP_OFF", 1);
			}
			if(lm3530_get_state() ==1)
				LGF_SendKey(KEY_END);
//LGE_CHANGE_S [jinhwan.do][2012.03.07]TestMode patch source merge from LS696 Model [End]
	      		remote_set_ftm_boot(0);
	      		if_condition_is_on_air_plain_mode = 1;
	      		remote_set_operation_mode(0);
			break;

		case AIR_PLAIN_MODE_OFF:
	  		remote_set_ftm_boot(0);
	  		if_condition_is_on_air_plain_mode = 0;
	  		remote_set_operation_mode(1);
			break;
			
//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [Start]
		case SLEEP_MODE_STATUS:
			if(lm3530_get_state() == 2)
				pRsp->test_mode_rsp.sleep_status = 1;
			else
				pRsp->test_mode_rsp.sleep_status = 0;
			break;

		case AIR_PLAIN_MODE_STATUS:
			pRsp->test_mode_rsp.airplane_status = if_condition_is_on_air_plain_mode;
			break;
//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [End]

		default:
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
			break;
	}

	return pRsp; 
}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-16, Test Mode Porting of PowerSaveMode [END]

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of TestScript [Start]
void* LGF_TestScriptItemSet(test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type* pRsp)
{
// BEGIN: 0009720 sehyuny.kim@lge.com 2010-10-06
// MOD 0009720: [Modem] It add RF X-Backup feature
  	int mtd_op_result = 0;

  	const MmcPartition *pMisc_part; 
  	unsigned long factoryreset_bytes_pos_in_emmc = 0;
  	DIAG_TEST_MODE_F_req_type req_ptr;
	unsigned int rsp_len=0;

  	req_ptr.sub_cmd_code = TEST_MODE_TEST_SCRIPT_MODE;
  	req_ptr.test_mode_req.test_mode_test_scr_mode = pReq->test_mode_test_scr_mode;

	rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type) + sizeof(test_mode_req_test_script_mode_type);

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

  	printk("LGF_TestScriptItemSet> mmc info sec : 0x%x, size : 0x%x type : 0x%x frst sec: 0x%lx\n", pMisc_part->dfirstsec, pMisc_part->dsize, pMisc_part->dtype, factoryreset_bytes_pos_in_emmc);

  	switch(pReq->test_mode_test_scr_mode)
  	{
    	case TEST_SCRIPT_ITEM_SET:
#if 1 // def CONFIG_LGE_MTD_DIRECT_ACCESS
      		mtd_op_result = lge_erase_block(factoryreset_bytes_pos_in_emmc, (FACTORY_RESET_STR_SIZE+2) );

      		if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))  // LS696_me : Clear factory reset flag 
      		{
      	 		printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
      	 		pRsp->ret_stat_code = TEST_FAIL_S;
      	 		break;
      		} else
#endif /*CONFIG_LGE_MTD_DIRECT_ACCESS*/
      // LG_FW khlee 2010.03.16 - They want to ACL on state in test script state.
      		{
//LGE_CHANGE_S [jinhwan.do][2012.03.07]TestMode patch source merge from LS696 Model [Start]
//				if(diagpdev != NULL)		
//	      	 		update_diagcmd_state(diagpdev, "ALC", 1);
//LGE_CHANGE_S [jinhwan.do][2012.03.07]TestMode patch source merge from LS696 Model [End]

      	 		//send_to_arm9((void*)(((byte*)pReq) -sizeof(diagpkt_header_type) - sizeof(word)) , pRsp);
      	 		//send_to_arm9((void*)&req_ptr, (void*)pRsp, sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type) + sizeof(test_mode_req_test_script_mode_type));
      	 		send_to_arm9((void*)&req_ptr, (void*)pRsp, rsp_len);
        		printk(KERN_INFO "%s, result : %s\n", __func__, pRsp->ret_stat_code==TEST_OK_S?"OK":"FALSE");
      		}
      		break;
#if 0		
  		case CAL_DATA_BACKUP:
  		case CAL_DATA_RESTORE:
  		case CAL_DATA_ERASE:
  		case CAL_DATA_INFO:
  			diagpkt_free(pRsp);
  			return 0;			
  			break;
#endif		
    	default:
      		//send_to_arm9((void*)(((byte*)pReq) -sizeof(diagpkt_header_type) - sizeof(word)) , pRsp);
      		//send_to_arm9((void*)&req_ptr, (void*)pRsp, sizeof(DIAG_TEST_MODE_F_rsp_type));
      		send_to_arm9((void*)&req_ptr, (void*)pRsp, rsp_len);
      		printk(KERN_INFO "%s, cmd : %d, result : %s\n", __func__, pReq->test_mode_test_scr_mode, \
	  										pRsp->ret_stat_code==TEST_OK_S?"OK":"FALSE");
      		if(pReq->test_mode_test_scr_mode == TEST_SCRIPT_MODE_CHECK)
      		{
        		switch(pRsp->test_mode_rsp.test_mode_test_scr_mode)
        		{
          			case 0:
            			printk(KERN_INFO "%s, mode : %s\n", __func__, "USER SCRIPT");
            			break;
          			case 1:
            			printk(KERN_INFO "%s, mode : %s\n", __func__, "TEST SCRIPT");
            			break;
          			default:
            			printk(KERN_INFO "%s, mode : %s, returned %d\n", __func__, "NO PRL", pRsp->test_mode_rsp.test_mode_test_scr_mode);
            			break;
        		}
      		}
      		break;
  	}  
      
// END: 0009720 sehyuny.kim@lge.com 2010-10-06
  	return pRsp;
}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of TestScript [END]


//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of VolmeLevel [Start]
void* LGT_TestModeVolumeLevel(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp)
{
	pRsp->ret_stat_code = TEST_OK_S;

	if (diagpdev != NULL){
		update_diagcmd_state(diagpdev, "VOLUMELEVEL", pReq->volume_level);
		msleep(1000);	//hongsic.kim@lge.com, Fixed CMD OK response after process
	}
	else
	{
		printk("\n[%s] error VOLUMELEVEL", __func__ );
		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}

  	return pRsp;
}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of VolmeLevel [END]

static int first_booting_chg_mode_status = -1;
void set_first_booting_chg_mode_status(int status)
{
	first_booting_chg_mode_status = status;
	printk("%s, status : %d\n", __func__, first_booting_chg_mode_status);
}

int get_first_booting_chg_mode_status(void)
{
	printk("%s, status : %d\n", __func__, first_booting_chg_mode_status);
	return first_booting_chg_mode_status;
}

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of FirstBoot [Start]
void* LGF_TestModeFboot(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp)
{
	switch( pReq->fboot)
	{
		case FIRST_BOOTING_COMPLETE_CHECK:
			printk("[Testmode] First Boot info ?? ====> %d \n", boot_complete_info);
			if (boot_complete_info)
				pRsp->ret_stat_code = TEST_OK_S;
			else
				pRsp->ret_stat_code = TEST_FAIL_S;
			break;
/* BEGIN: 0015566 jihoon.lee@lge.com 20110207 */
/* ADD 0015566: [Kernel] charging mode check command */
/*
 * chg_status 0 : in the charging mode
 * chg_status 1 : normal boot mode
 */
#ifdef CONFIG_LGE_CHARGING_MODE_INFO
		case FIRST_BOOTING_CHG_MODE_CHECK:
			if(get_first_booting_chg_mode_status() == 1)
				pRsp->ret_stat_code = FIRST_BOOTING_IN_CHG_MODE;
			else
				pRsp->ret_stat_code = FIRST_BOOTING_NOT_IN_CHG_MODE;
			break;
#endif
/* END: 0015566 jihoon.lee@lge.com 20110207 */
	    default:
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
			break;
	}

    return pRsp;
}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of FirstBoot [END]

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of PowerReset [Start]
void* LGF_TestModePowerReset(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp)
{
	
	if(diagpdev !=NULL)
	{
//LGE_CHANGE_S [jinhwan.do][2012.03.07]TestMode patch source merge from LS696 Model [Start]
		switch(pReq->power_reset)
		{
			case POWER_RESET:
				update_diagcmd_state(diagpdev, "REBOOT", 0);
				pRsp->ret_stat_code = TEST_OK_S;
				break;
			case POWER_OFF:
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
				break;
		}
//LGE_CHANGE_S [jinhwan.do][2012.03.07]TestMode patch source merge from LS696 Model [End]
	}else
	{
		pRsp->ret_stat_code = TEST_FAIL_S;
	}
	
	return pRsp;
}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of PowerReset [END]

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-16, Test Mode Porting of DB Check [Start]
void* LGF_TestModeDBIntegrityCheck(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp)
{
	printk(KERN_ERR "[_DBCHECK_] [%s:%d] DBCHECKSubCmd=<%d>\n", __func__, __LINE__, pReq->bt);

	memset(integrity_ret.ret, 0, 32);
	pRsp->ret_stat_code = TEST_OK_S;

	if (diagpdev != NULL){
		update_diagcmd_state(diagpdev, "DBCHECK", pReq->db_check);
		switch(pReq->db_check)
		{
			case DB_INTEGRITY_CHECK:
				while ( !db_integrity_ready )
					msleep(10);
				db_integrity_ready = 0;

				msleep(100); // wait until the return value is written to the file
				{
					unsigned int crc_val; //LGE_CHANGE_S[jinhwan.do@lge.com] 2012-02-16 compile error is fixed
					//crc_val = simple_strtoul(integrity_ret.ret+1,NULL,10);
					crc_val = simple_strtoul(integrity_ret.ret+1,NULL,16);
					sprintf(pRsp->test_mode_rsp.str_buf, "0x%08X", crc_val);
						
					printk(KERN_INFO "%s\n", integrity_ret.ret);
					printk(KERN_INFO "%d\n", crc_val);//LGE_CHANGE_S[jinhwan.do@lge.com] 2012-02-16 compile error is fixed
					printk(KERN_INFO "%s\n", pRsp->test_mode_rsp.str_buf);
				}

				/* MANUFACTURE requested not to check the status, just return CRC
				if ( integrity_ret.ret[0] == '0' )
					pRsp->ret_stat_code = TEST_OK_S;
				else
					pRsp->ret_stat_code = TEST_FAIL_S;
				*/
				pRsp->ret_stat_code = TEST_OK_S;
				break;
					
			case FPRI_CRC_CHECK:
				while ( !fpri_crc_ready )
					msleep(10);
				fpri_crc_ready = 0;

				msleep(100); // wait until the return value is written to the file
				{
					unsigned int  crc_val; //LGE_CHANGE_S[jinhwan.do@lge.com] 2012-02-16 compile error is fixed
					crc_val = simple_strtoul(integrity_ret.ret+1,NULL,10);
					sprintf(pRsp->test_mode_rsp.str_buf, "0x%08X", crc_val);
					
					printk(KERN_INFO "%s\n", integrity_ret.ret);
					printk(KERN_INFO "%d\n", crc_val); //LGE_CHANGE_S[jinhwan.do@lge.com] 2012-02-16 compile error is fixed
					printk(KERN_INFO "%s\n", pRsp->test_mode_rsp.str_buf);
				}

				/* MANUFACTURE requested not to check the status, just return CRC
				if ( integrity_ret.ret[0] == '0' )
					pRsp->ret_stat_code = TEST_OK_S;
				else
					pRsp->ret_stat_code = TEST_FAIL_S;
				*/
					
				/*
				if ( integrity_ret.ret[0] == '0' )
				{
					unsigned long crc_val;
					pRsp->ret_stat_code = TEST_OK_S;
					memcpy(pRsp->test_mode_rsp.str_buf,integrity_ret.ret, 1);
					
					crc_val = simple_strtoul(integrity_ret.ret+1,NULL,10);
					sprintf(pRsp->test_mode_rsp.str_buf + 1, "%08x", crc_val);
				} else {
					pRsp->ret_stat_code = TEST_FAIL_S;
				}
				*/
				pRsp->ret_stat_code = TEST_OK_S;
				break;
					
			case FILE_CRC_CHECK:
			{
				while ( !file_crc_ready )
					msleep(10);
				file_crc_ready = 0;

				msleep(100); // wait until the return value is written to the file

				{
					unsigned int crc_val; //LGE_CHANGE_S[jinhwan.do@lge.com] 2012-02-16 compile error is fixed
					crc_val = simple_strtoul(integrity_ret.ret+1,NULL,10);
					sprintf(pRsp->test_mode_rsp.str_buf, "0x%08X", crc_val);
						
					printk(KERN_INFO "%s\n", integrity_ret.ret);
					printk(KERN_INFO "%d\n", crc_val); //LGE_CHANGE_S[jinhwan.do@lge.com] 2012-02-16 compile error is fixed
					printk(KERN_INFO "%s\n", pRsp->test_mode_rsp.str_buf);
				}

				/* MANUFACTURE requested not to check the status, just return CRC
				if ( integrity_ret.ret[0] == '0' )
					pRsp->ret_stat_code = TEST_OK_S;
				else
					pRsp->ret_stat_code = TEST_FAIL_S;
				*/
				pRsp->ret_stat_code = TEST_OK_S;
				break;
					
				/*
				int mtd_op_result = 0;
				char sec_buf[512];
				const MmcPartition *pSystem_part; 
				unsigned long system_bytes_pos_in_emmc = 0;
				unsigned long system_sec_remained = 0;
					
				printk(KERN_INFO"FILE_CRC_CHECK read block1\n");
					
				mtd_op_result = lge_mmc_scan_partitions();
				if(mtd_op_result < 0)
				{
					printk(KERN_ERR "NOT READY! unable to scan partitions\n");
					return 0;
				}
					
				pSystem_part = lge_mmc_find_partition_by_name("system");
				if ( pSystem_part == NULL )
				{
					
					printk(KERN_INFO"NO System\n");
					return 0;
				}
				system_bytes_pos_in_emmc = (pSystem_part->dfirstsec*512);
				system_sec_remained = pSystem_part->dsize;
				memset(sec_buf, 0 , 512);

				do 
				{
					mtd_op_result = lge_read_block(system_bytes_pos_in_emmc, sec_buf, 512);
					system_bytes_pos_in_emmc += mtd_op_result;
					system_sec_remained -= 1;
					printk(KERN_INFO"FILE_CRC_CHECK> system_sec_remained %d \n", system_sec_remained);
				} while ( mtd_op_result != 0 && system_sec_remained != 0 );
				*/	
#if 0					
				while ( !file_crc_ready )
					msleep(10);
				file_crc_ready = 0;
#else
//					pRsp->ret_stat_code = TEST_OK_S;

#endif					
				break;
			}
			case CODE_PARTITION_CRC_CHECK:
#if 0					
				while ( !code_partition_crc_ready )
					msleep(10);
				code_partition_crc_ready = 0;
#else
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;					
#endif					
				break;
			case TOTAL_CRC_CHECK:
#if 0 					
				while ( !total_crc_ready )
					msleep(10);
				total_crc_ready = 0;
#else
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;										
#endif					
				break;

// hojung7.kim@lge.com Add  (MS910)
			case DB_DUMP:
#if 0 					
				while ( !total_crc_ready )
					msleep(10);
				total_crc_ready = 0;
#else
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;													
#endif					
				break;

			case DB_COPY:
#if 0 					
				while ( !total_crc_ready )
					msleep(10);
				total_crc_ready = 0;
#else
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;										
#endif					
				break;
// hojung7.kim@lge.com Add  (MS910)
		}
	}
	else
	{
		printk("\n[%s] error DBCHECK", __func__ );
		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}
	printk(KERN_ERR "[_DBCHECK_] [%s:%d] DBCHECK Result=<%s>\n", __func__, __LINE__, integrity_ret.ret);

	return pRsp;
}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-16, Test Mode Porting of DB Check [END]

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-17, Test Mode Porting of WIFI  [Start]
static char wifi_get_rx_packet_info(rx_packet_info_t* rx_info)
{
	const char* src = "/data/misc/wifi/diag_wifi_result";
	char return_value = TEST_FAIL_S;
	char *dest = (void *)0;
	char buf[30];
	off_t fd_offset;
	int fd;
	char *tok, *holder = NULL;
	char *delimiter = ":\r\n";
	substring_t args[MAX_OPT_ARGS];	
	int token;	
	char tmpstr[10];

    mm_segment_t old_fs=get_fs();
    set_fs(get_ds());

	if (rx_info == NULL) {
		goto file_fail;
	}
	
	memset(buf, 0x00, sizeof(buf));

    if ( (fd = sys_open((const char __user *)src, O_CREAT | O_RDWR, 0) ) < 0 )
    {
        printk(KERN_ERR "[Testmode Wi-Fi] sys_open() failed!!\n");
        goto file_fail;
    }

    if ( (dest = kmalloc(30, GFP_KERNEL)) )
    {
        fd_offset = sys_lseek(fd, 0, 0);

        if ((sys_read(fd, (char __user *) dest, 30)) < 0)
        {
            printk(KERN_ERR "[Testmode Wi-Fi] can't read path %s \n", src);
            goto file_fail;
        }

#if 0 
		/*	sscanf(dest, "%d:%d", &(rx_info->goodpacket), &(rx_info->badpacket));    */
		strncpy(buf, (const char *)dest, sizeof(buf) - 1) ;

		tok = strtok_r(dest, delimiter, &holder);

		if ( holder != NULL && tok != NULL)
		{
			rx_info->goodpacket = simple_strtoul(tok, (char**)NULL, 10);
			tok = strtok_r(NULL, delimiter, &holder);
			rx_info->badpacket = simple_strtoul(tok, (char**)NULL, 10);
			printk(KERN_ERR "[Testmode Wi-Fi] rx_info->goodpacket %lu, rx_info->badpacket = %lu \n",
				rx_info->goodpacket, rx_info->badpacket);
			return_value = TEST_OK_S;
		}
#else
		if ((memcmp(dest, "30", 2)) == 0) {
			printk(KERN_INFO "rx_packet_cnt read error \n");
			goto file_fail;
		}

		strncpy(buf, (const char *)dest, sizeof(buf) - 1);
		buf[sizeof(buf)-1] = 0;
		holder = &(buf[2]); // skip index, result
		
		while (holder != NULL) {
			tok = strsep(&holder, delimiter);
			
			if (!*tok)
				continue;

			token = match_token(tok, param_tokens, args);
			switch (token) {
			case Param_goodpacket:
				memset(tmpstr, 0x00, sizeof(tmpstr));
				if (0 == match_strlcpy(tmpstr, &args[0], sizeof(tmpstr)))
				{
					printk(KERN_ERR "Error GoodPacket %s", args[0].from);
					continue;
				}
				rx_info->goodpacket = simple_strtol(tmpstr, NULL, 0);
				printk(KERN_INFO "[Testmode Wi-Fi] rx_info->goodpacket = %d", rx_info->goodpacket);
				break;

			case Param_badpacket:
				memset(tmpstr, 0x00, sizeof(tmpstr));
				if (0 == match_strlcpy(tmpstr, &args[0], sizeof(tmpstr)))
				{
					printk(KERN_ERR "Error BadPacket %s\n", args[0].from);
					continue;
				}

				rx_info->badpacket = simple_strtol(tmpstr, NULL, 0);
				printk(KERN_INFO "[Testmode Wi-Fi] rx_info->badpacket = %d", rx_info->badpacket);
				return_value = TEST_OK_S;
				break;

			case Param_end:
			case Param_err:
			default:
				/* silently ignore unknown settings */
				printk(KERN_ERR "[Testmode Wi-Fi] ignore unknown token %s\n", tok);
				break;
			}
		}
#endif
    }

	printk(KERN_INFO "[Testmode Wi-Fi] return_value %d!!\n", return_value);
	
file_fail:    
    kfree(dest);
    sys_close(fd);
    set_fs(old_fs);
    sys_unlink((const char __user *)src);
    return return_value;
}


static char wifi_get_test_results(int index)
{
	const char* src = "/data/misc/wifi/diag_wifi_result";
    char return_value = TEST_FAIL_S;
    char *dest = (void *)0;
	char buf[4]={0};
    off_t fd_offset;
    int fd;
    mm_segment_t old_fs=get_fs();
    set_fs(get_ds());

    if ( (fd = sys_open((const char __user *)src, O_CREAT | O_RDWR, 0) ) < 0 )
    {
        printk(KERN_ERR "[Testmode Wi-Fi] sys_open() failed!!\n");
        goto file_fail;
    }

    if ( (dest = kmalloc(20, GFP_KERNEL)) )
    {
        fd_offset = sys_lseek(fd, 0, 0);

        if ((sys_read(fd, (char __user *) dest, 20)) < 0)
        {
            printk(KERN_ERR "[Testmode Wi-Fi] can't read path %s \n", src);
            goto file_fail;
        }

		sprintf(buf, "%d""1", index);
		buf[3]='\0';
        printk(KERN_INFO "[Testmode Wi-Fi] result %s!!\n", buf);

        if ((memcmp(dest, buf, 2)) == 0)
            return_value = TEST_OK_S;
        else
            return_value = TEST_FAIL_S;
		
        printk(KERN_ERR "[Testmode Wi-Fi] return_value %d!!\n", return_value);

    }
	
file_fail:
    kfree(dest);
    sys_close(fd);
    set_fs(old_fs);
    sys_unlink((const char __user *)src);

    return return_value;
}

static char wifi_get_test_mode_check(DIAG_TEST_MODE_F_rsp_type *pRsp)
{
    const char *src = "/data/misc/wifi/diag_wifi_result";
    char return_value = TEST_FAIL_S;
    char *dest = (void *)0;
    char buf[4] = {0};
    off_t fd_offset;
    int fd;
    mm_segment_t old_fs=get_fs();
    set_fs(get_ds());

    if ((fd = sys_open((const char __user *)src, O_CREAT | O_RDWR, 0) ) < 0)  {
        printk(KERN_ERR "[Testmode Wi-Fi] sys_open() failed!!\n");
        goto file_fail;
    }

    if ((dest = kmalloc(20, GFP_KERNEL))) {
        fd_offset = sys_lseek(fd, 0, 0);

        if ((sys_read(fd, (char __user *) dest, 20)) < 0) {
            printk(KERN_ERR "[Testmode Wi-Fi] can't read path %s \n", src);
            goto file_fail;
        }
        memcpy(buf, dest, 2);
        buf[3]='\0';
        printk(KERN_INFO "[Testmode Wi-Fi] result %s!!\n", buf);

        if (buf[0] == '0' && (buf[1] == '0' || buf[1] == '1')) {
            pRsp->ret_stat_code = TEST_OK_S;
        } else {
            pRsp->ret_stat_code = TEST_FAIL_S;
        }

        if (buf[1] == '1') {
            pRsp->test_mode_rsp.wlan_status = 1;
        } else {
            pRsp->test_mode_rsp.wlan_status = 0;
        }

        printk(KERN_ERR "[Testmode Wi-Fi] return_value %d!!\n", pRsp->test_mode_rsp.wlan_status);

    }
	
file_fail:
    kfree(dest);
    sys_close(fd);
    set_fs(old_fs);
    sys_unlink((const char __user *)src);

    return return_value;
}


static test_mode_ret_wifi_ctgry_t divide_into_wifi_category(test_mode_req_wifi_type input)
{
	test_mode_ret_wifi_ctgry_t sub_category = WLAN_TEST_MODE_CTGRY_NOT_SUPPORTED;
	
	if ( input == WLAN_TEST_MODE_CHK) {
		sub_category = WLAN_TEST_MODE_CTGRY_CHK;
	} else if ( input == WLAN_TEST_MODE_54G_ON || 
		WL_IS_WITHIN(WLAN_TEST_MODE_11B_ON, WLAN_TEST_MODE_11A_CH_RX_START, input)) {
		sub_category = WLAN_TEST_MODE_CTGRY_ON;
	} else if ( input == WLAN_TEST_MODE_OFF ) {
		sub_category = WLAN_TEST_MODE_CTGRY_OFF;
	} else if ( input == WLAN_TEST_MODE_RX_RESULT ) {
		sub_category = WLAN_TEST_MODE_CTGRY_RX_STOP;
	} else if ( WL_IS_WITHIN(WLAN_TEST_MODE_RX_START, WLAN_TEST_MODE_RX_RESULT, input) || 
			WL_IS_WITHIN(WLAN_TEST_MODE_LF_RX_START, WLAN_TEST_MODE_MF_TX_START, input)) {
        sub_category = WLAN_TEST_MODE_CTGRY_RX_START;
	} else if ( WL_IS_WITHIN(WLAN_TEST_MODE_TX_START, WLAN_TEST_MODE_TXRX_STOP, input) || 
			WL_IS_WITHIN( WLAN_TEST_MODE_MF_TX_START, WLAN_TEST_MODE_11B_ON, input)) {
		sub_category = WLAN_TEST_MODE_CTGRY_TX_START;
	} else if ( input == WLAN_TEST_MODE_TXRX_STOP) {
		sub_category = WLAN_TEST_MODE_CTGRY_TX_STOP;
	}
	
	printk(KERN_INFO "[divide_into_wifi_category] input = %d, sub_category = %d!!\n", input, sub_category );
	
	return sub_category;	
}

void* LGF_TestModeWLAN(
        test_mode_req_type*	pReq,
        DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
	int i;
	static int first_on_try = 10;
	test_mode_ret_wifi_ctgry_t wl_category;

	if (diagpdev != NULL)
	{
		update_diagcmd_state(diagpdev, "WIFI_TEST_MODE", pReq->wifi);

		printk(KERN_ERR "[WI-FI] [%s:%d] WiFiSubCmd=<%d>\n", __func__, __LINE__, pReq->wifi);

		wl_category = divide_into_wifi_category(pReq->wifi);

		/* Set Test Mode */
		switch (wl_category) {
			case WLAN_TEST_MODE_CTGRY_CHK:
				msleep(1000);
				wifi_get_test_mode_check(pRsp);
				printk(KERN_INFO "[WI-FI] ret_stat_code = %d, wlan_status = %d", pRsp->ret_stat_code, pRsp->test_mode_rsp.wlan_status);

				break;
			case WLAN_TEST_MODE_CTGRY_ON:
				//[10sec timeout] when wifi turns on, it takes about 9seconds to bring up FTM mode.
				msleep(7000);
				
				first_on_try = 5;

				pRsp->ret_stat_code = wifi_get_test_results(wl_category);
				pRsp->test_mode_rsp.wlan_status = !(pRsp->ret_stat_code);
				break;

			case WLAN_TEST_MODE_CTGRY_OFF:
				//5sec timeout
				msleep(3000);
				pRsp->ret_stat_code = wifi_get_test_results(wl_category);
				break;

			case WLAN_TEST_MODE_CTGRY_RX_START:
				msleep(2000);
				pRsp->ret_stat_code = wifi_get_test_results(wl_category);
				pRsp->test_mode_rsp.wlan_status = !(pRsp->ret_stat_code);
				break;

			case WLAN_TEST_MODE_CTGRY_RX_STOP:
			{
				rx_packet_info_t rx_info;
				int total_packet = 0;
				int m_rx_per = 0;
				// init
				rx_info.goodpacket = 0;
				rx_info.badpacket = 0;
				// wait 3 sec
				msleep(3000);
				
				pRsp->test_mode_rsp.wlan_rx_results.packet = 0;
				pRsp->test_mode_rsp.wlan_rx_results.per = 0;

				pRsp->ret_stat_code = wifi_get_rx_packet_info(&rx_info);
				if (pRsp->ret_stat_code == TEST_OK_S) {
					total_packet = rx_info.badpacket + rx_info.goodpacket;
					if(total_packet > 0) {
						m_rx_per = (rx_info.badpacket * 1000 / total_packet);
						printk(KERN_INFO "[WI-FI] per = %d, rx_info.goodpacket = %d, rx_info.badpacket = %d ",
							m_rx_per, rx_info.goodpacket, rx_info.badpacket);
					}
					pRsp->test_mode_rsp.wlan_rx_results.packet = rx_info.goodpacket;
					pRsp->test_mode_rsp.wlan_rx_results.per = m_rx_per;
				}				
				break;
			}

			case WLAN_TEST_MODE_CTGRY_TX_START:
				for (i = 0; i< 2; i++)
					msleep(1000);
				pRsp->ret_stat_code = wifi_get_test_results(wl_category);
				pRsp->test_mode_rsp.wlan_status = !(pRsp->ret_stat_code);
				break;

			case WLAN_TEST_MODE_CTGRY_TX_STOP:
				for (i = 0; i< 2; i++)
					msleep(1000);
				pRsp->ret_stat_code = wifi_get_test_results(wl_category);
				pRsp->test_mode_rsp.wlan_status = !(pRsp->ret_stat_code);
				break;

			default:
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
				break;
		}
	}
	else
	{
		printk(KERN_ERR "[WI-FI] [%s:%d] diagpdev %d ERROR\n", __func__, __LINE__, pReq->wifi);
		pRsp->ret_stat_code = TEST_FAIL_S;
	}

	return pRsp;
}
/* 2011-10-13, dongseok.ok@lge.com, Add Wi-Fi Testmode function [END] */
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-17, Test Mode Porting of WIFI  [END]

// LGE_CHANGE_S, bill.jung@lge.com, 20110808, WiFi MAC R/W Function by DIAG
void* LGF_TestModeWiFiMACRW(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
// LGE_CHANGE_S, real-wifi@lge.com, 20110928, [WLAN TEST MODE]
#if 0
	DIAG_TEST_MODE_F_req_type req_ptr;

	req_ptr.sub_cmd_code = TEST_MODE_MAC_READ_WRITE;
	printk(KERN_ERR "[LGF_TestModeWiFiMACRW] req_type=%d, wifi_mac_addr=[%s]\n", pReq->wifi_mac_ad.req_type, pReq->wifi_mac_ad.wifi_mac_addr);

	if (diagpdev != NULL)
	{
		pRsp->ret_stat_code = TEST_FAIL_S;
		if( pReq->wifi_mac_ad.req_type == 0) {
			req_ptr.test_mode_req.wifi_mac_ad.req_type = 0;
			memcpy(req_ptr.test_mode_req.wifi_mac_ad.wifi_mac_addr, (void*)(pReq->wifi_mac_ad.wifi_mac_addr), WIFI_MAC_ADDR_CNT);
			send_to_arm9((void*)&req_ptr, (void*)pRsp, sizeof(DIAG_TEST_MODE_F_rsp_type));
			printk(KERN_INFO "[Wi-Fi] %s, result : %s\n", __func__, pRsp->ret_stat_code==TEST_OK_S?"OK":"FAILURE");
		} else if ( pReq->wifi_mac_ad.req_type == 1) {
			req_ptr.test_mode_req.wifi_mac_ad.req_type = 1;
			send_to_arm9((void*)&req_ptr, (void*)pRsp, sizeof(DIAG_TEST_MODE_F_rsp_type));
			printk(KERN_INFO "[Wi-Fi] %s, result : %s\n", __func__, pRsp->ret_stat_code==TEST_OK_S?"OK":"FAILURE");
		}
		else{
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
		}
	}
	else
	{
		printk(KERN_ERR "[WI-FI] [%s:%d] diagpdev %d ERROR\n", __func__, __LINE__, pReq->wifi_mac_ad.req_type);
		pRsp->ret_stat_code = TEST_FAIL_S;
	}

	return pRsp;
#endif
// LGE_CHANGE_E, real-wifi@lge.com, 20110928, [WLAN TEST MODE]
#if 1
	int fd=0;
	int i=0;
        char *src = (void *)0;
        mm_segment_t old_fs=get_fs();
        set_fs(get_ds());

	printk(KERN_ERR "[LGF_TestModeWiFiMACRW] req_type=%d, wifi_mac_addr=[%s]\n", pReq->wifi_mac_ad.req_type, pReq->wifi_mac_ad.wifi_mac_addr);

	if (diagpdev != NULL)
	{
		if( pReq->wifi_mac_ad.req_type == 0 )
		{
			printk(KERN_ERR "[LGF_TestModeWiFiMACRW] WIFI_MAC_ADDRESS_WRITE.\n");

			if ( (fd = sys_open((const char __user *) "/data/misc/wifi/diag_mac", O_CREAT | O_RDWR, 0777) ) < 0 )
			{
				printk(KERN_ERR "[LGF_TestModeWiFiMACRW] Can not open file.\n");
				pRsp->ret_stat_code = TEST_FAIL_S;
				goto file_fail;
			}

			if ( (src = kmalloc(20, GFP_KERNEL)) )
			{
				sprintf( src,"%c%c%c%c%c%c%c%c%c%c%c%c", pReq->wifi_mac_ad.wifi_mac_addr[0],
					pReq->wifi_mac_ad.wifi_mac_addr[1], pReq->wifi_mac_ad.wifi_mac_addr[2],
					pReq->wifi_mac_ad.wifi_mac_addr[3], pReq->wifi_mac_ad.wifi_mac_addr[4],
					pReq->wifi_mac_ad.wifi_mac_addr[5], pReq->wifi_mac_ad.wifi_mac_addr[6],
					pReq->wifi_mac_ad.wifi_mac_addr[7], pReq->wifi_mac_ad.wifi_mac_addr[8],
					pReq->wifi_mac_ad.wifi_mac_addr[9], pReq->wifi_mac_ad.wifi_mac_addr[10],
					pReq->wifi_mac_ad.wifi_mac_addr[11]
					);

				if ((sys_write(fd, (const char __user *) src, 12)) < 0)
				{
					printk(KERN_ERR "[LGF_TestModeWiFiMACRW] Can not write file.\n");
					pRsp->ret_stat_code = TEST_FAIL_S;
					goto file_fail;
				}
			}

			msleep(500);

			update_diagcmd_state(diagpdev, "WIFIMACWRITE", 0);

			pRsp->ret_stat_code = TEST_OK_S;

		}
		else if(  pReq->wifi_mac_ad.req_type == 1 )
		{
			printk(KERN_ERR "[LGF_TestModeWiFiMACRW] WIFI_MAC_ADDRESS_READ.\n");

			update_diagcmd_state(diagpdev, "WIFIMACREAD", 0);

			for( i=0; i< 2; i++ )
			{
				msleep(500);
			}

			if ( (fd = sys_open((const char __user *) "/data/misc/wifi/diag_mac", O_CREAT | O_RDWR, 0777) ) < 0 )
			{
				printk(KERN_ERR "[LGF_TestModeWiFiMACRW] Can not open file.\n");
				pRsp->ret_stat_code = TEST_FAIL_S;
				goto file_fail;
			}

			if ( (src = kmalloc(20, GFP_KERNEL)) )
			{
				if ((sys_read(fd, (char __user *) src, 12)) < 0)
				{
					printk(KERN_ERR "[LGF_TestModeWiFiMACRW] Can not read file.\n");
					pRsp->ret_stat_code = TEST_FAIL_S;
					goto file_fail;
				}
			}

			for( i=0; i<14; i++)
			{
				pRsp->test_mode_rsp.key_pressed_buf[i] = 0;
			}

			for( i=0; i< 12; i++ )
			{
				pRsp->test_mode_rsp.read_wifi_mac_addr[i] = src[i];
			}

			sys_unlink((const char __user *)"/data/misc/wifi/diag_mac");

			pRsp->ret_stat_code = TEST_OK_S;
		}
		else
		{
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
		}
	}
	else
	{
		pRsp->ret_stat_code = TEST_FAIL_S;
	}

file_fail:
	kfree(src);

	sys_close(fd);
	set_fs(old_fs);

	return pRsp;
#endif
}
// LGE_CHANGE_E, bill.jung@lge.com, 20110808, WiFi MAC R/W Function by DIAG

//LGE_CHANGE_S [US730] [TestMode] [jinhwan.do@lge.com] 2012-02-16, Test Mode Porting of FOTA ID CHECK [Start]
extern int fota_id_check;
extern char fota_id_read[20];
void* LGF_TestModeFOTAIDCheck(test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type* pRsp)
{
    int i;    
    
    pRsp->ret_stat_code = TEST_OK_S; // LGE_FOTA_BSP miracle.kim@lge.com temp block for TEST_OK result
    if (diagpdev != NULL)
    {
        switch( pReq->fota_id_check)
        {
		case FOTA_ID_CHECK:
			fota_id_check = 1;
			update_diagcmd_state(diagpdev, "FOTAIDCHECK", 0);
			msleep(500);

                if(fota_id_check == 0)
                {
			printk("[Testmode] TEST_OK_S\n");
			pRsp->ret_stat_code = TEST_OK_S;
                }
		else
		{
			printk("[Testmode] TEST_FAIL_S\n");
			pRsp->ret_stat_code = TEST_FAIL_S;
		}
                break;

		case FOTA_ID_READ:
			memset(fota_id_read, 0x00, sizeof(fota_id_read));

			update_diagcmd_state(diagpdev, "FOTAIDREAD", 0);
			msleep(500);

	                for(i = 0; i < sizeof(fota_id_read) ; i++)
	                    pRsp->test_mode_rsp.fota_id_read[i] = fota_id_read[i];

	                printk(KERN_ERR "%s, rsp.read_fota_id : %s\n", __func__, (char *)pRsp->test_mode_rsp.fota_id_read);
	                pRsp->ret_stat_code = TEST_OK_S;
                break;

		default:
			printk("[Testmode] TEST_NOT_SUPPORTED_S\n");
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
		break;
        }
    }
    else
    {
    	pRsp->ret_stat_code = TEST_FAIL_S;
    }

    return pRsp;
}
//LGE_CHANGE_S [US730] [TestMode] [jinhwan.do@lge.com] 2012-02-16, Test Mode Porting of FOTA ID CHECK [END]

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of KeyLock [Start]
int lgf_key_lock=0;
extern void lm3530_backlight_off(void);
extern void lm3530_backlight_on(void);

void* LGF_TestModeKEYLOCK(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
	pRsp->ret_stat_code = TEST_OK_S;
	if (diagpdev != NULL)
	{
		switch( pReq->req_key_lock)
		{
			case KEY_LOCK_REQ:				
				pRsp->ret_stat_code = TEST_OK_S;
				printk("[Testmode KEY_LOCK] key lock on\n");
				lm3530_backlight_off();				
				lgf_key_lock = 1;
				break;
			case KEY_UNLOCK_REQ:
				pRsp->ret_stat_code = TEST_OK_S;
				printk("[Testmode KEY_LOCK] key lock off\n");
				lm3530_backlight_on();
				mdelay(50);
				lgf_key_lock = 0;
				break;
//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [Start]
			case KEY_STATUS_REQ:
				printk("[Testmode KEY_LOCK] KEY_STATUS_REQ\n");
				pRsp->test_mode_rsp.key_lock_status = lgf_key_lock;
				break;
//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [End]
			default:
				printk("[Testmode KEY_LOCK]not support\n");
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
				break;				
		}
	}
	else
		pRsp->ret_stat_code = TEST_FAIL_S;

	printk("%s() : lgf_key_lock = %d, resp = %d\n", __func__, lgf_key_lock, pRsp->ret_stat_code);

	return pRsp;
}
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-02-14, Test Mode Porting of KeyLock [END]

#ifdef CONFIG_LGE_DIAG_MFT
void* LGF_TestMode_NotSupported(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
	pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	return pRsp;
}
 
 //LGE_CHANGE [jinhwan.do][2012.07.09] MFT Camera/Camcorder Get funtion is changed for kernel panic [Start]
 //LGE_CHANGE [jinhwan.do][2012.05.15] Merge from VS930 MFT command's source [Start]
 int save_file(char *file, char *data, int length)
{
	int read;
	mm_segment_t oldfs;
	

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	if((read = sys_open((const char __user *)file, O_RDONLY , 0)) < 0){
		printk("save_file : fp is NULL!\n");
		return -1;
	}
	
	if ((sys_write(read, (const char __user *) data, length)) < 0)
	{
		printk("save_file : wirte is failed!\n");
		return -1;
	}
	
	set_fs(oldfs);
	sys_close(read);

	return 0;
}

unsigned int get_file_length(char *file)
{
	int read;
	unsigned int fd_offset = 0;
	unsigned int fd_end = 0;
	mm_segment_t oldfs;	

	oldfs = get_fs();
   	set_fs(get_ds());
	if((read = sys_open((const char __user *)file, O_RDONLY , 0)) < 0){
		printk("get_file_length : fp is NULL!\n");
		return 0;
	}
	
	fd_end = sys_lseek(read, 0, 2);
	fd_offset = sys_lseek(read, 0, 0);
	total_size = fd_end - fd_offset;
	printk("get_file_length :fd_end : %u    fd_offset  :  %u    total_size : %u  \n", fd_end, fd_offset ,total_size);

	sys_close(read);
	set_fs(oldfs);
	
	printk("File size : %d\n",total_size);
	return total_size;
}

unsigned int get_file_data(char* rsp, char *file)
{
	struct file *phMscd_Filp = NULL;
	mm_segment_t old_fs;
	unsigned int remain = 0;
	ssize_t read_size = 0;
	
	old_fs=get_fs();
	set_fs(get_ds());
	
	phMscd_Filp = filp_open(file, O_RDONLY |O_LARGEFILE, 0);
	if(IS_ERR(phMscd_Filp)) {
		printk(KERN_ERR "%s, %s open failed !\n", __func__, file);
		set_fs(old_fs);
		return 0;
	}
	
	phMscd_Filp->f_pos = (loff_t)position;
	phMscd_Filp->f_op->llseek(phMscd_Filp, phMscd_Filp->f_pos, SEEK_SET);
	
	remain = total_size - position;

	if(remain <= 0) {
		printk("get_file_data : Read size is 0!\n");
		return 0;
	} else if(remain >= FILE_BUF_SIZE) {
		read_size = phMscd_Filp->f_op->read(phMscd_Filp, rsp, FILE_BUF_SIZE, &phMscd_Filp->f_pos);
		if(read_size <= 0)
		{
			printk(KERN_ERR "%s, %s read failed !, size : %d\n", __func__, file, read_size);
			set_fs(old_fs);
			return 0;
		}
		else
		{
			printk(KERN_ERR "%s, %s read OK !, size : %d\n", __func__, file, read_size);
		}
		position += FILE_BUF_SIZE;
		return FILE_BUF_SIZE;
	} else {
		read_size = phMscd_Filp->f_op->read(phMscd_Filp, rsp, remain, &phMscd_Filp->f_pos);
		if(read_size <= 0)
		{
			printk(KERN_ERR "%s, %s read failed !, size : %d\n", __func__, file, read_size);
			set_fs(old_fs);
			return 0;
		}
		else
		{
			printk(KERN_ERR "%s, %s read OK !, size : %d\n", __func__, file, read_size);
		}
		position += remain;
	}
	
	filp_close(phMscd_Filp,NULL);
	set_fs(old_fs);

	return remain;
}

int get_file_command(int req_type, char *file, m_file_type *rsp)
{
	printk("get_file_command (type): %d\n", req_type);

	rsp->flag = req_type;
	switch(req_type)
	{
		case REQ_START:
			position = 0;
			total_size = 0;
			rsp->transfered_size = get_file_length(file);
			rsp->current_size = 0;
//LGE_CHANGE [jinhwan.do][2012.07.06] MFT Camera Patch code adjust from G1_TDR [Start]			
			printk("get_file_command : data size is %d.\n", rsp->transfered_size);
			if(rsp->transfered_size <= 0)
				return -1;
//LGE_CHANGE [jinhwan.do][2012.07.06] MFT Camera Patch code adjust from G1_TDR [End]				
			break;
		case REQ_GET_BUF:
			rsp->current_size = get_file_data(rsp->data, file);
			rsp->transfered_size = position;
			break;
		case REQ_END:
			rsp->transfered_size = total_size;
			total_size = 0;
			position = 0;
			break;
		default:
			return -1;
	}

	return 0;
}
//LGE_CHANGE [jinhwan.do][2012.06.02] MFT of Camera/Camcorder error is fixed  [Start]
 //LGE_CHANGE [jinhwan.do][2012.07.09] MFT Camera/Camcorder Get funtion is changed for kernel panic [End]
 
//LGE_CHANGE [jinhwan.do][2012.04.21] Test Mode 9.0 MFT Camera/Camcorder [Start]
void* LGF_TestMode_MFT_Camera(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{	
	test_mode_req_mft_camera_type *req = NULL;
	DIAG_TEST_MODE_mft_camera_rsp_type *get_rsp = NULL;
	int mft_camera_get_parameter = -1;
	
	pRsp->ret_stat_code = TEST_OK_S;
	req = (test_mode_req_mft_camera_type*)pReq;	
	mft_camera_get_parameter = pReq->mft_camera.camera_parameter -48;
	printk("%s() : pReq  sub2 = %d, req.mft_camera_parameter = %d \n", __func__, pReq->mft_camera.sub2, pReq->mft_camera.camera_parameter);
	printk("%s() : pRsp  xx_header = %d, sub_cmd_code = %d   ret_stat_code = %d\n", __func__, pRsp->xx_header.opaque_header, pRsp->sub_cmd_code , pRsp->ret_stat_code);
	
	switch( pReq->mft_camera.sub2)
	{
		case MFT_CAMERA_INIT:
//LGE_CHANGE [jinhwan.do][2012.06.11] When LCD turn off, push home key for LCD turn on [Start]
			if(lm3530_get_state() == 2)
			{
				LGF_SendKey(KEY_HOME);
				msleep(300);
			}
//LGE_CHANGE [jinhwan.do][2012.06.11] When LCD turn off, push home key for LCD turn on [End]
		case MFT_CAMERA_EXIT:
		case MFT_CAMERA_SHOT:
			if((pReq->mft_camera.sub2 == MFT_CAMERA_INIT) || (pReq->mft_camera.sub2 == MFT_CAMERA_EXIT))
			{
				mft_cam_led_status = 0;
				mft_cam_af_status = 0;
				msleep(50);
			}
			if(mft_camera_get_parameter == 0)
			{
				if (diagpdev != NULL)
				{
					update_diagcmd_state(diagpdev, "MFT_CAMERA", pReq->mft_camera.sub2);
					msleep(1500);	//hongsic.kim@lge.com, Fixed CMD OK response after process 
				}
				else
				{
					printk("\n[MFT_CAMERA_FLASH] diagpdev is NULL \n");
				}
			}
			else
			{
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
				return pRsp;
			}
			break;
			
		case MFT_CAMERA_GET:
			get_rsp = (DIAG_TEST_MODE_mft_camera_rsp_type *)diagpkt_alloc(TEST_MODE_MFT_CAMERA_TEST, sizeof(DIAG_TEST_MODE_mft_camera_rsp_type));
			memset(&get_rsp->get_data, 0x00, sizeof(m_file_type));		
			if(get_file_command((int)pReq->mft_camera.camera_parameter, LGE_CAMERA_FILE, &get_rsp->get_data) < 0)
			{
				get_rsp->xx_header = pRsp->xx_header;	
				get_rsp->sub_cmd_code = pReq->mft_camera.sub2;	
				get_rsp->ret_stat_code = TEST_FAIL_S;
			}
			else
			{
				get_rsp->xx_header = pRsp->xx_header;	
				get_rsp->sub_cmd_code = pReq->mft_camera.sub2;
				get_rsp->ret_stat_code = TEST_OK_S;
			}
			if(pReq->mft_camera.camera_parameter == 2)
			{
				printk("\n[%s] camera_parameter is 2 ", __func__ );
				msleep(1000);
			}
			break;
			
		case MFT_CAMERA_FLASH:
			mft_cam_led_status = mft_camera_get_parameter;
			msleep(100);
			if(mft_cam_led_status > 1)			
			{
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
				return pRsp;
			}
			if (diagpdev != NULL)
			{
				update_diagcmd_state(diagpdev, "MFT_CAMERA", pReq->mft_camera.sub2);
			}
			else
			{
				printk("\n[MFT_CAMERA_FLASH] diagpdev is NULL \n");
			}
			break;
			
		case MFT_CAMERA_AUTO_FOCUS:
			mft_cam_af_status = mft_camera_get_parameter;
			msleep(100);	
			if(mft_cam_af_status > 1)			
			{
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
				return pRsp;
			}
			if (diagpdev != NULL)
			{
				update_diagcmd_state(diagpdev, "MFT_CAMERA", pReq->mft_camera.sub2);
			}
			else
			{
				printk("\n[MFT_CAMERA_AUTO_FOCUS] diagpdev is NULL \n");
			}
			break;
			
		case MFT_CAMERA_FLASH_STATUS:			
			pRsp->test_mode_rsp.mft_cam_led_status = mft_cam_led_status;
			break;
			
		case MFT_CAMERA_AUTO_FOCUS_STATUS:			
			pRsp->test_mode_rsp.mft_cam_af_status = mft_cam_af_status;
			break;
				
		default:
			printk("\n[%s] pReq->mft_camcorder.sub2 is'nt supported", __func__ );
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
			break;
	}	
	if(get_rsp != NULL)
	{
		return get_rsp;
	}
	
	return pRsp;
}


void* LGF_TestMode_MFT_Camcorder(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
	test_mode_req_mft_camcorder_type *req = NULL;
	DIAG_TEST_MODE_mft_camcorder_rsp_type *get_rsp = NULL;
	int mft_camcorder_get_parameter = -1;
	
	pRsp->ret_stat_code = TEST_OK_S;
	req = (test_mode_req_mft_camcorder_type*)pReq;
	mft_camcorder_get_parameter = pReq->mft_camcorder.camcorder_parameter -48;
	printk("%s() : pReq  sub2 = %d, req.mft_camcorder_parameter = %d \n", __func__, pReq->mft_camcorder.sub2, pReq->mft_camcorder.camcorder_parameter);
	printk("%s() : pRsp  xx_header = %d, sub_cmd_code = %d   ret_stat_code = %d\n", __func__, pRsp->xx_header.opaque_header, pRsp->sub_cmd_code , pRsp->ret_stat_code);
	
	switch( pReq->mft_camcorder.sub2)
	{
		case MFT_CAM_INIT:
//LGE_CHANGE [jinhwan.do][2012.06.11] When LCD turn off, push home key for LCD turn on [Start]
			if(lm3530_get_state() == 2)
			{
				LGF_SendKey(KEY_HOME);
				msleep(300);
			}
//LGE_CHANGE [jinhwan.do][2012.06.11] When LCD turn off, push home key for LCD turn on [End]
		case MFT_CAM_EXIT:
		case MFT_CAM_PLAY:
		case MFT_CAM_REC_STOP:
			if((pReq->mft_camera.sub2 == MFT_CAM_INIT) || (pReq->mft_camera.sub2 == MFT_CAM_EXIT))
			{
				mft_avr_led_status = 0;
				msleep(50);
			}
			if (diagpdev != NULL)
			{
				update_diagcmd_state(diagpdev, "MFT_CAMCORDER", pReq->mft_camcorder.sub2);
				msleep(1500);	//hongsic.kim@lge.com, Fixed CMD OK response after process
			}
			else
			{
				printk("\n[%s] diagpdev is NULL ", __func__ );
				return pRsp;
			}
			break;
			
		case MFT_CAM_REC_SRART:
			mft_avr_record_time = 0;
			mft_avr_record_time = pReq->mft_camcorder.camcorder_parameter;
			msleep(100);		
			if (diagpdev != NULL)
			{
				update_diagcmd_state(diagpdev, "MFT_CAMCORDER", pReq->mft_camcorder.sub2);
			}
			else
			{
				printk("\n[%s] diagpdev is NULL ", __func__ );
				return pRsp;
			}
			break;

		case MFT_CAM_GET:
			get_rsp = (DIAG_TEST_MODE_mft_camcorder_rsp_type *)diagpkt_alloc(TEST_MODE_MFT_CAM_TEST, sizeof(DIAG_TEST_MODE_mft_camcorder_rsp_type));
			memset(&get_rsp->get_data, 0x00, sizeof(m_file_type));		
			if(get_file_command((int)pReq->mft_camcorder.camcorder_parameter, LGE_CAMCORDER_FILE, &get_rsp->get_data) < 0)
			{
				get_rsp->xx_header = pRsp->xx_header;	
				get_rsp->sub_cmd_code = pReq->mft_camcorder.sub2;	
				get_rsp->ret_stat_code = TEST_FAIL_S;
			}
			else
			{
				get_rsp->xx_header = pRsp->xx_header;	
				get_rsp	->sub_cmd_code = pReq->mft_camcorder.sub2;	
				get_rsp->ret_stat_code = TEST_OK_S;
			}
			if(pReq->mft_camcorder.camcorder_parameter == 2)
			{
				printk("\n[%s] camcorder_parameter is 2 ", __func__ );
				msleep(1000);
			}
			break;
			
		case MFT_CAM_FLASH:
			mft_avr_led_status = mft_camcorder_get_parameter;
			msleep(100);
			if(mft_avr_led_status > 1)			
			{
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
				return pRsp;
			}		
			if (diagpdev != NULL)
			{
				update_diagcmd_state(diagpdev, "MFT_CAMCORDER", pReq->mft_camcorder.sub2);
			}
			else
			{
				printk("\n[%s] diagpdev is NULL ", __func__ );
				return pRsp;
			}
			break;
			
		case MFT_CAM_PLAY_STATUS:				
			if (diagpdev != NULL)
			{
				update_diagcmd_state(diagpdev, "MFT_CAMCORDER", pReq->mft_camcorder.sub2);
			}
			else
			{
				printk("\n[%s] diagpdev is NULL ", __func__ );
				return pRsp;
			}
			msleep(100);
			pRsp->test_mode_rsp.mft_avr_play_status = mft_avr_play_status;
			break;
			
		case MFT_CAM_FLASH_STATUS:				
			pRsp->test_mode_rsp.mft_avr_led_status = mft_avr_led_status;
			printk("%s() : sub2 = %d, mft_avr_led_status = %d   mft_avr_led_status = %d\n", __func__, pReq->mft_camcorder.sub2, pRsp->test_mode_rsp.mft_avr_led_status, mft_avr_led_status);
			break;
				
		default:
			printk("\n[%s] pReq->mft_camcorder.sub2 is'nt supported", __func__ );
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
			break;
	}
	
	if(get_rsp != NULL)
	{
		return get_rsp;
	}
	
	return pRsp;
}
//LGE_CHANGE [jinhwan.do][2012.04.21] Test Mode 9.0 MFT Camera/Camcorder [End]
//LGE_CHANGE [jinhwan.do][2012.06.02] MFT of Camera/Camcorder error is fixed  [End]

void* LGF_TestMode_MFT_Key(test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
	test_mode_req_mft_key_type *req = (test_mode_req_mft_key_type*)pReq;
	static int key_input_buffer_size = 0;
	
	pRsp->ret_stat_code = TEST_OK_S;
	
	switch( req->sub2)
	{
		case MFT_KEY_INIT:	
			key_input_buffer_size = req->key_data_len;									
			if(key_input_buffer_size > (MAX_MFT_KEY_BUFF_SIZE - 1) )
			{
				pRsp->ret_stat_code = TEST_FAIL_S;
				break;				
			}				
			memset((void *)mft_key_buf,0x00,MAX_MFT_KEY_BUFF_SIZE);
			count_mft_key_buf=0;
			diag_event_log_start();		
			break;
			
		case MFT_KEY_EXIT:	
			memset((void *)mft_key_buf,0x00,MAX_MFT_KEY_BUFF_SIZE);
			diag_event_log_end();		
			break;
			
//LGE_CHANGE [jinhwan.do][2012.06.03]Add MFT Key Screen Test Menu [Start]
		case MFT_KEY_SCREEN_SET:	
			update_diagcmd_state(diagpdev, "MFT_KEY_SCREEN", 1);
			msleep(1000);	//hongsic.kim@lge.com, Fixed CMD OK response after process
			break;
			
		case MFT_KEY_SCREEN_END:	
			update_diagcmd_state(diagpdev, "MFT_KEY_SCREEN", 0);
			msleep(1000);	//hongsic.kim@lge.com, Fixed CMD OK response after process
			break;
//LGE_CHANGE [jinhwan.do][2012.06.03]Add MFT Key Screen Test Menu [End]
	 
		case MFT_KEY_READ:
			key_input_buffer_size = req->key_data_len;
			if(key_input_buffer_size >(MAX_MFT_KEY_BUFF_SIZE - 1) )
			{
				pRsp->ret_stat_code = TEST_FAIL_S;
				break;				
			}
			memcpy(pRsp->test_mode_rsp.mft_key_pressed_buf, mft_key_buf, key_input_buffer_size);
			printk("%s() : sub2 = %d, key_len = %d    mft_key_buf = %s \n", __func__, req->sub2, req->key_data_len, mft_key_buf);
			memset((void *)mft_key_buf,0x00,MAX_MFT_KEY_BUFF_SIZE);	
			break;
		default:
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
		break;				
	}
	printk("\n  %s() : sub2 = %d, key_len = %d    mft_key_pressed_buf = %s \n", __func__,   req->sub2,  req->key_data_len,pRsp->test_mode_rsp.mft_key_pressed_buf);
	return pRsp;
}
//LGE_CHANGE [jinhwan.do][2012.05.15] Merge from VS930 MFT command's source [End]

void* LGF_TestMode_MFT_Volume(test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
	test_mode_req_mft_volume_type *req = (test_mode_req_mft_volume_type*)pReq;
	
	pRsp->ret_stat_code = TEST_OK_S;
	if (diagpdev != NULL){
		update_diagcmd_state(diagpdev, "GET_VOLUME_LEVEL", req->sub2);
	}	
	switch( req->sub2)
	{
		case MFT_VOLUME_READ:
			msleep(100);
			pRsp->test_mode_rsp.spk_vol_level = spk_vol_level;
			break;
			
		case MFT_VOLUME_CHANGE:	
			if (diagpdev != NULL){
				if(req->spk_volume_level < 7)
				{
					update_diagcmd_state(diagpdev, "SET_VOLUME_LEVEL", req->spk_volume_level);
					msleep(1000);	//hongsic.kim@lge.com, Fixed CMD OK response after process
				}
				else
					pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
			}
			else
			{
				printk("\n[%s] error SET_VOLUME_LEVEL", __func__ );
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
			}	
			break;
			
		default:
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
		break;				
	}
	printk("%s() : sub2 = %d, volume = %d,  spk_vol_level = %d  \n", __func__,   req->sub2, req->spk_volume_level, spk_vol_level);
	return pRsp;
}


void* LGF_TestMode_MFT_TouchDraw(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
	pRsp->ret_stat_code = TEST_OK_S;
	switch( pReq->mft_touch_draw)
	{
		case MFT_TOUCH_DRAW_INIT:
//LGE_CHANGE [jinhwan.do][2012.06.11] When LCD turn off, push home key for LCD turn on [Start]		
			if(lm3530_get_state() == 2)
			{
				LGF_SendKey(KEY_HOME);
				msleep(300);
			}
//LGE_CHANGE [jinhwan.do][2012.06.11] When LCD turn off, push home key for LCD turn on [End]
		case MFT_TOUCH_DRAW_EXIT:
		case MFT_TOUCH_DRAW_START:
			if (diagpdev != NULL)
			{
				update_diagcmd_state(diagpdev, "TOUCHDRAW", pReq->mft_touch_draw);
				msleep(1000);	//hongsic.kim@lge.com, Fixed CMD OK response after process
			}
			else
			{
				printk("\n[%s] error TOUCHDRAW", __func__ );
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
			}
		break;
		default:
			printk("[Testmode MFT_TOUCHDRAW]not support\n");
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
		break;				
	}
	printk("%s() : mft_touch_draw = %d, resp = %d \n", __func__,  pReq->mft_touch_draw, pRsp->ret_stat_code);	
	return pRsp;
}	

static ssize_t fp_offset = 0;
static ssize_t fp_total = 0;
extern int mft_lcd_input_data_1;  
extern int mft_lcd_input_data_2;  
extern int mft_lcd_input_data_3;  

void* LGF_TestMode_Mft_Lcd(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp)
{
	unsigned int x = 0, y = 0, w = 0, h = 0;
	int result = -1;
	struct file *phMscd_Filp = NULL;
	mm_segment_t old_fs;
	ssize_t read_size = 0, buf_size = 0;

	if(pReq->mft_lcd.sub_cmd2 == MFT_LCD_GET)
	{
		pRsp->test_mode_rsp.get_lcd_data.flag = pReq->mft_lcd.flag;
		switch(pReq->mft_lcd.flag)
		{
			case REQ_MFT_LCD_GET_START:
				printk(KERN_INFO "LCD MFT : REQ_START\n");

				x = pReq->mft_lcd.info[0];
				y = pReq->mft_lcd.info[1];
				w = pReq->mft_lcd.info[2];
				h = pReq->mft_lcd.info[3];
				
				printk(KERN_INFO "LCD MFT : x=%d, y=%d, w=%d, h=%d\n", x, y, w, h);

				if (x>LCD_MAIN_WIDTH || y>LCD_MAIN_HEIGHT || w>LCD_MAIN_WIDTH || h>LCD_MAIN_HEIGHT)
				{
					printk(KERN_ERR "%s out of range !\n", __func__);
					pRsp->ret_stat_code = TEST_FAIL_S;
					return pRsp;
				}
				
				result = read_Framebuffer_Testmode(LCD_CAPTURED_FILE, x, y, w, h);
				if(result != 0)
				{
					printk(KERN_ERR "%s failed to read the frame buffer !\n", __func__);
					pRsp->ret_stat_code = TEST_FAIL_S;
					return pRsp;
				}

				old_fs=get_fs();
				set_fs(get_ds());

				phMscd_Filp = filp_open(LCD_CAPTURED_FILE, O_RDONLY |O_LARGEFILE, 0);
				if(IS_ERR(phMscd_Filp)) {
					printk(KERN_ERR "%s, %s open failed !\n", __func__, LCD_CAPTURED_FILE);
					pRsp->ret_stat_code = TEST_FAIL_S;
					set_fs(old_fs);
					return pRsp;
				}

				fp_total = fp_offset = 0;
				
				phMscd_Filp->f_pos = 0;
				fp_total = phMscd_Filp->f_op->llseek(phMscd_Filp, phMscd_Filp->f_pos, SEEK_END);
				pRsp->test_mode_rsp.get_lcd_data.sended_bytes = (unsigned int)fp_total;
				pRsp->test_mode_rsp.get_lcd_data.num_bytes = (unsigned int)fp_offset;
				pRsp->ret_stat_code = TEST_OK_S;

				filp_close(phMscd_Filp,NULL);
				set_fs(old_fs);

				printk(KERN_INFO "LCD MFT : tot size = 0x%X, curr pos = 0x%X\n", fp_total, fp_offset);
				break;
				
			case REQ_MFT_LCD_GET_GETBUF:
#ifdef MFT_LCD_GET_DBG
				printk(KERN_INFO "LCD MFT : REQ_GETBUF\n");
				printk(KERN_INFO "LCD MFT : tot size = 0x%X, curr = 0x%X, remained : 0x%X\n", fp_total, fp_offset, (fp_total-fp_offset));
#endif
				old_fs=get_fs();
				set_fs(get_ds());
				
				phMscd_Filp = filp_open(LCD_CAPTURED_FILE, O_RDONLY |O_LARGEFILE, 0);
				if(IS_ERR(phMscd_Filp)) {
					printk(KERN_ERR "%s, %s open failed !\n", __func__, LCD_CAPTURED_FILE);
					pRsp->ret_stat_code = TEST_FAIL_S;
					set_fs(old_fs);
					return pRsp;
				}

				phMscd_Filp->f_pos = (loff_t)fp_offset;
				phMscd_Filp->f_op->llseek(phMscd_Filp, phMscd_Filp->f_pos, SEEK_SET);

				buf_size = fp_total - fp_offset;
				if(buf_size >= MFT_LCD_BUF_SIZE)
					buf_size = MFT_LCD_BUF_SIZE;
				//memset(pRsp->test_mode_rsp.get_lcd_data.buf, 0, sizeof(pRsp->test_mode_rsp.get_lcd_data.buf));
				read_size = phMscd_Filp->f_op->read(phMscd_Filp, pRsp->test_mode_rsp.get_lcd_data.buf, buf_size, &phMscd_Filp->f_pos);
				if(read_size <= 0)
				{
					printk(KERN_ERR "%s, %s read failed !, size : %d\n", __func__, LCD_CAPTURED_FILE, read_size);
					pRsp->ret_stat_code = TEST_FAIL_S;
					set_fs(old_fs);
					return pRsp;
				}

				fp_offset+=read_size;

				pRsp->test_mode_rsp.get_lcd_data.sended_bytes = (unsigned int)fp_offset;
				pRsp->test_mode_rsp.get_lcd_data.num_bytes = (unsigned int)buf_size;
				pRsp->ret_stat_code = TEST_OK_S;

				filp_close(phMscd_Filp,NULL);
				set_fs(old_fs);
				break;
				
			case REQ_MFT_LCD_GET_END:
				printk(KERN_INFO "LCD MFT : REQ_END\n");
				printk(KERN_INFO "LCD MFT : tot size = %d, curr pos = %d\n", fp_total, fp_offset);
				sys_unlink(LCD_CAPTURED_FILE);				
				pRsp->ret_stat_code = TEST_OK_S;
				break;
				
			default :
				printk(KERN_ERR "%s unknown command flag : %d !\n", __func__, pReq->mft_lcd.flag);
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
				break;
		}
		return pRsp;
	}
	else if (pReq->mft_lcd.sub_cmd2 == MFT_LCD_GET_INFO)
	{
		// other commands will be handled in lg_diag_app
		diagpkt_free(pRsp);
		return 0;
	}
	else if(pReq->mft_lcd.sub_cmd2 == MFT_LCD_DISPLAY_CHART)
	{
		pRsp->ret_stat_code = TEST_OK_S;

		mft_lcd_input_data_1=pReq->mft_lcd.info[0];
		update_diagcmd_state(diagpdev, "LCD",pReq->mft_lcd.sub_cmd2 );
		msleep(700);
		
		return pRsp;
	}
	else if(pReq->mft_lcd.sub_cmd2 == MFT_LCD_DISPLAY_PATTERN_CHART)
	{
		char mft_lcd_data_buf[10] = "0";
		pRsp->ret_stat_code = TEST_OK_S;
		
		memcpy(mft_lcd_data_buf, pReq->mft_lcd.info, 10);
		mft_lcd_input_data_1=  mft_lcd_data_buf[0];
		mft_lcd_input_data_2=  mft_lcd_data_buf[1];
		mft_lcd_input_data_3=  mft_lcd_data_buf[2];
		//mft_lcd_input_data_1=  pReq->mft_lcd.info[0];
		//mft_lcd_input_data_2=  pReq->mft_lcd.info[1];
		//mft_lcd_input_data_3=  pReq->mft_lcd.info[2];
		update_diagcmd_state(diagpdev, "LCD",pReq->mft_lcd.sub_cmd2 );
	
		msleep(700);
		return pRsp;
	}
	else
	{
		pRsp->ret_stat_code = TEST_OK_S;
		update_diagcmd_state(diagpdev, "LCD",pReq->mft_lcd.sub_cmd2 );
		msleep(1000);
		return pRsp;
	}
}

#endif /* CONFIG_LGE_DIAG_MFT */
//LGE_CHANGE_S[TestMode][hongsic.kim@lge.com] 2012-04-10, TESTMODE CMD ADD Earjack_check porting [Start]
void* LGF_TestModeEarJackCheck(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp)
{
	pRsp->ret_stat_code = TEST_OK_S;

	if(gpio_get_value(SYS_GPIO_EARJACK_DET)== 0)
	{
		pRsp->test_mode_rsp.earjack_check = TEST_FAIL_S;
	}
	else
	{
		pRsp->test_mode_rsp.earjack_check = TEST_OK_S;
	}

	return pRsp;
}
//LGE_CHANGE_S[TestMode][hongsic.kim@lge.com] 2012-04-10, TESTMODE CMD ADD Earjack_check porting [End]

// +s AAT apk manager cwgf.lee@lge.com

#define AAT_INSTALL_WAIT_MAX_CNT 100
#define AAT_INSTALL_STATUS_FILE "/persist/aatstatus"

char aat_install_return[50];
EXPORT_SYMBOL(aat_install_return);

static int lg_diag_aat_apk_return_read(void)
{
	int read_fd = 0;
	int wait_count = 0;
	int install_condition = 0;
	char return_str[256];
	int read_cnt = 0;


	while(wait_count < AAT_INSTALL_WAIT_MAX_CNT)
	{
		msleep(100);
		wait_count++;

		memset((void *)return_str, 0x00, 256);
		
		if((read_fd = sys_open((const char __user *)AAT_INSTALL_STATUS_FILE, O_RDONLY , 0)) < 1)
		{			
			printk("install status file : fp is NULL!\n");
			continue;
		}
		
		if ((read_cnt = sys_read(read_fd , (char __user *) return_str, 255)) < 1)
		{
			printk("install status file : READ ERROR %d\n", read_cnt); 
			continue;
		}
		else
		{
			sys_close(read_fd);
		}
		
		if(strcmp(return_str, "INSTALL_START") == 0)
		{
			if(install_condition != 1)
			{
				printk("\n[%s] AATAPKTRANSFER return Install Start %s\n", __func__, return_str );			
				install_condition = 1;
			}
			else
			{
				printk("\n[%s] AATAPKTRANSFER return Install wait : %d %s\n", __func__, wait_count, return_str );	
			}
		}
		else if(strcmp(return_str, "INFO_FILE_ERROR") == 0)
		{
			printk("\n[%s] AATAPKTRANSFER return AAT Infomation file error", __func__ );		
			install_condition = 2;
			break;
		}
		else if(strcmp(return_str, "PACKAGE_FAULT") == 0)
		{
			printk("\n[%s] AATAPKTRANSFER return Install success", __func__ );		
			install_condition = 3;
			break;
		}
		else if(strcmp(return_str, "INSTALL_FAIL") == 0)
		{
			printk("\n[%s] AATAPKTRANSFER return Install success", __func__ );
			install_condition = 4;
			break;
		}
		else if(strcmp(return_str, "INSTALL_SUCCESS") == 0)
		{
			printk("\n[%s] AATAPKTRANSFER return Install success", __func__ );
			install_condition = 5;
			break;
		}
	}

	sys_unlink(AAT_INSTALL_STATUS_FILE);
	
	if(install_condition == 5)
	{
		return 0;
	}
	else 
	{
		if(install_condition == 1 || install_condition == 0)
		{
			printk("\n[%s] error AATAPKTRANSFER return timeout", __func__ );
		}
		return -1;
	}

}

void* LGF_TestMode_AAT_Transfer(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type* pRsp)
{
	pRsp->ret_stat_code = TEST_OK_S;
	switch( pReq->aat_transfer.sub2)
	{
		case AAT_TRANSFER_INIT:
		case AAT_TRANSFER_NAME:
		case AAT_TRANSFER_SIZE:
		case AAT_TRANSFER_DATA:
		case AAT_TRANSFER_END:
		case AAT_TRANSFER_STOP:
			diagpkt_free(pRsp);
			pRsp = NULL;
		break;
		case AAT_APK_INSTALL:
		case AAT_APK_COMPARE:
			if (diagpdev != NULL)
			{
				update_diagcmd_state(diagpdev, "AATAPKTRANSFER", pReq->aat_transfer.sub2);
				if(lg_diag_aat_apk_return_read() == 0)
					pRsp->ret_stat_code = TEST_OK_S;
				else
					pRsp->ret_stat_code = TEST_FAIL_S;
			}
			else
			{
				printk("\n[%s] error AATAPKTRANSFER", __func__ );
				pRsp->ret_stat_code = TEST_FAIL_S;
			}
		break;
		default:
			printk("[Testmode AATAPKTRANSFER]not support\n");
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
		break;				
	}
	printk("%s() : AAT Transfer \n", __func__);	
	return pRsp;
}



// +e AAT apk manager cwgf.lee@lge.com

