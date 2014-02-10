#ifndef LG_DIAG_TESTMODE_H
#define LG_DIAG_TESTMODE_H

#include "lge_comdef.h"

/*********************** BEGIN PACK() Definition ***************************/
#if defined __GNUC__
  #define PACK(x)       x __attribute__((__packed__))
  #define PACKED        __attribute__((__packed__))
#elif defined __arm
  #define PACK(x)       __packed x
  #define PACKED        __packed
#else
  #error No PACK() macro defined for this compiler
#endif
/********************** END PACK() Definition *****************************/

/* BEGIN: 0014654 jihoon.lee@lge.com 20110124 */
/* MOD 0014654: [TESTMODE] SYNC UP TESTMODE PACKET STRUCTURE TO KERNEL */
//#define MAX_KEY_BUFF_SIZE    200
//LGE_CHANGE [jinhwan.do][2012.06.28] response packet data filter [Start]
//#define MAX_KEY_BUFF_SIZE    201
#define MAX_KEY_BUFF_SIZE    20
//LGE_CHANGE [jinhwan.do][2012.06.28] response packet data filter [End]
/* END: 0014654 jihoon.lee@lge.com 2011024 */
//LGE_CHANGE [jinhwan.do][2012.05.15] Merge from VS930 MFT command's source [Start]
#define MAX_MFT_KEY_BUFF_SIZE 17
#define FILE_BUF_SIZE 2048
//LGE_CHANGE [jinhwan.do][2012.05.15] Merge from VS930 MFT command's source [End]
//LGE_CHANGE [myungsu.choi][2012.07.04] Version string size definition for fixing webdload failure [Start]
#define STR_BUF_SIZE 15
//LGE_CHANGE [myungsu.choi][2012.07.04] Version string size definition for fixing webdload failure [End]

#ifdef CONFIG_LGE_DIAG_MFT
#define LCD_CAPTURED_FILE "/data/img/mft_lcd_img"

#ifndef LCD_MAIN_WIDTH
#define LCD_MAIN_WIDTH   480
#endif
#ifndef LCD_MAIN_HEIGHT
#define LCD_MAIN_HEIGHT  800
#endif
#endif

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-03-14, SWVersion [Start]
typedef enum
{
  	VER_SW=0,		//Binary Revision
  	VER_DSP=1,      /* Camera DSP */
  	VER_MMS=2,
  	VER_CONTENTS=3,
  	VER_PRL=4,
  	VER_ERI=5,
  	VER_BREW=6,
  	VER_MODEL=7,  // 250-0-7 Test Mode Version
  	VER_HW=8,
  	REV_DSP=9,
  	CAMERA_REV=10,   // Ver 7.3
  	CONTENTS_SIZE=11,
  	JAVA_FILE_CNT=13,
  	JAVA_FILE_SIZE,
  	VER_JAVA,
  	BANK_ON_CNT=16,
  	BANK_ON_SIZE,
  	MODULE_FILE_CNT,
  	MODULE_FILE_SIZE,
  	MP3_DSP_OS_VER=21,
  	TOUCH_MODULE_REV, // Ver 7.3
  	TCC_OS_VER_MODULE, // Ver 7.3
  	VER_LCD_REVISION=24,
  	VER_GSM_SW=25,
  	VER_SP_VER=26
} test_mode_req_version_type;
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-03-14, SWVersion [End]

/* 2011-10-13, dongseok.ok@lge.com, Add Wi-Fi Testmode function [START] */
#define WIFI_MAC_ADDR_CNT 12

typedef enum
{
	WLAN_TEST_MODE_CHK=3,
	WLAN_TEST_MODE_54G_ON=4,
	WLAN_TEST_MODE_OFF,
	WLAN_TEST_MODE_RX_START,
	WLAN_TEST_MODE_RX_RESULT=9,
	WLAN_TEST_MODE_TX_START=10,
	WLAN_TEST_MODE_TXRX_STOP=13,
	WLAN_TEST_MODE_LF_RX_START=31,
	WLAN_TEST_MODE_MF_TX_START=44,
	WLAN_TEST_MODE_11B_ON=57,
	WLAN_TEST_MODE_11N_MIXED_LONG_GI_ON=69,
	WLAN_TEST_MODE_11N_MIXED_SHORT_GI_ON=77,
	WLAN_TEST_MODE_11N_GREEN_LONG_GI_ON=85,
	WLAN_TEST_MODE_11N_GREEN_SHORT_GI_ON=93,
	WLAN_TEST_MODE_11A_CH_RX_START=101, // not support
	WLAN_TEST_MODE_11BG_CH_TX_START=128,
	WLAN_TEST_MODE_11A_ON=155,
	WLAN_TEST_MODE_11AN_MIXED_LONG_GI_ON=163,
	WLAN_TEST_MODE_MAX=195,
} test_mode_req_wifi_type;

typedef enum
{
	WLAN_TEST_MODE_CTGRY_ON,
	WLAN_TEST_MODE_CTGRY_OFF,
	WLAN_TEST_MODE_CTGRY_RX_START,
	WLAN_TEST_MODE_CTGRY_RX_STOP,
	WLAN_TEST_MODE_CTGRY_TX_START,
	WLAN_TEST_MODE_CTGRY_TX_STOP,
	WLAN_TEST_MODE_CTGRY_CHK,
	WLAN_TEST_MODE_CTGRY_NOT_SUPPORTED,
} test_mode_ret_wifi_ctgry_t;


typedef enum
{
	WIFI_MAC_ADDRESS_WRITE = 0,
	WIFI_MAC_ADDRESS_READ = 1,
} test_mode_req_wifi_addr_req_type;

typedef struct {
	//test_mode_req_wifi_addr_req_type req_type;
	byte req_type;
	byte wifi_mac_addr[WIFI_MAC_ADDR_CNT];
} test_mode_req_wifi_addr_type;

/* 2011-10-13, dongseok.ok@lge.com, Add Wi-Fi Testmode function [END] */

//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [Start]
typedef enum
{
  	MOTOR_OFF,
  	MOTOR_ON,
  	MOTOR_STATUS,
}test_mode_req_motor_type;
//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [End]

//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-16, TestMode 9.0, get Acoustic Status [Start]
typedef enum
{
  	ACOUSTIC_OFF=0,
  	ACOUSTIC_ON,
  	HEADSET_PATH_OPEN,
  	HANDSET_PATH_OPEN,
  	ACOUSTIC_LOOPBACK_ON,
  	ACOUSTIC_LOOPBACK_OFF,
  	ACOUSTIC_LOOPBACK_STATUS = 10,
} test_mode_req_acoustic_type;
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-16, TestMode 9.0, get Acoustic Status [End]

typedef enum
{
	CAM_TEST_MODE_OFF = 0,
	CAM_TEST_MODE_ON,
	CAM_TEST_SHOT,
	CAM_TEST_SAVE_IMAGE,
	CAM_TEST_CALL_IMAGE,
	CAM_TEST_ERASE_IMAGE,
	CAM_TEST_FLASH_ON,
	CAM_TEST_FLASH_OFF = 9,
	CAM_TEST_CAMCORDER_MODE_OFF,
	CAM_TEST_CAMCORDER_MODE_ON,
	CAM_TEST_CAMCORDER_SHOT,
	CAM_TEST_CAMCORDER_SAVE_MOVING_FILE,
	CAM_TEST_CAMCORDER_PLAY_MOVING_FILE,
	CAM_TEST_CAMCORDER_ERASE_MOVING_FILE,
	CAM_TEST_CAMCORDER_FLASH_ON,
	CAM_TEST_CAMCORDER_FLASH_OFF,
	CAM_TEST_STROBE_LIGHT_ON,
	CAM_TEST_STROBE_LIGHT_OFF,
	CAM_TEST_CAMERA_SELECT = 22,
} test_mode_req_cam_type;

//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [Start]
typedef enum
{	
	KEY_DATA_END = 0,
	KEY_DATA_START = 1,
	KEY_DATA_STATUS = 2,		
} test_mode_req_key_type;
//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [End]

typedef enum
{
  	EXTERNAL_SOCKET_MEMORY_CHECK,
  	EXTERNAL_FLASH_MEMORY_SIZE,
  	EXTERNAL_SOCKET_ERASE,
  	EXTERNAL_FLASH_MEMORY_USED_SIZE = 4,
  	EXTERNAL_SOCKET_ERASE_SDCARD_ONLY = 0xE,
  	EXTERNAL_SOCKET_ERASE_FAT_ONLY = 0xF,
} test_mode_req_socket_memory;

#ifndef LG_BTUI_TEST_MODE
typedef enum
{
  	BT_GET_ADDR, //no use anymore
  	BT_TEST_MODE_1=1,
  	BT_TEST_MODE_CHECK=2,
  	BT_TEST_MODE_RELEASE=5,
  	BT_TEST_MODE_11=11 // 11~42
} test_mode_req_bt_type;
#endif

//20110930, addy.kim@lge.com,  [START]
typedef enum
{
  NFC_TEST_MODE_ON=0,
  NFC_TEST_MODE_OFF,
  NFC_TEST_MODE_SWP,
  NFC_TEST_MODE_ANT,
  NFC_TEST_MODE_READER,
  NFC_TEST_MODE_FIRMWARE_FILE_VERSION,
  NFC_TEST_MODE_FIMEWARE_UPDATE,
  NFC_TEST_MODE_FIRMWARE_CHIP_VERSION
}test_mode_req_nfc_type;
//20110930, addy.kim@lge.com,  [END]


typedef enum
{
  	MP3_128KHZ_0DB,
  	MP3_128KHZ_0DB_L,
  	MP3_128KHZ_0DB_R,
  	MP3_MULTISINE_20KHZ,
  	MP3_PLAYMODE_OFF,
  	MP3_SAMPLE_FILE,
  	MP3_NoSignal_LR_128k
} test_mode_req_mp3_test_type;

typedef enum 
{
  	MANUAL_TEST_ON,
  	MANUAL_TEST_OFF,
  	MANUAL_MODE_CHECK,
  	MANUAL_MODE_ALLAUTO_RESULT
} test_mode_req_manual_test_mode_type;

typedef enum
{
  	MEMORY_TOTAL_SIZE_TEST = 0 ,
  	MEMORY_FORMAT_MEMORY_TEST = 1,
} test_mode_req_memory_size_type;

typedef enum
{
  	MEMORY_TOTAL_CAPA_TEST,
  	MEMORY_USED_CAPA_TEST,
  	MEMORY_REMAIN_CAPA_TEST
} test_mode_req_memory_capa_type;

//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [Start]
typedef enum
{
  	SLEEP_MODE_ON,
 	AIR_PLAIN_MODE_ON,
  	FTM_BOOT_ON,
  	AIR_PLAIN_MODE_OFF,
  	SLEEP_MODE_STATUS,
  	AIR_PLAIN_MODE_STATUS,
} test_mode_sleep_mode_type;
//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [End]

typedef enum
{
  	SPEAKER_PHONE_OFF,
  	SPEAKER_PHONE_ON,
  	NOMAL_Mic1,
  	NC_MODE_ON,
  	ONLY_MIC2_ON_NC_ON,
  	ONLY_MIC1_ON_NC_ON
} test_mode_req_speaker_phone_type;

typedef enum
{
  	TEST_SCRIPT_ITEM_SET,
  	TEST_SCRIPT_MODE_CHECK,
  	CAL_DATA_BACKUP,
  	CAL_DATA_RESTORE,
  	CAL_DATA_ERASE,
  	CAL_DATA_INFO
} test_mode_req_test_script_mode_type;

typedef enum
{
  	FACTORY_RESET_CHECK,
  	FACTORY_RESET_COMPLETE_CHECK,
  	FACTORY_RESET_STATUS_CHECK,
  	FACTORY_RESET_COLD_BOOT,
  	FACTORY_RESET_ERASE_USERDATA = 0x0F, // for NPST dll
} test_mode_req_factory_reset_mode_type;

typedef enum
{
  	FACTORY_RESET_START = 0,
  	FACTORY_RESET_INITIAL = 1,
  	FACTORY_RESET_ARM9_END = 2,
  	FACTORY_RESET_COLD_BOOT_START = 3,
  	FACTORY_RESET_COLD_BOOT_END = 5,
	FACTORY_RESET_HOME_SCREEN_END = 6,
  	FACTORY_RESET_NA = 7,
} test_mode_factory_reset_status_type;

typedef enum
{
  	VOL_LEV_OFF,
  	VOL_LEV_MIN,
  	VOL_LEV_MEDIUM,
  	VOL_LEV_MAX
} test_mode_req_volume_level_type;

typedef enum
{
  	FIRST_BOOTING_COMPLETE_CHECK,
/* BEGIN: 0015566 jihoon.lee@lge.com 20110207 */
/* ADD 0015566: [Kernel] charging mode check command */
#ifdef CONFIG_LGE_CHARGING_MODE_INFO
  	FIRST_BOOTING_CHG_MODE_CHECK=0xF, // charging mode check, temporal
#endif
/* END: 0015566 jihoon.lee@lge.com 20110207 */
} test_mode_req_fboot;

/* TEST_MODE_PID_TEST */
typedef enum
{
  PID_WRITE,
  PID_READ
}test_mode_req_subcmd_type;

typedef  struct{
  test_mode_req_subcmd_type	pid_subcmd;
  byte PID[30];
}PACKED test_mode_req_pid_type;
typedef enum
{
	DB_INTEGRITY_CHECK=0,
	FPRI_CRC_CHECK=1,
	FILE_CRC_CHECK=2,
	CODE_PARTITION_CRC_CHECK=3,
	TOTAL_CRC_CHECK=4,	
	DB_DUMP=5,  //hojung7.kim@lge.com Add  (MS910)
	DB_COPY=6   //hojung7.kim@lge.com Add  (MS910)
} test_mode_req_db_check;

typedef enum
{
  	FIRST_BOOTING_IN_CHG_MODE,
  	FIRST_BOOTING_NOT_IN_CHG_MODE
} test_mode_first_booting_chg_mode_type;

typedef enum
{
  	RESET_FIRST_PRODUCTION,
  	RESET_FIRST_PRODUCTION_CHECK,
  	RESET_REFURBISH=2,
  	RESET_REFURBISH_CHECK,
} test_mode_req_reset_production_type;

//LGE_FOTA_ID_CHECK
typedef enum
{
	FOTA_ID_CHECK,
	FOTA_ID_READ
}test_mode_req_fota_id_check_type;

extern int key_lock;

//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [Start]
typedef enum
{
	KEY_LOCK_REQ=0,
	KEY_UNLOCK_REQ=1,	
	KEY_STATUS_REQ=2,	
}test_mode_req_key_lock_type;
//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [End]

#if defined (CONFIG_LGE_LCD_K_CAL)
typedef struct
{
	byte MaxRGB[10];
}test_mode_req_lcd_cal;
#endif

//LGE_CHANGE_S [jinhwan.do][2012.03.07]TestMode patch source merge from LS696 Model [Start]
typedef enum
{
	POWER_RESET,
	POWER_OFF,
}test_mode_req_power_reset_type;
//LGE_CHANGE_S [jinhwan.do][2012.03.07]TestMode patch source merge from LS696 Model [End]

#ifdef CONFIG_LGE_DIAG_MFT	//LGE_CHANGE jinhwan.do 20120326 test mode MFT featrue porting
//LGE_CHANGE [jinhwan.do][2012.05.15] Merge from VS930 MFT command's source [Start]
typedef enum
{
	REQ_START = 0,
	REQ_GET_BUF,
	REQ_END,
}test_mode_req_mft_get_file_type ;
//LGE_CHANGE [jinhwan.do][2012.05.15] Merge from VS930 MFT command's source [End]

typedef enum
{
    MFT_LCD_INIT=0,
    MFT_LCD_GET_INFO=1,
    MFT_LCD_ON=2,
    MFT_LCD_OFF=3,
    MFT_LCD_DISPLAY_CHART=4,
    MFT_LCD_DISPLAY_PATTERN_CHART=5,
    MFT_LCD_GET=6, 	 
} test_mode_req_mft_lcd_sub_cmd2;
typedef enum {
    REQ_MFT_LCD_GET_START = 0,
    REQ_MFT_LCD_GET_GETBUF,
    REQ_MFT_LCD_GET_END,
} test_mode_req_mft_lcd_get_flag;

typedef struct 
{
	unsigned char sub_cmd2; /* enum test_mode_req_mft_lcd_type */
	unsigned int info[4]; /* x, y, w, h */
	int flag; /* enum test_mode_req_mft_lcd_get_flag */
} PACKED test_mode_req_mft_lcd_type;

#define MFT_LCD_BUF_SIZE 2048
typedef struct
{
    int flag; /* enum test_mode_req_mft_lcd_get_flag */
    unsigned int sended_bytes;
    unsigned int num_bytes;
    char buf[MFT_LCD_BUF_SIZE];
} PACKED test_mode_rsp_mft_lcd_get_type;

typedef enum
{
  	MFT_LED_ON=0,
  	MFT_LED_OFF=1,
  	MFT_LED_STATUS=2,
} test_mode_req_mft_led_type;

//LGE_CHANGE [jinhwan.do][2012.06.02] MFT of Camera/Camcorder error is fixed  [Start]
//LGE_CHANGE [jinhwan.do][2012.04.21] Test Mode 9.0 MFT Camera/Camcorder [Start]
typedef enum
{
  	MFT_CAMERA_INIT=0,
	MFT_CAMERA_EXIT=1,
	MFT_CAMERA_SHOT=2,
	MFT_CAMERA_GET=3,
	MFT_CAMERA_FLASH=4,
	MFT_CAMERA_AUTO_FOCUS=5,
	MFT_CAMERA_CLOSEST_FOCUS=6,
	MFT_CAMERA_FLASH_STATUS=7,
	MFT_CAMERA_AUTO_FOCUS_STATUS=8,
	MFT_CAMERA_CLOSEST_FOCUS_STATUS=9,		
} test_mode_req_mft_camera_id;

typedef  struct{
	byte sub2;
	byte camera_parameter;
}PACKED test_mode_req_mft_camera_type;

typedef enum
{
  	MFT_CAM_INIT=0,
  	MFT_CAM_EXIT=1,	
  	MFT_CAM_REC_SRART=2,
  	MFT_CAM_REC_STOP=3,
  	MFT_CAM_PLAY=4,
  	MFT_CAM_PLAY_STATUS=5,
  	MFT_CAM_GET=6,
  	MFT_CAM_FLASH=7,  
  	MFT_CAM_FLASH_STATUS=8,  		
} test_mode_req_mft_camcorder_id;

typedef  struct{
	byte sub2;
	byte camcorder_parameter;
}PACKED test_mode_req_mft_camcorder_type;
//LGE_CHANGE [jinhwan.do][2012.04.21] Test Mode 9.0 MFT Camera/Camcorder [End]
//LGE_CHANGE [jinhwan.do][2012.06.02] MFT of Camera/Camcorder error is fixed  [End]

typedef enum
{
  	MFT_KEY_INIT=0,
	MFT_KEY_EXIT=1,
	MFT_KEY_READ=2,	 
//LGE_CHANGE [jinhwan.do][2012.06.03]Add MFT Key Screen Test Menu [Start]
	MFT_KEY_SCREEN_SET=3, 
	MFT_KEY_SCREEN_END=4,  
//LGE_CHANGE [jinhwan.do][2012.06.03]Add MFT Key Screen Test Menu [End]
} test_mode_req_mft_key_id;

typedef struct 
{
  byte sub2;
  byte key_data_len;
} PACKED test_mode_req_mft_key_type;

typedef enum
{
  	MFT_PROXIMITY_SENSOR_INIT=0,
  	MFT_PROXIMITY_SENSOR_EXIT=1,
  	MFT_PROXIMITY_SENSOR_CHECK=2
} test_mode_req_mft_proximity_sensor_type;

typedef enum
{
	MFT_ILLUMINATION_SENSOR_INIT=0,
	MFT_ILLUMINATION_SENSOR_EXIT=1,
	MFT_ILLUMINATION_SENSOR_CHECK_LEVEL=2,
	MFT_ILLUMINATION_SENSOR_CHECK_VALUE=3,
		
} test_mode_req_mft_illumination_sensor_type;

typedef enum
{
	MFT_VOLUME_READ=0,
	MFT_VOLUME_CHANGE=1		
} test_mode_req_mft_volume_id;

typedef struct 
{
  byte sub2;
  byte spk_volume_level;
} PACKED test_mode_req_mft_volume_type;

typedef enum
{
	MFT_TOUCH_DRAW_INIT=0,
	MFT_TOUCH_DRAW_EXIT=1,
	MFT_TOUCH_DRAW_START=2
} test_mode_req_mft_touch_draw_type;
#endif /* CONFIG_LGE_DIAG_MFT */
//LGE_CHANGE_S[TestMode][hongsic.kim@lge.com] 2012-04-10, TESTMODE CMD ADD Earjack_check porting [Start]
typedef enum
{
	EARJACK_CHECK = 0
} test_mode_req_earjack_check_type;
//LGE_CHANGE_S[TestMode][hongsic.kim@lge.com] 2012-04-10, TESTMODE CMD ADD Earjack_check porting [End]
//LGE_CHANGE jini1711 20120409 add AAT SET diag command [Start]
#define LGE_TESTMODE_AATSET_DATA		100
typedef enum
{	
  AATSET_TEST_READ = 0,	
  AATSET_TEST_WRITE = 1,	
  AATSET_TEST_FULL = 2
} test_mode_req_aatset_request_type;

typedef struct {
	byte sub2;
	byte aatset_data[LGE_TESTMODE_AATSET_DATA];
} PACKED test_mode_req_aatset_type;
//LGE_CHANGE jini1711 20120409 add AAT SET diag command [End]

#define MEID_ESN_BUF_SIZE 30
typedef enum
{
  TESTMODE_DLOAD_MEID_READ = 0,
  TESTMODE_DLOAD_ESN_READ = 1,
}test_mode_req_dload_meid_subcmd2_type;

#define MODEL_NAME_BUF_SIZE 50
typedef enum
{
  TESTMODE_DLOAD_MODEL_NAME_READ = 0,
}test_mode_req_dload_model_name_subcmd2_type;

//+s AAT apk manager cwgf.lee@lge.com
typedef PACK(enum)
{    
	AAT_TRANSFER_INIT = 0,
	AAT_TRANSFER_NAME,
	AAT_TRANSFER_SIZE,
	AAT_TRANSFER_DATA,
	AAT_TRANSFER_END,
	AAT_TRANSFER_STOP,
	AAT_APK_INSTALL,
	AAT_APK_COMPARE,
	AAT_APK_STATUS_NONE,
} aat_transfer_request_id;

typedef struct
{
	byte sub2;
	int size;
} PACKED test_mode_req_aat_transfer_type;
//+e AAT apk manager cwgf.lee@lge.com


typedef union
{
	test_mode_req_version_type				version;
	test_mode_req_motor_type            	motor;
	test_mode_req_acoustic_type         	acoustic;
	test_mode_req_cam_type              	camera;
//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [Start]
	test_mode_req_key_type                             	key_test_start;
//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [End]
	test_mode_req_socket_memory         	esm;
#ifndef LG_BTUI_TEST_MODE
	test_mode_req_bt_type               	bt;
#endif
	//20110930, chan2.kim@lge.com,	[START]
	test_mode_req_nfc_type 					nfc;
	//20110930, chan2.kim@lge.com,	[END]
	test_mode_req_mp3_test_type         	mp3_play;
	test_mode_req_manual_test_mode_type		test_manual_mode;
	test_mode_req_memory_size_type      	memory_format;
	test_mode_req_pid_type			pid;
	word                                	key_data;
	test_mode_req_memory_capa_type      	mem_capa;
	test_mode_sleep_mode_type           	sleep_mode;
	test_mode_req_speaker_phone_type		speaker_phone;
	test_mode_req_test_script_mode_type 	test_mode_test_scr_mode;
	test_mode_req_factory_reset_mode_type  	factory_reset;
	test_mode_req_volume_level_type			volume_level;
	test_mode_req_fboot 					fboot;
	test_mode_req_db_check					db_check;
	test_mode_req_reset_production_type 	reset_production_cmd;
	/* 2011-10-13, dongseok.ok@lge.com, Add Wi-Fi Testmode [START] */
	test_mode_req_wifi_type wifi;
	test_mode_req_wifi_addr_type wifi_mac_ad;
	/* 2011-10-13, dongseok.ok@lge.com, Add Wi-Fi Testmode [END] */
#if defined (CONFIG_LGE_LCD_K_CAL)
	test_mode_req_lcd_cal lcd_cal; 
#endif
	//LGE_FOTA_ID_CHECK
	test_mode_req_fota_id_check_type		fota_id_check;
	//Testmode Key Lock
	test_mode_req_key_lock_type				req_key_lock;
	//LGE_CHANGE_S [jinhwan.do][2012.03.07]TestMode patch source merge from LS696 Model [Start]
	test_mode_req_power_reset_type		power_reset;
	//LGE_CHANGE_S [jinhwan.do][2012.03.07]TestMode patch source merge from LS696 Model [End]

#ifdef CONFIG_LGE_DIAG_MFT	//LGE_CHANGE jinhwan.do 20120326 test mode MFT featrue porting
	test_mode_req_mft_lcd_type		mft_lcd;
	test_mode_req_mft_led_type		mft_led;
	test_mode_req_mft_camera_type		mft_camera;
	test_mode_req_mft_camcorder_type		mft_camcorder;
	test_mode_req_mft_key_type		mft_key;
	test_mode_req_mft_proximity_sensor_type		mft_proximity_sensor;
	test_mode_req_mft_illumination_sensor_type		mft_illumination_sensor;
	test_mode_req_mft_volume_type		mft_volume;
	test_mode_req_mft_touch_draw_type		mft_touch_draw;
#endif /* CONFIG_LGE_DIAG_MFT */
//LGE_CHANGE jini1711 20120409 add AAT SET diag command [Start] 
	test_mode_req_aatset_type				aatset;
//LGE_CHANGE jini1711 20120409 add AAT SET diag command [End]
	char meid_read;
	char model_name_read;
//+s AAT apk manager cwgf.lee@lge.com
	test_mode_req_aat_transfer_type		aat_transfer;
//+e AAT apk manager cwgf.lee@lge.com

} test_mode_req_type;

typedef struct diagpkt_header
{
	byte opaque_header;
} PACKED diagpkt_header_type;

typedef struct DIAG_TEST_MODE_F_req_tag {
	diagpkt_header_type						xx_header;
	word									sub_cmd_code;
	test_mode_req_type						test_mode_req;
} PACKED DIAG_TEST_MODE_F_req_type;

typedef enum
{
  	TEST_OK_S,
  	TEST_FAIL_S,
  	TEST_NOT_SUPPORTED_S
} PACKED test_mode_ret_stat_type;


/* 2011-10-13, dongseok.ok@lge.com, Add Wi-Fi Testmode [START] */
typedef struct
{
	int packet;
	int per;
} PACKED WlRxResults;
/* 2011-10-13, dongseok.ok@lge.com, Add Wi-Fi Testmode [END] */

typedef union
{
  	test_mode_req_version_type				version;
  	byte									str_buf[STR_BUF_SIZE];
//LGE_CHANGE jini1711 20120409 add AAT SET diag command [Start]
	byte						str_aat_buf[LGE_TESTMODE_AATSET_DATA];
//LGE_CHANGE jini1711 20120409 add AAT SET diag command [End]
	char 									key_pressed_buf[MAX_KEY_BUFF_SIZE];
	int                                     manual_test;
	char  									memory_check;
	test_mode_req_pid_type				pid;
// BEGIN : munho.lee@lge.com 2011-01-15
// MOD: 0013541: 0014142: [Test_Mode] To remove Internal memory information in External memory test when SD-card is not exist   
  	uint32    								socket_memory_size;
  	uint32    								socket_memory_usedsize;
//  int    socket_memory_size
//  int    socket_memory_usedsize;
// END : munho.lee@lge.com 2011-01-15
	unsigned int 							mem_capa;
	/* 2011-10-13, dongseok.ok@lge.com, Add Wi-Fi Testmode [START] */
	byte wlan_status;
	WlRxResults wlan_rx_results;
	byte read_wifi_mac_addr[12];
	/* 2011-10-13, dongseok.ok@lge.com, Add Wi-Fi Testmode [END] */
	test_mode_req_test_script_mode_type 	test_mode_test_scr_mode;
	test_mode_req_factory_reset_mode_type	factory_reset;
#if defined (CONFIG_LGE_LCD_K_CAL)
	test_mode_req_lcd_cal lcd_cal;
#endif
   	byte 	fota_id_read[30];
#ifdef CONFIG_LGE_DIAG_MFT	//LGE_CHANGE jinhwan.do 20120326 test mode MFT featrue porting
//LGE_CHANGE [jinhwan.do][2012.05.15] Merge from VS930 MFT command's source [Start]
	char 							mft_key_pressed_buf[MAX_MFT_KEY_BUFF_SIZE];
//LGE_CHANGE [jinhwan.do][2012.05.15] Merge from VS930 MFT command's source [End]
	int							spk_vol_level;
	test_mode_rsp_mft_lcd_get_type get_lcd_data;
#endif /*CONFIG_LGE_DIAG_MFT*/
//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [Start]
	int							motor_status;
	int							key_data_status;
	int							sleep_status;
	int							airplane_status;
	int							key_lock_status;
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-16, TestMode 9.0, get Acoustic Status [Start]
	int							acoustic_status;
//LGE_CHANGE_S[TestMode][jinhwan.do@lge.com] 2012-04-16, TestMode 9.0, get Acoustic Status [End]
//LGE_CHANGE [jinhwan.do][2012.06.02] MFT of Camera/Camcorder error is fixed  [Start]
//LGE_CHANGE [jinhwan.do][2012.04.21] Test Mode 9.0 MFT Camera/Camcorder [Start]
	int							mft_cam_led_status;
	int							mft_cam_af_status;
	int							mft_avr_led_status;
	int							mft_avr_play_status;
//LGE_CHANGE [jinhwan.do][2012.04.21] Test Mode 9.0 MFT Camera/Camcorder [End]
//LGE_CHANGE [jinhwan.do][2012.06.02] MFT of Camera/Camcorder error is fixed  [End]
//LGE_CHANGE [jinhwan.do@lge.com][2012.04.10] Test Mode 9.0 key/led/motor/sleep status check [End]
//LGE_CHANGE_S[TestMode][hongsic.kim@lge.com] 2012-04-10, TESTMODE CMD ADD Earjack_check porting [Start]
	char	earjack_check;
//LGE_CHANGE_S[TestMode][hongsic.kim@lge.com] 2012-04-10, TESTMODE CMD ADD Earjack_check porting [End]	
	byte	lcd_data[256];
	char meid_esn_buf[MEID_ESN_BUF_SIZE];
	char model_name_buf[MODEL_NAME_BUF_SIZE];
} PACKED test_mode_rsp_type;

typedef struct DIAG_TEST_MODE_F_rsp_tag {
	diagpkt_header_type		xx_header;
	word					sub_cmd_code;
	test_mode_ret_stat_type	ret_stat_code;
	test_mode_rsp_type		test_mode_rsp;
} PACKED DIAG_TEST_MODE_F_rsp_type;

typedef enum
{
	TEST_MODE_VERSION=0,
	TEST_MODE_LCD=1,
#if defined (CONFIG_LGE_LCD_K_CAL)
	TEST_MODE_LCD_CAL = 2,
#endif
	TEST_MODE_MOTOR=3,
	TEST_MODE_ACOUSTIC=4,
	TEST_MODE_CAM=7,
	TEST_MODE_KEY_TEST=22,
	TEST_MODE_EXT_SOCKET_TEST=23,
	TEST_MODE_BATT_TEST=25,
	TEST_MODE_MP3_TEST=27,
#ifndef LG_BTUI_TEST_MODE
  	TEST_MODE_BLUETOOTH_TEST=24,
#endif
	TEST_MODE_ACCEL_SENSOR_TEST=31,
	TEST_MODE_ALCOHOL_SENSOR_TEST=32, //[2011-10-6] addy.kim@lge.com, add Test Number 32
	TEST_MODE_WIFI_TEST=33,
	TEST_MODE_MANUAL_MODE_TEST=36,
	TEST_MODE_FORMAT_MEMORY_TEST=38,
	TEST_MODE_KEY_DATA_TEST=40,
	TEST_MODE_MEMORY_CAPA_TEST=41,
	TEST_MODE_SLEEP_MODE_TEST=42,
	TEST_MODE_SPEAKER_PHONE_TEST=43,
    TEST_MODE_VCO_SELF_TUNNING_TEST=46,
    TEST_MODE_MRD_USB_TEST=47,
	TEST_MODE_TEST_SCRIPT_MODE=48,
	TEST_MODE_PROXIMITY_SENSOR_TEST=49, 
 	TEST_MODE_FACTORY_RESET_CHECK_TEST=50,
	TEST_MODE_VOLUME_TEST=51,
	TEST_MODE_CGPS_MEASURE_CNO=54,
	TEST_MODE_FIRST_BOOT_COMPLETE_TEST = 58,
	TEST_MODE_LED_TEST=60,	
	TEST_MODE_PID_TEST=70,
	TEST_MODE_SW_VERSION=71,
	TEST_MODE_IME_TEST=72,
	TEST_MODE_IMPL_TEST=73,
	TEST_MODE_UNLOCK_CODE_TEST=75,
	TEST_MODE_IDDE_TEST=76,
	TEST_MODE_FULL_SIGNATURE_TEST=77,
	TEST_MODE_NT_CODE_TEST=78,
	TEST_MODE_CAL_CHECK=82,
#ifndef LG_BTUI_TEST_MODE
  	TEST_MODE_BLUETOOTH_TEST_RW=83,
#endif
	TEST_MODE_SKIP_WELCOM_TEST=87,
	TEST_MODE_MAC_READ_WRITE=88,
	TEST_MODE_DB_INTEGRITY_CHECK=91,
	TEST_MODE_NVCRC_CHECK=92,
    TEST_MODE_SENSOR_CALIBRATION_TEST = 93,
	TEST_MODE_RELEASE_CURRENT_LIMIT=94,
	TEST_MODE_RESET_PRODUCTION=96,
	//LGE_FOTA_ID_CHECK
	TEST_MODE_FOTA_ID_CHECK = 98,
	// Key lock
	TEST_MODE_KEY_LOCK =99,
	TEST_MODE_ACCEL_SENSOR_ONOFF_TEST=100,
	TEST_MODE_COMPASS_SENSOR_TEST=102,
	TEST_MODE_GNSS_MEASURE_CNO = 103,
	TEST_MODE_POWER_RESET=105,
//LGE_CHANGE_S mschoi 20120531 add JTAG disable cmd (LG_FW_JTAG_DISABLE)[Start]
	TEST_MODE_JTAG_DISABLE = 109,
//LGE_CHANGE jini1711 20120409  add JTAG disable cmd (LG_FW_JTAG_DISABLE)[End]
#ifdef CONFIG_LGE_DIAG_MFT	//LGE_CHANGE jinhwan.do 20120326 test mode MFT featrue porting
	TEST_MODE_MFT_LCD_TEST=116,
	TEST_MODE_MFT_LED_TEST=117,
	TEST_MODE_MFT_CAMERA_TEST=118,
	TEST_MODE_MFT_CAM_TEST=119,
	TEST_MODE_MFT_KEY_TEST=120,
	TEST_MODE_MFT_PROXIMITY_SENSOR_TEST=121,
	TEST_MODE_MFT_ILLUMINATION_SENSOR_TEST=122,
	TEST_MODE_MFT_VOLUME_TEST=123,
	TEST_MODE_MFT_TOUCH_DRAW_TEST=124,
#endif /* CONFIG_LGE_DIAG_MFT */
	TESTMODE_DLOAD_MEID = 125,
//LGE_CHANGE jini1711 20120409 add AAT SET diag command [Start]
    TEST_MODE_MFT_AAT_SET=130,
//LGE_CHANGE jini1711 20120409 add AAT SET diag command [End]
//LGE_CHANGE_S [ihyunju.kim@lge.com] 2012-04-19 US730 TestMode 	
//LG_SYS_MISC_TESTMODE_LOOPBACK_CALL
        TEST_MODE_LOOPBACK_CALL=131,
//LGE_CHANGE_E [ihyunju.kim@lge.com] 2012-04-19 
	TESTMODE_DLOAD_MODEL_NAME = 132,
//LGE_CHANGE_S[TestMode][hongsic.kim@lge.com] 2012-04-10, TESTMODE CMD ADD Earjack_check porting [Start]	
	TEST_MODE_EARJACK_CHECK=133,
//LGE_CHANGE_S[TestMode][hongsic.kim@lge.com] 2012-04-10, TESTMODE CMD ADD Earjack_check porting [End]	
// +s AAT apk manager cwgf.lee@lge.com
  	TEST_MODE_MFT_AAT_TRANSFER = 190,
// +e AAT apk manager cwgf.lee@lge.com
  	MAX_TEST_MODE_SUBCMD = 0xFFFF
} PACKED test_mode_sub_cmd_type;
 
#define TESTMODE_MSTR_TBL_SIZE   128

#define ARM9_PROCESSOR		0
#define ARM11_PROCESSOR		1

typedef void*(* testmode_func_type)(test_mode_req_type * , DIAG_TEST_MODE_F_rsp_type * );

typedef struct
{
  	word cmd_code;
	testmode_func_type func_ptr;
  	byte  which_procesor;             // to choose which processor will do act.
} testmode_user_table_entry_type;

typedef struct DIAG_TEST_MODE_KEY_F_rsp_tag {
  	diagpkt_header_type		xx_header;
  	word					sub_cmd_code;
  	test_mode_ret_stat_type	ret_stat_code;
  	char key_pressed_buf[MAX_KEY_BUFF_SIZE];
} PACKED DIAG_TEST_MODE_KEY_F_rsp_type;

#if 1	//def LG_FW_USB_ACCESS_LOCK
#define PPE_UART_KEY_LENGTH 6
#define PPE_DES3_KEY_LENGTH 128
/* LGE_CHANGE_S [START] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */
#define USB_LOCK_KEY_LENGTH 16 
/* LGE_CHANGE_S [END] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */

typedef enum {
  TF_SUB_CHECK_PORTLOCK = 0,
  TF_SUB_LOCK_PORT,
  TF_SUB_UNLOCK_PORT,
  TF_SUB_KEY_VERIFY,  
  TF_SUB_GET_CARRIER,
  TF_SUB_PROD_FLAG_STATUS,  
  TF_SUB_PROD_KEY_VERIFY,  
/* LGE_CHANGE_S [START] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */  
#ifdef CONFIG_LGE_DIAG_USB_PERMANENT_LOCK
  TF_SUB_CLEAR_USB_UNLOCK_FAIL_CNT,
  TF_SUB_GET_USB_UNLOCK_FAIL_CNT,
#endif 
#ifdef CONFIG_LGE_DIAG_USBLOCK_EFS_SYNC
  TF_SUB_CHECK_EFS_SYNC = 9,  
#endif 
/* LGE_CHANGE_S [END] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */
} nvdiag_tf_sub_cmd_type;

// oskwon 090606 : Lock 상태에서 SUB 커맨드 3개만 허용, 패킷 길이는 3가지 다 허용 
typedef struct	
{
  byte cmd_code;                      /* Command code */
  byte sub_cmd;                       /* Sub Command */
} PACKED DIAG_TF_F_req_type1;

typedef struct
{
  byte cmd_code;                      /* Command code */
  byte sub_cmd;                       /* Sub Command */
  byte keybuf[PPE_UART_KEY_LENGTH];   /* Uart Lock Key - 6 Digit */
} PACKED DIAG_TF_F_req_type2;

typedef struct
{
  byte cmd_code;                      /* Command code */
  byte sub_cmd;                       /* Sub Command */
  union {
/* LGE_CHANGE_S [START] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */  
#ifdef CONFIG_LGE_DIAG_USB_ACCESS_LOCK_SHA1_ENCRYPTION
		byte keybuf[USB_LOCK_KEY_LENGTH]; /* Uart Lock Key - 16 Digit */
#else
		byte keybuf[PPE_UART_KEY_LENGTH];	/* Uart Lock Key - 16 Digit */
#endif
/* LGE_CHANGE_S [END] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */
  byte probuf[PPE_DES3_KEY_LENGTH];   /* Production Key - 128 Byte */
  } PACKED buf;
} PACKED DIAG_TF_F_req_type;

typedef struct
{
  byte cmd_code;                      /* Command code */ 
  byte sub_cmd;                       /* Sub Command */
  byte result;                        /* Status of operation */
} PACKED DIAG_TF_F_rsp_type;

typedef enum {
  TF_STATUS_FAIL = 0,       	//Fail Unknown Reason
  TF_STATUS_SUCCESS,        	//General Success
  TF_STATUS_PORT_LOCK = 12,    	//TF_SUB_CHECK_PORTLOCK -> LOCK
  TF_STATUS_PORT_UNLOCK,    	//TF_SUB_CHECK_PORTLOCK -> UNLOCK
  TF_STATUS_VER_KEY_OK,     	//TF_SUB_KEY_VERIFY -> OK 
  TF_STATUS_VER_KEY_NG,     	//TF_SUB_KEY_VERIFY -> NG
  TF_STATUS_P_FLAG_ENABLE,   	//PRODUCTION FLAG 1 
  TF_STATUS_P_FLAG_DISABLE,		//PRODUCTION FLAG 0
  TF_STATUS_VER_P_KEY_OK,		// PPE KEY OK
  TF_STATUS_VER_P_KEY_NG,		// PPE KEY NG
} DIAG_TF_F_sub_cmd_result_type;

typedef struct
{
  byte cmd_code;                      /* Command code */
  dword seccode;                       /* security code */
} PACKED DIAG_TF_SB_F_req_type;

typedef struct
{
  byte cmd_code;                      /* Command code */ 
} PACKED DIAG_TF_SB_F_rsp_type;

#endif

/* LGE_CHANGE_S [START] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */    
#ifdef CONFIG_LGE_DIAG_USB_ACCESS_LOCK_SHA1_ENCRYPTION
typedef struct
{
  byte cmd_code;                      /* Command code */
  byte key[USB_LOCK_KEY_LENGTH];   /* Uart Lock Key - 16 Digit */
} PACKED DIAG_USB_LOCK_KEY_F_req_type;

typedef struct
{
  byte cmd_code;                      /* Command code */ 
  byte result;                        /* Status of operation */
} PACKED DIAG_USB_LOCK_KEY_F_rsp_type;
#endif
/* LGE_CHANGE_S [END] 2012.05.30 choongnam.kim@lge.com to enable ##USBLOCK# */
 
 //LGE_CHANGE [jinhwan.do][2012.05.15] Merge from VS930 MFT command's source [Start]
 typedef struct
{
	int flag;
	unsigned int transfered_size;
	unsigned int current_size;
	char data[FILE_BUF_SIZE];
} PACKED m_file_type;

typedef struct DIAG_TEST_MODE_mft_camera_rsp_tag
{
  	diagpkt_header_type		xx_header;
  	word					sub_cmd_code;
  	test_mode_ret_stat_type	ret_stat_code;
	m_file_type get_data;
} PACKED DIAG_TEST_MODE_mft_camera_rsp_type;

//LGE_CHANGE [jinhwan.do][2012.06.02] MFT of Camera/Camcorder error is fixed  [Start]
typedef struct DIAG_TEST_MODE_mft_camcorder_rsp_tag
{
  	diagpkt_header_type		xx_header;
  	word					sub_cmd_code;
  	test_mode_ret_stat_type	ret_stat_code;
	m_file_type get_data;
} PACKED DIAG_TEST_MODE_mft_camcorder_rsp_type;
//LGE_CHANGE [jinhwan.do][2012.06.02] MFT of Camera/Camcorder error is fixed  [End]
//LGE_CHANGE [jinhwan.do][2012.05.15] Merge from VS930 MFT command's source [End]

#endif /* LG_DIAG_TESTMODE_H */
