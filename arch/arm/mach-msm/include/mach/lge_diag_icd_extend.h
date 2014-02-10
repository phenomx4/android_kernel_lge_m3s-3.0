#ifndef LG_DIAG_ICD_KEY_H
#define LG_DIAG_ICD_KEY_H

//#include <linux/lge_at_cmd.h> 

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

/*
typedef struct {
	int hold;
	unsigned int key_code;
}PACKED log_data_key_event_type;

typedef struct {
	unsigned int action;
	int x;
	int y;
}PACKED log_data_touch_event_type;

typedef union {
	log_data_key_event_type key;
	log_data_touch_event_type touch;
}PACKED icd_log_data_type;

typedef struct {
	int id;
	unsigned int time;
	icd_log_data_type data;
}PACKED icd_log_type;
*/

typedef enum {
	ICD_LOG_ID_KEY = 1,
	ICD_LOG_ID_TOUCH = 2,
	ICD_LOG_ID_MAX,
}icd_log_id_type;

/** ICD requset type**/
typedef struct
{
	unsigned char cmd_code;
	unsigned char sub_cmd;
} PACKED icd_key_req_hdr_type;

typedef struct {
	icd_key_req_hdr_type hdr;
	unsigned char icd_key_read;
}PACKED icd_key_get_req_type;

typedef union
{
	icd_key_get_req_type get_key_data_req_info;
} PACKED icd_key_req_type;

typedef union{
	icd_key_req_hdr_type hdr;
	icd_key_req_type icd_key_req;
} PACKED DIAG_ICD_KEY_F_req_type;
/** ICD requset type**/

//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [Start]
typedef struct {
	icd_key_req_hdr_type hdr;
	char key_data[201];
}PACKED icd_key_get_rsq_type;
//LGE_CHANGE jinhwan.do 20120323 Slate source merge from LS696 [End]

typedef union
{
	icd_key_get_rsq_type get_key_data_rsq_info;
} PACKED icd_key_rsp_type;

typedef union
{
	icd_key_req_hdr_type hdr;
	icd_key_rsp_type icd_key_rsp;
} PACKED DIAG_ICD_KEY_F_rsp_type;

typedef DIAG_ICD_KEY_F_rsp_type*(* icd_key_func_type)(DIAG_ICD_KEY_F_req_type *);

typedef struct
{
	unsigned short cmd_code;
	icd_key_func_type func_ptr;
	unsigned char which_procesor;             // to choose which processor will do act.
}icd_key_user_table_entry_type;

#endif /* LG_DIAG_ICD_KEY_H */
