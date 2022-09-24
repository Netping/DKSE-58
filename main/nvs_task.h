/*
 * nvs_task.h
 *
 *  Created on: 16 ����. 2021 �.
 *      Author: ivanov
 */

#ifndef COMPONENTS_NVS_TASK_H_
#define COMPONENTS_NVS_TASK_H_

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
///#include <nvs_flash.h>
#include <sys/param.h>
#include "C:\Espressif\frameworks\esp-idf-v4.4.2\components\nvs_flash\include\nvs_flash.h"
#include "esp_netif.h"
#include "esp_eth.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "config_pj.h"
#include "../main/input/input.h"
#include "../main/notify/notify.h"



//enum in_stype_t {
//	WEB,
//	SNMP,
//	RASP,
//	WDT,
//	TERMO,
//	IO,
//	SMTP,
//	SETT,
//	MAIN,
//	SYS
//};


typedef struct
{
	uint8_t  source;
	uint8_t smtp;
	uint8_t log;
	uint8_t syslog;
	uint8_t snmp;
	s32_t OID_out[12];
	uint8_t canal;
}event_struct_t;
enum logs_events_t {
  NO_RUN,
  RESETL,
  UPDATE_FW,
  TEMP_OVER,
 TEMP_NORM,
 TEMP_LOW,
 TEMP_ERR,
  POWER_ON,
  LOAD_DEF_DATA,
  SAVE_DATA_SETT,
  WDT_RES,
  WDT_STOP,
  WDT_REPID,
  IN_RISE,
  IN_FALL,
  OUT_SET,
  OUT_RES,
  OUT_TOL,
  OUT_SET_N,
  OUT_RES_N,
  OUT_TOL_N
};

enum logs_source_t {
  NULLS,
  SYS,
  LOG,
  SETT,
  SMTP,
  NTP,
  SNMP,
  IN,
  OUT,
  TERM,
  UPD,
  NTF
};


typedef struct
{
uint8_t dicr ;//0
//event_struct_t event_cfg;
uint8_t type_event;//1
uint8_t reple_hours;//2
uint8_t reple_minuts;//3
uint8_t reple_seconds;//4
uint8_t day;//5
uint8_t dweek;//6
uint8_t month;//7
uint16_t year;//9-10
uint8_t source;//11/logs_source_t enum //0-SYS/1-log/2-sett/3-smtp/4-ntp/5-snmp/6-input/7-out/8-termo/
uint8_t line;//12
}log_reple_t;

typedef struct
{
 uint16_t CRC16;
 log_reple_t log_reple[200];
} logs_t;

typedef struct
{
uint8_t flag_event;
uint8_t event;
uint8_t source[16];
uint8_t n_slot;
uint32_t fswich_point[16];
char mess[200];
}FW_log_t;

typedef struct
{
 uint8_t on_swich_m;
 uint8_t on_swich_h;
 uint8_t off_swich_m;
 uint8_t off_swich_h;
} day_swich_elem;
typedef struct
{
 uint8_t day;
 uint8_t month;
 uint8_t year;
} data_swich_elem;
typedef struct
{
   uint8_t data_param;

}nvs_flags_t;

typedef struct
{
 day_swich_elem time_data[6];
 uint8_t set_up_day;
} V_D_TIME_type;

typedef struct
{
 data_swich_elem data[10];
 uint8_t restore_day[10];
} V_RD_DATA_type;

typedef struct
{
	uint8_t LOAD_DEF_FLAG;
	uint16_t  V_CRC_APP  ;
	uint16_t  V_CRC_BOOT  ;
	uint8_t V_FW1_VER[4] ;
	uint32_t V_FW1_LEN ;
	uint32_t V_BOOT_VER ;
	uint16_t V_CRC_DATA;
	char V_GEOM_NAME[86];
	uint8_t V_ID_MAC[8];
	char V_Name_dev[86];
	char V_CALL_DATA[86];
	signed char V_NTP_CIRCL;
	uint8_t V_L_TIME;
	uint8_t V_TYPE_OUT;
	uint8_t V_IP_SOURCE[4] ;
	uint8_t V_MASK_SOURCE ;
	uint8_t V_ECO_EN ;


}FW_system_t;
typedef struct
{
	uint8_t V_IP_CONFIG[4] ;
	uint8_t V_IP_MASK[4] ;
	uint8_t V_IP_GET[4] ;
	uint8_t V_DHCP;
	uint8_t V_IP_DNS[4] ;
	uint8_t V_IP_NTP1[4];
	uint8_t V_IP_NTP2[4];
	char N_NTP1[32];
	char N_NTP2[32];
	uint8_t SAMMER;
	uint16_t V_PORT_NTP;

	uint16_t V_HTTP_IP;
	uint8_t V_IP_SYSL[4];
	uint8_t V_IP_SYSL1[4];
	uint8_t V_IP_SYSL2[4];
	char V_N_SYSL[32];
	char V_N_SYSL1[32];
	char V_N_SYSL2[32];
	char N_SLOG[32];
	uint8_t facility;
	uint8_t severity;

}FW_network_t;

typedef struct
{
	uint8_t V_EMAIL_ERR[32];
	char V_NAME_SMTP[64];
	char V_LOGIN_SMTP[32];
	char V_PASSWORD_SMTP[32];
	uint8_t V_FLAG_EN_EMAIL;
	uint8_t V_FLAG_DEF_EMAIL;
	uint16_t V_FLAG_EMAIL_PORT;
	char V_EMAIL_ADDR[32];
	char V_EMAIL_FROM[32];
	char V_EMAIL_TO[32];
	char V_EMAIL_CC1[32];
	char V_EMAIL_CC2[32];
	char V_EMAIL_CC3[32];
}FW_smtp_t;
typedef struct
{
	uint16_t V_SNMP;
	uint16_t V_PORT_SNMP;
	char V_COMMUNITY[16];
    char V_COMMUNITY_WRITE[16];
	uint8_t V_IP_SNMP[32];
	uint8_t V_IP_SNMP_S[32];


	uint8_t V_REFR_TRAP;
}FW_snmp_t;
typedef struct
{
	uint8_t V_N_PING;
	uint8_t V_FLAG_EN_WATCHMAN;
	uint8_t V_EN_WATCHDOG;
	uint8_t V_EN_WATCHDOG_CN_A;
	uint8_t V_EN_WATCHDOG_CN_B;
	uint8_t V_EN_WATCHDOG_CN_C;
	uint8_t V_IP_WDT_ADDR_CN_A[4];
	uint8_t V_IP_WDT_ADDR_CN_B[4];
	uint8_t V_IP_WDT_ADDR_CN_C[4];
	uint16_t V_CT_RES_ALLSTART;
	uint16_t V_T_SEND_PING;
	uint16_t V_TIME_RESEND_PING;
	uint16_t V_MAX_REPID_PING;
	uint16_t V_TIME_RESET_PULSE;
	uint16_t V_PAUSE_RESET_TO_REPID;
	uint16_t V_MAX_RESEND_PACET_RESET;
	uint8_t V_TYPE_LOGIC;
	uint16_t V_N_OUT;


	uint8_t V_EVENT_L;
	uint8_t V_EVENT_SL;
	uint8_t V_EVENT_E;
	uint8_t V_EVENT_S;
	uint8_t V_EVENT_T;

	uint8_t V_RESET_L;
	uint8_t V_RESET_SL;
	uint8_t V_RESET_E;
	uint8_t V_RESET_S;
	uint8_t V_RESET_T;
	uint8_t V_RELOG_E;

	char V_NAME[16];
}FW_wdt_t;
typedef struct
{
	char V_LOGIN[16];
	char V_PASSWORD[16];
	uint16_t V_WEB_PORT;
	char V_ON_MESS[32];
	char V_OFF_MESS[32];
	uint8_t V_FLAG_EN_MAN;
	uint8_t V_FLAG_EN_WEB;

}FW_http_t;
typedef struct
{
uint8_t V_FLAG_EN_RASP;
V_D_TIME_type V_D_TIME[10]; //260
V_RD_DATA_type V_RD_DATA; //78
uint16_t V_IP_PING_TIME;
uint16_t V_TIME_SEND;
uint16_t V_TIME_READ;
uint16_t V_N_RERID;
uint16_t V_TIME_RESET;
uint16_t V_DELAY_PING;
uint8_t V_SOST_RESET;
uint8_t V_SOST_ERR_RASP;
}FW_rasp_t;







typedef struct
{
	FW_system_t sys;
	FW_network_t net;
	FW_smtp_t smtp;
	FW_snmp_t snmp;
	FW_wdt_t wdt[2];
	FW_http_t http;
	FW_rasp_t rasp;
	FW_log_t log;
//	FW_gpio_t gpio;

//uint8_t V_resv[956];
logs_t V_logs_struct;
}FW_data_t;

extern FW_data_t FW_data;
extern nvs_handle_t nvs_data_handle;
extern nvs_flags_t nvs_flags;

void nvs_task(void *pvParameters);
uint8_t load_struct_flash_data (void);
esp_err_t save_data_blok(void);

event_struct_t  event_io,event_wdt,event_termo,event_main;


#endif /* COMPONENTS_NVS_TASK_H_ */
