#include <stdint.h>
//#include "flash_if.h"
#include "nvs_task.h"
#define max_log_mess 50
void save_reple_log (log_reple_t reple2);
uint8_t logs_read (uint16_t n_mess,char* mess);
void GET_reple (log_reple_t* reple);
//void form_reple_to_save (event_struct_t cfg);
void log_sett_save_mess(uint8_t event);
void log_log_save_mess(uint8_t event) ;
void save_reple_log(log_reple_t reple2);
void decode_reple (char* out,log_reple_t* reple);
void form_reple_to_smtp (event_struct_t cfg);
void log_task(void *pvParameters);
void swich_mess_event(log_reple_t* reply_sw, char *mess);
void swich_mess_event_en(log_reple_t* reply_sw, char *mess);
void log_update_save_mess(uint8_t event);
void log_start_update(void);

enum update_event_t {
  UPD_START,
  UPD_GOOD,
  UPD_BAD,
  UPD_ERR
};
enum sett_event_t {
  SETT_START,
  SETT_EDIT,
  SETT_NDHCP,
  SETT_EDHCP,
  SETT_DNS,
  SETT_ERR,
  SETT_DNS_GETE
};
enum log_event_t {
  LOG_START,
  LOG_ERR,
  SLOG_ERR
};


extern log_reple_t reple_to_save;
extern log_reple_t reple_to_email;
//extern RTC_DateTypeDef dates;
//extern RTC_TimeTypeDef times;
extern struct tm timeinfo;
extern SemaphoreHandle_t flag_global_save_log;
extern uint32_t timeup;
extern time_t now;

