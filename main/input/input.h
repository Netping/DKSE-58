/*
 * input.h
 *
 *  Created on: 20 ���. 2022 �.
 *      Author: ivanov
 */
#include "../main/config_pj.h"
#include "C:\Espressif\frameworks\esp-idf-v4.4.2\components\esp_http_server\include\esp_http_server.h"
#ifndef MAIN_INPUT_INPUT_H_
#define MAIN_INPUT_INPUT_H_

#define in_ver 1
#define in_rev 2

#if MAIN_APP_IN_PORT == 1
typedef struct
{
 uint8_t sost_raw;
 uint8_t sost_filtr_old;
 uint8_t sost_filtr;
 uint8_t sost_rise;
 uint8_t sost_fall;
 uint32_t filtr_time;
 uint32_t filtr_count;
 uint32_t count;
 uint32_t semple_count;
 uint8_t event;
 char name[32];
 char 	set_name[32];
 char clr_name[32];

}input_port_t;

enum in_event_t {
  IN_START,
  IN_SETE,
  IN_CLRE,
  IN_SETT,
  IN_ERR
};
extern input_port_t IN_PORT[in_port_n];
void http_var_init_input(httpd_handle_t server);
esp_err_t load_data_input(void);
esp_err_t save_data_input(void);
void load_def_input (void);
void log_in_save_mess(uint8_t event, uint8_t line);
void input_port(void *pvParameters);
void http_var_web_input(httpd_handle_t server);
#endif

#endif /* MAIN_INPUT_INPUT_H_ */
