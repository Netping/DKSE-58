/*
 * output.h
 *
 *  Created on: 20 ���. 2022 �.
 *      Author: ivanov
 */
#include "../main/config_pj.h"
#include "C:\Espressif\frameworks\esp-idf-v4.4.2\components\esp_http_server\include\esp_http_server.h"
#ifndef MAIN_OUTPUT_OUTPUT_H_
#define MAIN_OUTPUT_OUTPUT_H_
#if MAIN_APP_OUT_PORT == 1
typedef struct
{
	uint8_t event; // 0-no 0-clr 1-set 2-pulse 3-rise 4-fall
	uint8_t type_logic; //0-norm 1-invert 3-pulse
	uint8_t sost; //1-set 0-clr
	uint8_t old_sost;
	uint8_t realtime;
	uint16_t delay;
	uint8_t aflag;
	uint32_t count;
	SemaphoreHandle_t S_gpio_port;
	char name[32];
	char set_name[32];
	char clr_name[32];
	uint8_t polar_pulse;
	uint8_t ALL_EVENT;





	uint8_t RISE_L[out_port_n];
	uint8_t RISE_SL[out_port_n];
	uint8_t RISE_E[out_port_n];
	uint8_t RISE_SM[out_port_n];
	uint8_t RISE_SN[out_port_n];
	uint8_t FALL_L[out_port_n];
	uint8_t FALL_SL[out_port_n];
	uint8_t FALL_E[out_port_n];
	uint8_t FALL_SM[out_port_n];
    uint8_t FALL_SN[out_port_n];
    uint8_t CIKL_E[out_port_n];
    uint8_t SET_COLOR[out_port_n];
    uint8_t CLR_COLOR[out_port_n];
    char mess_low[out_port_n][16];
    char mess_hi[out_port_n][16];
    uint8_t reactiv[out_port_n];
    uint8_t cicle_t[out_port_n];

}output_port_t;
enum out_event_t {
  OUT_START,
  OUT_SETE,
  OUT_CLRE,
  OUT_SETT,
  OUT_ERR
};
void output_port(void *pvParameters);
void http_var_init_out(httpd_handle_t server);
void set_outport(uint8_t index, uint8_t sost);
esp_err_t save_data_output(void);
void log_out_save_mess(uint8_t event, uint8_t line);
esp_err_t load_data_output(void);
void load_def_output(void);
extern output_port_t OUT_PORT[out_port_n];
extern uint8_t  polar_snmp[out_port_n];
#endif
#endif /* MAIN_OUTPUT_OUTPUT_H_ */
