/*
 * app_owb.h
 *
 *  Created on: 8 ����. 2021 �.
 *      Author: ivanov
 */
#include "..\main\config_pj.h"

#if MAIN_APP_OWB_H_ == 1

#include "C:\Espressif\frameworks\esp-idf-v4.4.2\components\esp_http_server\include\esp_http_server.h"
#include "mime.h"
#include "C:\Espressif\frameworks\esp-idf-v4.4.2\components\app_update\include\esp_ota_ops.h"
#include "ds18b20.h"

#include <string.h>

#include "owb.h"

#define pin_1w_out (10)
#define pin_1w_in (9)
#define GPIO_DS18B20_0       pin_1w_out
#define MAX_DEVICES          max_sensor

#define SAMPLE_PERIOD        (3000)   // milliseconds

#define TERMO_COUNT MAX_DEVICES



extern float readings[MAX_DEVICES];
extern const httpd_uri_t termo_get_api;
extern const httpd_uri_t termo_data_api;
extern const httpd_uri_t np_html_uri_termo;
extern FW_termo_t termo[MAX_DEVICES];
esp_err_t termo_data_cgi_api_handler(httpd_req_t *req);
esp_err_t termo_get_cgi_api_handler(httpd_req_t *req);

void http_var_init_owb (httpd_handle_t server);
void lwip_privmib_init_termo(void);
void app_owb(void *pvParameters);
esp_err_t load_data_termo(void);
esp_err_t save_data_termo(void);
uint8_t load_def_termo(void);
#endif /* MAIN_APP_OWB_H_ */
