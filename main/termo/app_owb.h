/*
 * app_owb.h
 *
 *  Created on: 8 но€б. 2021 г.
 *      Author: ivanov
 */
#include "..\main\config_pj.h"

#if MAIN_APP_OWB_H_ == 1

#include "C:\esp-idf-2\components\esp_http_server\include\esp_http_server.h"
#include "mime.h"
#include "C:\esp-idf-2\components\app_update\include\esp_ota_ops.h"
#include "ds18b20.h"

#include <string.h>

#include "owb.h"


#define GPIO_DS18B20_0       (10)
#define MAX_DEVICES          (2)

#define SAMPLE_PERIOD        (10000)   // milliseconds

#define TERMO_COUNT MAX_DEVICES



extern float readings[MAX_DEVICES];
extern const httpd_uri_t termo_get_api;
extern const httpd_uri_t termo_data_api;
extern const httpd_uri_t np_html_uri_termo;
extern FW_termo_t termo[2];
esp_err_t termo_data_cgi_api_handler(httpd_req_t *req);
esp_err_t termo_get_cgi_api_handler(httpd_req_t *req);

void http_var_init_owb (httpd_handle_t server);
void lwip_privmib_init_termo(void);
void app_owb(void *pvParameters);
esp_err_t load_data_termo(void);
esp_err_t save_data_termo(void);
uint8_t load_def_termo(void);
#endif /* MAIN_APP_OWB_H_ */
