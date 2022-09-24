
/*
 * notify.h
 *
 *  Created on: 19 θών. 2022 γ.
 *      Author: ivanov
 */

#if MAIN_APP_NOTIF == 1

#include "../main/config_pj.h"



void http_var_init_notify(httpd_handle_t server);
esp_err_t save_data_notify(void);
esp_err_t load_data_notify(void);
void log_notf_save_mess(uint8_t event, uint8_t line);
void load_def_notify(void);
void notify_app (void *pvParameters) ;
void decode_expr (int* out_ex,char* mess,size_t len);
void dec_email(char *in, char *mail1, char *mail2, char *mail3);
uint8_t notife_run(int *in, int val);
#endif
