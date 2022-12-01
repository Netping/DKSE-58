/*
 * input.c
 *
 *
 *
 *  Created on: 20 апр. 2022 г.
 *      Author: ivanov
 */

#include "config_pj.h"
#include "includes_base.h"
#include "../main/input/input.h"
#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "LOGS.h"
#include "../components/mime.h"
#include "http_var.h"
log_reple_t input_reply_gl;

#if MAIN_APP_IN_PORT == 1
#define get_name(x) #x

input_port_t IN_PORT[in_port_n];
uint8_t PORT_I[in_port_n] = { P_I0, P_I1 };

static void char2_to_hex(char *in, uint8_t *out, uint32_t len) {
	//	Bcd_To_Hex((unsigned char *)in, (unsigned char *)out, len);
	for (uint32_t ct = 0; ct < len; ct++) {
		if (in[ct * 2] > 0x46) {
			in[ct * 2] = in[ct * 2] - 0x57;
		} else if (in[ct * 2] > 0x40) {
			in[ct * 2] = in[ct * 2] - 0x37;
		} else if (in[ct * 2] > 0x2f) {
			in[ct * 2] = in[ct * 2] - 0x30;
		} else {
			break;
		}

		if (in[ct * 2 + 1] > 0x46) {
			in[ct * 2 + 1] = in[ct * 2 + 1] - 0x57;
		} else if (in[ct * 2 + 1] > 0x40) {
			in[ct * 2 + 1] = in[ct * 2 + 1] - 0x37;
		} else if (in[ct * 2 + 1] > 0x2f) {
			in[ct * 2 + 1] = in[ct * 2 + 1] - 0x30;
		} else {
			break;
		}
		out[ct] = (in[ct * 2] << 4) | in[ct * 2 + 1];
	}

}
static uint8_t read_mess_smtp(char *in, uint8_t *out) {
	uint8_t len = 0;
	if (in[0] > 0x46) {
		in[0] = in[0] - 0x57;
	} else if (in[0] > 0x40) {
		in[0] = in[0] - 0x37;
	} else if (in[0] > 0x2f) {
		in[0] = in[0] - 0x30;
	} else {
		return 0;
	}
	if (in[1] > 0x46) {
		in[1] = in[1] - 0x57;
	} else if (in[1] > 0x40) {
		in[1] = in[1] - 0x37;
	} else if (in[1] > 0x2f) {
		in[1] = in[1] - 0x30;
	} else {
		return 0;
	}
	len = (in[0] << 4) | in[1];
	char2_to_hex((char*) (in + 2), (uint8_t*) out, len);
	return len;
}

static esp_err_t in_set_post_handler(httpd_req_t *req) {
	esp_err_t err;
	nvs_handle_t my_handle;
	uint8_t len;
//	char buf[1000];
	char buf_temp[256] = { 0 };
	io_set_t data;
	int ret, remaining = req->content_len;
	uint8_t ct;

	httpd_resp_set_status(req, "303 See Other");
	httpd_resp_set_hdr(req, "Location", "/in.html");
	httpd_resp_set_hdr(req, "Cache-Control",
			"no-store, no-cache, must-revalidate");
	httpd_resp_set_type(req, mime_sse);
	httpd_resp_set_hdr(req, "Connection", "Close");

	if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {

	}

	for (ct = 0; ct < (in_port_n); ct++) {
		len = read_mess_smtp((char*) (buf + ct * 224 + 5), (uint8_t*) buf_temp);

		memset(&(IN_PORT[ct].name), 0, 32);
		memcpy(&(IN_PORT[ct].name), (char*) (buf_temp), len);
		memset(buf_temp, 0, 256);

		len = read_mess_smtp((char*) (buf + ct * 224 + 73),
				(uint8_t*) buf_temp);

		memset(&(IN_PORT[ct].set_name), 0, 32);
		memcpy(&(IN_PORT[ct].set_name), (char*) (buf_temp), len);
		memset(buf_temp, 0, 256);

		len = read_mess_smtp((char*) (buf + ct * 224 + 141),
				(uint8_t*) buf_temp);

		memset(&(IN_PORT[ct].clr_name), 0, 32);
		memcpy(&(IN_PORT[ct].clr_name), (char*) (buf_temp), len);
		memset(buf_temp, 0, 256);
	}

	nvs_flags.data_param = 1;

	httpd_resp_send_chunk(req, NULL, 0);
	return ESP_OK;
}
static esp_err_t in_get_cgi_handler(httpd_req_t *req) {
#warning "******** where is no error processing !  *******"
	uint8_t ct;
	httpd_resp_set_hdr(req, "Cache-Control",
			"no-store, no-cache, must-revalidate");
	httpd_resp_set_type(req, mime_sse);
	httpd_resp_set_hdr(req, "Connection", "Close");

	const esp_partition_t *running = esp_ota_get_running_partition();
	esp_app_desc_t app_desc;
	esp_err_t ret = esp_ota_get_partition_description(running, &app_desc);
	if (ret != ESP_OK) {
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
				"Can't read FW version!");
		return ESP_FAIL;
	}
//	page_sost = IO;
//	char buf[2048];
	char buf_temp[256];
	uint16_t gpio_status, ct_s;
	gpio_status = 0;
	memset((uint8_t*) buf, 0, 2048);

	for (ct_s = 0; ct_s < in_port_n; ct_s++) {
		gpio_status = gpio_status | ((IN_PORT[ct_s].sost_filtr & 0x01) << ct_s);
	}

	printf("input sost=%d\n\r",gpio_status);

	if ((req->uri[12] == 'a') & (req->uri[13] == 'd') & (req->uri[14] == 'd')) {
		sprintf(buf,
				"var  packfmt={name:{offs:0,len:34},log1_desc:{offs:34,len:34},log0_desc:{offs:68,len:34},__len:112};");
		sprintf(buf_temp, "var data_status=15;");
		strcat(buf, buf_temp);
		sprintf(buf_temp, " var data=[");
		for (ct = 0; ct < in_port_n - 1; ct++) {
			strcat(buf, buf_temp);
			sprintf(buf_temp,
					"{name:\"%s\",log1_desc:\"%s\",log0_desc:\"%s\",log:%d,pulse_dur:255,pulse_polar:1,colors:82},",
					IN_PORT[ct].name, IN_PORT[ct].set_name,
					IN_PORT[ct].clr_name, IN_PORT[ct].sost_filtr);
		}
		strcat(buf, buf_temp);
		sprintf(buf_temp,
				"{name:\"%s\",log1_desc:\"%s\",log0_desc:\"%s\",log:%d,pulse_dur:255,pulse_polar:1,colors:82}];",
				IN_PORT[in_port_n - 1].name, IN_PORT[in_port_n - 1].set_name,
				IN_PORT[in_port_n - 1].clr_name,
				IN_PORT[in_port_n - 1].sost_filtr);

		strcat(buf, buf_temp);
		sprintf(buf_temp, "var devname='%s';", FW_data.sys.V_Name_dev);
		strcat(buf, buf_temp);
		const esp_partition_t *running = esp_ota_get_running_partition();
		esp_ota_get_partition_description(running, &app_desc);
		sprintf(buf_temp, "var fwver='v%.31s';", app_desc.version);
		strcat(buf, buf_temp);
		sprintf(buf_temp, "var hwver=%d;", hw_config);
		strcat(buf, buf_temp);
		sprintf(buf_temp, "var sys_name='%s';", FW_data.sys.V_Name_dev);
		strcat(buf, buf_temp);
		sprintf(buf_temp, "var sys_location='%s';", FW_data.sys.V_GEOM_NAME);
		strcat(buf, buf_temp);
		sprintf(buf_temp, "var hwmodel=%d;", 6);
		strcat(buf, buf_temp);
		sprintf(buf_temp, "var data_status='%d';", gpio_status);
		strcat(buf, buf_temp);
		gpio_status = MENU_CONFG;
		sprintf(buf_temp, "var menu_data='%d';", gpio_status);
		strcat(buf, buf_temp);
		printf(buf_temp, " pack(packfmt, data);\n");
		strcat(buf, buf_temp);
	} else {
		sprintf(buf_temp,
				"<pre>\nretry: 2000\n\nevent: in_state\ndata: %d\n\n</pre>",
				gpio_status);
		strcat(buf, buf_temp);

	}

//	sprintf(buf, " retry: 2000\n");
//	strcat(buf, buf_temp);

//	sprintf(buf_temp, "\n event: in_state\n""data: %d%d\n\n",
//				IN_PORT[0].sost_filtr,
//				IN_PORT[1].sost_filtr);
//	strcat(buf, buf_temp);
//
//	sprintf(buf_temp, "event: sse_ping:\n""data: -\n\n");
//	strcat(buf, buf_temp);

//
//	sprintf(buf, " retry: 2000;\n");
//	strcat(buf, buf_temp);
//
//	sprintf(buf_temp, " event: io_state\n" "data: %d%d\n\n",
//			IN_PORT[0].sost_filtr,
//			IN_PORT[1].sost_filtr);
//	strcat(buf, buf_temp);

	httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

static esp_err_t in_web1_handler(httpd_req_t *req) {
#warning "******** where is no error processing !  *******"
	uint8_t ct;
	httpd_resp_set_hdr(req, "Cache-Control",
			"no-store, no-cache, must-revalidate");
	httpd_resp_set_type(req, mime_text);
	httpd_resp_set_hdr(req, "Connection", "Close");

//	char buf[2048];
	char buf_temp[256];
	uint16_t gpio_status, ct_s;
	gpio_status = 0;
	memset((uint8_t*) buf, 0, 2048);

	printf("\n\rin %s  len=%d \n\r", req->uri, strlen(req->uri));
	memcpy(buf_temp, req->uri, 10);
	printf("\n\rout %s  len=%d \n\r", buf_temp, strlen(buf_temp));
	if ((strcmp(buf_temp, "/in.cgi?in") == 0)
			&& (req->uri[strlen(req->uri) + 1] == 0)) {
		printf("\n\rGood web hook\n\r");
		memset((uint8_t*) buf_temp, 0, 256);
		uint8_t fault=1;
		for (ct_s = 1; ct_s < in_port_n+1; ct_s++) {

			sprintf(buf_temp, "%d", ct_s);
//			if(((req->uri[strlen(req->uri) - 1])>0x30)||(((req->uri[strlen(req->uri) - 1])==0x30)&&((req->uri[strlen(req->uri) - 2])>0x30)))
//			{
			if ((ct_s == (req->uri[strlen(req->uri) - 1] - 0x30))
					&& ((strlen(req->uri) - 10) == 1)) {
				sprintf(buf, "in_result('ok', -1, %d, %d)",
						IN_PORT[ct_s-1].sost_filtr, IN_PORT[ct_s-1].count);
				printf("\n\rhook %d %s\n\r", ct_s, buf);
				fault=0;
			}

			if (((buf_temp[0]-0x30) == (req->uri[strlen(req->uri) - 2]) - 0x30)
					&& ((buf_temp[1]-0x30) == (req->uri[strlen(req->uri) - 1]) - 0x30)
					&& ((strlen(req->uri) - 10) == 2)) {
				sprintf(buf, "in_result('ok', -1, %d, %d)",
						IN_PORT[ct_s-1].sost_filtr, IN_PORT[ct_s-1].count);
				printf("\n\rhook %d %s\n\r", ct_s, buf);
				fault=0;
			}
	//	}
		}
		if (fault==1)
			{
			printf("\n\rFall  web hook\n\r");
			sprintf(buf, "in_result('error')");
			}

	}
	else {
		printf("\n\rFall  web hook\n\r");
		sprintf(buf, "in_result('error')");
	}

	httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

static esp_err_t in_web2_handler(httpd_req_t *req) {
#warning "******** where is no error processing !  *******"
	uint8_t ct;
	httpd_resp_set_hdr(req, "Cache-Control",
			"no-store, no-cache, must-revalidate");
	httpd_resp_set_type(req, mime_text);
	httpd_resp_set_hdr(req, "Connection", "Close");

//	char buf[2048];
	char buf_temp[256];
	uint16_t gpio_status, ct_s;
	gpio_status = 0;
	memset((uint8_t*) buf, 0, 2048);

	printf("\n\rin %s  len=%d \n\r", req->uri, strlen(req->uri));
	memcpy(buf_temp, req->uri, strlen(req->uri));
	printf("\n\rcopy %s  len=%d \n\r", buf_temp, strlen(req->uri));
	if ((buf_temp[9]=='?')&&(buf_temp[10]=='i')&&(buf_temp[11]=='n'))
		 {


		char data[in_port_n]={0};
		memset((char*) data, 0, in_port_n);
		for (ct_s = 0; ct_s<in_port_n; ct_s++)
		{
		//data=data<<1;
		if (IN_PORT[ct_s].sost_filtr==0)
			{

			 data[ct_s]='0';
			}
		else
			{
			 data[ct_s]='1';
			}

		}
		printf("\n\rGood data=%s\n\r",data);

		memset((uint8_t*) buf, 0, 2048);
		sprintf(buf, "in_result('ok', %s)",data);




    	}
	else {
		printf("\n\rFall  web hook\n\r");
		sprintf(buf, "in_result('error')");
	}
	httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

static const httpd_uri_t in_web1 = { .uri = "/in.cgi", .method = HTTP_GET,
		.handler = in_web1_handler, .user_ctx = 0 };

static const httpd_uri_t in_web2 = { .uri = "/in=n.cgi", .method = HTTP_GET,
		.handler = in_web2_handler, .user_ctx = 0 };

static const httpd_uri_t in_set = { .uri = "/in_set.cgi", .method = HTTP_POST,
		.handler = in_set_post_handler, .user_ctx = NULL };

static const httpd_uri_t in_get_cgi = { .uri = "/in_get.cgi",
		.method = HTTP_GET, .handler = in_get_cgi_handler, .user_ctx = 0 };

void load_def_input(void) {
	uint8_t ct;
	uint8_t name_line[32];
	for (ct = 0; ct < in_port_n; ct++) {
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "Линия%d", ct);
		memset((uint8_t*) &IN_PORT[ct].name, 0, 32);
		memcpy((uint8_t*) &IN_PORT[ct].name, (uint8_t*) name_line,
				sizeof(name_line));
		IN_PORT[ct].filtr_time = 500;

		sprintf((char*) name_line, "Включено");
		memset((uint8_t*) &IN_PORT[ct].set_name, 0, 32);
		memcpy((uint8_t*) &IN_PORT[ct].set_name, (uint8_t*) name_line,
				sizeof(name_line));

		sprintf((char*) name_line, "Выключено");
		memset((uint8_t*) &IN_PORT[ct].clr_name, 0, 32);
		memcpy((uint8_t*) &IN_PORT[ct].clr_name, (uint8_t*) name_line,
				sizeof(name_line));
		IN_PORT[ct].filtr_time = 10;

	}
}
esp_err_t load_data_input(void) {
	esp_err_t err = 0;
	uint8_t ct;
	uint8_t name_line[32];
	size_t lens = 16;
	for (ct = 0; ct < in_port_n; ct++) {
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "in_t%d", ct);
		err =  nvs_get_u16(nvs_data_handle, (char*) name_line,
						&(IN_PORT[ct].filtr_time));

		if (err != ESP_OK) {
							ESP_LOGE("IN_READ", "Error %X read data to flash-IN_PORT[%d].filtr_time", err, ct);
						}

		printf("Name load%d=%s\n\r",ct,IN_PORT[ct].name);

		lens = 16;
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "in_name_%d", ct);
		err =  nvs_get_blob(nvs_data_handle, (char*) name_line,
						&(IN_PORT[ct].name), &lens);

		if (err != ESP_OK) {
									ESP_LOGE("IN_READ", "Error %X read data to flash-IN_PORT[%d].name", err, ct);
								}

		lens = 16;


		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "in_set_%d", ct);

		err = nvs_get_blob(nvs_data_handle, (char*) name_line,
						&(IN_PORT[ct].set_name), &lens);
		if (err != ESP_OK) {
											ESP_LOGE("IN_READ", "Error %X read data to flash-IN_PORT[%d].set_name", err, ct);
										}

		lens = 16;

		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "in_clr_%d", ct);
		err =  nvs_get_blob(nvs_data_handle, (char*) name_line,
						&(IN_PORT[ct].clr_name), &lens);
		if (err != ESP_OK) {
													ESP_LOGE("IN_READ", "Error %X read data to flash-IN_PORT[%d].clr_name", err, ct);
												}
	}
	return err;
}

esp_err_t save_data_input(void) {
	esp_err_t err = 0;
	uint8_t ct;
	uint8_t name_line[32];

	for (ct = 0; ct < in_port_n; ct++) {
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "in_t%d", ct);
		err = err
				| nvs_set_u16(nvs_data_handle, (char*) name_line,
						IN_PORT[ct].filtr_time);

		printf("Name save%d=%s\n\r",ct,IN_PORT[ct].name);


		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "in_name_%d", ct);
		err = err
				| nvs_set_blob(nvs_data_handle, (char*) name_line,
						&(IN_PORT[ct].name), 16);

		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "in_set_%d", ct);
		err = err
				| nvs_set_blob(nvs_data_handle, (char*) name_line,
						&(IN_PORT[ct].set_name), 16);

		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "in_clr_%d", ct);
		err = err
				| nvs_set_blob(nvs_data_handle, (char*) name_line,
						&(IN_PORT[ct].clr_name), 16);
		if (err != ESP_OK) {
							ESP_LOGE("IN_SAVE", "Error %X save data to flash %d", err, ct);
						}

	}
	return err;
}
void read_in(input_port_t *inpin, uint8_t pin) {
	inpin->sost_raw = pin;
	if (inpin->semple_count < inpin->filtr_time) {
		inpin->semple_count++;
		if (inpin->sost_raw != 0) {
			inpin->filtr_count++;
		}
	} else {
		inpin->semple_count = 0;
		inpin->sost_filtr_old = inpin->sost_filtr;
		if (inpin->filtr_count < (inpin->filtr_time / 2)) {
			inpin->sost_filtr = 0;
		} else {
			inpin->sost_filtr = 1;
		}
		inpin->filtr_count = 0;
		while (inpin->event != 0) {
			vTaskDelay(30 / portTICK_PERIOD_MS);
		}

		if (inpin->sost_filtr_old > inpin->sost_filtr) {
			inpin->sost_fall = 1;
			inpin->sost_rise = 0;
			inpin->event = 1;
		}
		if (inpin->sost_filtr_old < inpin->sost_filtr) {
			inpin->sost_fall = 0;
			inpin->sost_rise = 1;
			inpin->count++;
			inpin->event = 1;
		}
		if (inpin->event == 0) {
			inpin->sost_fall = 0;
			inpin->sost_rise = 0;
		}
	}
}
//enum in_event_t {
//  IN_START,
//  IN_SETE,
//  IN_CLRE,
//  IN_SETT,
//  IN_ERR
//};
void log_swich_in(char *out, log_reple_t *input_reply) {
	char out_small[200] = { 0 };
	sprintf(out_small, "%02d.%02d.%d  %02d:%02d:%02d [digital-inputs] ",
			input_reply->day, input_reply->month, input_reply->year,
			input_reply->reple_hours, input_reply->reple_minuts,
			input_reply->reple_seconds);
	strcat(out, out_small);
	switch (input_reply->type_event) {

	case IN_START:
		sprintf(out_small, "Старт модуля входов v%d.%d\n\r",in_ver,in_rev);
		break;
	case IN_CLRE:
		sprintf(out_small, "%s %s\n\r", IN_PORT[input_reply->line].name,
				IN_PORT[input_reply->line].clr_name);
		break;
	case IN_SETE:
		sprintf(out_small, "%s %s\n\r", IN_PORT[input_reply->line].name,
				IN_PORT[input_reply->line].set_name);

		break;
	case IN_SETT:
		sprintf(out_small, "%s %s", IN_PORT[input_reply->line].name,
				"Изменение настроек");
		break;
	case IN_ERR:
		sprintf(out_small, "%s %s", IN_PORT[input_reply->line].name,
				"Ошибка модуля");
		break;
	default:
		sprintf(out_small, "%s %s", IN_PORT[input_reply->line].name,
				"Ошибка журнала");
	}
	strcat(out, out_small);
}

void log_start_in(void) {
	xSemaphoreTake(flag_global_save_log, (TickType_t) 1000);
	{
		for (uint8_t ct_l = 0; ct_l < 16; ct_l++) {
			if (FW_data.log.source[ct_l] == NULLS) {
				FW_data.log.source[ct_l] = IN;

				//	memcpy(&(FW_data.log.header[ct_l][0]), (char*)"[digital-inputs]",sizeof("[digital-inputs]"));
				FW_data.log.fswich_point[ct_l] = log_swich_in;
				ct_l = 17;
			}
		}
	}
	xSemaphoreGive(flag_global_save_log);
	log_in_save_mess(IN_START, 0);
}

void log_in_save_mess(uint8_t event, uint8_t line) {
	xSemaphoreTake(flag_global_save_log, (TickType_t) 1000);
	{
		log_reple_t save_reply;
		GET_reple(&save_reply);
		save_reply.line = line;
		save_reply.type_event = event;
		save_reply.source = IN;
		save_reple_log(save_reply);

	}
	xSemaphoreGive(flag_global_save_log);
}
void log_in_event_cntl(input_port_t *inpin, uint8_t line) {
	if (inpin->event == 1) {
		if (inpin->sost_rise == 1) {
			log_in_save_mess(IN_SETE, line);
			inpin->sost_rise = 0;
		}
		if (inpin->sost_fall == 1) {
			log_in_save_mess(IN_CLRE, line);
			inpin->sost_fall = 0;
		}
		inpin->event = 0;
	}
}
void input_port(void *pvParameters) {
	for (uint8_t ct_i = 0; ct_i < in_port_n; ct_i++) {
		gpio_set_direction(PORT_I[ct_i], GPIO_MODE_INPUT);
	}
	log_start_in();

//	FW_data.gpio.IN_PORT[0].filtr_time = 2;
//	FW_data.gpio.IN_PORT[1].filtr_time = 2;
//	FW_data.gpio.OUT_PORT[0].input_str.filtr_time=2;
//	FW_data.gpio.OUT_PORT[1].input_str.filtr_time=2;

	while (1) {

		for (uint8_t ct_i = 0; ct_i < in_port_n; ct_i++) {
			read_in(&(IN_PORT[ct_i]), gpio_get_level(PORT_I[ct_i]));
			log_in_event_cntl(&(IN_PORT[ct_i]), ct_i);
		}

		vTaskDelay(10 / portTICK_PERIOD_MS);
	}

}
void http_var_web_input(httpd_handle_t server) {
	httpd_register_uri_handler(server, &in_web1);
	httpd_register_uri_handler(server, &in_web2);
}

void http_var_init_input(httpd_handle_t server) {
	http_var_web_input(server);
	httpd_register_uri_handler(server, &in_set);
	httpd_register_uri_handler(server, &in_get_cgi);
}

#endif
