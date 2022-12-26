/*
 * output.c
 *
 *  Created on: 20 пїЅпїЅпїЅ. 2022 пїЅ.
 *      Author: ivanov
 */
#include "config_pj.h"
#if MAIN_APP_OUT_PORT == 1
#include "includes_base.h"
#include "private_mib.h"
#include "../main/output/output.h"
#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "esp_http_server.h"
#include "../components/mime.h"
#include "LOGS.h"
#include "http_var.h"

#define get_name(x) #x

uint8_t PORT_O[out_port_n] = { P_O0, P_O1 };

output_port_t OUT_PORT[out_port_n];

uint8_t polar_snmp[out_port_n];

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
void set_outport(uint8_t index, uint8_t sost) {

		if (sost == 1) {
			OUT_PORT[index].sost = 0;
			OUT_PORT[index].type_logic = 0;
		}else if (sost == 2) {
			OUT_PORT[index].sost = 1;
			OUT_PORT[index].type_logic = 0;
		}else if (sost == 3) {
			if (OUT_PORT[index].sost==1)
			{
			OUT_PORT[index].sost = 0;
			OUT_PORT[index].type_logic = 0;
			}
			else
			{
			   OUT_PORT[index].sost = 1;
			   OUT_PORT[index].type_logic = 0;
			}
		} else {
//		if (OUT_PORT[index].realtime == 0) {
		OUT_PORT[index].sost = 1;
		OUT_PORT[index].type_logic = 3;
//		} else {
//			OUT_PORT[index].sost = 0;
//			OUT_PORT[index].type_logic = 3;
		}


	OUT_PORT[index].aflag = 1;
}
static esp_err_t out_get_cgi_handler(httpd_req_t *req) {
#warning "******** where is no error processing !  *******"
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
	uint8_t ct;
//	page_sost = IO;
//	char buf[2048];
	char buf_temp[256];
	uint16_t gpio_status, ct_s;
	gpio_status = 0;
	memset((uint8_t*) buf, 0, 2048);

	for (ct_s = 0; ct_s < out_port_n; ct_s++) {
		gpio_status = gpio_status | ((OUT_PORT[ct_s].realtime & 0x01) << ct_s);
	}
//	printf("output sost=%d\n\r", gpio_status);

	if ((req->uri[13] == 'a') & (req->uri[14] == 'd') & (req->uri[15] == 'd')) {
		sprintf(buf,
				"var  packfmt={name:{offs:0,len:34},log1_desc:{offs:34,len:34},log0_desc:{offs:68,len:34},pulse_dur:{offs:102,len:2},pulse_polar:{offs:104,len:1},__len:105};\n");

		sprintf(buf_temp, "var data_status=15;");
		strcat(buf, buf_temp);
		sprintf(buf_temp, " var data=[");
		uint8_t color_d = 82;

		for (ct = 0; ct < out_port_n - 1; ct++) {
			strcat(buf, buf_temp);
//		if (OUT_PORT[ct].realtime == 0) {
//			color_d = 81;
//		} else {
			color_d = 82;
//		}
			sprintf(buf_temp,
					"{name:\"%s\",log1_desc:\"%s\",log0_desc:\"%s\",pulse_dur:%d,pulse_polar:%d,colors:%d},\n",
					OUT_PORT[ct].name, OUT_PORT[ct].set_name,
					OUT_PORT[ct].clr_name, OUT_PORT[ct].delay,
					OUT_PORT[ct].polar_pulse, color_d);
		}
		strcat(buf, buf_temp);
//	if (OUT_PORT[ct].realtime == 0) {
//		color_d = 81;
//	} else {
		color_d = 82;
//	}
		sprintf(buf_temp,
				"{name:\"%s\",log1_desc:\"%s\",log0_desc:\"%s\",pulse_dur:%d,pulse_polar:%d,colors:%d}];\n",
				OUT_PORT[out_port_n - 1].name,
				OUT_PORT[out_port_n - 1].set_name,
				OUT_PORT[out_port_n - 1].clr_name, OUT_PORT[ct].delay,
				OUT_PORT[ct].polar_pulse, color_d);

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
	} else {
		sprintf(buf_temp,
				"<pre>\nretry: 500\n\nevent: out_status\ndata: %d\n\n<\pre>",
				gpio_status);
		strcat(buf, buf_temp);

	}

	httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

static esp_err_t out_switch_post_handler(httpd_req_t *req) {
	esp_err_t err;
	nvs_handle_t my_handle;
	uint8_t len;
//	char buf[1000];
	char buf_temp[256] = { 0 };
	io_set_t data;
	int ret, remaining = req->content_len;
	uint8_t ct;

	httpd_resp_set_status(req, "303 See Other");
	httpd_resp_set_hdr(req, "Location", "/out.html");
	httpd_resp_set_hdr(req, "Cache-Control",
			"no-store, no-cache, must-revalidate");
	httpd_resp_set_type(req, mime_js);
	httpd_resp_set_hdr(req, "Connection", "Close");

	if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {

	}
	char2_to_hex((char*) (buf + 5), (uint8_t*) buf_temp, 5);
	printf("output%d sost=%d\n\r",buf_temp[0],buf_temp[1]);
	set_outport((uint8_t) buf_temp[0], (uint8_t) buf_temp[1]);

//	nvs_flags.data_param = 1;

	httpd_resp_send_chunk(req, NULL, 0);
	return ESP_OK;
}

static esp_err_t out_set_post_handler(httpd_req_t *req) {
	esp_err_t err;
	nvs_handle_t my_handle;
	uint8_t len;
//	char buf[1000];
	char buf_temp[256] = { 0 };
	io_set_t data;
	int ret, remaining = req->content_len;
	uint8_t ct;

	httpd_resp_set_status(req, "303 See Other");
	httpd_resp_set_hdr(req, "Location", "\out.html");
	httpd_resp_set_hdr(req, "Cache-Control",
			"no-store, no-cache, must-revalidate");
	httpd_resp_set_type(req, mime_sse);
	httpd_resp_set_hdr(req, "Connection", "Close");

	if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {

	}

//	for (ct = 0; ct < (in_port_n); ct++) {
//		len = read_mess_smtp((char*) (buf + ct * 224 + 5), (uint8_t*) buf_temp);
//
//		memset(&(IN_PORT[ct].name), 0, 32);
//		memcpy(&(IN_PORT[ct].name), (char*) (buf_temp), len);
//		memset(buf_temp, 0, 256);
//
//		len = read_mess_smtp((char*) (buf + ct * 224 + 73), (uint8_t*) buf_temp);
//
//		memset(&(IN_PORT[ct].set_name), 0, 32);
//		memcpy(&(IN_PORT[ct].set_name), (char*) (buf_temp), len);
//		memset(buf_temp, 0, 256);
//
//		len = read_mess_smtp((char*) (buf + ct * 224 + 141), (uint8_t*) buf_temp);
//
//				memset(&(IN_PORT[ct].clr_name), 0, 32);
//				memcpy(&(IN_PORT[ct].clr_name), (char*) (buf_temp), len);
//				memset(buf_temp, 0, 256);
//		}
//
	printf("output set post \n\r");
	for (ct = 0; ct < out_port_n; ct++) {

		len = read_mess_smtp((char*) (buf + ct * 210 + 5), (uint8_t*) buf_temp);

		memset(&(OUT_PORT[ct].name[0]), 0, 32);
		memcpy(&(OUT_PORT[ct].name[0]), (char*) (buf_temp), len);
		memset(buf_temp, 0, 256);

		len = read_mess_smtp((char*) (buf + ct * 210 + 73),
				(uint8_t*) buf_temp);
		memset(OUT_PORT[ct].set_name, 0, 32);
		memcpy(OUT_PORT[ct].set_name, (char*) (buf_temp), len);
		memset(buf_temp, 0, 256);

		len = read_mess_smtp((char*) (buf + ct * 210 + 141),
				(uint8_t*) buf_temp);
		memset(OUT_PORT[ct].clr_name, 0, 32);
		memcpy(OUT_PORT[ct].clr_name, (char*) (buf_temp), len);
		memset(buf_temp, 0, 256);

//		char2_to_hex((char*) (buf + 84 + ct * 96), (uint8_t*) buf_temp, 5);

//			OUT_PORT[ct].input_str.filtr_time = ((buf[ct * 96 + 88]
//					<< 8) | (buf[ct * 96 + 85] << 4) | (buf[ct * 96 + 86]));
//		OUT_PORT[ct].sost = buf[92 + ct * 96];
//		OUT_PORT[ct].aflag = 1;

		char2_to_hex((char*) (buf + 209 + ct * 210), (uint8_t*) buf_temp, 2);
		OUT_PORT[ct].delay = ((buf_temp[1] << 8) | (buf_temp[0]));

		char2_to_hex((char*) (buf + 213 + ct * 210), (uint8_t*) buf_temp, 1);
		OUT_PORT[ct].polar_pulse = buf_temp[0];

//				* 100;

		if (OUT_PORT[ct].sost > OUT_PORT[ct].old_sost) {
			OUT_PORT[ct].old_sost = OUT_PORT[ct].sost;
			//FW_data.gpio.OUT_PORT[ct].event = WEB_OUT_PORT0_SET + ct;
			reple_to_save.type_event = OUT_SET;
			//	reple_to_save.event_cfg.canal = ct;
			//	reple_to_save.event_cfg.source = WEB;
			//	reple_to_save.dicr = 1;

		} else if (OUT_PORT[ct].sost < OUT_PORT[ct].old_sost) {
			OUT_PORT[ct].old_sost = OUT_PORT[ct].sost;
			//reple_to_save.type_event = OUT_SET;
			//	FW_data.gpio.OUT_PORT[ct].event = WEB_OUT_PORT0_RES + ct;
			//	reple_to_save.type_event = OUT_RES;
			//	reple_to_save.event_cfg.canal = ct;
			//	reple_to_save.event_cfg.source = WEB;
			//	reple_to_save.dicr = 1;

		}

	}

	nvs_flags.data_param = 1;

	httpd_resp_send_chunk(req, NULL, 0);
	return ESP_OK;
}
static esp_err_t out_set_pulse_post_handler(httpd_req_t *req) {
	esp_err_t err;
	nvs_handle_t my_handle;
	uint8_t len;
//	char buf[1000];
	char buf_temp[256] = { 0 };
	io_set_t data;
	int ret, remaining = req->content_len;
	uint8_t ct;

	httpd_resp_set_status(req, "303 See Other");
	httpd_resp_set_hdr(req, "Location", "\io.html");
	httpd_resp_set_hdr(req, "Cache-Control",
			"no-store, no-cache, must-revalidate");
	httpd_resp_set_type(req, mime_sse);
	httpd_resp_set_hdr(req, "Connection", "Close");

	if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {

	}
	printf("output set pulse post\n\r");
//	if (buf[6] == 0x31) {
	char2_to_hex((char*) (buf + 7), (uint8_t*) buf_temp, 2);
	OUT_PORT[(buf[6] - 0x30)].delay = ((buf_temp[1] << 8) | (buf_temp[0]));
	OUT_PORT[(buf[6] - 0x30)].event = OUT_TOL;
	//	reple_to_save.type_event = OUT_TOL;
	//	reple_to_save.event_cfg.canal = 1;
	//	reple_to_save.event_cfg.source = WEB;
	//	reple_to_save.dicr = 1;
	set_outport((buf[6] - 0x30), 4);
//		OUT_PORT[1].sost = 1;
//		OUT_PORT[1].type_logic = 3;
//		OUT_PORT[1].aflag = 1;

//	} else {
//		char2_to_hex((char*) (buf + 7), (uint8_t*) buf_temp, 2);
//		OUT_PORT[0].delay = ((buf_temp[1] << 8) | (buf_temp[0]));
//		OUT_PORT[0].event = OUT_TOL;
//		//	reple_to_save.type_event = OUT_TOL;
//		//	reple_to_save.event_cfg.canal = 0;
//		//	reple_to_save.event_cfg.source = WEB;
//		//	reple_to_save.dicr = 1;
//		set_outport(0, 3);
////		OUT_PORT[0].sost = 1;
////		OUT_PORT[0].type_logic = 3;
////		OUT_PORT[0].aflag = 1;
//	}

	httpd_resp_send_chunk(req, NULL, 0);
	return ESP_OK;
}

static esp_err_t out_web1_handler(httpd_req_t *req) {
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
	memset((uint8_t*) buf_temp, 0, 256);

	printf("\n\rout %s  len=%d \n\r", req->uri, strlen(req->uri));
	memcpy(buf_temp, req->uri, 12);
	printf("\n\rout %s  len=%d \n\r", buf_temp, strlen(buf_temp));
	if ((strcmp(buf_temp, "/out.cgi?out") == 0)
			&& (req->uri[strlen(req->uri) + 1] == 0)) {
		printf("\n\rGood web hook\n\r");
		memset((uint8_t*) buf_temp, 0, 256);
		uint8_t fault = 1;
		for (ct_s = 0; ct_s < out_port_n; ct_s++) {

			sprintf(buf_temp, "%d", ct_s);
			if(req->uri[12]>0x30)
			{
			if ((ct_s == (req->uri[12] - 0x31)))
			/*&& ((strlen(req->uri) - 12) == 3))*/{

				if (req->uri[15] != ',') {
					if (req->uri[14] != 'f') {
						sprintf(buf, "out_result('ok')");
						printf("\n\rct_s %d %d\n\r", (req->uri[12] - 0x31),
								(req->uri[14] - 0x30));

						set_outport((uint8_t) (req->uri[12] - 0x31),
								(uint8_t) 1 + (req->uri[14] - 0x30));

						printf("\n\rhook set %d %s\n\r", ct_s, req->uri);
						fault = 0;
					}
					if (req->uri[14] == 'f') {
						sprintf(buf, "out_result('ok')");
						printf("\n\rct_s %d %d\n\r", (req->uri[12] - 0x31),
								(req->uri[14] - 0x30));
						if (OUT_PORT[(uint8_t) (req->uri[12] - 0x31)].realtime
								== 0) {
							set_outport((uint8_t) (req->uri[12] - 0x31), 2);
						} else {
							set_outport((uint8_t) (req->uri[12] - 0x31), 1);

						}

						printf("\n\rhook reload%d %s\n\r", ct_s, req->uri);
						fault = 0;
					}

				} else {

					if (req->uri[14] == '1') {
						printf("\n\r17_18 %x %x \n\r", req->uri[17],
								req->uri[18]);
						if (req->uri[17] == 0) {

							OUT_PORT[(uint8_t) (req->uri[12] - 0x31)].delay =
									(req->uri[16] - 0x30) * 1000;
							printf("\n\rhook pulse3 %d %d %c %c\n\r", ct_s,
									OUT_PORT[(uint8_t) (req->uri[12] - 0x31)].delay,
									req->uri[16], req->uri[17]);
							set_outport((uint8_t) (req->uri[12] - 0x31), 3);
							sprintf(buf, "out_result('ok')");
							fault = 0;
						} else {
							if (req->uri[18] == 0) {
								OUT_PORT[(uint8_t) (req->uri[12] - 0x31)].delay =
										((req->uri[16] - 0x30) * 10
												+ (req->uri[17] - 0x30)) * 1000;
								printf("\n\rhook pulse3 %d %d %c %c\n\r", ct_s,
										OUT_PORT[(uint8_t) (req->uri[12] - 0x31)].delay,
										req->uri[16], req->uri[17]);
								set_outport((uint8_t) (req->uri[12] - 0x31), 3);
								sprintf(buf, "out_result('ok')");
								fault = 0;
							}
						}
					} else {
						if (req->uri[17] == 0) {
							OUT_PORT[(uint8_t) (req->uri[12] - 0x31)].delay =
									(req->uri[16] - 0x30) * 1000;
							printf("\n\rhook pulse4 %d %d %c %c\n\r", ct_s,
									OUT_PORT[(uint8_t) (req->uri[12] - 0x31)].delay,
									req->uri[16], req->uri[17]);
							set_outport((uint8_t) (req->uri[12] - 0x31), 4);
							sprintf(buf, "out_result('ok')");
							fault = 0;
						} else {
							if (req->uri[18] == 0) {
								OUT_PORT[(uint8_t) (req->uri[12] - 0x31)].delay =
										((req->uri[16] - 0x30) * 10
												+ (req->uri[17] - 0x30)) * 1000;
								printf("\n\rhook pulse4 %d %d %c %c\n\r", ct_s,
										OUT_PORT[(uint8_t) (req->uri[12] - 0x31)].delay,
										req->uri[16], req->uri[17]);
								set_outport((uint8_t) (req->uri[12] - 0x31), 4);
								sprintf(buf, "out_result('ok')");
								fault = 0;
							}
						}

					}

				}

			}
			}
//			if (((buf_temp[0]-0x30) == (req->uri[strlen(req->uri) - 2]) - 0x30)
//					&& ((buf_temp[1]-0x30) == (req->uri[strlen(req->uri) - 1]) - 0x30)
//					&& ((strlen(req->uri) - 10) == 2)) {
//				sprintf(buf, "in_result('ok', -1, %d, %d)",
//						IN_PORT[ct_s].sost_filtr, IN_PORT[ct_s].count);
//				printf("\n\rhook %d %s\n\r", ct_s, buf);
//				fault=0;
//			}
		}
		if (fault == 1) {
			printf("\n\rFall  web hook\n\r");
			sprintf(buf, "out_result('error')");
		}

	} else {
		printf("\n\rFall  web hook\n\r");
		sprintf(buf, "out_result('error')");
	}

	httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

static const httpd_uri_t out_set_pulse = { .uri = "/out_set_pulse.cgi",
		.method = HTTP_POST, .handler = out_set_pulse_post_handler, .user_ctx =
		NULL };
static const httpd_uri_t out_get_cgi = { .uri = "/out_get.cgi", .method =
		HTTP_GET, .handler = out_get_cgi_handler, .user_ctx = 0 };
static const httpd_uri_t out_set = { .uri = "/out_set.cgi", .method = HTTP_POST,
		.handler = out_set_post_handler, .user_ctx = NULL };

static const httpd_uri_t out_switch = { .uri = "/out_switch.cgi", .method =
		HTTP_POST, .handler = out_switch_post_handler, .user_ctx = NULL };
static const httpd_uri_t out_web1 = { .uri = "/out.cgi", .method = HTTP_GET,
		.handler = out_web1_handler, .user_ctx = 0 };
void http_var_init_out(httpd_handle_t server) {
	httpd_register_uri_handler(server, &out_web1);
	httpd_register_uri_handler(server, &out_set_pulse);
	httpd_register_uri_handler(server, &out_set);
	httpd_register_uri_handler(server, &out_switch);
	httpd_register_uri_handler(server, &out_get_cgi);
}
//enum out_event_t {
//  OUT_START,
//  OUT_SETE,
//  OUT_CLRE,
//  OUT_SETT,
//  OUT_ERR
//};
void log_swich_out(char *out, log_reple_t *output_reply) {
	//GET_reple(&output_reply);
	char out_small[200] = { 0 };
	memset(out, 0, 256);
	sprintf(out_small, "%02d.%02d.%d  %02d:%02d:%02d [digital-outputs] ",
			output_reply->day, output_reply->month, output_reply->year,
			output_reply->reple_hours, output_reply->reple_minuts,
			output_reply->reple_seconds);
	strcat(out, out_small);
	switch (output_reply->type_event) {

	case OUT_START:
		sprintf(out_small, "Старт модуля выходов v%d.%d\n\r",out_ver,out_rev);
		break;

	case OUT_CLRE:
		sprintf(out_small, "%s %s\n\r", OUT_PORT[output_reply->line].name,
				OUT_PORT[output_reply->line].clr_name);
		break;
	case OUT_SETE:
		sprintf(out_small, "%s %s\n\r", OUT_PORT[output_reply->line].name,
				OUT_PORT[output_reply->line].set_name);
		break;
	case OUT_SETT:
		sprintf(out_small, "%s %s\n\r", OUT_PORT[output_reply->line].name,
				"Изменены настройки модуля");
		break;
	case OUT_ERR:
		sprintf(out_small, "%s %s\n\r", OUT_PORT[output_reply->line].name,
				"Error mess");

		break;
	default:
		sprintf(out_small, "%s %s\n\r", OUT_PORT[output_reply->line].name,
				"Неизвестное событие");
	}
	strcat(out, out_small);
}

void log_out_save_mess(uint8_t event, uint8_t line) {
	xSemaphoreTake(flag_global_save_log, (TickType_t ) 1000);
	{
		log_reple_t save_reply;
		GET_reple(&save_reply);
		save_reply.line = line;
		save_reply.type_event = event;
		save_reply.source = OUT;
		save_reple_log(save_reply);

	}
	xSemaphoreGive(flag_global_save_log);
}

void log_out_event_cntl(output_port_t *outpin, uint8_t line) {
	if (outpin->aflag == 1) {
		if (outpin->realtime == 1) {
			log_out_save_mess(OUT_SETE, line);

		}
		if (outpin->realtime == 0) {
			log_out_save_mess(OUT_CLRE, line);

		}

//		outpin->event = 0;
	}
}

void log_start_out(void) {
	xSemaphoreTake(flag_global_save_log, (TickType_t ) 1000);
	{
		for (uint8_t ct_l = 0; ct_l < 16; ct_l++) {
			if (FW_data.log.source[ct_l] == NULLS) {
				FW_data.log.source[ct_l] = OUT;

				FW_data.log.fswich_point[ct_l] = log_swich_out;
				ct_l = 17;
			}
		}
	}
	xSemaphoreGive(flag_global_save_log);
	log_out_save_mess(OUT_START, 0);
}

void output_port(void *pvParameters) {
	s32_t OID_out[] = { 1, 3, 6, 1, 4, 1, 25728, 5500, 6, 100, 1 };
	log_start_out();
	for (uint8_t ct = 0; ct < out_port_n; ct++) {
		//	FW_data.gpio.OUT_PORT[ct].delay = 3000;
		gpio_iomux_out(PORT_O[ct], 2, 1);
		gpio_set_direction(PORT_O[ct], GPIO_MODE_OUTPUT);
		OUT_PORT[ct].S_gpio_port = xSemaphoreCreateMutex();
		xSemaphoreGive(OUT_PORT[ct].S_gpio_port);
	}

	while (1) {
		//	printf("OUT test \n\r");H
		for (uint8_t ct = 0; ct < out_port_n; ct++) {
//			printf("OUT%d=%d\n\r",ct,OUT_PORT[ct].sost);
//			printf("OUT%d_REALT=%d\n\r",ct,OUT_PORT[ct].realtime);
			if (xSemaphoreTake(OUT_PORT[ct].S_gpio_port,
					(TickType_t ) 100) == pdTRUE) {

				if (OUT_PORT[ct].aflag == 1) {

					if (OUT_PORT[ct].type_logic == 3) {
						if (OUT_PORT[ct].sost == 0) {
							gpio_set_level(PORT_O[ct], 0);

							OUT_PORT[ct].realtime = 0;
							log_out_event_cntl(&(OUT_PORT[ct]), ct);
							vTaskDelay(OUT_PORT[ct].delay / portTICK_PERIOD_MS);
							gpio_set_level(PORT_O[ct], 1);

							OUT_PORT[ct].count++;
							OUT_PORT[ct].realtime = 1;
							log_out_event_cntl(&(OUT_PORT[ct]), ct);
							OUT_PORT[ct].type_logic = 0;
						} else {
							gpio_set_level(PORT_O[ct], 1);

							OUT_PORT[ct].count++;
							OUT_PORT[ct].realtime = 1;
							log_out_event_cntl(&(OUT_PORT[ct]), ct);
							vTaskDelay(OUT_PORT[ct].delay / portTICK_PERIOD_MS);
							gpio_set_level(PORT_O[ct], 0);

							OUT_PORT[ct].realtime = 0;
							log_out_event_cntl(&(OUT_PORT[ct]), ct);
							OUT_PORT[ct].type_logic = 0;

						}

					} else {
						if (OUT_PORT[ct].sost == 0) {

							gpio_set_level(PORT_O[ct], 0);

							OUT_PORT[ct].realtime = 0;
							log_out_event_cntl(&(OUT_PORT[ct]), ct);
						} else {
							gpio_set_level(PORT_O[ct], 1);

							OUT_PORT[ct].count++;
							OUT_PORT[ct].realtime = 1;
							log_out_event_cntl(&(OUT_PORT[ct]), ct);

						}
					}
					OUT_PORT[ct].aflag = 0;
				}
				xSemaphoreGive(OUT_PORT[ct].S_gpio_port);
			}

		}

		vTaskDelay(300 / portTICK_PERIOD_MS);
	}

}

esp_err_t save_data_output(void) {
	esp_err_t err = 0;
	uint8_t ct_s;
	char name[64];
	for (ct_s = 0; ct_s < out_port_n; ct_s++) {

		memset((uint8_t*) name, 0, 64);
		sprintf(name, "gpio_out_d%d", ct_s);
		err = err
				| nvs_set_u16(nvs_data_handle, (char*) name,
						OUT_PORT[ct_s].delay);
		if (err != ERR_OK) {
			ESP_LOGW("OUT_SAVE", "Error %X save data to flash1-%d", err, ct_s);
		}

		memset((uint8_t*) name, 0, 64);
		sprintf(name, "gpio_out_p%d", ct_s);
		err = err
				| nvs_set_u16(nvs_data_handle, (char*) name,
						OUT_PORT[ct_s].polar_pulse);
		if (err != ERR_OK) {
			ESP_LOGW("OUT_SAVE", "Error %X save data to flash2-%d", err, ct_s);
		}

		memset((uint8_t*) name, 0, 64);
		sprintf(name, "gpio_t_log_%d", ct_s);
		err = err
				| nvs_set_u16(nvs_data_handle, (char*) name,
						OUT_PORT[ct_s].type_logic);
		if (err != ERR_OK) {
			ESP_LOGW("OUT_SAVE", "Error %X save data to flash3-%d", err, ct_s);
		}
		memset((uint8_t*) name, 0, 64);
		sprintf(name, "gpio_name_%d", ct_s);
		err = err
				| nvs_set_blob(nvs_data_handle, (char*) name,
						&(OUT_PORT[ct_s].name), 32);
		if (err != ERR_OK) {
			ESP_LOGW("OUT_SAVE", "Error %X save data to flash4-%d", err, ct_s);
		}
		memset((uint8_t*) name, 0, 64);
		sprintf(name, "gpio_s_name_%d", ct_s);
		err = err
				| nvs_set_blob(nvs_data_handle, (char*) name,
						&(OUT_PORT[ct_s].set_name), 32);
		if (err != ERR_OK) {
			ESP_LOGW("OUT_SAVE", "Error %X save data to flash5-%d", err, ct_s);
		}
		memset((uint8_t*) name, 0, 64);
		sprintf(name, "gpio_c_name_%d", ct_s);
		err = err
				| nvs_set_blob(nvs_data_handle, (char*) name,
						&(OUT_PORT[ct_s].clr_name), 32);
		if (err != ESP_OK) {
			ESP_LOGW("OUT_SAVE", "Error %X save data to flash6-%d", err, ct_s);
		}
	}



	return err;
}

esp_err_t load_data_output(void) {
	esp_err_t err = 0;
	size_t lens = 16;
	uint8_t ct_s;
	char name[64];
	for (ct_s = 0; ct_s < out_port_n; ct_s++) {

		memset((uint8_t*) name, 0, 64);
		sprintf(name, "gpio_out_d%d", ct_s);
		err = err
				| nvs_get_u16(nvs_data_handle, (char*) name,
						&OUT_PORT[ct_s].delay);

		if (err != ERR_OK) {
			ESP_LOGE("OUT_READ", "Error %s=%x read data to flash1-%d", name,
					err, ct_s);
		}

		memset((uint8_t*) name, 0, 64);
		sprintf(name, "gpio_out_p%d", ct_s);
		err = err
				| nvs_get_u16(nvs_data_handle, (char*) name,
						&OUT_PORT[ct_s].polar_pulse);

		if (err != ERR_OK) {
			ESP_LOGE("OUT_READ", "Error %s=%x read data to flash2-%d", name,
					err, ct_s);
		}

		memset((uint8_t*) name, 0, 64);
		sprintf(name, "gpio_t_log_%d", ct_s);
		err = err
				| nvs_get_u16(nvs_data_handle, (char*) name,
						&OUT_PORT[ct_s].type_logic);
		if (err != ERR_OK) {
			ESP_LOGE("OUT_READ", "Error %s=%x read data to flash3-%d", name,
					err, ct_s);
		}
		lens = 32;

		memset((uint8_t*) name, 0, 64);
		sprintf(name, "gpio_name_%d", ct_s);
//		ESP_LOGW("OUT_READ", "Error %s=%x read data to flash4-%d", name,
//							err, ct_s);
		err = err
				| nvs_get_blob(nvs_data_handle, (char*) name,
						&(OUT_PORT[ct_s].name), &lens);
		if (err != ERR_OK) {
			ESP_LOGE("OUT_READ", "Error %s=%x read data to flash4-%d", name,
					err, ct_s);
		}

		lens = 32;

		memset((uint8_t*) name, 0, 64);
		sprintf(name, "gpio_s_name_%d", ct_s);
//		ESP_LOGE("OUT_READ", "Error %s=%x read data to flash5-%d", name,
//							err, ct_s);
		err = err
				| nvs_get_blob(nvs_data_handle, (char*) name,
						&(OUT_PORT[ct_s].set_name), &lens);
		if (err != ERR_OK) {
			ESP_LOGE("OUT_READ", "Error %s=%x read data to flash5-%d", name,
					err, ct_s);
		}
		lens = 32;

		memset((uint8_t*) name, 0, 64);
		sprintf(name, "gpio_c_name_%d", ct_s);
		err = err
				| nvs_get_blob(nvs_data_handle, (char*) name,
						&(OUT_PORT[ct_s].clr_name), &lens);
		if (err != ERR_OK) {
			ESP_LOGE("OUT_READ", "Error %s=%x read data to flash6-%d", name,
					err, ct_s);
		}
	}



	return err;

}

void load_def_output(void) {
	uint8_t ct_s;
	char name[64];
	for (ct_s = 0; ct_s < out_port_n; ct_s++) {

		memset((uint8_t*) name, 0, 64);
		sprintf(name, "Линия %d", ct_s);
		memset((uint8_t*) &OUT_PORT[ct_s].name, 0, 32);
		memcpy((uint8_t*) &OUT_PORT[ct_s].name, name, strlen(name));

		memset((uint8_t*) &OUT_PORT[ct_s].clr_name, 0, 32);
		memcpy((uint8_t*) &OUT_PORT[ct_s].clr_name, (uint8_t*) "Выключить",
				sizeof("Выключить"));

		memset((uint8_t*) &OUT_PORT[ct_s].set_name, 0, 32);
		memcpy((uint8_t*) &OUT_PORT[ct_s].set_name, (uint8_t*) "Включить",
				sizeof("Включить"));

		OUT_PORT[ct_s].delay = 5000;
		OUT_PORT[ct_s].type_logic = 0;
		OUT_PORT[ct_s].polar_pulse = 0;
	}

}

#endif
