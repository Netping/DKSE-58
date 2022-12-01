#include "includes_base.h"
#include "../main/notify/notify.h"
#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "LOGS.h"
#include "../components/mime.h"
#include "http_var.h"
#include "../main/output/output.h"
#include "../main/input/input.h"
#include "app.h"
#include "smtp.h"
#include "lwip/dns.h"
#include "private_mib.h"
//log_reple_t input_reply_gl;

#if MAIN_APP_NOTIF == 1

typedef struct {
	char name[64];
	uint8_t active;
//	char signal[64];
	uint8_t signal_n;
	//char method[64];
	uint8_t method_n;
	char expr[64];
	uint8_t expr_n[5];
	char text_mess[128];
	char send_to[64];
} notify_data_t;

enum notf_event_t {
	NOTF_START, NOTF_SETE, NOTF_SETT, NOTF_ERR,NOTF_ACTIV
};
ip_addr_t ip_trap;
notify_data_t NOTF[not_max_n];
char mails1[64];
char mails2[64];
char mails3[64];
uint8_t notf_save = 0;
int ex_data[5] = { 0 };

int trap_dns_found(const char *hostname, ip_addr_t *ipaddr, void *arg) {
//  struct smtp_session *s = (struct smtp_session*)arg;
//  struct altcp_pcb *pcb;
	err_t *err;
	u8_t result;

	err = (struct smtp_session*) arg;
	LWIP_UNUSED_ARG(hostname);

	if (ipaddr != NULL) {
		*err = ERR_OK;
		return;
	} else {
		*err = ERR_ARG;
		return;
	}

}

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

esp_err_t notify_set_post_handler(httpd_req_t *req) {
	esp_err_t err;
	nvs_handle_t my_handle;
	uint8_t len;

	char buf_temp[6 * 1024] = { 0 };
	io_set_t data;
	int ret, remaining = req->content_len;
	uint16_t ct;

//	httpd_resp_set_status(req, "303 See Other");
//	httpd_resp_set_hdr(req, "Location", "\notify.html");
//	httpd_resp_set_hdr(req, "Cache-Control",
//			"no-store, no-cache, must-revalidate");
//	httpd_resp_set_type(req, mime_html);
//	httpd_resp_set_hdr(req, "Connection", "Close");

	httpd_resp_set_status(req, "302 Temporary Redirect");
	httpd_resp_set_hdr(req, "Location", " /notify.html");
	httpd_resp_set_hdr(req, "Cache-Control",
			"no-store, no-cache, must-revalidate");
	httpd_resp_set_type(req, mime_html);
	httpd_resp_set_hdr(req, "Connection", "Close");
	//char* buf=malloc(remaining);

//	memset(buf,1,size_1k_buff * 1044);
//	printf("Read init buf= %d\n\r", strlen(buf));
	memset(buf, 0, size_1k_buff * 1044);

//	if ((ret = httpd_req_recv(req, buf,1024*size_1k_buff)) <= 0) {
//
//	}

	printf("Read POST remaining= %d\n\r", remaining);
	if (remaining > 0) {
		for (; remaining > 0;) {
			ret = httpd_req_recv(req, buf_temp, 1024 * size_1k_buff);
			if (ret != 0) {
				strcat(buf, buf_temp);
				memset(buf_temp, 0, sizeof(buf_temp));
				remaining = remaining - ret;
			}

		}

	}

	printf("Read POST\n\r");

	printf("Read POST buf= %d\n\r", strlen(buf));
	printf("Read POST remaining= %d\n\r", remaining);

	notf_save = ((buf[5] - 0x30) << 4) | (buf[6] - 0x30);
	printf("Read n %d\n\r", notf_save);

//	for (ct = 0; ct < (40 * (notf_save)); ct++) {
//		memset(buf_temp, 0, 600);
//		memcpy(buf_temp, (char*) (buf + 50 * ct + 6), 10);
//		printf("%d %s ",ct*50, buf_temp);
//
//		memset(buf_temp, 0, 600);
//		memcpy(buf_temp, (char*) (buf + 50 * ct + 16), 10);
//		printf("%s ", buf_temp);
//
//		memset(buf_temp, 0, 600);
//		memcpy(buf_temp, (char*) (buf + 50 * ct + 26), 10);
//		printf("%s ", buf_temp);
//
//		memset(buf_temp, 0, 600);
//		memcpy(buf_temp, (char*) (buf + 50 * ct + 36), 10);
//		printf("%s ", buf_temp);
//
//		memset(buf_temp, 0, 600);
//		memcpy(buf_temp, (char*) (buf + 50 * ct + 46), 10);
//		printf("%s \n\r", buf_temp);
//
//	}

//	printf("notf_save=%d\n\r", notf_save);
	if (notf_save < not_max_n + 1) {
		for (ct = 0; ct < (notf_save); ct++) {

			len = read_mess_smtp((char*) (buf + ct * 1044 + 7),
					(uint8_t*) buf_temp);

			memset(&(NOTF[ct].name), 0, 64);
			memcpy(&(NOTF[ct].name), (char*) (buf_temp), len);
			memset(buf_temp, 0, 256);
			//		printf("NOTF[%d].name=%s\n\r", ct, NOTF[ct].name);

			NOTF[ct].signal_n = (((buf[132 + 7 + ct * 1044] - 0x30) << 8)
					| (buf[133 + 7 + ct * 1044] - 0x30));
			//		printf("NOTF[%d].signal_n=%d\n\r", ct, NOTF[ct].signal_n);

			NOTF[ct].method_n = ((buf[134 + 7 + ct * 1044] - 0x30) << 8)
					| (buf[135 + 7 + ct * 1044] - 0x30);
			//		printf("NOTF[%d].method_n=%d\n\r", ct, NOTF[ct].method_n);

			len = read_mess_smtp((char*) (buf + ct * 1044 + 136 + 7),
					(uint8_t*) buf_temp);
			memset(&(NOTF[ct].expr), 0, 64);
			memcpy(&(NOTF[ct].expr), (char*) (buf_temp), len);
			memset(buf_temp, 0, 256);
			printf("NOTF[%d].expr=%s\n\r", ct, NOTF[ct].expr);

			//	decode_expr(ex_data, NOTF[ct].expr, strlen(NOTF[ct].expr));

			len = read_mess_smtp((char*) (buf + ct * 1044 + 198 * 2 + 7),
					(uint8_t*) buf_temp);
			memset(&(NOTF[ct].text_mess), 0, 64);
			memcpy(&(NOTF[ct].text_mess), (char*) (buf_temp), len);
			memset(buf_temp, 0, 256);
			///		printf("NOTF[%d].text_mess=%s\n\r", ct, NOTF[ct].text_mess);

			len = read_mess_smtp((char*) (buf + ct * 1044 + 890 + 7),
					(uint8_t*) buf_temp);
			memset(&(NOTF[ct].send_to), 0, 64);
			memcpy(&(NOTF[ct].send_to), (char*) (buf_temp), len);
			//		dec_email(NOTF[ct].send_to,mails1,mails2,mails3);

			memset(buf_temp, 0, 256);
//			printf("NOTF[%d].send_to=%s\n\r", ct, NOTF[ct].send_to);
//			printf("mails1=%s\n\r", mails1);
//			printf("mails2=%s\n\r", mails2);
//			printf("mails3=%s\n\r", mails3);

			uint8_t active_t = ((buf[1042 + 7 + ct * 1044] - 0x30) << 8)
					| (buf[1043 + 7 + ct * 1044] - 0x30);
			if ((NOTF[ct].active == 0) && (active_t == 1)) {

				log_notf_save_mess(NOTF_SETE, ct);
			}

			if (active_t == 0) {
				NOTF[ct].active = 0;
			} else {
				if (NOTF[ct].active == 0) {

					log_notf_save_mess(NOTF_SETE, ct);
				}
				NOTF[ct].active = 1;
			}

			//		printf("NOTF[%d].active=%d\n\r", ct, NOTF[ct].active);

		}

	} else {
		notf_save = not_max_n;
	}

	log_notf_save_mess(NOTF_SETT, 0);
	nvs_flags.data_param = 1;

	httpd_resp_send_chunk(req, NULL, 0);
//	free(buf);
	return ESP_OK;
}

esp_err_t notify_get_cgi_handler(httpd_req_t *req) {

	uint8_t ct;
	httpd_resp_set_hdr(req, "Cache-Control",
			"no-store, no-cache, must-revalidate");
	httpd_resp_set_type(req, mime_js);
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
	//char buf[1024 *6] = { 0 };
	char buf_temp[1024] = { 0 };
	uint16_t gpio_status, ct_s;

	uint8_t start_ct = 0;
	gpio_status = 0;
	memset(buf, 0, size_1k_buff * 1044);

//	for (ct_s = 0; ct_s < not_max_n; ct_s++) {
//		gpio_status = gpio_status | ((IN_PORT[ct_s].sost_filtr & 0x01) << ct_s);
//	}
//	if ((req->uri[12] == 'a') & (req->uri[13] == 'd') & (req->uri[14] == 'd')) {

//		sprintf(buf_temp, "var data_status=15;");
//		strcat(buf, buf_temp);

	sprintf(buf, "var data=[");
	if (notf_save != 0) {
		for (ct_s = 0; ct_s < notf_save - 1; ct_s++) {

			sprintf(buf_temp,
					"{name:\"%s\",\"active\":%d,signal:\"%d\",method:\"%d\",expr:\"%s\",text_mess:\"%s\",send_to:\"%s\"},",
					NOTF[ct_s].name, NOTF[ct_s].active, NOTF[ct_s].signal_n,
					NOTF[ct_s].method_n, NOTF[ct_s].expr, NOTF[ct_s].text_mess,
					NOTF[ct_s].send_to);
			strcat(buf, buf_temp);
		}
		sprintf(buf_temp,
				"{name:\"%s\",\"active\":%d,signal:\"%d\",method:\"%d\",expr:\"%s\",text_mess:\"%s\",send_to:\"%s\"}",
				NOTF[notf_save - 1].name, NOTF[notf_save - 1].active,
				NOTF[notf_save - 1].signal_n, NOTF[notf_save - 1].method_n,
				NOTF[notf_save - 1].expr, NOTF[notf_save - 1].text_mess,
				NOTF[notf_save - 1].send_to);

		strcat(buf, buf_temp);

	}
	sprintf(buf_temp, "];\n\r");
	strcat(buf, buf_temp);

	sprintf(buf_temp, "var devname='%s';\n\r", FW_data.sys.V_Name_dev);
	strcat(buf, buf_temp);
	running = esp_ota_get_running_partition();
	esp_ota_get_partition_description(running, &app_desc);
	sprintf(buf_temp, "var fwver='v%.31s';\n\r", app_desc.version);
	strcat(buf, buf_temp);
	sprintf(buf_temp, "var hwmodel=%d;\n\r", 6);
	strcat(buf, buf_temp);
	sprintf(buf_temp, "var hwver=%d;\n\r", hw_config);
	strcat(buf, buf_temp);
	sprintf(buf_temp, "var sys_name='%s';\n\r", FW_data.sys.V_Name_dev);
	strcat(buf, buf_temp);
	sprintf(buf_temp, "var sys_location='%s';\n\r", FW_data.sys.V_GEOM_NAME);
	strcat(buf, buf_temp);
	gpio_status = MENU_CONFG;
	sprintf(buf_temp, "var menu_data='%d';\n\r", gpio_status);
	strcat(buf, buf_temp);

	sprintf(buf_temp, "var signal_arr = [");
	strcat(buf, buf_temp);

#if MAIN_APP_IN_PORT == 1

	for (ct_s = 0; ct_s < in_port_n - 1; ct_s++) {

		sprintf(buf_temp, "{value:\"%d\",text:\"Вход%d\"},", ct_s, ct_s);
		strcat(buf, buf_temp);

	}

	sprintf(buf_temp, "{value:\"%d\",text:\"Вход%d\"}", in_port_n - 1,
	in_port_n - 1);
	strcat(buf, buf_temp);
	start_ct = in_port_n;

#endif
#if MAIN_APP_OUT_PORT == 1
	sprintf(buf_temp, ",");
	strcat(buf, buf_temp);
	for (ct_s = 0; ct_s < out_port_n - 1; ct_s++) {

		sprintf(buf_temp, "{value:\"%d\",text:\"Выход%d\"},", ct_s + start_ct,
				ct_s);
		strcat(buf, buf_temp);
	}
	sprintf(buf_temp, "{value:\"%d\",text:\"Выход%d\"}",
	out_port_n + start_ct - 1,
	out_port_n - 1);
	strcat(buf, buf_temp);
#endif

#if MAIN_APP_OWB_H_ == 1
	uint8_t sensor_cont;
	if (num_devices != 0) {
		sensor_cont = num_devices;
	} else {
		sensor_cont = max_sensor;

	}

	sprintf(buf_temp, ",");
	strcat(buf, buf_temp);
	for (ct_s = 0; ct_s < sensor_cont - 1; ct_s++) {

		sprintf(buf_temp, "{value:\"%d\",text:\"Термодатчик%d\"},",
				ct_s + start_ct, ct_s);
		strcat(buf, buf_temp);
	}
	sprintf(buf_temp, "{value:\"%d\",text:\"Термодатчик%d\"}",
	out_port_n + start_ct - 1,
	out_port_n - 1);
	strcat(buf, buf_temp);
#endif

	sprintf(buf_temp, "];\n\r");
	strcat(buf, buf_temp);

	sprintf(buf_temp, "var method_arr = [");
	strcat(buf, buf_temp);
	sprintf(buf_temp, "{value:\"0\",text:\"E-mail\"},");
	strcat(buf, buf_temp);
	sprintf(buf_temp, "{value:\"1\",text:\"SNMP\"}");
	strcat(buf, buf_temp);
	sprintf(buf_temp, "];\n\r");
	strcat(buf, buf_temp);
	sprintf(buf_temp,
			"packfmt={name:{offs:0,len:66},signal:{offs:66,len:1},method:{offs:67,len:1},expr:{offs:68,len:130},text_mess:{offs:198,len:257},send_to:{offs:445,len:66},active:{offs:521,len:1},__len:522};\n\r");
	strcat(buf, buf_temp);

//	sprintf(buf_temp, " pack(packfmt, data);\n");
//	strcat(buf, buf_temp);

	printf("Size buf_temp=%d\n\r", strlen(buf_temp));
	printf("Size buf=%d\n\r", strlen(buf));

//}
//	else {
//		sprintf(buf_temp,
//				"<pre>\nretry: 2000\n\nevent: in_state\ndata: %d\n\n</pre>",
//				gpio_status);
//		strcat(buf, buf_temp);
//
//	}

	httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

static const httpd_uri_t notify_set = { .uri = "/notify_set.cgi", .method =
		HTTP_POST, .handler = notify_set_post_handler, .user_ctx = NULL };

static const httpd_uri_t notify_get_cgi = { .uri = "/notify_get.cgi", .method =
		HTTP_GET, .handler = notify_get_cgi_handler, .user_ctx = 0 };

void http_var_init_notify(httpd_handle_t server) {
	httpd_register_uri_handler(server, &notify_set);
	httpd_register_uri_handler(server, &notify_get_cgi);
}

esp_err_t load_data_notify(void) {
	esp_err_t err = 0;
	uint8_t ct;
	uint8_t name_line[32];
	uint8_t lens = 64;

	memset((uint8_t*) &name_line, 0, 32);
	sprintf((char*) name_line, "notf_save");
	err = err | nvs_get_u8(nvs_data_handle, (char*) name_line, &(notf_save));

	for (ct = 0; ct < notf_save; ct++) {
		//memset((uint8_t*) &NOTF[ct].name, 0, 64);
		lens = 64;

		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_name", ct);
		err = err
				| nvs_get_blob(nvs_data_handle, (char*) name_line,
						&(NOTF[ct].name), &lens);

		//memset((uint8_t*) &NOTF[ct].expr, 0, 64);
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_expr", ct);
		err = err
				| nvs_get_blob(nvs_data_handle, (char*) name_line,
						&(NOTF[ct].expr), &lens);

		//memset((uint8_t*) &NOTF[ct].method, 0, 64);
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_method", ct);
		err = err
				| nvs_get_u8(nvs_data_handle, (char*) name_line,
						&(NOTF[ct].method_n));

		//memset((uint8_t*) &NOTF[ct].send_to, 0, 64);
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_send_to", ct);
		err = err
				| nvs_get_blob(nvs_data_handle, (char*) name_line,
						&(NOTF[ct].send_to), &lens);

		//memset((uint8_t*) &NOTF[ct].signal, 0, 64);
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_signal", ct);
		err = err
				| nvs_get_u8(nvs_data_handle, (char*) name_line,
						&(NOTF[ct].signal_n));

		lens = 128;

		//memset((uint8_t*) &NOTF[ct].text_mess, 0, 128);
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_text_mess", ct);
		err = err
				| nvs_get_blob(nvs_data_handle, (char*) name_line,
						&(NOTF[ct].text_mess), &lens);

		//NOTF[ct].avtive = 0;
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_avtive", ct);
		err = err
				| nvs_get_u8(nvs_data_handle, (char*) name_line,
						&(NOTF[ct].active));

		//NOTF[ct].method_n = 0;
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_method_n", ct);
		err = err
				| nvs_get_u8(nvs_data_handle, (char*) name_line,
						&(NOTF[ct].method_n));

		//NOTF[ct].signal_n = 0;

		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_signal_n", ct);
		err = err
				| nvs_get_u8(nvs_data_handle, (char*) name_line,
						&(NOTF[ct].signal_n));

//		NOTF[ct].expr_n[0] = 0;
//		NOTF[ct].expr_n[1] = 0;
//		NOTF[ct].expr_n[2] = 0;
//		NOTF[ct].expr_n[3] = 0;
//		NOTF[ct].expr_n[4] = 0;

		lens = 5;
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_expr_n", ct);
		err = err
				| nvs_get_blob(nvs_data_handle, (char*) name_line,
						&(NOTF[ct].expr_n), &lens);
	}
	memset((uint8_t*) &name_line, 0, 32);
	sprintf((char*) name_line, "notf_save");
	err = err | nvs_get_u8(nvs_data_handle, (char*) name_line, &(notf_save));
	return err;
}

esp_err_t save_data_notify(void) {
	esp_err_t err = 0;
	uint8_t ct;
	uint8_t name_line[32];

	memset((uint8_t*) &name_line, 0, 32);
	sprintf((char*) name_line, "notf_save");
	err = err | nvs_set_u8(nvs_data_handle, (char*) name_line, notf_save);

	for (ct = 0; ct < notf_save; ct++) {

		//memset((uint8_t*) &NOTF[ct].name, 0, 64);
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_name", ct);
		err = err
				| nvs_set_blob(nvs_data_handle, (char*) name_line,
						&(NOTF[ct].name), 64);

		//memset((uint8_t*) &NOTF[ct].expr, 0, 64);
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_expr", ct);
		printf("NOTF%d_expr=%s\n\r", ct, NOTF[ct].expr);
		err = err
				| nvs_set_blob(nvs_data_handle, (char*) name_line,
						&(NOTF[ct].expr), 64);

		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_method_n", ct);
		err = err
				| nvs_set_u8(nvs_data_handle, (char*) name_line,
						NOTF[ct].method_n);

		//memset((uint8_t*) &NOTF[ct].send_to, 0, 64);
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_send_to", ct);
		err = err
				| nvs_set_blob(nvs_data_handle, (char*) name_line,
						&(NOTF[ct].send_to), 64);

		//	memset((uint8_t*) &NOTF[ct].signal, 0, 64);
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_signal", ct);
		err = err
				| nvs_set_u8(nvs_data_handle, (char*) name_line,
						NOTF[ct].signal_n);

		//	memset((uint8_t*) &NOTF[ct].text_mess, 0, 128);
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_text_mess", ct);
		err = err
				| nvs_set_blob(nvs_data_handle, (char*) name_line,
						&(NOTF[ct].text_mess), 128);

		//	NOTF[ct].avtive = 0;
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_avtive", ct);
		err = err
				| nvs_set_u8(nvs_data_handle, (char*) name_line,
						NOTF[ct].active);

		//NOTF[ct].method_n = 0;
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_method_n", ct);
		err = err
				| nvs_set_u8(nvs_data_handle, (char*) name_line,
						NOTF[ct].method_n);
		//NOTF[ct].signal_n = 0;
		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_signal_n", ct);
		err = err
				| nvs_set_u8(nvs_data_handle, (char*) name_line,
						NOTF[ct].signal_n);

//		NOTF[ct].expr_n[0] = 0;
//		NOTF[ct].expr_n[1] = 0;
//		NOTF[ct].expr_n[2] = 0;
//		NOTF[ct].expr_n[3] = 0;
//		NOTF[ct].expr_n[4] = 0;

		memset((uint8_t*) &name_line, 0, 32);
		sprintf((char*) name_line, "NOTF%d_expr_n", ct);
		err = err
				| nvs_set_blob(nvs_data_handle, (char*) name_line,
						&(NOTF[ct].expr_n), 5);

	}
	memset((uint8_t*) &name_line, 0, 32);
	sprintf((char*) name_line, "notf_save");
	err = err | nvs_set_u8(nvs_data_handle, (char*) name_line, notf_save);

	return err;
}
void load_def_notify(void) {
	uint8_t ct;
	uint8_t name_notif[32];
	for (ct = 0; ct < not_max_n; ct++) {
		memset((uint8_t*) &NOTF[ct].name, 0, 64);
		memset((uint8_t*) &NOTF[ct].expr, 0, 64);
		//	memset((uint8_t*) &NOTF[ct].method, 0, 64);
		memset((uint8_t*) &NOTF[ct].send_to, 0, 64);
		//	memset((uint8_t*) &NOTF[ct].signal, 0, 64);
		memset((uint8_t*) &NOTF[ct].text_mess, 0, 128);
		NOTF[ct].active = 0;
		NOTF[ct].method_n = 0;
		NOTF[ct].signal_n = 0;

	}

	sprintf((char*) NOTF[0].name, "Контроль входной двери");
//	sprintf((char*) NOTF[0].method, "E-mail");
	sprintf((char*) NOTF[0].send_to, "ntp1.stratum2.ru");
//	sprintf((char*) NOTF[0].signal, "input0");
	sprintf((char*) NOTF[0].text_mess,
			"Концевой выключатель входной двери замкнут");
	sprintf((char*) NOTF[0].expr, "%sVAL%s>0", "%", "%");
	NOTF[0].method_n = 1;
	NOTF[0].signal_n = 1;

	notf_save = 1;
}
//enum in_event_t {
//  NOTF_START,
//  NOTF_SETE,
//  NOTF_SETT,
//  NOTF_ERR
//};
void log_swich_notf(char *out, log_reple_t *input_reply) {
	char out_small[200] = { 0 };
	sprintf(out_small, "%02d.%02d.%d  %02d:%02d:%02d [notification] ",
			input_reply->day, input_reply->month, input_reply->year,
			input_reply->reple_hours, input_reply->reple_minuts,
			input_reply->reple_seconds);
	strcat(out, out_small);
	switch (input_reply->type_event) {

	case NOTF_START:
		sprintf(out_small, "Старт модуля уведомления v%d.%d\n\r",not_ver,not_rev);
		break;
	case NOTF_SETE:
		sprintf(out_small, "Активировано уведомление: %s \n\r",
				NOTF[input_reply->line].name);

		break;
		case NOTF_ACTIV:
		sprintf(out_small, "Сработка уведомление: %s \n\r",
				NOTF[input_reply->line].name);

		break;
	case NOTF_SETT:
		sprintf(out_small, "%s", "Изменение настроек уведомлений \n\r");
		break;
	case NOTF_ERR:
		sprintf(out_small, "%s %s", "Ошибка модуля: \n\r",
				NOTF[input_reply->line].name);
		break;
	default:
		sprintf(out_small, "%s %s", NOTF[input_reply->line].name,
				"Ошибка журнала\n\r");
	}
	strcat(out, out_small);
}

void log_start_notf(void) {
	xSemaphoreTake(flag_global_save_log, (TickType_t ) 1000);
	{
		for (uint8_t ct_l = 0; ct_l < 16; ct_l++) {
			if (FW_data.log.source[ct_l] == NULLS) {
				FW_data.log.source[ct_l] = NTF;

				//	memcpy(&(FW_data.log.header[ct_l][0]), (char*)"[digital-inputs]",sizeof("[digital-inputs]"));
				FW_data.log.fswich_point[ct_l] = log_swich_notf;
				ct_l = 17;
			}
		}
	}
	xSemaphoreGive(flag_global_save_log);
	log_notf_save_mess(NOTF_START, 0);
}

void log_notf_save_mess(uint8_t event, uint8_t line) {
	xSemaphoreTake(flag_global_save_log, (TickType_t ) 1000);
	{
		log_reple_t save_reply;
		GET_reple(&save_reply);
		save_reply.line = line;
		save_reply.type_event = event;
		save_reply.source = NTF;
		save_reple_log(save_reply);

	}
	xSemaphoreGive(flag_global_save_log);
}

void notify_app(void *pvParameters) {

	log_start_notf();
	uint8_t nf_ct = 0;
	uint8_t event_nf = 0;
	uint8_t event_nf_old[not_max_n] = {0};
	char subj[64] = { };
	uint8_t err;
	uint8_t ipt[4];
	uint8_t indexi, indexn = 0;
	uint8_t index_old;
	char mess[256] = { 0 };
	uint8_t sensor_cont;

	while (1) {
		if (num_devices != 0) {
			sensor_cont = num_devices;
		} else {
			sensor_cont = max_sensor;

		}
		for (nf_ct = 0; nf_ct < notf_save; nf_ct++) {
			if (NOTF[nf_ct].active == 1) {
				event_nf = 0;
				memset((char*) ex_data, 0, 20);
				decode_expr(ex_data, NOTF[nf_ct].expr,
						strlen(NOTF[nf_ct].expr));
				if (NOTF[nf_ct].signal_n < in_port_n) {

					printf("event_nf=%d.%d.%d.%d.%d\n\r", ex_data[0],
							ex_data[1], ex_data[2], ex_data[3], ex_data[4]);
					printf("IN_PORT[NOTF[nf_ct].signal_n].sost_filtr=%d\n\r",
							IN_PORT[NOTF[nf_ct].signal_n].sost_filtr);

					event_nf = notife_run(ex_data,
							IN_PORT[NOTF[nf_ct].signal_n].sost_filtr);

					printf("event_nf=%d\n\r", event_nf);
					if ((event_nf != 0) && (event_nf_old[nf_ct] == 0)) {
										event_nf = 1;
										log_notf_save_mess(NOTF_ACTIV, nf_ct);
									} else {
										event_nf = 0;
									}

				} else if (NOTF[nf_ct].signal_n < (in_port_n + out_port_n)) {
					event_nf =
							notife_run(ex_data,
									OUT_PORT[NOTF[nf_ct].signal_n - in_port_n].realtime);
					if ((event_nf != 0) && (event_nf_old[nf_ct] == 0)) {
										event_nf = 1;
										log_notf_save_mess(NOTF_ACTIV, nf_ct);
									} else {
										event_nf = 0;
									}
				} else if (NOTF[nf_ct].signal_n
						< (in_port_n + out_port_n + sensor_cont)) {
					event_nf = notife_run(ex_data,
							termo[NOTF[nf_ct].signal_n
									- (in_port_n + out_port_n)].temper);
					if ((event_nf != 0) && (event_nf_old[nf_ct] == 0)) {
										event_nf = 1;
										log_notf_save_mess(NOTF_ACTIV, nf_ct);
									} else {
										event_nf = 0;
									}
				}


				printf("NOTF[%d].method_n=%d\n\r", nf_ct, NOTF[nf_ct].method_n);
				memset((uint8_t*) mess, 0, 256);
				memcpy(mess, NOTF[nf_ct].text_mess,
						strlen(NOTF[nf_ct].text_mess));
				strcat(mess, "..");
				if ((event_nf == 1) && (NOTF[nf_ct].method_n == 0)) {
					memset(FW_data.smtp.V_EMAIL_TO, 0, 32);
					memset(FW_data.smtp.V_EMAIL_CC1, 0, 32);
					memset(FW_data.smtp.V_EMAIL_CC2, 0, 32);
					dec_email(NOTF[nf_ct].send_to, FW_data.smtp.V_EMAIL_TO,
							FW_data.smtp.V_EMAIL_CC1, FW_data.smtp.V_EMAIL_CC2);

					printf("%s %s %s\n\r", FW_data.smtp.V_EMAIL_TO,
							FW_data.smtp.V_EMAIL_CC1, FW_data.smtp.V_EMAIL_CC2);

					if (NOTF[nf_ct].name[0] == 0) {
						printf("NOTF[nf_ct].name[0] == 0\n\r");
						if (NOTF[nf_ct].signal_n < in_port_n) {
							sprintf(subj, "NetPing %c%d%c %cпїЅпїЅпїЅпїЅ%d%c", '%',
									serial_id, '%', '%', NOTF[nf_ct].signal_n,
									'%');
						} else if (NOTF[nf_ct].signal_n
								< (in_port_n + out_port_n)) {
							sprintf(subj, "NetPing %c%d%c %cпїЅпїЅпїЅпїЅпїЅ%d%c", '%',
									serial_id, '%', '%',
									NOTF[nf_ct].signal_n - in_port_n, '%');
						}
						send_smtp_mess(mess, subj);
					} else {
						printf(
								"send_smtp_mess(NOTF[nf_ct].text_mess, NOTF[nf_ct].name)\n\r");
						send_smtp_mess(mess, NOTF[nf_ct].name);

					}

				}

				if ((event_nf == 1) && (NOTF[nf_ct].method_n == 1)) {
					///		if ((NOTF[nf_ct].method_n == 1)) {

					if ((NOTF[nf_ct].send_to[0] < 0x3a)
							&& (NOTF[nf_ct].send_to[0] > 0x2f)) {
						printf("TR_N=%s\n\r", NOTF[nf_ct].send_to);
//						ipt[0]=(NOTF[nf_ct].send_to[0]-0x30)*100+(NOTF[nf_ct].send_to[1]-0x30)*10+(NOTF[nf_ct].send_to[2]-0x30);
//						ipt[1]=(NOTF[nf_ct].send_to[3]-0x30)*100+(NOTF[nf_ct].send_to[4]-0x30)*10+(NOTF[nf_ct].send_to[5]-0x30);
//						ipt[2]=(NOTF[nf_ct].send_to[7]-0x30)*100+(NOTF[nf_ct].send_to[8]-0x30)*10+(NOTF[nf_ct].send_to[9]-0x30);
//						ipt[3]=(NOTF[nf_ct].send_to[11]-0x30)*100+(NOTF[nf_ct].send_to[12]-0x30)*10+(NOTF[nf_ct].send_to[13]-0x30);
						indexi = 0;
						indexn = 0;
						index_old = 0;
						ipt[0] = 0;
						ipt[1] = 0;
						ipt[2] = 0;
						ipt[3] = 0;
						for (uint8_t ipct = 0;
								ipct < strlen(NOTF[nf_ct].send_to) + 1;
								ipct++) {
							if ((NOTF[nf_ct].send_to[ipct] == '.')
									|| (ipct == strlen(NOTF[nf_ct].send_to))) {
								indexi = ipct - index_old;
								index_old = ipct + 1;
								if (indexi >= 3) {
									ipt[indexn] = (NOTF[nf_ct].send_to[ipct - 3]
											- 0x30) * 100;
									//		printf("NOTF[%d].send_to[%d]=%c\n\r",nf_ct,indexi,(NOTF[nf_ct].send_to[ipct-3]));
								}
								if (indexi >= 2) {
									ipt[indexn] = ipt[indexn]
											+ (NOTF[nf_ct].send_to[ipct - 2]
													- 0x30) * 10;
									//		printf("NOTF[%d].send_to[%d]=%c\n\r",nf_ct,indexi+1,(NOTF[nf_ct].send_to[ipct-2]));
								}
								if (indexi >= 1) {
									ipt[indexn] = ipt[indexn]
											+ (NOTF[nf_ct].send_to[ipct - 1]
													- 0x30);
									//		printf("NOTF[%d].send_to[%d]=%c\n\r",nf_ct,indexi+2,(NOTF[nf_ct].send_to[ipct-1]));
								}

								//	printf("indexn=%d\n\r",indexn);
								indexn++;
								//	printf("indexi=%d\n\r",indexi);
								//	printf("ipct=%d\n\r",ipct-1);
								indexi++;

							}

						}
						printf("IPT=%d.%d.%d.%d\n\r", ipt[0], ipt[1], ipt[2],
								ipt[3]);
						err = ERR_CONN;

					} else {
						dns_gethostbyname(NOTF[nf_ct].send_to, &ip_trap,
								trap_dns_found, &err);
						printf("IPN=%x\n\r", ip_trap.addr);
					}

					if (err != ERR_OK) {

						IP4_ADDR(&ip_trap, ipt[0], ipt[1], ipt[2],
								ipt[3]);
//						ip_syslog1.type = 4;
//						ip_syslog1.u_addr.ip4.addr = ip4_syslog.addr;
						printf("IPN=%x\n\r", ip_trap.addr);
						err = 0;
					}

					if (NOTF[nf_ct].signal_n < in_port_n) {
						printf("in_send_mess_trap=%d\n\r", event_nf);
						in_send_mess_trap(&(ip_trap.addr),
								NOTF[nf_ct].signal_n);
					} else if (NOTF[nf_ct].signal_n
							< (in_port_n + out_port_n)) {
						printf("out_send_mess_trap=%d\n\r", event_nf);
						out_send_mess_trap(&(ip_trap.addr),
								(NOTF[nf_ct].signal_n - in_port_n));
					}

				}

			}
		}

		vTaskDelay(10 * 1000 / portTICK_PERIOD_MS);
	}

}

void decode_expr(int *out_ex, char *mess, size_t len) {
	uint8_t ct = 0;
	uint8_t index = 0;
	uint8_t index1 = 0;
	for (ct = 0; ct < len; ct++) {

		if (memcmp(mess, "%VAL%", 5) == 0) {
			index = ct;
			break;
		}
	}

	index += 5;
//    printf("index=%d\n\r",index);
//    printf("char=%c\n\r",mess[index]);
	out_ex[0] = 0;
	if ((mess[index + 1] > 0x29) && (mess[index + 1] < 0x3a)) {

		if (memcmp((char*) (mess + index), "<", 1) == 0) {
			out_ex[0] = 1;
			printf("<=%c\n\r", mess[index]);
		} else if (memcmp((char*) (mess + index), ">", 1) == 0) {
			printf(">=%c\n\r", mess[index]);
			out_ex[0] = 2;
		} else if (memcmp((char*) (mess + index), "=", 1) == 0) {
			printf("==%c\n\r", mess[index]);
			out_ex[0] = 3;
		}
		index += 1;
	} else {
		if (memcmp((char*) (mess + index), "<=", 2) == 0) {
			printf("<==%c\n\r", mess[index]);
			out_ex[0] = 4;
		} else if (memcmp((char*) (mess + index), ">=", 2) == 0) {
			printf(">==%c\n\r", mess[index]);
			out_ex[0] = 5;
		} else if (memcmp((char*) (mess + index), "!=", 2) == 0) {
			printf("!=%c\n\r", mess[index]);
			out_ex[0] = 6;
		} else if (memcmp((char*) (mess + index), "==", 2) == 0) {
			printf("==%c\n\r", mess[index]);
			out_ex[0] = 7;
		}
		index += 2;
	}
	printf("out_ex[0]=%d\n\r", out_ex[0]);
	printf("len-index=%d\n\r", len - index);
	out_ex[1] = 0;
	if ((len - index) < 4) {
		if (len == index) {
			return;
		}
		if ((len - index) == 1) {
			out_ex[1] = mess[index] - 0x30;
			return;
		}
		if ((len - index) == 2) {
			out_ex[1] = (mess[index + 1] - 0x30) + ((mess[index] - 0x30) * 10);
			return;
		}
		if ((len - index) == 3) {
			out_ex[1] = (mess[index + 2] - 0x30)
					+ ((mess[index + 1] - 0x30) * 10)
					+ ((mess[index] - 0x30) * 100);
			return;
		}

	} else {
		for (ct = 0; ct < 4; ct++) {

			if (memcmp((char*) (mess + index + ct), "%", 1) == 0) {

				//	printf("ct=%d\n\r", ct);
				index1 = ct;
				break;
			}
		}

		if (index1 == 1) {
			out_ex[1] = mess[index] - 0x30;
			//	return;
		}
		if (index1 == 2) {
			out_ex[1] = (mess[index + 1] - 0x30) + ((mess[index] - 0x30) * 10);
			//	return;
		}
		if (index1 == 3) {
			out_ex[1] = (mess[index + 2] - 0x30)
					+ ((mess[index + 1] - 0x30) * 10)
					+ ((mess[index] - 0x30) * 100);
			//	return;
		}
		index += index1;

	}
	printf("out_ex[1]=%d\n\r", out_ex[1]);
	printf("char=%c\n\r", mess[index]);
	out_ex[2] = 0;
	if (memcmp((char*) (mess + index), "%OR%", 4) == 0) {
		index += 4;
		out_ex[2] = 1;
	} else if (memcmp((char*) (mess + index), "%AND%", 5) == 0) {
		index += 5;
		out_ex[2] = 2;
	} else {
		return;
	}

	printf("out_ex[2]=%d\n\r", out_ex[2]);

	if (memcmp((char*) (mess + index), "VAL%", 4) != 0) {
		return;
	}
	index += 4;

	out_ex[3] = 0;
	if ((mess[index + 1] > 0x29) && (mess[index + 1] < 0x3a)) {

		if (memcmp((char*) (mess + index), "<", 1) == 0) {
			out_ex[3] = 1;
			printf("<=%c\n\r", mess[index]);
		} else if (memcmp((char*) (mess + index), ">", 1) == 0) {
			printf(">=%c\n\r", mess[index]);
			out_ex[3] = 2;
		} else if (memcmp((char*) (mess + index), "=", 1) == 0) {
			printf("==%c\n\r", mess[index]);
			out_ex[3] = 3;
		}
		index += 1;
	} else {
		if (memcmp((char*) (mess + index), "<=", 2) == 0) {
			printf("<==%c\n\r", mess[index]);
			out_ex[3] = 4;
		} else if (memcmp((char*) (mess + index), ">=", 2) == 0) {
			printf(">==%c\n\r", mess[index]);
			out_ex[3] = 5;
		} else if (memcmp((char*) (mess + index), "!=", 2) == 0) {
			printf("!==%c\n\r", mess[index]);
			out_ex[3] = 6;
		} else if (memcmp((char*) (mess + index), "==", 2) == 0) {
			printf("===%c\n\r", mess[index]);
			out_ex[3] = 7;
		}
		index += 2;
	}
	printf("out_ex[3]=%d\n\r", out_ex[3]);
	printf("len-index=%d\n\r", len - index);
	out_ex[4] = 0;

	if (len == index) {
		return;
	}
	if ((len - index) == 1) {
		out_ex[4] = mess[index] - 0x30;
//			return;
	}
	if ((len - index) == 2) {
		out_ex[4] = (mess[index + 1] - 0x30) + ((mess[index] - 0x30) * 10);
//			return;
	}
	if ((len - index) == 3) {
		out_ex[4] = (mess[index + 2] - 0x30) + ((mess[index + 1] - 0x30) * 10)
				+ ((mess[index] - 0x30) * 100);
//
	}

	printf("out_ex[4]=%d\n\r", out_ex[4]);
	return;
//	if (memcmp(((char*) (mess + index), "%OR%", 5) == 0) {
//				index = ct;
//
//			}
//	else if (memcmp(((char*) (mess + index), "%OR%", 5) == 0) {
//						index = ct;
//
//					}

}

uint8_t notife_run(int *in, int val) {
	uint8_t result1 = 0;
	uint8_t result2 = 0;
	uint8_t out = 0;
//	printf("in[0]=%d\n\r",in[0]);
///	printf("in[1]=%d\n\r",in[1]);
	switch (in[0]) {
	case 1: {
		if (val < in[1]) {
			result1 = 1;
//			printf("result1=%d\n\r",result1);
		}
	}
		break;
	case 2: {
		if (val > in[1]) {
			result1 = 1;
		}
	}
		break;
	case 3: {
		if (val == in[1]) {
			result1 = 1;
		}
	}
		break;
	case 4: {
		if (val <= in[1]) {
			result1 = 1;
		}
	}
		break;
	case 5: {
		if (val >= in[1]) {
			result1 = 1;
		}
	}
		break;
	case 6: {
		if (val != in[1]) {
			result1 = 1;
		}
	}
		break;
	case 7: {
		if (val == in[1]) {
			result1 = 1;
		}
	}
		break;
	default:
		result1 = 0;
	}

	switch (in[3]) {
	case 1: {
		if (val < in[4]) {
			result2 = 1;
		}
	}
		break;
	case 2: {
		if (val > in[4]) {
			result2 = 1;
		}
	}
		break;
	case 3: {
		if (val == in[4]) {
			result2 = 1;
		}
	}
		break;
	case 4: {
		if (val <= in[4]) {
			result2 = 1;
		}
	}
		break;
	case 5: {
		if (val >= in[4]) {
			result2 = 1;
		}
	}
		break;
	case 6: {
		if (val != in[4]) {
			result2 = 1;
		}
	}
		break;
	default:
		result2 = 0;
	}

	if (in[2] != 0) {
		switch (in[2]) {
		case 1: {
			if ((result1 == 1) || (result2 == 1)) {
				out = 1;
				printf("out=%d\n\r", out);
			}
		}
			break;
		case 2: {
			if ((result1 == 1) && (result2 == 1)) {
				out = 1;
			}
		}
			break;
		default:
			out = 0;
		}
	} else {
		out = result1;
	}

	return out;
}

void dec_email(char *in, char *mail1, char *mail2, char *mail3) {
	uint8_t ct;
	uint8_t in_len = strlen(in);
	uint8_t len1 = 0;
	uint8_t len2 = 0;
	uint8_t len3 = 0;
	uint8_t step = 0;
	for (ct = 0; ct < in_len; ct++) {
		if (in[ct] == ';') {
			if (step == 0) {
				len1 = ct;
				memcpy(mail1, in, len1);
				len1++;
				step++;
				//	ct++;
			} else if (step == 1) {
				//ct++;
				len2 = ct - len1;
				memcpy(mail2, (char*) (in + len1), len2);
				len2++;
				step++;

			} else if (step == 2) {
				//	ct++;
				len3 = ct - len2 - len1;
				memcpy(mail3, (char*) (in + len2 + len1), len3);
				step++;

			}
		}
	}

	if (step == 0) {
		memcpy(mail1, in, in_len);
	}
	if (step == 1) {
		len2 = in_len - len1;
		memcpy(mail2, (char*) (in + len1), len2);
	}
	if (step == 2) {
		len3 = in_len - len2 - len1;
		memcpy(mail3, (char*) (in + len2 + len1), len3);
	}
}
#endif

