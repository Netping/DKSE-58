/*
 * http_var.c
 *
 *  Created on: 18 сент. 2021 г.
 *      Author: ivanov
 */
#include "../main/includes_base.h"
#include "config_pj.h"
#include "esp_tls_crypto.h"
//#include "html_data_simple_ru.h"
#include "update.h"
#include "mime.h"
#include "app.h"
#include "LOGS.h"
#include "smtp.h"
#include "http_var.h"
#include "../main/termo/app_owb.h"
#include "../main/output/output.h"
#include "../main/notify/notify.h"

#define STORAGE_NAMESPACE "storage"
#define TAG_http "http mes"
#define HTTPD_401      "401 UNAUTHORIZED"           /*!< HTTP Response 401 */
#define HTTPD_302      "302 Found Location: /index.html"           /*!< HTTP Response 401 */
/* An HTTP GET handler */
const uint8_t mask_dec[33][4] = { { 255, 255, 255, 255 },
		{ 255, 255, 255, 254 }, { 255, 255, 255, 252 }, { 255, 255, 255, 248 },
		{ 255, 255, 255, 240 }, { 255, 255, 255, 224 }, { 255, 255, 255, 192 },
		{ 255, 255, 255, 128 }, { 255, 255, 255, 000 }, { 255, 255, 254, 000 },
		{ 255, 255, 252, 000 }, { 255, 255, 248, 000 }, { 255, 255, 240, 000 },
		{ 255, 255, 224, 000 }, { 255, 255, 192, 000 }, { 255, 255, 128, 000 },
		{ 255, 255, 000, 000 }, { 255, 254, 000, 000 }, { 255, 252, 000, 000 },
		{ 255, 248, 000, 000 }, { 255, 240, 000, 000 }, { 255, 224, 000, 000 },
		{ 255, 192, 000, 000 }, { 255, 128, 000, 000 }, { 255, 000, 000, 000 },
		{ 254, 000, 000, 000 }, { 252, 000, 000, 000 }, { 248, 000, 000, 000 },
		{ 240, 000, 000, 000 }, { 224, 000, 000, 000 }, { 192, 000, 000, 000 },
		{ 128, 000, 000, 000 }, { 000, 000, 000, 000 } };

uint8_t page_sost;

char buf[size_1k_buff * 1044] = { 0 };

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

static esp_err_t hello_get_handler(httpd_req_t *req) {
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
}

static esp_err_t log_get_cgi_handler(httpd_req_t *req) {
#warning "******** where is no error processing !  *******"
	httpd_resp_set_hdr(req, "Cache-Control",
			"no-store\, no-cache\, must-revalidate");
	httpd_resp_set_type(req, mime_text);

//	char buf[128 * 50] = { 0 };
	memset(buf, 0, size_1k_buff * 1044);
	char buf_temp[256];
	uint8_t number_mess;

	esp_err_t err = nvs_open_from_partition("nvs", "storage", NVS_READWRITE,
			&nvs_data_handle);
	err = nvs_get_u16(nvs_data_handle, "number_mess", &number_mess);
	err = nvs_commit(nvs_data_handle);
	nvs_close(nvs_data_handle);

	//
	//buf[0] = ' ';
	printf("\n\rRead number %d\n\r", number_mess);
	for (uint16_t i = number_mess; i > 0; i--) {
		logs_read(i, buf_temp);
		printf("\n\rRead %d messege logs: %s \n\r", i, buf_temp);
		strcat(buf, buf_temp);

	}
	for (uint16_t i = max_log_mess; i < number_mess; i--) {
		logs_read(i, buf_temp);
		printf("\n\rRead %d messege logs: %s \n\r", i, buf_temp);
		strcat(buf, buf_temp);
//		printf("\n\rRead %d messege logs\n\r", i);
	}

	//sprintf(buf,"11.05.21 Tu 07:38:15.040 Watchdog: reset of chan.1 \"Сигнал\"\. A (8.8.8.8) no reply, B (192.168.0.55) no reply, C (124.211.45.11) is ignored.\r\n");

	httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

static esp_err_t email_send_test_cgi_handler(httpd_req_t *req) {
	httpd_resp_set_type(req, mime_sse);
	httpd_resp_set_status(req, HTTPD_200);
	httpd_resp_set_hdr(req, "Connection", "Close");
	//char buf[256];
	memset(buf, 0, size_1k_buff * 1044);
	my_smtp_test();

	httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

static esp_err_t setup_get_cgi_handler(httpd_req_t *req) {
	httpd_resp_set_hdr(req, "Cache-Control",
			"no-store, no-cache, must-revalidate");
	httpd_resp_set_type(req, mime_js);
	httpd_resp_set_hdr(req, "Connection", "Close");

	esp_err_t err;
	nvs_handle_t my_handle;

//	char buf[14096];

	char buf_temp[256];
	page_sost = SETT;
	memset(buf, 0, size_1k_buff * 1044);
	int mac = 5566223;

sprintf(buf,
		"var packfmt={mac:{offs:0,len:6},ip:{offs:6,len:4},gate:{offs:10,len:4},mask:{offs:14,len:1}, dst:{offs:15,len:1},"
		"http_port:{offs:16,len:2},uname:{offs:18,len:18}, passwd:{offs:36,len:18}, community_r:{offs:54,len:18},"
		"community_w:{offs:72,len:18},filt_ip1:{offs:90,len:4},filt_mask1:{offs:94,len:1},dhcp:{offs:95,len:1},"
		"powersaving:{offs:96,len:1},trap_refresh:{offs:97,len:1},trap_ip1:{offs:105,len:4},trap_ip2:{offs:109,len:4},"
		"ntp_ip1:{offs:113,len:4}, ntp_ip2:{offs:117,len:4},timezone:{offs:121,len:1},syslog_ip1:{offs:122,len:4},"
		"facility:{offs:130,len:1},severity:{offs:131,len:1},snmp_port: {offs:132,len:2},notification_email: {offs:134,len:48},"
		"hostname: {offs:184,len:64},contact: {offs:248,len:64},location: {offs:312,len:64},dns_ip1: {offs:376,len:4},"
		"trap_hostname1: {offs:384,len:64},trap_hostname2: {offs:448,len:64},ntp_hostname1: {offs:512,len:64},"
		"ntp_hostname2: {offs:576,len:64},syslog_hostname1: {offs:640,len:64},syslog_ip2: {offs:704,len:4},syslog_ip3: {offs:708,len:4},"
		"syslog_hostname2: {offs:712,len:64},syslog_hostname3: {offs:778,len:64},__len:842};\n");
	sprintf(buf_temp, "var data={serial:\"SN: %6d\"", serial_id);
	strcat( buf, buf_temp);
	sprintf(buf_temp, ",serialnum:%d,", serial_id);
	strcat( buf, buf_temp);
	struct netif *_netif;
	esp_err_t ret = tcpip_adapter_get_netif(TCPIP_ADAPTER_IF_ETH,
			(void**) &_netif);

	sprintf(buf_temp, "mac:'%02x:%02x:%02x:%02x:%02x:%02x',", _netif->hwaddr[0],
			_netif->hwaddr[1], _netif->hwaddr[2], _netif->hwaddr[3],
			_netif->hwaddr[4], _netif->hwaddr[5]);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "ip:'%d.%d.%d.%d',", FW_data.net.V_IP_CONFIG[0],
			FW_data.net.V_IP_CONFIG[1], FW_data.net.V_IP_CONFIG[2],
			FW_data.net.V_IP_CONFIG[3]);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "gate:'%d.%d.%d.%d',", FW_data.net.V_IP_GET[0],
			FW_data.net.V_IP_GET[1], FW_data.net.V_IP_GET[2],
			FW_data.net.V_IP_GET[3]);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "mask:'%d.%d.%d.%d',", FW_data.net.V_IP_MASK[0],
			FW_data.net.V_IP_MASK[1], FW_data.net.V_IP_MASK[2],
			FW_data.net.V_IP_MASK[3]);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "dst:%d,", FW_data.net.SAMMER);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "http_port:%d,", FW_data.http.V_WEB_PORT);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "uname:\"%s\",", FW_data.http.V_LOGIN);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "passwd:\"%s\",", FW_data.http.V_PASSWORD);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "community_r:\"%s\"", FW_data.snmp.V_COMMUNITY);
	strcat( buf, buf_temp);
	sprintf(buf_temp, ",community_w:\"%s\",", FW_data.snmp.V_COMMUNITY_WRITE);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "filt_ip1:'%d.%d.%d.%d',", FW_data.sys.V_IP_SOURCE[0],
			FW_data.sys.V_IP_SOURCE[1], FW_data.sys.V_IP_SOURCE[2],
			FW_data.sys.V_IP_SOURCE[3]);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "filt_mask1:'%d.%d.%d.%d',",
			mask_dec[32 - FW_data.sys.V_MASK_SOURCE][0],
			mask_dec[32 - FW_data.sys.V_MASK_SOURCE][1],
			mask_dec[32 - FW_data.sys.V_MASK_SOURCE][2],
			mask_dec[32 - FW_data.sys.V_MASK_SOURCE][3]);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "dhcp:%d,", FW_data.net.V_DHCP);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "powersaving:%d,", FW_data.sys.V_ECO_EN);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "trap_refresh:%d,", FW_data.snmp.V_REFR_TRAP);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "trap_ip1:'%d.%d.%d.%d',", FW_data.snmp.V_IP_SNMP[0],
			FW_data.snmp.V_IP_SNMP[1], FW_data.snmp.V_IP_SNMP[2],
			FW_data.snmp.V_IP_SNMP[3]);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "trap_ip2:'%d.%d.%d.%d',", FW_data.snmp.V_IP_SNMP_S[0],
			FW_data.snmp.V_IP_SNMP_S[1], FW_data.snmp.V_IP_SNMP_S[2],
			FW_data.snmp.V_IP_SNMP_S[3]);
	strcat( buf, buf_temp);

	sprintf(buf_temp, "ntp_ip1:'%d.%d.%d.%d',", FW_data.net.V_IP_NTP1[0],
			FW_data.net.V_IP_NTP1[1], FW_data.net.V_IP_NTP1[2],
			FW_data.net.V_IP_NTP1[3]);
	strcat( buf, buf_temp);

	sprintf(buf_temp, "ntp_ip2:'%d.%d.%d.%d',", FW_data.net.V_IP_NTP2[0],
			FW_data.net.V_IP_NTP2[1], FW_data.net.V_IP_NTP2[2],
			FW_data.net.V_IP_NTP2[3]);
	strcat( buf, buf_temp);

	sprintf(buf_temp, "timezone:%d,", FW_data.sys.V_NTP_CIRCL);
	strcat( buf, buf_temp);

	sprintf(buf_temp, "syslog_ip1:'%d.%d.%d.%d',", FW_data.net.V_IP_SYSL[0],
			FW_data.net.V_IP_SYSL[1], FW_data.net.V_IP_SYSL[2],
			FW_data.net.V_IP_SYSL[3]);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "syslog_ip2:'%d.%d.%d.%d',", FW_data.net.V_IP_SYSL1[0],
			FW_data.net.V_IP_SYSL1[1], FW_data.net.V_IP_SYSL1[2],
			FW_data.net.V_IP_SYSL1[3]);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "syslog_ip3:'%d.%d.%d.%d',", FW_data.net.V_IP_SYSL2[0],
			FW_data.net.V_IP_SYSL2[1], FW_data.net.V_IP_SYSL2[2],
			FW_data.net.V_IP_SYSL2[3]);
	strcat( buf, buf_temp);

	sprintf(buf_temp, "facility:%d,severity:%d,", FW_data.net.facility,
			FW_data.net.severity);

	strcat( buf, buf_temp);
	sprintf(buf_temp, "snmp_port:%d,", FW_data.snmp.V_PORT_SNMP);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "notification_email:\"\"");
	strcat( buf, buf_temp);
	sprintf(buf_temp, ",hostname:\"%s\",", FW_data.sys.V_Name_dev);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "location:\"%s\",", FW_data.sys.V_GEOM_NAME);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "contact:\"%s\",", FW_data.sys.V_CALL_DATA);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "dns_ip1:'%d.%d.%d.%d',", FW_data.net.V_IP_DNS[0],
			FW_data.net.V_IP_DNS[1], FW_data.net.V_IP_DNS[2],
			FW_data.net.V_IP_DNS[3]);
	strcat( buf, buf_temp);

	sprintf(buf_temp, "trap_hostname1:\"%s\",", "NO");
	strcat( buf, buf_temp);

	sprintf(buf_temp, "trap_hostname2:\"%s\",", "NO");
	strcat( buf, buf_temp);

	sprintf(buf_temp, "ntp_hostname1:'%s',", FW_data.net.N_NTP1);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "ntp_hostname2:'%s',", FW_data.net.N_NTP2);
	strcat( buf, buf_temp);

//	sprintf(buf_temp, "active_settings:%d,", FW_data.sys.V_ECO_EN);
//		strcat(buf, buf_temp);

//	sprintf(buf_temp, "syslog_ip1:'10%d.%d.%d.%d',", FW_data.net.V_IP_SYSL[0],
//			FW_data.net.V_IP_SYSL[1], FW_data.net.V_IP_SYSL[2],
//			FW_data.net.V_IP_SYSL[3]);
//	strcat(buf, buf_temp);
//	sprintf(buf_temp, "syslog_ip2:'10%d.%d.%d.%d',", FW_data.net.V_IP_SYSL1[0],
//			FW_data.net.V_IP_SYSL1[1], FW_data.net.V_IP_SYSL1[2],
//			FW_data.net.V_IP_SYSL1[3]);
//	strcat(buf, buf_temp);
//	sprintf(buf_temp, "syslog_ip3:'10%d.%d.%d.%d',", FW_data.net.V_IP_SYSL2[0],
//			FW_data.net.V_IP_SYSL2[1], FW_data.net.V_IP_SYSL2[2],
//			FW_data.net.V_IP_SYSL2[3]);

	sprintf(buf_temp, "syslog_hostname1:\"%s\",", FW_data.net.V_N_SYSL);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "syslog_hostname2:\"%s\",", FW_data.net.V_N_SYSL1);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "syslog_hostname3:\"%s\",", FW_data.net.V_N_SYSL2);
	strcat( buf, buf_temp);

	sprintf(buf_temp, "active_settings:\%d\};", 0x37);//37
	strcat( buf, buf_temp);

	uint32_t rtc_time;
	rtc_time = timeinfo.tm_sec + timeinfo.tm_min * 60 + timeinfo.tm_hour * 3600
			+ timeinfo.tm_yday * 86400 + (timeinfo.tm_year - 70) * 31557600;
	sprintf(buf_temp, "var data_rtc=%d;", rtc_time);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "var uptime_100ms=%d;", rtc_time - timeup);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "var devname='%s';", FW_data.sys.V_Name_dev);
	strcat( buf, buf_temp);
	const esp_partition_t *running = esp_ota_get_running_partition();
	esp_ota_get_partition_description(running, &app_desc);
	sprintf(buf_temp, "var fwver='v%.31s';", app_desc.version);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "var hwver=%d;", hw_config);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "var sys_name='%s';", FW_data.sys.V_Name_dev);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "var sys_location='%s';", FW_data.sys.V_GEOM_NAME);
	strcat( buf, buf_temp);
	sprintf(buf_temp, "var hwmodel=%d;", 6);
	strcat( buf, buf_temp);

	sprintf(buf_temp, "var menu_data='%d';\n", MENU_CONFG);
	strcat( buf, buf_temp);

	httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

static esp_err_t rtcset_post_handler(httpd_req_t *req) {
	esp_err_t err;
	nvs_handle_t my_handle;
	httpd_resp_set_status(req, "303 See Other");
	httpd_resp_set_hdr(req, "Location", "\settings.html");
	httpd_resp_set_hdr(req, "Cache-Control",
			"no-store, no-cache, must-revalidate");
	httpd_resp_set_type(req, mime_sse);
	httpd_resp_set_hdr(req, "Connection", "Close");

	//char buf[1000] = { 0 };
	char buf_temp[256] = { 0 };
	int ret, remaining = req->content_len;
	memset(buf, 0, size_1k_buff * 1044);
	if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {
//				if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
//					/* Retry receiving if timeout occurred */
//					continue;
//				}
//				return ESP_FAIL;
	}
	char2_to_hex((char*) (buf + 9), (uint8_t*) buf_temp, 8);
	struct timeval tv;
	struct timezone tz;
	struct timezone tz_utc = { 0, 0 };

	// Set a fixed time of 2020-09-26 00:00:00, UTC  //1601216683 391618146

	tv.tv_sec = (buf_temp[0] | (buf_temp[1] << 8) | (buf_temp[2] << 16)
			| (buf_temp[3] << 24));
	tv.tv_usec = 0;

	//tv.tv_sec =1654535940;
	tz.tz_minuteswest = FW_data.sys.V_NTP_CIRCL * 60;
	tz.tz_dsttime = 0;
	settimeofday(&tv, &tz_utc);
	tzset();

	time(&now);
	localtime_r(&now, &timeinfo);
	timeup = timeinfo.tm_sec + timeinfo.tm_min * 60 + timeinfo.tm_hour * 3600
			+ timeinfo.tm_yday * 86400 + (timeinfo.tm_year - 70) * 31557600;

	nvs_flags.data_param = 1;

	//	}

	// End response
	httpd_resp_send_chunk(req, NULL, 0);
	return ESP_OK;
}
static esp_err_t ip_set_post_handler(httpd_req_t *req) {
	esp_err_t err;
	nvs_handle_t my_handle;
	httpd_resp_set_status(req, "303 See Other");
	httpd_resp_set_hdr(req, "Location", "\settings.html");
	httpd_resp_set_hdr(req, "Cache-Control",
			"no-store, no-cache, must-revalidate");
	httpd_resp_set_type(req, mime_sse);
	httpd_resp_set_hdr(req, "Connection", "Close");
	//char buf[1000];
	memset(buf, 0, size_1k_buff * 1044);

	int ret, remaining = req->content_len;

	//	while (remaining > 0) {
	/* Read the data for the request */
	if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {
//				if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
//					/* Retry receiving if timeout occurred */
//					continue;
//				}
//				return ESP_FAIL;
	}

	FW_data.net.V_IP_CONFIG[0] = buf[6];
	FW_data.net.V_IP_CONFIG[1] = buf[7];
	FW_data.net.V_IP_CONFIG[2] = buf[8];
	FW_data.net.V_IP_CONFIG[3] = buf[9];

	FW_data.net.V_IP_GET[0] = buf[10];
	FW_data.net.V_IP_GET[1] = buf[11];
	FW_data.net.V_IP_GET[2] = buf[12];
	FW_data.net.V_IP_GET[3] = buf[13];
	uint32_t mask_temp = 0xffffffff << (32 - buf[14]);

	FW_data.net.V_IP_MASK[3] = mask_temp & 0x000000ff;
	FW_data.net.V_IP_MASK[2] = 0x000000ff & (mask_temp >> 8);
	FW_data.net.V_IP_MASK[1] = 0x000000ff & (mask_temp >> 16);
	FW_data.net.V_IP_MASK[0] = 0x000000ff & (mask_temp >> 24);

	FW_data.net.V_DHCP = buf[95];

	FW_data.snmp.V_PORT_SNMP = buf[132];

	FW_data.net.V_IP_DNS[0] = buf[376];
	FW_data.net.V_IP_DNS[1] = buf[377];
	FW_data.net.V_IP_DNS[2] = buf[378];
	FW_data.net.V_IP_DNS[3] = buf[379];

	FW_data.http.V_WEB_PORT = buf[16];

	nvs_flags.data_param = 1;

	httpd_resp_send_chunk(req, NULL, 0);
	return ESP_OK;
}

static esp_err_t wdog_set_post_handler(httpd_req_t *req) {
	esp_err_t err;
	nvs_handle_t my_handle;
	uint8_t len;
	//char buf[2048];

	char buf_temp[1024] = { 0 };
	io_set_t data;
	int ret, remaining = req->content_len;
	memset(buf, 0, size_1k_buff * 1044);
	uint8_t ct;

	httpd_resp_set_status(req, "303 See Other");
	httpd_resp_set_hdr(req, "Location", "\wdog.html");
	httpd_resp_set_hdr(req, "Cache-Control",
			"no-store, no-cache, must-revalidate");
	httpd_resp_set_type(req, mime_sse);
	httpd_resp_set_hdr(req, "Connection", "Close");

	if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {

	}

	len = read_mess_smtp((char*) (buf + 13), (uint8_t*) buf_temp);
	memset(FW_data.wdt[0].V_NAME, 0, 16);
	memcpy(FW_data.wdt[0].V_NAME, (char*) (buf_temp), len);
	char2_to_hex((char*) (buf + 13 + 64), (uint8_t*) buf_temp, 219);
	FW_data.wdt[0].V_IP_WDT_ADDR_CN_A[0] = buf_temp[0];
	FW_data.wdt[0].V_IP_WDT_ADDR_CN_A[1] = buf_temp[1];
	FW_data.wdt[0].V_IP_WDT_ADDR_CN_A[2] = buf_temp[2];
	FW_data.wdt[0].V_IP_WDT_ADDR_CN_A[3] = buf_temp[3];

	FW_data.wdt[0].V_IP_WDT_ADDR_CN_B[0] = buf_temp[4];
	FW_data.wdt[0].V_IP_WDT_ADDR_CN_B[1] = buf_temp[5];
	FW_data.wdt[0].V_IP_WDT_ADDR_CN_B[2] = buf_temp[6];
	FW_data.wdt[0].V_IP_WDT_ADDR_CN_B[3] = buf_temp[7];

	FW_data.wdt[0].V_IP_WDT_ADDR_CN_C[0] = buf_temp[8];
	FW_data.wdt[0].V_IP_WDT_ADDR_CN_C[1] = buf_temp[9];
	FW_data.wdt[0].V_IP_WDT_ADDR_CN_C[2] = buf_temp[10];
	FW_data.wdt[0].V_IP_WDT_ADDR_CN_C[3] = buf_temp[11];

	FW_data.wdt[0].V_T_SEND_PING = (buf_temp[13] << 8) | buf_temp[12];
	FW_data.wdt[0].V_TIME_RESEND_PING = (buf_temp[15] << 8) | buf_temp[14];
	FW_data.wdt[0].V_TIME_RESET_PULSE = (buf_temp[17] << 8) | buf_temp[16];
	FW_data.wdt[0].V_PAUSE_RESET_TO_REPID = (buf_temp[19] << 8) | buf_temp[18];
	FW_data.wdt[0].V_MAX_REPID_PING = buf_temp[20];
	FW_data.wdt[0].V_MAX_RESEND_PACET_RESET = buf_temp[21];
	FW_data.wdt[0].V_EN_WATCHDOG_CN_A = (buf_temp[23]) & 0x01;
	FW_data.wdt[0].V_EN_WATCHDOG_CN_B = (buf_temp[23] >> 1) & 0x01;
	FW_data.wdt[0].V_EN_WATCHDOG_CN_C = (buf_temp[23] >> 2) & 0x01;
	FW_data.wdt[0].V_EN_WATCHDOG = (buf_temp[23] >> 7) & 0x01;
	FW_data.wdt[0].V_N_OUT = (buf_temp[27] << 8) | buf_temp[26];
	FW_data.wdt[0].V_TYPE_LOGIC = buf_temp[24];

	len = read_mess_smtp((char*) (buf + 525), (uint8_t*) buf_temp);
	memset(FW_data.wdt[1].V_NAME, 0, 16);
	memcpy(FW_data.wdt[1].V_NAME, (char*) (buf_temp), len);
	char2_to_hex((char*) (buf + 589), (uint8_t*) buf_temp, 219);
	FW_data.wdt[1].V_IP_WDT_ADDR_CN_A[0] = buf_temp[0];
	FW_data.wdt[1].V_IP_WDT_ADDR_CN_A[1] = buf_temp[1];
	FW_data.wdt[1].V_IP_WDT_ADDR_CN_A[2] = buf_temp[2];
	FW_data.wdt[1].V_IP_WDT_ADDR_CN_A[3] = buf_temp[3];

	FW_data.wdt[1].V_IP_WDT_ADDR_CN_B[0] = buf_temp[4];
	FW_data.wdt[1].V_IP_WDT_ADDR_CN_B[1] = buf_temp[5];
	FW_data.wdt[1].V_IP_WDT_ADDR_CN_B[2] = buf_temp[6];
	FW_data.wdt[1].V_IP_WDT_ADDR_CN_B[3] = buf_temp[7];

	FW_data.wdt[1].V_IP_WDT_ADDR_CN_C[0] = buf_temp[8];
	FW_data.wdt[1].V_IP_WDT_ADDR_CN_C[1] = buf_temp[9];
	FW_data.wdt[1].V_IP_WDT_ADDR_CN_C[2] = buf_temp[10];
	FW_data.wdt[1].V_IP_WDT_ADDR_CN_C[3] = buf_temp[11];

	FW_data.wdt[1].V_T_SEND_PING = (buf_temp[13] << 8) | buf_temp[12];
	FW_data.wdt[1].V_TIME_RESEND_PING = (buf_temp[15] << 8) | buf_temp[14];
	FW_data.wdt[1].V_TIME_RESET_PULSE = (buf_temp[17] << 8) | buf_temp[16];
	FW_data.wdt[1].V_PAUSE_RESET_TO_REPID = (buf_temp[19] << 8) | buf_temp[18];
	FW_data.wdt[1].V_MAX_REPID_PING = buf_temp[20];
	FW_data.wdt[1].V_MAX_RESEND_PACET_RESET = buf_temp[21];
	FW_data.wdt[1].V_EN_WATCHDOG_CN_A = (buf_temp[23]) & 0x01;
	FW_data.wdt[1].V_EN_WATCHDOG_CN_B = (buf_temp[23] >> 1) & 0x01;
	FW_data.wdt[1].V_EN_WATCHDOG_CN_C = (buf_temp[23] >> 2) & 0x01;
	FW_data.wdt[1].V_EN_WATCHDOG = (buf_temp[23] >> 7) & 0x01;
	FW_data.wdt[1].V_N_OUT = (buf_temp[27] << 8) | buf_temp[26];
	FW_data.wdt[1].V_TYPE_LOGIC = buf_temp[24];

	nvs_flags.data_param = 1;

	httpd_resp_send_chunk(req, NULL, 0);
	return ESP_OK;
}

static esp_err_t setup_set_post_handler(httpd_req_t *req) {
	esp_err_t err;
	nvs_handle_t my_handle;
	httpd_resp_set_status(req, "303 See Other");
	httpd_resp_set_hdr(req, "Location", "\settings.html");
	httpd_resp_set_hdr(req, "Cache-Control",
			"no-store, no-cache, must-revalidate");
	httpd_resp_set_type(req, mime_sse);
	httpd_resp_set_hdr(req, "Connection", "Close");
	//char buf[1000] = { 0 };
	char buf_temp[256];
	uint8_t len;
	memset(buf, 0, size_1k_buff * 1044);
	int ret, remaining = req->content_len;

	if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {
//				if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
//					/* Retry receiving if timeout occurred */
//					continue;
//				}
//				return ESP_FAIL;
	}

	memset(FW_data.http.V_LOGIN, 0, 16);
	memcpy(FW_data.http.V_LOGIN, (char*) (buf + 19), buf[18]);
	memset(FW_data.http.V_PASSWORD, 0, 16);
	memcpy(FW_data.http.V_PASSWORD, (char*) (buf + 37), buf[36]);

	memset(FW_data.snmp.V_COMMUNITY, 0, 16);
	memcpy(FW_data.snmp.V_COMMUNITY, (char*) (buf + 55), buf[54]);
	memset(FW_data.snmp.V_COMMUNITY_WRITE, 0, 16);
	memcpy(FW_data.snmp.V_COMMUNITY_WRITE, (char*) (buf + 73), buf[72]);

	FW_data.net.V_IP_CONFIG[0] = buf[6];
	FW_data.net.V_IP_CONFIG[1] = buf[7];
	FW_data.net.V_IP_CONFIG[2] = buf[8];
	FW_data.net.V_IP_CONFIG[3] = buf[9];

	FW_data.net.V_IP_GET[0] = buf[10];
	FW_data.net.V_IP_GET[1] = buf[11];
	FW_data.net.V_IP_GET[2] = buf[12];
	FW_data.net.V_IP_GET[3] = buf[13];
	FW_data.sys.V_MASK_SOURCE = buf[14];
	FW_data.net.SAMMER = buf[15];

	memset(FW_data.http.V_LOGIN, 0, 16);
	memcpy(FW_data.http.V_LOGIN, (char*) (buf + 19), buf[18]);
	memset(FW_data.http.V_PASSWORD, 0, 16);
	memcpy(FW_data.http.V_PASSWORD, (char*) (buf + 37), buf[36]);

	memset(FW_data.snmp.V_COMMUNITY, 0, 16);
	memcpy(FW_data.snmp.V_COMMUNITY, (char*) (buf + 55), buf[54]);
	memset(FW_data.snmp.V_COMMUNITY_WRITE, 0, 16);
	memcpy(FW_data.snmp.V_COMMUNITY_WRITE, (char*) (buf + 73), buf[72]);

	FW_data.sys.V_IP_SOURCE[0] = buf[90];
	FW_data.sys.V_IP_SOURCE[1] = buf[91];
	FW_data.sys.V_IP_SOURCE[2] = buf[92];
	FW_data.sys.V_IP_SOURCE[3] = buf[93];

	FW_data.sys.V_MASK_SOURCE = buf[94];

	FW_data.net.V_DHCP = buf[95];

	FW_data.snmp.V_IP_SNMP[0] = buf[105];
	FW_data.snmp.V_IP_SNMP[1] = buf[106];
	FW_data.snmp.V_IP_SNMP[2] = buf[107];
	FW_data.snmp.V_IP_SNMP[3] = buf[108];

	FW_data.snmp.V_IP_SNMP_S[0] = buf[109];
	FW_data.snmp.V_IP_SNMP_S[1] = buf[110];
	FW_data.snmp.V_IP_SNMP_S[2] = buf[111];
	FW_data.snmp.V_IP_SNMP_S[3] = buf[112];

	FW_data.sys.V_NTP_CIRCL = buf[121];

	FW_data.net.facility = buf[130];
	FW_data.net.severity = buf[131];

	FW_data.snmp.V_PORT_SNMP = buf[132];

	FW_data.net.V_IP_NTP1[0] = buf[113];
	FW_data.net.V_IP_NTP1[1] = buf[114];
	FW_data.net.V_IP_NTP1[2] = buf[115];
	FW_data.net.V_IP_NTP1[3] = buf[116];

	if (FW_data.net.V_IP_NTP1[3] == 0) {
		memcpy(FW_data.net.N_NTP1, (char*) (buf + 513), buf[512]);
	} else {
		memset(FW_data.net.N_NTP1, 0, 32);
	}

	FW_data.net.V_IP_NTP2[0] = buf[117];
	FW_data.net.V_IP_NTP2[1] = buf[118];
	FW_data.net.V_IP_NTP2[2] = buf[119];
	FW_data.net.V_IP_NTP2[3] = buf[120];

	if (FW_data.net.V_IP_NTP2[3] == 0) {
		memcpy(FW_data.net.N_NTP2, (char*) (buf + 577), buf[576]);
	} else {
		memset(FW_data.net.N_NTP2, 0, 32);
	}

	//memset(buf_temp, 0, 256);
	//len = read_mess_smtp((char*) (buf + 512), (uint8_t*) buf_temp);

//	memset(buf_temp, 0, 256);
	//len = read_mess_smtp((char*) (buf + 576), (uint8_t*) buf_temp);

//	FW_data.net.V_IP_SYSL1[0] = buf[641] - 0x30;
//	FW_data.net.V_IP_SYSL1[1] = buf[643] - 0x30;
//	FW_data.net.V_IP_SYSL1[2] = buf[645] - 0x30;
//	FW_data.net.V_IP_SYSL1[3] = buf[647] - 0x30;
//
//	FW_data.net.V_IP_SYSL2[0] = buf[122];
//	FW_data.net.V_IP_SYSL2[1] = buf[123];
//	FW_data.net.V_IP_SYSL2[2] = buf[124];
//	FW_data.net.V_IP_SYSL2[3] = buf[125];

	FW_data.sys.V_L_TIME = buf[15];
	memcpy(FW_data.sys.V_Name_dev, (char*) (buf + 185), buf[184]);
	memcpy(FW_data.sys.V_CALL_DATA, (char*) (buf + 249), buf[248]);
	memcpy(FW_data.sys.V_GEOM_NAME, (char*) (buf + 313), buf[312]);

	FW_data.net.V_IP_DNS[0] = buf[376];
	FW_data.net.V_IP_DNS[1] = buf[377];
	FW_data.net.V_IP_DNS[2] = buf[378];
	FW_data.net.V_IP_DNS[3] = buf[379];

	memcpy(FW_data.net.N_NTP1, (char*) (buf + 513), buf[512]);
	memcpy(FW_data.net.N_NTP2, (char*) (buf + 577), buf[576]);

	printf("N_NTP1 %s\n\r", FW_data.net.N_NTP1);
	printf("N_NTP2 %s\n\r", FW_data.net.N_NTP2);

	FW_data.net.V_IP_SYSL[0] = buf[122];
	FW_data.net.V_IP_SYSL[1] = buf[123];
	FW_data.net.V_IP_SYSL[2] = buf[124];
	FW_data.net.V_IP_SYSL[3] = buf[125];

	printf("IP_SYSL0 %d.%d.%d.%d\n\r", FW_data.net.V_IP_SYSL[0],
			FW_data.net.V_IP_SYSL[1], FW_data.net.V_IP_SYSL[2],
			FW_data.net.V_IP_SYSL[3]);

	if (FW_data.net.V_IP_SYSL[3] == 0) {
		memcpy(FW_data.net.N_SLOG, (char*) (buf + 641), buf[640]);
	} else {
		memset(FW_data.net.N_SLOG, 0, 32);
	}

	FW_data.net.V_IP_SYSL1[0] = buf[704];
	FW_data.net.V_IP_SYSL1[1] = buf[705];
	FW_data.net.V_IP_SYSL1[2] = buf[706];
	FW_data.net.V_IP_SYSL1[3] = buf[707];

	printf("IP_SYSL1 %d.%d.%d.%d\n\r", FW_data.net.V_IP_SYSL1[0],
			FW_data.net.V_IP_SYSL1[1], FW_data.net.V_IP_SYSL1[2],
			FW_data.net.V_IP_SYSL1[3]);

	if (FW_data.net.V_IP_SYSL1[3] == 0) {
		memcpy(FW_data.net.V_N_SYSL1, (char*) (buf + 713), buf[712]);
	} else {
		memset(FW_data.net.V_N_SYSL1, 0, 32);
	}

	FW_data.net.V_IP_SYSL2[0] = buf[708];
	FW_data.net.V_IP_SYSL2[1] = buf[709];
	FW_data.net.V_IP_SYSL2[2] = buf[710];
	FW_data.net.V_IP_SYSL2[3] = buf[711];

	printf("IP_SYSL2 %d.%d.%d.%d\n\r", FW_data.net.V_IP_SYSL2[0],
			FW_data.net.V_IP_SYSL2[1], FW_data.net.V_IP_SYSL2[2],
			FW_data.net.V_IP_SYSL2[3]);

	if (FW_data.net.V_IP_SYSL2[3] == 0) {
		memcpy(FW_data.net.V_N_SYSL2, (char*) (buf + 779), buf[778]);
	} else {
		memset(FW_data.net.V_N_SYSL2, 0, 32);
	}

	printf("IP_SYSLN %s\n\r", FW_data.net.N_SLOG);
	printf("IP_SYSLN1 %s\n\r", FW_data.net.V_N_SYSL1);
	printf("IP_SYSLN2 %s\n\r", FW_data.net.V_N_SYSL2);

//		        FW_data.net.V_IP_CONFIG[0] = buf[6];
//				FW_data.net.V_IP_CONFIG[1] = buf[7];
//				FW_data.net.V_IP_CONFIG[2] = buf[8];
//				FW_data.net.V_IP_CONFIG[3] = buf[9];
//
//				FW_data.net.V_IP_GET[0] = buf[10];
//				FW_data.net.V_IP_GET[1] = buf[11];
//				FW_data.net.V_IP_GET[2] = buf[12];
//				FW_data.net.V_IP_GET[3] = buf[13];
//
//				uint32_t mask_temp = 0xffffffff << (32 - buf[14]);
//
//				FW_data.net.V_IP_MASK[3] = mask_temp & 0x000000ff;
//				FW_data.net.V_IP_MASK[2] = 0x000000ff & (mask_temp >> 8);
//				FW_data.net.V_IP_MASK[1] = 0x000000ff & (mask_temp >> 16);
//				FW_data.net.V_IP_MASK[0] = 0x000000ff & (mask_temp >> 24);
//				FW_data.smtp.V_FLAG_DEF_EMAIL = 1;
//				FW_data.smtp.V_FLAG_EN_EMAIL = 1;

	nvs_flags.data_param = 1;

	//	}

	// End response
	httpd_resp_send_chunk(req, NULL, 0);
	return ESP_OK;
}

static esp_err_t sendmail_set_post_handler(httpd_req_t *req) {
	esp_err_t err;
	nvs_handle_t my_handle;
	uint8_t len;
//	httpd_resp_set_status(req, HTTPD_200);
//	httpd_resp_set_status(req, "303 See Other");
	httpd_resp_set_status(req, "302 Temporary Redirect");
	httpd_resp_set_hdr(req, "Location", "\sendmail.html");
	httpd_resp_set_hdr(req, "Cache-Control",
			"no-store, no-cache, must-revalidate");
	httpd_resp_set_type(req, mime_html);
	httpd_resp_set_hdr(req, "Connection", "Close");
//	char buf[1000] = { 0 };
	char buf_temp[256] = { 0 };
	memset(buf, 0, 1000);
	memset(buf, 0, size_1k_buff * 1044);
	int ret, remaining = req->content_len;

	if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {

	}

	len = read_mess_smtp((char*) (buf + 5), (uint8_t*) buf_temp);

	memset(FW_data.smtp.V_EMAIL_ADDR, 0, 32);
	memcpy(FW_data.smtp.V_EMAIL_ADDR, (char*) (buf_temp), len);
	memset(buf_temp, 0, 256);
	char2_to_hex((char*) (buf + 133), (uint8_t*) buf_temp, 2);
	FW_data.smtp.V_FLAG_EMAIL_PORT = ((buf_temp[1] << 8) | buf_temp[0]);
	memset(buf_temp, 0, 256);
	len = read_mess_smtp((char*) (buf + 141), (uint8_t*) buf_temp);
	memset(FW_data.smtp.V_LOGIN_SMTP, 0, 32);
	memcpy(FW_data.smtp.V_LOGIN_SMTP, (char*) (buf_temp), len);
	memset(buf_temp, 0, 256);
	len = read_mess_smtp((char*) (buf + 237), (uint8_t*) buf_temp);
	memset(FW_data.smtp.V_PASSWORD_SMTP, 0, 32);
	memcpy(FW_data.smtp.V_PASSWORD_SMTP, (char*) (buf_temp), len);
	memset(buf_temp, 0, 256);
	len = read_mess_smtp((char*) (buf + 301), (uint8_t*) buf_temp);
	memset(FW_data.smtp.V_EMAIL_FROM, 0, 32);
	memcpy(FW_data.smtp.V_EMAIL_FROM, (char*) (buf_temp), len);
	memset(buf_temp, 0, 256);
	len = read_mess_smtp((char*) (buf + 397), (uint8_t*) buf_temp);
	memset(FW_data.smtp.V_EMAIL_TO, 0, 32);
	memcpy(FW_data.smtp.V_EMAIL_TO, (char*) (buf_temp), len);
	memset(buf_temp, 0, 256);
	len = read_mess_smtp((char*) (buf + 621), (uint8_t*) buf_temp);
	memset(FW_data.smtp.V_EMAIL_CC1, 0, 32);
	memcpy(FW_data.smtp.V_EMAIL_CC1, (char*) (buf_temp), len);
	memset(buf_temp, 0, 256);
	len = read_mess_smtp((char*) (buf + 717), (uint8_t*) buf_temp);
	memset(FW_data.smtp.V_EMAIL_CC2, 0, 32);
	memcpy(FW_data.smtp.V_EMAIL_CC2, (char*) (buf_temp), len);
	memset(buf_temp, 0, 256);
	len = read_mess_smtp((char*) (buf + 813), (uint8_t*) buf_temp);
	memset(FW_data.smtp.V_EMAIL_CC3, 0, 32);
	memcpy(FW_data.smtp.V_EMAIL_CC3, (char*) (buf_temp), len);
	uint8_t flag = buf[138] - 0x30;
	if ((flag & 0x01) != 0) {
		FW_data.smtp.V_FLAG_DEF_EMAIL = 1;
	} else {
		FW_data.smtp.V_FLAG_DEF_EMAIL = 0;
	}
	if ((flag & 0x02) != 0) {
		FW_data.smtp.V_FLAG_EN_EMAIL = 1;
	} else {
		FW_data.smtp.V_FLAG_EN_EMAIL = 0;
	}
	nvs_flags.data_param = 1;
//	save_data_blok();

	//	}

	// End response
	httpd_resp_send_chunk(req, NULL, 0);
	return ESP_OK;
}

// HTTP Error (404) Handler - Redirects all requests to the root page
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err) {
	// Set status
	httpd_resp_set_status(req, "302 Temporary Redirect");
	// Redirect to the "/" root directory
	httpd_resp_set_hdr(req, "Location", "/");
	// iOS requires content in the response to detect a captive portal, simply redirecting is not sufficient.
	httpd_resp_send(req, "Redirect to the captive portal",
	HTTPD_RESP_USE_STRLEN);

	ESP_LOGI(TAG_http, "Redirecting to root");
	return ESP_OK;
}

typedef struct {
	char *username;
	char *password;
} basic_auth_info_t;

static char* http_auth_basic(const char *username, const char *password) {
	int out;
	char *user_info = NULL;
	char *digest = NULL;
	size_t n = 0;
	asprintf(&user_info, "%s:%s", username, password);
	if (!user_info) {
		ESP_LOGE(TAG_http, "No enough memory for user information");
		return NULL;
	}
	esp_crypto_base64_encode(NULL, 0, &n, (const unsigned char*) user_info,
			strlen(user_info));

	/* 6: The length of the "Basic " string
	 * n: Number of bytes for a base64 encode format
	 * 1: Number of bytes for a reserved which be used to fill zero
	 */
	digest = calloc(1, 6 + n + 1);
	if (digest) {
		strcpy(digest, "Basic ");
		esp_crypto_base64_encode((unsigned char*) digest + 6, n, (size_t*) &out,
				(const unsigned char*) user_info, strlen(user_info));
	}
	free(user_info);
	return digest;
}

/* An HTTP GET handler */
static esp_err_t basic_auth_get_handler(httpd_req_t *req) {
	char *bufd = NULL;
	size_t buf_len = 0;
	basic_auth_info_t *basic_auth_info = req->user_ctx;

	buf_len = httpd_req_get_hdr_value_len(req, "Authorization") + 1;
	if (buf_len > 1) {
		bufd = calloc(1, buf_len);
		if (!bufd) {
			ESP_LOGE(TAG_http, "No enough memory for basic authorization");
			return ESP_ERR_NO_MEM;
		}

		if (httpd_req_get_hdr_value_str(req, "Authorization", bufd,
				buf_len) == ESP_OK) {
			ESP_LOGI(TAG_http, "Found header => Authorization: %s", bufd);
		} else {
			ESP_LOGE(TAG_http, "No auth value received");
		}

		char *auth_credentials = http_auth_basic(basic_auth_info->username,
				basic_auth_info->password);
		if (!auth_credentials) {
			ESP_LOGE(TAG_http,
					"No enough memory for basic authorization credentials");
			free(bufd);
			return ESP_ERR_NO_MEM;
		}

		if (strncmp(auth_credentials, bufd, buf_len)) {
			ESP_LOGE(TAG_http, "Not authenticated");
			httpd_resp_set_status(req, HTTPD_401);
			httpd_resp_set_type(req, "application/json");
			httpd_resp_set_hdr(req, "Connection", "keep-alive");
			httpd_resp_set_hdr(req, "WWW-Authenticate",
					"Basic realm=\"Hello\"");
			httpd_resp_send(req, NULL, 0);
		} else {
			ESP_LOGI(TAG_http, "Authenticated!");
			char *basic_auth_resp = NULL;
//            httpd_resp_set_status(req, HTTPD_200);HTTPD_302
//            httpd_resp_set_type(req, "application/json");
//            httpd_resp_set_hdr(req, "Connection", "keep-alive");
			httpd_resp_set_status(req, "302 Temporary Redirect");
			// Redirect to the "/" root directory
			httpd_resp_set_hdr(req, "Location", "/index.html");
			// iOS requires content in the response to detect a captive portal, simply redirecting is not sufficient.
			httpd_resp_send(req, "Redirect to the captive portal",
			HTTPD_RESP_USE_STRLEN);

			//    asprintf(&basic_auth_resp, "{\"authenticated\": true,\"user\": \"%s\"}", basic_auth_info->username);
			if (!basic_auth_resp) {
				ESP_LOGE(TAG_http,
						"No enough memory for basic authorization response");
				free(auth_credentials);
				free(bufd);
				return ESP_ERR_NO_MEM;
			}
			httpd_resp_send(req, basic_auth_resp, strlen(basic_auth_resp));
			free(basic_auth_resp);
		}
		free(auth_credentials);
		free(bufd);
	} else {
		ESP_LOGE(TAG_http, "No auth header received");
		httpd_resp_set_status(req, HTTPD_401);
		httpd_resp_set_type(req, "application/json");
		httpd_resp_set_hdr(req, "Connection", "keep-alive");
		httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Hello\"");
		httpd_resp_send(req, NULL, 0);
	}

	return ESP_OK;
}

static httpd_uri_t basic_auth = { .uri = "/", .method = HTTP_GET, .handler =
		basic_auth_get_handler, };

static void httpd_register_basic_auth(httpd_handle_t server) {
	basic_auth_info_t *basic_auth_info = calloc(1, sizeof(basic_auth_info_t));
	if (basic_auth_info) {
		basic_auth_info->username = FW_data.http.V_LOGIN;
		basic_auth_info->password = FW_data.http.V_PASSWORD;

		basic_auth.user_ctx = basic_auth_info;
		httpd_register_uri_handler(server, &basic_auth);
	}
}

static const httpd_uri_t setup_get_cgi = { .uri = "/setup_get.cgi", .method =
		HTTP_GET, .handler = setup_get_cgi_handler, .user_ctx = 0 };
//static const httpd_uri_t wdog_get_cgi = { .uri = "/wdog_get.cgi", .method =
//		HTTP_GET, .handler = wdog_get_cgi_handler, .user_ctx = 0 };

//static const httpd_uri_t email_send_test_cgi = { .uri = "/email_send_test.cgi",
//		.method = HTTP_GET, .handler = email_send_test_cgi_handler, .user_ctx =
//				0 };
//static const httpd_uri_t sendmail_get_cgi = { .uri = "/sendmail_get.cgi",
//		.method = HTTP_GET, .handler = sendmail_get_cgi_handler, .user_ctx = 0 };
//static const httpd_uri_t io_get_api = { .uri = "/io.cgi", .method = HTTP_GET,
//		.handler = io_cgi_api_handler, .user_ctx = 0 };

static const httpd_uri_t log_get_cgi = { .uri = "/log.cgi", .method = HTTP_GET,
		.handler = log_get_cgi_handler, .user_ctx = 0 };//({reset:19,suspend:19,report:0})

//static const httpd_uri_t notify_get_cgi = { .uri = "/notify_get.cgi", .method =
//		HTTP_GET, .handler = notify_get_cgi_handler, .user_ctx = 0 };

//static const httpd_uri_t sse_get_cgi = { .uri = "/sse.cgi", .method = HTTP_GET,
//		.handler = sse_get_cgi_handler, .user_ctx = 0 };
//static const httpd_uri_t eeprom_clone_get = { .uri = "/eeprom_clone_get.cgi",
//		.method = HTTP_GET, .handler = hello_get_handler,
/* Let's pass response string in user
 * context to demonstrate it's usage */
//		.user_ctx = 0 //(char*)(data__IOv3_setup_bin+data__IOv3_setup_bin_shift)
//		};
static const httpd_uri_t setup_set = { .uri = "/setup_set.cgi", .method =
		HTTP_POST, .handler = setup_set_post_handler, .user_ctx = NULL };

//static const httpd_uri_t wdog_set = { .uri = "/wdog_set.cgi", .method =
//		HTTP_POST, .handler = wdog_set_post_handler, .user_ctx = NULL };
//
//
//
//static const httpd_uri_t notify_set = { .uri = "/notify_set.cgi", .method =
//		HTTP_POST, .handler = notify_set_post_handler, .user_ctx = NULL };

//static const httpd_uri_t sendmail_set = { .uri = "/sendmail_set.cgi", .method =
//		HTTP_POST, .handler = sendmail_set_post_handler, .user_ctx = NULL };

static const httpd_uri_t ip_set = { .uri = "/ip_set.cgi", .method = HTTP_POST,
		.handler = ip_set_post_handler, .user_ctx = NULL };

static const httpd_uri_t rtcset = { .uri = "/rtcset.cgi", .method = HTTP_POST,
		.handler = rtcset_post_handler, .user_ctx = NULL };

httpd_handle_t start_webserver(void) {
	httpd_handle_t server = NULL;
	const httpd_config_t config = { .task_priority = tskIDLE_PRIORITY + 5,
			.stack_size = 48 * 1024, .core_id = tskNO_AFFINITY, .server_port =
					FW_data.http.V_WEB_PORT, .ctrl_port = 32768,
			.max_open_sockets = 7, .max_uri_handlers = 50, /*12*/
			.max_resp_headers = 8, .backlog_conn = 5, .lru_purge_enable = true, /**/
			.recv_wait_timeout = 5, .send_wait_timeout = 5, .global_user_ctx =
			NULL, .global_user_ctx_free_fn = NULL, .global_transport_ctx = NULL,
			.global_transport_ctx_free_fn = NULL, .open_fn = NULL, .close_fn =
			NULL, .uri_match_fn = NULL };
	//config.lru_purge_enable = true;

	// Start the httpd server
	ESP_LOGI(TAG_http, "Starting server on port: '%d'", config.server_port);
	if (httpd_start(&server, &config) == ESP_OK) {
		// Set URI handlers
		ESP_LOGI(TAG_http, "Registering URI handlers");

		httpd_register_uri_handler(server, &ip_set);
		httpd_register_uri_handler(server, &rtcset);
		httpd_register_uri_handler(server, &setup_set); // kotbazilioi@ngs.ru
//		httpd_register_uri_handler(server, &sendmail_set);
				//httpd_register_uri_handler(server, &eeprom_clone_get);
				//httpd_register_uri_handler(server, &wdog_get_cgi);
		httpd_register_uri_handler(server, &setup_get_cgi);
#if  MAIN_APP_OWB_H_ == 1
		http_var_init_owb(server);
		#endif

#if  MAIN_APP_IN_PORT == 1
		http_var_init_input(server);
#endif

#if  MAIN_APP_OUT_PORT == 1
		http_var_init_out(server);
#endif
#if  MAIN_APP_NOTIF == 1

		http_var_init_notify(server);
#endif
//		httpd_register_uri_handler(server, &email_send_test_cgi);
		httpd_register_uri_handler(server, &log_get_cgi); //
		//	httpd_register_uri_handler(server, &wdog_set);

		//	httpd_register_uri_handler(server, &sendmail_get_cgi);
		for (int i = 0; i < NP_HTML_HEADERS_NUMBER; ++i) {
			httpd_register_uri_handler(server, &np_html_uri[i]);
		}
		httpd_register_uri_handler(server, &np_html_uri_main);
		//	httpd_register_uri_handler(server, &np_html_uri_wdog);

		httpd_register_uri_handler(server, &np_html_uri_update);
		httpd_register_uri_handler(server, &np_html_uri_setings);
		//	httpd_register_uri_handler(server, &np_html_uri_sendmail);
		httpd_register_uri_handler(server, &np_html_uri_update_set);
		httpd_register_uri_handler(server, &np_html_uri_devname_cgi);
		httpd_register_uri_handler(server, &np_html_uri_reboot_cgi);
		httpd_register_err_handler(server, HTTPD_404_NOT_FOUND,
				http_404_error_handler);
		//	  #if CONFIG_EXAMPLE_BASIC_AUTH
		httpd_register_basic_auth(server);
		//	 #endif
		return server;
	}

	ESP_LOGI(TAG_http, "Error starting server!");
	return NULL;
}

void stop_webserver(httpd_handle_t server) {
	httpd_stop(server);
}

__attribute__((used)) void disconnect_handler(void *arg,
		esp_event_base_t event_base, int32_t event_id, void *event_data) {
	httpd_handle_t *server = (httpd_handle_t*) arg;
	if (*server) {
		ESP_LOGI(TAG_http, "Stopping webserver");
		stop_webserver(*server);
		*server = NULL;
	}
}

__attribute__((used)) void connect_handler(void *arg,
		esp_event_base_t event_base, int32_t event_id, void *event_data) {
	httpd_handle_t *server = (httpd_handle_t*) arg;
	if (*server == NULL) {
		ESP_LOGI(TAG_http, "Starting webserver");
		*server = start_webserver();
	}
}

