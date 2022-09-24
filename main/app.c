/*
 * app.c
 *
 *  Created on: 22 сент. 2021 г.
 *      Author: ivanov
 */
#include "includes_base.h"
#include "LOGS.h"

#include "sntp_task.h"
#include "smtp.h"
#include "tcpip_adapter.h"
#include "tcpip_adapter_types.h"
#include "app.h"
#include "config_pj.h"
#include "ping.h"
#include "private_mib.h"
#include "../main/termo/app_owb.h"
#include "syslog.h"
#include "lwip/ip4_addr.h"
#include "lwip/dns.h"
#include "smtp.h"
#include "../main/input/input.h"
#include "../main/output/output.h"
#include "http_var.h"

#include "snmp.h"

TaskHandle_t xHandleNTP = NULL;

uint8_t chipid[6];
uint32_t serial_id;

//void gpio1_task(void *pvParameters) {
//	gpio_set_direction(PORT_O2, GPIO_MODE_OUTPUT);
//	gpio_set_direction(PORT_I0, GPIO_MODE_INPUT);
//	while (1) {
//		//	IN_PORT[0]=gpio_get_level(PORT_I0);
//		if (S_gpio_port1 != NULL) {
//
//			if (xSemaphoreTake(S_gpio_port1, (TickType_t) 100) == pdTRUE) {
//				gpio_set_level(PORT_O1, port_data[0]);
//
//				xSemaphoreGive(S_gpio_port1);
//			} else {
//				/* We could not obtain the semaphore and can therefore not access
//				 the shared resource safely. */
//			}
//		}
//		vTaskDelay(300 / portTICK_PERIOD_MS);
//	}
//}
//void gpio2_task(void *pvParameters) {
//	gpio_set_direction(PORT_O2, GPIO_MODE_OUTPUT);
//	gpio_set_direction(PORT_I1, GPIO_MODE_INPUT);
//	while (1) {
//		//	IN_PORT[1]=gpio_get_level(PORT_I1);
//		if (S_gpio_port2 != NULL) {
//
//			if (xSemaphoreTake(S_gpio_port2, (TickType_t) 100) == pdTRUE) {
//				gpio_set_level(PORT_O2, port_data[1]);
//
//				xSemaphoreGive(S_gpio_port2);
//			} else {
//				/* We could not obtain the semaphore and can therefore not access
//				 the shared resource safely. */
//			}
//		}
//		vTaskDelay(300 / portTICK_PERIOD_MS);
//	}
//}

int syslog_dns_found(const char *hostname, ip_addr_t *ipaddr, void *arg) {
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
void sett_task(void *pvParameters) {
	err_t err;
	flag_global_save_log = xSemaphoreCreateMutex();
	ip_addr_t ip_syslog;
	ip_addr_t ip_syslog1;
	ip_addr_t ip_syslog2;
	char syslog_server[32] = "ya.ru";
	xSemaphoreGive(flag_global_save_log);

	log_sett_save_mess(0);

if (FW_data.net.V_IP_SYSL[3]!=0)
{
	dns_gethostbyname(FW_data.net.N_SLOG, &ip_syslog, syslog_dns_found, &err);

}
else
{
	err=ERR_CONN;
}


if (err != ERR_OK) {
	IP4_ADDR(&ip4_syslog, FW_data.net.V_IP_SYSL[0],
			FW_data.net.V_IP_SYSL[1], FW_data.net.V_IP_SYSL[2],
			FW_data.net.V_IP_SYSL[3]);
	ip_syslog.type = 4;
	ip_syslog.u_addr.ip4.addr = ip4_syslog.addr;
}





if (FW_data.net.V_IP_SYSL1[3]!=0)
{
	dns_gethostbyname(FW_data.net.N_SLOG, &ip_syslog1, syslog_dns_found, &err);

}
else
{
	err=ERR_CONN;
}


if (err != ERR_OK) {
	IP4_ADDR(&ip4_syslog, FW_data.net.V_IP_SYSL1[0],
			FW_data.net.V_IP_SYSL1[1], FW_data.net.V_IP_SYSL1[2],
			FW_data.net.V_IP_SYSL1[3]);
	ip_syslog1.type = 4;
	ip_syslog1.u_addr.ip4.addr = ip4_syslog.addr;
}



if (FW_data.net.V_IP_SYSL2[3]!=0)
{
	dns_gethostbyname(FW_data.net.N_SLOG, &ip_syslog2, syslog_dns_found, &err);

}
else
{
	err=ERR_CONN;
}


if (err != ERR_OK) {
	IP4_ADDR(&ip4_syslog, FW_data.net.V_IP_SYSL2[0],
			FW_data.net.V_IP_SYSL2[1], FW_data.net.V_IP_SYSL2[2],
			FW_data.net.V_IP_SYSL2[3]);
	ip_syslog2.type = 4;
	ip_syslog2.u_addr.ip4.addr = ip4_syslog.addr;
}



	syslog_init(ip_syslog,ip_syslog1,ip_syslog2);


	xTaskCreate(&vTaskNTP, "vTaskNTP", 2048, NULL, 5, &xHandleNTP);

	xTaskCreate(&start_task, "start_task", 12048, NULL, 10, NULL);
	log_sett_save_mess(SETT_START);
	log_update_save_mess(UPD_START);
	vTaskDelete(NULL);
}
void start_task(void *pvParameters) {

	err_t err;
	esp_mac_type_t eth = 3;
	esp_read_mac(chipid, eth);
	serial_id = (chipid[3] << 24) | (chipid[2] << 16) | (chipid[1] << 8)
			| (chipid[0]);

	//flag_global_save_log = xSemaphoreCreateMutex();
	//vSemaphoreCreateBinary( flag_global_save_log );

	xTaskCreate(&mdns_example_task, "mdns_example_task", 2048, NULL, 5, NULL);
	xTaskCreate(&nvs_task, "nvs_task", 4096, NULL, 5, NULL);

#if MAIN_APP_OWB_H_ == 1
	xTaskCreate(&app_owb, "app_owb", 4096, NULL, 10, NULL);
#endif

#if MAIN_APP_IN_PORT == 1
	xTaskCreate(&input_port, "input_port", 4096, NULL, 10, NULL);
	vTaskDelay(10 / portTICK_PERIOD_MS);
#endif

#if MAIN_APP_OUT_PORT == 1
	xTaskCreate(&output_port, "output_port", 4096, NULL, 10, NULL);
	vTaskDelay(10 / portTICK_PERIOD_MS);
#endif

	xTaskCreate(&log_task, "log_task", 2024, NULL, 10, NULL);

#if MAIN_APP_NOTIF == 1
	xTaskCreate(&notify_app, "notify_app", 4096, NULL, 10, NULL);
#endif

#if MAIN_APP_SMTP == 1
	//xTaskCreate(&send_smtp_task, "send_smtp_task", 4096, NULL, 10, NULL);
#endif

#if MAIN_APP_PING == 1
	ping_init();
#endif

	vTaskDelay(1000 / portTICK_PERIOD_MS);


//	gpio_set_direction(OUT_POW_EN, GPIO_MODE_OUTPUT);
//	gpio_set_level(OUT_POW_EN, 1);


	printf("all app run\n\r");
	timeup = timeinfo.tm_sec + timeinfo.tm_min * 60 + timeinfo.tm_hour * 60 * 60
			+ timeinfo.tm_yday * 60 * 60 * 24
			+ (timeinfo.tm_year - 70) * 60 * 60 * 8766;
	vTaskDelete(NULL);
}

