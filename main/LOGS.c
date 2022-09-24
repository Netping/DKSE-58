//#include "main.h"
#include <string.h>
#include "LOGS.h"
#include "nvs_task.h";
#include "app.h";
#include "smtp.h"
#include "config_pj.h"
#include "ping.h"
#include "syslog.h"
#include "../main/input/input.h"
#include "../main/output/output.h"
//#include "flash_if.h"
//#include "syslog.h"
log_reple_t reple_to_save;
log_reple_t reple_to_email;
// RTC_DateTypeDef dates;
// RTC_TimeTypeDef times;
struct tm timeinfo;
time_t now;
uint32_t timeup;
SemaphoreHandle_t flag_global_save_log = NULL;

void GET_reple(log_reple_t *reple) {

	time_t now;
	time(&now);
	localtime_r(&now, &timeinfo);

	reple->reple_hours = timeinfo.tm_hour;
	reple->reple_minuts = timeinfo.tm_min;
	reple->reple_seconds = timeinfo.tm_sec;
	reple->dweek = timeinfo.tm_wday;
	reple->day = timeinfo.tm_mday;
	reple->month = timeinfo.tm_mon;
	reple->year = 2000 + timeinfo.tm_year - 100;
	reple->dicr = 0x0;
	if (reple->year == 2000) {
		reple->year = 2001;
	}

}
void decode_reple(char *out, log_reple_t *reple) {
	char out_small[200] = { 0 };
//  memset()
	sprintf(out_small, "%02d.%02d.%d  %02d:%02d:%02d    ", reple->day,
			reple->month, reple->year, reple->reple_hours, reple->reple_minuts,
			reple->reple_seconds);
	strcat(out, out_small);
	memset(out_small, 0, 200);
	swich_mess_event(reple->type_event, out_small);
	strcat(out, out_small);
}
void decode_reple_en(char *out, log_reple_t *reple) {
	char out_small[200] = { 0 };
//  memset()
	sprintf(out_small, "%02d.%02d.%d  %02d:%02d:%02d    ", reple->day,
			reple->month, reple->year, reple->reple_hours, reple->reple_minuts,
			reple->reple_seconds);
	strcat(out, out_small);
	memset(out_small, 0, 200);
	swich_mess_event_en(reple, out_small);
	strcat(out, out_small);
}

void form_reple_to_smtp(event_struct_t cfg) {
	char mess_syslog[200] = { 0 };

	GET_reple(&reple_to_save);
	decode_reple_en(mess_syslog, &reple_to_save);

	////////////// xSemaphoreTake (flag_global_save_log, (TickType_t) 100);
}

uint8_t logs_read(uint16_t n_mess, char *mess) {
	char name_str[10] = { 0 };
	uint8_t logs_data[13] = { 0 };
	uint8_t good_fl = 0;
	log_reple_t reply_read;
	uint16_t size = 12;
	memset(mess, 0, 256);
	sprintf(name_str, "mess%d", n_mess);
	esp_err_t err = nvs_open_from_partition("nvs", "storage", NVS_READWRITE,
			&nvs_data_handle);

	err = err | nvs_get_blob(nvs_data_handle, name_str, logs_data, &size);

	memcpy(((uint8_t*) &(reply_read)), logs_data, 12);

	if (reply_read.source != NULLS) {
		for (uint8_t ct_l = 0; ct_l < 16; ct_l++) {
			if (FW_data.log.source[ct_l] == reply_read.source) {
				FW_data.log.n_slot = ct_l;
				ct_l = 16;
				good_fl = 1;
			}
		}
		if (good_fl == 1) {
			void (*swich_mess)(char *messege, log_reple_t *reply_read);
			swich_mess = FW_data.log.fswich_point[FW_data.log.n_slot];
			swich_mess(mess, &reply_read);
		}
	}
	err = nvs_commit(nvs_data_handle);
	//printf((err != ESP_OK) ? "Failed mess save!\n" : "\n");
	nvs_close(nvs_data_handle);
	return 0;
}
//enum sett_event_t {
//  SETT_START,
//  SETT_EDIT,
//  SETT_NDHCP,
//  SETT_EDHCP,
//  SETT_DNS,
//  SETT_ERR
//};
void log_swich_sett(char *out, log_reple_t *input_reply) {
	char out_small[200] = { 0 };
	sprintf(out_small, "%02d.%02d.%d  %02d:%02d:%02d [network] ",
			input_reply->day, input_reply->month, input_reply->year,
			input_reply->reple_hours, input_reply->reple_minuts,
			input_reply->reple_seconds);
	strcat(out, out_small);
	switch (input_reply->type_event) {
	case SETT_START:
		sprintf(out_small, "Старт модуля настройки\n\r");
		break;
	case SETT_EDIT:
		sprintf(out_small, "Изменение конфигурации сетевых настроек\n\r");

		break;
	case SETT_NDHCP:
		sprintf(out_small,
				"Получение новой сетевой конфигурации от DHCP сервера\n\r");

		break;
	case SETT_EDHCP:
		sprintf(out_small,
				"Ошибка в получение  сетевой конфигурации от DHCP сервера\n\r");

		break;
	case SETT_DNS:
		sprintf(out_small,
				"Изменения доступности указанного адреса DNS сервера по ICMP\n\r");

		break;
	case SETT_DNS_GETE:
			sprintf(out_small,
					"Ошибка получения адреса по DNS имени \n\r");

			break;
	case SETT_ERR:
		sprintf(out_small, "ошибки возникающие в модуле ");

			break;
	default:
		sprintf(out_small, "ошибка журнала ");
	}
	strcat(out, out_small);
}
//enum update_event_t {
//  UPD_START,
//  UPD_GOOD,
//  UPD_BAD,
//  UPD_ERR
//};
void log_swich_update(char *out, log_reple_t *input_reply) {
	char out_small[200] = { 0 };
	sprintf(out_small, "%02d.%02d.%d  %02d:%02d:%02d [update] ",
			input_reply->day, input_reply->month, input_reply->year,
			input_reply->reple_hours, input_reply->reple_minuts,
			input_reply->reple_seconds);
	strcat(out, out_small);
	switch (input_reply->type_event) {
	case UPD_START:
		sprintf(out_small, "Старт модуля обновления\n\r");
		break;
	case UPD_GOOD:
		sprintf(out_small, "Процедура обновления ПО прошла успешно\n\r");

		break;
	case UPD_BAD:
		sprintf(out_small,
				"Процедура обновления ПО прошла неуспешно\n\r");

		break;
	case UPD_ERR:
		sprintf(out_small,
								"Ошибка в модуле обновления\n\r");

			break;
	default:
		sprintf(out_small,
						"Ошибка журнала\n\r");

	}
	strcat(out, out_small);
}
void log_swich_log(char *out, log_reple_t *input_reply) {
	char out_small[200] = { 0 };
	sprintf(out_small, "%02d.%02d.%d  %02d:%02d:%02d [logs] ",
			input_reply->day, input_reply->month, input_reply->year,
			input_reply->reple_hours, input_reply->reple_minuts,
			input_reply->reple_seconds);
	strcat(out, out_small);
	switch (input_reply->type_event) {
	case LOG_START:
		sprintf(out_small, "Старт модуля журнала\n\r");
		break;
	case LOG_ERR:
		sprintf(out_small,	"Ошибка в модуле журнала\n\r");

			break;
	case SLOG_ERR:
			sprintf(out_small,	"Ошибка отправки по одному из адресов syslog\n\r");

				break;

	default:
		sprintf(out_small,
						"Ошибка журнала\n\r");

	}
	strcat(out, out_small);
}

void log_sett_save_mess(uint8_t event) {
//	if (xSemaphoreTake(flag_global_save_log, (TickType_t) 1000)
	//			== pdTRUE)
	xSemaphoreTake(flag_global_save_log, (TickType_t) 1000);
	{
		log_reple_t save_reply;
		GET_reple(&save_reply);
		save_reply.line = 0;
		save_reply.type_event = event;
		save_reply.source = SETT;
		save_reple_log(save_reply);

	}
	xSemaphoreGive(flag_global_save_log);

}
void log_log_save_mess(uint8_t event) {
	xSemaphoreTake(flag_global_save_log, (TickType_t) 1000);
	{
		log_reple_t save_reply;
		GET_reple(&save_reply);
		save_reply.line = 0;
		save_reply.type_event = event;
		save_reply.source = LOG;
		save_reple_log(save_reply);

	}
	xSemaphoreGive(flag_global_save_log);

}
void log_start_sett(void) {
	xSemaphoreTake(flag_global_save_log, (TickType_t) 1000);
	{
		for (uint8_t ct_l = 0; ct_l < 16; ct_l++) {
			if (FW_data.log.source[ct_l] == NULLS) {
				FW_data.log.source[ct_l] = SETT;

				//	memcpy(&(FW_data.log.header[ct_l][0]), (char*)"[digital-inputs]",sizeof("[digital-inputs]"));
				FW_data.log.fswich_point[ct_l] = log_swich_sett;
				ct_l = 17;
			}
		}
	}
	xSemaphoreGive(flag_global_save_log);

}
void log_start_log(void) {
	xSemaphoreTake(flag_global_save_log, (TickType_t) 1000);
	{
		for (uint8_t ct_l = 0; ct_l < 16; ct_l++) {
			if (FW_data.log.source[ct_l] == NULLS) {
				FW_data.log.source[ct_l] = LOG;

				//	memcpy(&(FW_data.log.header[ct_l][0]), (char*)"[digital-inputs]",sizeof("[digital-inputs]"));
				FW_data.log.fswich_point[ct_l] = log_swich_log;
				ct_l = 17;
			}
		}
	}
	xSemaphoreGive(flag_global_save_log);
	log_log_save_mess(LOG_START);
}
void log_update_save_mess(uint8_t event) {
//	if (xSemaphoreTake(flag_global_save_log, (TickType_t) 1000)
	//			== pdTRUE)
	xSemaphoreTake(flag_global_save_log, (TickType_t) 1000);
	{
		log_reple_t save_reply;
		GET_reple(&save_reply);
		save_reply.line = 0;
		save_reply.type_event = event;
		save_reply.source = UPD;
		save_reple_log(save_reply);

	}
	xSemaphoreGive(flag_global_save_log);

}

void log_start_update(void) {
	xSemaphoreTake(flag_global_save_log, (TickType_t) 1000);
	{
		for (uint8_t ct_l = 0; ct_l < 16; ct_l++) {
			if (FW_data.log.source[ct_l] == NULLS) {
				FW_data.log.source[ct_l] = UPD;

				//	memcpy(&(FW_data.log.header[ct_l][0]), (char*)"[digital-inputs]",sizeof("[digital-inputs]"));
				FW_data.log.fswich_point[ct_l] = log_swich_update;
				ct_l = 17;
			}
		}
	}
	xSemaphoreGive(flag_global_save_log);

}


void save_reple_log(log_reple_t reple2) {
	char name_str[10];
	uint16_t number_mess;
	char mess[256] = { 0 };
	uint8_t good_fl = 0;

	if (reple2.source != NULLS) {
		for (uint8_t ct_l = 0; ct_l < 16; ct_l++) {
			if (FW_data.log.source[ct_l] == reple2.source) {
				FW_data.log.n_slot = ct_l;
				ct_l = 16;
				good_fl = 1;
			}
		}
		if (good_fl == 1) {
			void (*swich_mess)(char *messege, log_reple_t *reple2);
			swich_mess = FW_data.log.fswich_point[FW_data.log.n_slot];
			swich_mess(mess, &reple2);
		}
	}
	syslog_printf(mess);

	esp_err_t err = nvs_open_from_partition("nvs", "storage", NVS_READWRITE,
			&nvs_data_handle);

	err = nvs_get_u16(nvs_data_handle, "number_mess", &number_mess);
	if ((err == ESP_ERR_NVS_PART_NOT_FOUND) || (err == ESP_ERR_NVS_NOT_FOUND)) {
		err = ESP_OK;
		number_mess = 0;
	}
	printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

	if ((err == ESP_OK) && (number_mess < max_log_mess))
		number_mess++;
	if ((number_mess == max_log_mess) || (number_mess > max_log_mess))
		number_mess = 0;

	err = nvs_commit(nvs_data_handle);
	printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

	sprintf(name_str, "mess%d", number_mess);

	err = err
			| nvs_set_blob(nvs_data_handle, name_str, (uint8_t*) &(reple2),
					sizeof(reple2));

	err = nvs_set_u16(nvs_data_handle, "number_mess", number_mess);
	printf("save %d log mess committing updates in NVS ... ", number_mess);
	err = nvs_commit(nvs_data_handle);
	printf(
			(err != ESP_OK) ?
					"Failed mess save log !\n" : "Done mess save log \n");
	nvs_close(nvs_data_handle);
}

void log_task(void *pvParameters) {
	uint8_t ct;
	log_start_sett();
	log_start_update();
	log_start_log();
	while (1) {

		vTaskDelay(300 / portTICK_PERIOD_MS);

	}
}
