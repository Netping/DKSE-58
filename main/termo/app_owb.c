/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "..\main\config_pj.h"

#if  MAIN_APP_OWB_H_ == 1
#include <esp_event.h>

#include "http_var.h"

#include "private_mib.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

#include "app_owb.h"
#include "nvs_task.h"
#include "private_mib.h"
#include "update.h"
#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "esp_partition.h"

#include "..\main\html_struct.h"

#define get_name(x) #x
#include "..\termo\owb.h"
FW_termo_t termo[MAX_DEVICES];

#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_12_BIT)

const httpd_uri_t termo_get_api = { .uri = "/termo_get.cgi", .method = HTTP_GET,
		.handler = termo_get_cgi_api_handler, .user_ctx = 0 };

const httpd_uri_t termo_data_api = { .uri = "/termo_data.cgi", .method =
		HTTP_GET, .handler = termo_data_cgi_api_handler, .user_ctx = 0 };

const httpd_uri_t np_html_uri_termo = { "/termo.html", HTTP_GET,
		np_http_get_handler, (void*) &_html_page_termo_html };

float readings[MAX_DEVICES] = { 0 };
s32_t OID_termo[] = { 1, 3, 6, 1, 4, 1, 25728, 8800, 2, 0, 1 };

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
void app_owb(void *pvParameters) {
	// Override global log level
	esp_log_level_set("*", ESP_LOG_INFO);

	// To debug, use 'make menuconfig' to set default Log level to DEBUG, then uncomment:
	//esp_log_level_set("owb", ESP_LOG_DEBUG);
	//esp_log_level_set("ds18b20", ESP_LOG_DEBUG);

	// Stable readings require a brief period before communication
	vTaskDelay(2000.0 / portTICK_PERIOD_MS);

	// Create a 1-Wire bus, using the RMT timeslot driver
	OneWireBus *owb;
	owb_rmt_driver_info rmt_driver_info;
	owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1,
			RMT_CHANNEL_0);
	owb_use_crc(owb, true);  // enable CRC check for ROM code

	// Find all connected devices
	printf("Find devices:\n");
	OneWireBus_ROMCode device_rom_codes[MAX_DEVICES] = { 0 };
	int num_devices = 0;
	OneWireBus_SearchState search_state = { 0 };
	bool found = false;
	owb_search_first(owb, &search_state, &found);
	while (found) {
		char rom_code_s[17];
		owb_string_from_rom_code(search_state.rom_code, rom_code_s,
				sizeof(rom_code_s));
		printf("  %d : %s\n", num_devices, rom_code_s);
		device_rom_codes[num_devices] = search_state.rom_code;
		++num_devices;
		owb_search_next(owb, &search_state, &found);
	}
	printf("Found %d device%s\n", num_devices, num_devices == 1 ? "" : "s");

	// In this example, if a single device is present, then the ROM code is probably
	// not very interesting, so just print it out. If there are multiple devices,
	// then it may be useful to check that a specific device is present.

	if (num_devices == 1) {
		// For a single device only:
		OneWireBus_ROMCode rom_code;
		owb_status status = owb_read_rom(owb, &rom_code);
		if (status == OWB_STATUS_OK) {
			char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
			owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
			printf("Single device %s present\n", rom_code_s);
		} else {
			printf("An error occurred reading ROM code: %d", status);
		}
	} else {
		// Search for a known ROM code (LSB first):
		// For example: 0x1502162ca5b2ee28
		OneWireBus_ROMCode known_device = { .fields.family = { 0x28 },
				.fields.serial_number = { 0xee, 0xb2, 0xa5, 0x2c, 0x16, 0x02 },
				.fields.crc = { 0x15 }, };
		char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
		owb_string_from_rom_code(known_device, rom_code_s, sizeof(rom_code_s));
		bool is_present = false;

		owb_status search_status = owb_verify_rom(owb, known_device,
				&is_present);
		if (search_status == OWB_STATUS_OK) {
			printf("Device %s is %s\n", rom_code_s,
					is_present ? "present" : "not present");
		} else {
			printf("An error occurred searching for known device: %d",
					search_status);
		}
	}

	// Create DS18B20 devices on the 1-Wire bus
	DS18B20_Info *devices[MAX_DEVICES] = { 0 };
	for (int i = 0; i < num_devices; ++i) {
		DS18B20_Info *ds18b20_info = ds18b20_malloc();  // heap allocation
		devices[i] = ds18b20_info;

		if (num_devices == 1) {
			printf("Single device optimisations enabled\n");
			ds18b20_init_solo(ds18b20_info, owb);      // only one device on bus
		} else {
			ds18b20_init(ds18b20_info, owb, device_rom_codes[i]); // associate with bus and device
		}
		ds18b20_use_crc(ds18b20_info, true);    // enable CRC check on all reads
		ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);
	}

//    // Read temperatures from all sensors sequentially
//    while (1)
//    {
//        printf("\nTemperature readings (degrees C):\n");
//        for (int i = 0; i < num_devices; ++i)
//        {
//            float temp = ds18b20_get_temp(devices[i]);
//            printf("  %d: %.3f\n", i, temp);
//        }
//        vTaskDelay(1000 / portTICK_PERIOD_MS);
//    }

	// Check for parasitic-powered devices
	bool parasitic_power = false;
	ds18b20_check_for_parasite_power(owb, &parasitic_power);
	if (parasitic_power) {
		printf("Parasitic-powered devices detected");
	}

	// In parasitic-power mode, devices cannot indicate when conversions are complete,
	// so waiting for a temperature conversion must be done by waiting a prescribed duration
	owb_use_parasitic_power(owb, parasitic_power);

#ifdef CONFIG_ENABLE_STRONG_PULLUP_GPIO
    // An external pull-up circuit is used to supply extra current to OneWireBus devices
    // during temperature conversions.
    owb_use_strong_pullup_gpio(owb, CONFIG_STRONG_PULLUP_GPIO);
#endif

	// Read temperatures more efficiently by starting conversions on all devices at the same time
	int errors_count[MAX_DEVICES] = { 0 };
	int sample_count = 0;
	if (num_devices > 0) {
		TickType_t last_wake_time = xTaskGetTickCount();

		while (1) {
			ds18b20_convert_all(owb);

			// In this application all devices use the same resolution,
			// so use the first device to determine the delay
			ds18b20_wait_for_conversion(devices[0]);

			// Read the results immediately after conversion otherwise it may fail
			// (using printf before reading may take too long)

			DS18B20_ERROR errors[MAX_DEVICES] = { 0 };

			for (int i = 0; i < num_devices; ++i) {
				errors[i] = ds18b20_read_temp(devices[i], &readings[i]);
				termo[i].temper = readings[i];
				termo[i].ftemper = readings[i];
				termo[i].id[0] = device_rom_codes[i].bytes[7];
				termo[i].id[1] = device_rom_codes[i].bytes[6];
				termo[i].id[2] = device_rom_codes[i].bytes[5];
				termo[i].id[3] = device_rom_codes[i].bytes[4];

				termo[i].id[4] = device_rom_codes[i].bytes[3];
				termo[i].id[5] = device_rom_codes[i].bytes[2];
				termo[i].id[6] = device_rom_codes[i].bytes[1];
				termo[i].id[7] = device_rom_codes[i].bytes[0];
				if (termo[i].t_dw >= termo[i].temper) {
					termo[i].status = 1;
				} else if (termo[i].t_up <= termo[i].temper) {
					termo[i].status = 3;
				} else {
					termo[i].status = 2;
				}
				if (termo[i].status_old != termo[i].status) {
					send_mess_trap_termo(OID_termo, i);
					termo[i].status_old = termo[i].status;
				}

			}


//			typedef enum
//			{
//			    DS18B20_ERROR_UNKNOWN = -1,  ///< An unknown error occurred, or the value was not set
//			    DS18B20_OK = 0,        ///< Success
//			    DS18B20_ERROR_DEVICE,  ///< A device error occurred
//			    DS18B20_ERROR_CRC,     ///< A CRC error occurred
//			    DS18B20_ERROR_OWB,     ///< A One Wire Bus error occurred
//			    DS18B20_ERROR_NULL,    ///< A parameter or value is NULL
//			} DS18B20_ERROR;



			// Print results in a separate loop, after all have been read
			printf("\nTemperature readings (degrees C): sample %d\n",
					++sample_count);

			for (int i = 0; i < num_devices; ++i) {
				if (errors[i] != DS18B20_OK) {
					++errors_count[i];
				}

				printf("  %d: %.1f    %d errors\n", i, readings[i],
						errors_count[i]);
			}

			vTaskDelayUntil(&last_wake_time,
			SAMPLE_PERIOD / portTICK_PERIOD_MS);
		}
	} else {
		printf("\nNo DS18B20 devices detected!\n");
	}

	// clean up dynamically allocated data
	for (int i = 0; i < num_devices; ++i) {
		ds18b20_free(&devices[i]);
	}
	owb_uninitialize(owb);
	while (1) {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	//  esp_restart();
}

void lwip_privmib_init_termo(void) {

	u8_t i;

	memset(termo, 0, sizeof(termo));

	printf("SNMP private MIB start, detecting sensors.\n");

	for (i = 0; i < TERMO_COUNT; i++) {
		termo_snmp[i].num = (u8_t) (i + 1);
		snprintf(termo_snmp[i].file, sizeof(termo_snmp[i].file), "%d.txt", i);
		/* initialize sensor value to != zero */
		termo_snmp[i].value = 11 * (i + 1);
		/* !SENSORS_USE_FILES */
	}

}

esp_err_t termo_get_cgi_api_handler(httpd_req_t *req) {
	//#warning "******** where is no error processing !  *******"
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

	char buf_temp[256];
	sprintf(buf,
			"packfmt={name:{offs:0,len:34},termo_id:{offs:34,len:34},__len:68};");



	sprintf(buf_temp, " var data=[");
	strcat(buf, buf_temp);

	for (uint8_t ct = 0; ct < max_sensor - 1; ct++) {
		sprintf(buf_temp, "{name:\"%s\",", termo[ct].name);
		strcat(buf, buf_temp);
		printf("Name%d=%s\n\r",ct,termo[ct].name);
		sprintf(buf_temp, "termo_id:\"%02x%02x%02x%02x%02x%02x%02x%02x\",",
				termo[ct].id[0], termo[ct].id[1], termo[ct].id[2],
				termo[ct].id[3], termo[ct].id[4], termo[ct].id[5],
				termo[ct].id[6], termo[ct].id[7]);
		strcat(buf, buf_temp);
		printf("termo_id%d=%02x%02x%02x%02x%02x%02x%02x%02x\n\r",ct,termo[ct].id[0], termo[ct].id[1], termo[ct].id[2],
				termo[ct].id[3], termo[ct].id[4], termo[ct].id[5],
				termo[ct].id[6], termo[ct].id[7]);

		sprintf(buf_temp, "temp:%d}",
				(uint16_t) (termo[ct].temper * 10 + termo[ct].ftemper));
		strcat(buf, buf_temp);
		printf("temp%d=%d\n\r",ct,(uint16_t) (termo[ct].temper * 10 + termo[ct].ftemper));
		sprintf(buf_temp, ",");
		strcat(buf, buf_temp);
	}
	sprintf(buf_temp, "{name:\"%s\",", termo[max_sensor - 1].name);
	strcat(buf, buf_temp);
	sprintf(buf_temp, "termo_id:\"%02x%02x%02x%02x%02x%02x%02x%02x\",",
			termo[max_sensor - 1].id[0], termo[max_sensor - 1].id[1],
			termo[max_sensor - 1].id[2], termo[max_sensor - 1].id[3],
			termo[max_sensor - 1].id[4], termo[max_sensor - 1].id[5],
			termo[max_sensor - 1].id[6], termo[max_sensor - 1].id[7]);
	strcat(buf, buf_temp);
	sprintf(buf_temp, "temp:%d}",
			(uint16_t) (termo[max_sensor - 1].temper * 10
					+ termo[max_sensor - 1].ftemper));
	strcat(buf, buf_temp);

	sprintf(buf_temp, "];");
	strcat(buf, buf_temp);

	sprintf(buf_temp, "var devname='%s';", FW_data.sys.V_Name_dev);
		strcat(buf, buf_temp);

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
		sprintf(buf_temp, "var data_status='%d';", 0);
		strcat(buf, buf_temp);
	//	gpio_status = ;
		sprintf(buf_temp, "var menu_data='%d';", MENU_CONFG);
		strcat(buf, buf_temp);
		printf(buf_temp, " pack(packfmt, data);\n");
		strcat(buf, buf_temp);

	httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

esp_err_t termo_data_cgi_api_handler(httpd_req_t *req) {
	//#warning "******** where is no error processing !  *******"
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
//	char buf[1024];
	char buf_temp[256];
	if ((req->uri[13] == 'a') & (req->uri[14] == 'd') & (req->uri[15] == 'd')) {
	sprintf(buf,"packfmt={name:{offs:0,len:34},termo_id:{offs:34,len:34},__len:68};");



		sprintf(buf_temp, " var data=[");
		strcat(buf, buf_temp);

		for (uint8_t ct = 0; ct < max_sensor - 1; ct++) {
			sprintf(buf_temp, "{name:\"%s\",", termo[ct].name);
			strcat(buf, buf_temp);
			printf("Name%d=%s\n\r",ct,termo[ct].name);
			sprintf(buf_temp, "termo_id:\"%02x%02x %02x%02x %02x%02x %02x%02x\",",
					termo[ct].id[0], termo[ct].id[1], termo[ct].id[2],
					termo[ct].id[3], termo[ct].id[4], termo[ct].id[5],
					termo[ct].id[6], termo[ct].id[7]);
			strcat(buf, buf_temp);
			printf("termo_id%d=%02x%02x %02x%02x %02x%02x %02x%02x\n\r",ct,termo[ct].id[0], termo[ct].id[1], termo[ct].id[2],
					termo[ct].id[3], termo[ct].id[4], termo[ct].id[5],
					termo[ct].id[6], termo[ct].id[7]);

			sprintf(buf_temp, "temp:%d}",
					(uint16_t) (termo[ct].temper * 10 + termo[ct].ftemper));
			strcat(buf, buf_temp);
			printf("temp%d=%d\n\r",ct,(uint16_t) (termo[ct].temper * 10 + termo[ct].ftemper));
			sprintf(buf_temp, ",");
			strcat(buf, buf_temp);
		}
		sprintf(buf_temp, "{name:\"%s\",", termo[max_sensor - 1].name);
		strcat(buf, buf_temp);
		sprintf(buf_temp, "termo_id:\"%02x%02x %02x%02x %02x%02x %02x%02x\",",
				termo[max_sensor - 1].id[0], termo[max_sensor - 1].id[1],
				termo[max_sensor - 1].id[2], termo[max_sensor - 1].id[3],
				termo[max_sensor - 1].id[4], termo[max_sensor - 1].id[5],
				termo[max_sensor - 1].id[6], termo[max_sensor - 1].id[7]);
		strcat(buf, buf_temp);
		sprintf(buf_temp, "temp:%d}",
				(uint16_t) (termo[max_sensor - 1].temper * 10
						+ termo[max_sensor - 1].ftemper));
		strcat(buf, buf_temp);

		sprintf(buf_temp, "];");
		strcat(buf, buf_temp);

		sprintf(buf_temp, "var devname='%s';", FW_data.sys.V_Name_dev);
			strcat(buf, buf_temp);

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
			sprintf(buf_temp, "var data_status='%d';", 0);
			strcat(buf, buf_temp);
		//	gpio_status = ;
			sprintf(buf_temp, "var menu_data='%d';", MENU_CONFG);
			strcat(buf, buf_temp);
			printf(buf_temp, " pack(packfmt, data);\n");
			strcat(buf, buf_temp);
			strcat(buf, buf_temp);
		} else {
			sprintf(buf,
					"<pre>\nretry: 500\n\nevent: termo_status\ndata: %d\n\n<\pre>",
					111);
		//	strcat(buf, buf_temp);

		}
	httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}



static esp_err_t termo_set_post_handler(httpd_req_t *req) {
	esp_err_t err;
	nvs_handle_t my_handle;
	uint8_t len;
//	char buf[2048];
	char buf_temp[1024] = { 0 };
	io_set_t data;
	int ret, remaining = req->content_len;
	uint8_t ct;

	httpd_resp_set_status(req, "303 See Other");
		httpd_resp_set_hdr(req, "Location", "\termo.html");
		httpd_resp_set_hdr(req, "Cache-Control",
				"no-store, no-cache, must-revalidate");
		httpd_resp_set_type(req, mime_sse);
		httpd_resp_set_hdr(req, "Connection", "Close");

	if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {

	}
	for(uint16_t ct=0;ct<MAX_DEVICES;ct++)
	{
	len = read_mess_smtp((char*) (buf + 5+ct*68), (uint8_t*) buf_temp);
	memset(termo[ct].name, 0, 16);
	memcpy(termo[ct].name, (char*) (buf_temp), len);
	memset(buf_temp, 0, 1024);
	printf("Name%d=%s\n\r",ct,termo[ct].name);

//	len = read_mess_smtp((char*) (buf + 73), (uint8_t*) buf_temp);
//	memset(termo[1].name, 0, 16);
//	memcpy(termo[1].name, (char*) (buf_temp), len);

//char2_to_hex((char*) (buf + 40), (uint8_t*) buf_temp, 35);

//	termo[0].t_dw = buf[41] << 4 | buf[42];
//	termo[0].t_up = buf[43] << 4 | buf[44];

//	char2_to_hex((char*) (buf + 100), (uint8_t*) buf_temp, 70);
//	termo[1].t_dw = buf[109] << 4 | buf[110];
//	termo[1].t_up = buf[111] << 4 | buf[112];
	}

	nvs_flags.data_param = 1;

	httpd_resp_send_chunk(req, NULL, 0);
	return ESP_OK;
}
static esp_err_t thermo_web1_handler(httpd_req_t *req) {
#warning "******** where is no error processing !  *******"
	uint8_t ct;
	httpd_resp_set_hdr(req, "Cache-Control",
			"no-store, no-cache, must-revalidate");
	httpd_resp_set_type(req, mime_text);
	httpd_resp_set_hdr(req, "Connection", "Close");

//	char buf[2048];
	char buf_temp[256];
	uint16_t  ct_s;
//	gpio_status = 0;
	memset((uint8_t*) buf, 0, 2048);

	printf("\n\rin %s  len=%d \n\r", req->uri, strlen(req->uri));
	memset((uint8_t*) buf_temp, 0, 256);
	memcpy(buf_temp, req->uri, 13);

	printf("\n\rout %s  len=%d \n\r", buf_temp, strlen(buf_temp));

	if ((strcmp(buf_temp, "/thermo.cgi?t") == 0)
			&& (req->uri[strlen(req->uri) + 1] == 0)) {
		printf("\n\rGood web hook\n\r");
		memset((uint8_t*) buf_temp, 0, 256);
		uint8_t fault=1;
		for (ct_s = 0; ct_s < max_sensor; ct_s++) {

			sprintf(buf_temp, "%d", ct_s);

			if ((ct_s == (req->uri[strlen(req->uri) - 1] - 0x30))
					&& ((strlen(req->uri) - 13) == 1)) {
				sprintf(buf, "thermo_result('ok', %d, %d)",
						termo[ct_s].temper, termo[ct_s].status);
				printf("\n\rhook %d %s\n\r", ct_s, buf);
				fault=0;
			}

			if (((buf_temp[0]-0x30) == (req->uri[strlen(req->uri) - 2]) - 0x30)
					&& ((buf_temp[1]-0x30) == (req->uri[strlen(req->uri) - 1]) - 0x30)
					&& ((strlen(req->uri) - 13) == 2)) {
				sprintf(buf, "thermo_result('ok', %d, %d)",
						termo[ct_s].temper, termo[ct_s].status);
				printf("\n\rhook %d %s\n\r", ct_s, buf);
				fault=0;
			}
		}
		if (fault==1)
			{
			printf("\n\rFall  web hook\n\r");
			sprintf(buf, "thermo_result('error')");
			}

	}
	else {
		printf("\n\rFall  web hook\n\r");
		sprintf(buf, "thermo_result('error')");
	}

	httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}
static const httpd_uri_t thermo_web1 = { .uri = "/thermo.cgi", .method = HTTP_GET,
		.handler = thermo_web1_handler, .user_ctx = 0 };
static const httpd_uri_t termo_set = { .uri = "/termo_set.cgi", .method =
		HTTP_POST, .handler = termo_set_post_handler, .user_ctx = NULL };

void http_var_init_owb(httpd_handle_t server) {
	httpd_register_uri_handler(server, &thermo_web1);
	httpd_register_uri_handler(server, &termo_set);
	httpd_register_uri_handler(server, &termo_get_api);
	httpd_register_uri_handler(server, &termo_data_api);
	httpd_register_uri_handler(server, &np_html_uri_termo);

}

esp_err_t save_data_termo(void) {

	esp_err_t err = 0;

	err = err | nvs_set_i16(nvs_data_handle, get_name(t0_dw), termo[0].t_dw);
	err = err | nvs_set_i16(nvs_data_handle, get_name(t0_up), termo[0].t_up);
	err = err | nvs_set_u8(nvs_data_handle, get_name(t0_st), termo[0].status);
	err = err
			| nvs_set_blob(nvs_data_handle, get_name(t0_name),
					&(termo[0].name[0]), 16);

	err = err | nvs_set_i16(nvs_data_handle, get_name(t1_dw), termo[1].t_dw);
	err = err | nvs_set_i16(nvs_data_handle, get_name(t1_up), termo[1].t_up);
	err = err | nvs_set_u8(nvs_data_handle, get_name(t1_st), termo[1].status);
	err = err
			| nvs_set_blob(nvs_data_handle, get_name(t1_name),
					&(termo[1].name[0]), 16);


	err = err
			| nvs_set_u8(nvs_data_handle, get_name(repit_3r2),
					termo[1].repit_3r);

	return err;
}

esp_err_t load_data_termo(void) {

	esp_err_t err = 0;
	uint8_t lens = 4;
	lens = 16;

	err = err | nvs_get_i16(nvs_data_handle, get_name(t0_dw), &termo[0].t_dw);
	err = err | nvs_get_i16(nvs_data_handle, get_name(t0_up), &termo[0].t_up);
	err = err | nvs_get_u8(nvs_data_handle, get_name(t0_st), &termo[0].status);
	err = err
			| nvs_get_blob(nvs_data_handle, get_name(t0_name),
					&(termo[0].name[0]), &lens);

	err = err | nvs_get_i16(nvs_data_handle, get_name(t1_dw), &termo[1].t_dw);
	err = err | nvs_get_i16(nvs_data_handle, get_name(t1_up), &termo[1].t_up);
	err = err | nvs_get_u8(nvs_data_handle, get_name(t1_st), &termo[1].status);
	err = err
			| nvs_get_blob(nvs_data_handle, get_name(t1_name),
					&(termo[1].name[0]), &lens);

	err = err
			| nvs_get_u8(nvs_data_handle, get_name(repit_3r2),
					&termo[1].repit_3r);
	return err;
}
uint8_t load_def_termo(void) {

	memset((uint8_t*) termo[0].name, 0, 16);
	memcpy((uint8_t*) termo[0].name, (uint8_t*) "Termo1", sizeof("Termo1"));

	termo[0].t_up = 30;
	termo[0].t_dw = 10;
	termo[0].status = 2;

	memset((uint8_t*) termo[1].name, 0, 16);
	memcpy((uint8_t*) termo[1].name, (uint8_t*) "Termo2", sizeof("Termo2"));
	termo[1].t_up = 30;
	termo[1].t_dw = 10;
	termo[1].status = 2;
	return 0;

}

#endif
