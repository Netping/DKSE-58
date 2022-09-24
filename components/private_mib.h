/**
 * @file
 * Exports Private lwIP MIB 
 */

#ifndef LWIP_HDR_PRIVATE_MIB_H
#define LWIP_HDR_PRIVATE_MIB_H

#include "lwip/apps/snmp_opts.h"

#include "../components/snmp_core.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_MAX      10
#define termo_max      10
#define SENSOR_NAME_LEN 20


struct sensor_inf {
	u8_t num;

	char file[SENSOR_NAME_LEN + 1];

#if !SENSORS_USE_FILES
	/** When not using files, contains the value of the sensor */
	s32_t value;
#endif /* !SENSORS_USE_FILES */
};


/* export MIB */
extern const struct snmp_mib mib_private;
extern const struct snmp_mib mib_private5;
extern const struct snmp_mib new_mib_private;
extern const struct snmp_mib mib_np_private;
extern const struct snmp_mib mib_termo_private;
extern const struct snmp_mib out_mib_private;
extern struct sensor_inf termo_snmp[termo_max];
void lwip_privmib_init(void);
void send_mess_trap (s32_t* OID_TR,char* mess,uint16_t lens_mess,uint8_t canal);
void send_mess_trap_termo (s32_t* OID_TR,uint8_t canal);
void in_send_mess_trap(uint8_t* ipaddr, uint8_t canal);
void out_send_mess_trap(uint8_t* ipaddr, uint8_t canal);
#ifdef __cplusplus
}
#endif

#endif
