/*
 * config_pj.h
 *
 *  Created on: 19 ���. 2021 �.
 *      Author: ivanov
 */

#ifndef MAIN_CONFIG_PJ_H_
#define MAIN_CONFIG_PJ_H_

//*****************************************Main config*************************************************
#define Platform 58
#define rev 3
#define Assembly 0
#define Bild 0


#define CONFIG_BLINK_GPIO 2
#define BLINK_GPIO CONFIG_BLINK_GPIO
#define hw_config 1
#define MAIN_APP_DEFAULT_CONF 0
#define DEF_DHCP 0
//#define OUT_POW_EN 4
#define size_1k_buff 24
#define pin_def 35
#define NTP_UPDATE 20

//*****************************************1wire termo  port*******************************************
#define MAIN_APP_OWB_H_ 1

#if  MAIN_APP_OWB_H_ == 1
#define max_sensor 16
#include "..\main\termo\app_owb.h"

//#define OW_DEBUG 0
//#define PHY_DEBUG 1



#endif


//*****************************************Input  port*************************************************
#define MAIN_APP_IN_PORT 1


#define in_port_n 2

#define P_I0 37
#define P_I1 38


//*****************************************Output port*************************************************

#define MAIN_APP_OUT_PORT 1




#define out_port_n 2

#define P_O0 14
#define P_O1 12
//#define P_O2 4


//*****************************************SMTP OUTPUT*************************************************

#define MAIN_APP_SMTP 1

//*****************************************PING OUTPUT*************************************************

#define MAIN_APP_PING 0

//*****************************************ALL SETT APP*************************************************


#define MAIN_APP_UPDATE 1

#define MAIN_APP_LOG 1

//*****************************************NOTIFY MODULE*************************************************

#define MAIN_APP_NOTIF 1



#define not_max_n 9



//**************************************** SETT MODULE*************************************************


#define MAIN_APP_SETT 1

#define MAIN_APP_VIEW 1



#define MENU_CONFG ((MAIN_APP_UPDATE<<8)|(MAIN_APP_LOG<<7)|(MAIN_APP_NOTIF<<5)|(MAIN_APP_SETT<<4)|(MAIN_APP_OWB_H_<<3)|(MAIN_APP_OUT_PORT<<2)|(MAIN_APP_IN_PORT<<1)|(MAIN_APP_VIEW))
#endif
