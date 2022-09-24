                    
idf_component_register(SRCS
 "main.c" 
 "mdns_task.c"
 "nvs_task.c"
 "http_var.c"
 "update.c"
 "smtp.c"
 "LOGS.c"
 "app.c"
 "ping.c"
 "syslog.c"
  "connect.c"
  "../main/notify/notify.c"
 "../main/termo/app_owb.c"
 "../main/termo/owb.c"
 "../main/termo/owb_gpio.c"
 "../main/termo/owb_rmt.c"
 "../main/termo/ds18b20.c"
  "../main/input/input.c"
   "../main/output/output.c"
   
                    INCLUDE_DIRS ".")



#LBS force placing of 'custom_app_desc' segment
target_link_libraries(${COMPONENT_TARGET} "-u updater_js")


