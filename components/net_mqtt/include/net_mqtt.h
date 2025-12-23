#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t wifi_init_sta(const char *ssid, const char *pass);

esp_err_t mqtt_app_start(const char *broker_uri,
                         const char *username,
                         const char *password,
                         const char *topic);

void send_data_to_mqtt(int r, int g, int b);

#ifdef __cplusplus
}
#endif
