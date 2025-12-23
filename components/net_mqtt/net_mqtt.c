#include "net_mqtt.h"

#include <stdlib.h>
#include "esp_log.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "mqtt_client.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"

static const char *TAG = "net_mqtt";

static esp_mqtt_client_handle_t mqtt_client = NULL;
static const char *s_topic = NULL;

// ================= WIFI EVENT HANDLER (y như cũ) =================
static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Retrying to connect to the AP");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

esp_err_t wifi_init_sta(const char *ssid, const char *pass)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {0};
    snprintf((char*)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), "%s", ssid);
    snprintf((char*)wifi_config.sta.password, sizeof(wifi_config.sta.password), "%s", pass);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    return ESP_OK;
}

// ================= MQTT EVENT HANDLER (y như cũ) =================
static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    (void)handler_args; (void)base;
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT Error");
            break;
        default:
            break;
    }
}

esp_err_t mqtt_app_start(const char *broker_uri,
                         const char *username,
                         const char *password,
                         const char *topic)
{
    s_topic = topic;

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = broker_uri,
        .credentials.username = username,
        .credentials.authentication.password = password,

        // y như code cũ: dùng bundle
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!mqtt_client) return ESP_FAIL;

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client));

    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
    return ESP_OK;
}

void send_data_to_mqtt(int r, int g, int b)
{
    if (mqtt_client == NULL || s_topic == NULL) return;

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "red", r);
    cJSON_AddNumberToObject(root, "green", g);
    cJSON_AddNumberToObject(root, "blue", b);

    char *post_data = cJSON_PrintUnformatted(root);
    esp_mqtt_client_publish(mqtt_client, s_topic, post_data, 0, 1, 0);

    cJSON_Delete(root);
    free(post_data);
}
