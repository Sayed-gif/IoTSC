/**
 * @file mqtt.c
 * @author Frédéric Fauberteau
 * @brief Implement a secure MQTT client.
 * @see https://github.com/espressif/esp-idf/blob/master/examples/protocols/mqtt/ssl/main/app_main.c
 */

#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_event.h"
#include "esp_tls.h"
#include "mqtt_client.h"
#include "mqtt.h"

#define TAG "envmon:mqtt"

static mqtt_t mqtt;

static void mqtt_event_handler(void *, esp_event_base_t, int32_t, void *);

/**
 * @brief Initialize a connection to a MQTT broker.
 * @param uri The URI of the broker in the form [mqtt|mqtts]://hostname/.
 * @param username The username to connect to the broker.
 * @param password The password to connect to the broker.
 */
void mqtt_init(const char *uri, const char *username, const char *password)
{
    const esp_mqtt_client_config_t client_config = {
        .uri = uri,
        .username = username,
        .password = password,
    };

    mqtt.client = esp_mqtt_client_init(&client_config);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt.client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt.client));
}

/**
 * @brief Publish data on a MQTT topic.
 * @param topic The topic on which data will be plublish in the form /some/where.
 * @param data The data to publish.
 */
void mqtt_publish(const char *topic, const char *data)
{
    int message_id;

    message_id = esp_mqtt_client_publish(mqtt.client, topic, data, 0, 1, 0);
    if (message_id != -1)
        ESP_LOGI(TAG, "Message publication succeed: message ID=%d", message_id);
    else
        ESP_LOGI(TAG, "Message publication failed");
}

static void mqtt_event_handler(void *event_handler_arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    esp_mqtt_event_t *event = (esp_mqtt_event_t *)event_data;

    switch (event_id)
    {
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_ESP_TLS) {
                ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
                ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
            } else {
                ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
            }
            break;
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, message ID=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            break;
        case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
            break;
    }
}
