/**
 * This code is inspired from the Wi-Fi station example provided by Espressif
 * IoT Development Framework (IDF):
 * https://github.com/espressif/esp-idf/blob/master/examples/wifi/getting_started/station/main/station_example_main.c
 */

#include <string.h>
#include "wifi.h"

/* macro definitions */
#define CONNECTED_BIT BIT0
#define FAIL_BIT      BIT1
#define MAX_RETRY     16
#define TAG           "envmon:wifi"

/* static variables */
static EventGroupHandle_t event_group;
static int                n_retry;

/* static function prototypes */
static void event_handler(void*, esp_event_base_t, int32_t, void*);
static void nvs_init();
static void wifi_init_event();
static void wifi_init_netif();
static void wifi_config(const char *, const char *);
static void wifi_get_status(const char *);
static void wifi_deinit_event();

/**
 * Initialize the Wi-Fi station to connect to access point identified by the
 * given SSID.
 * \param ssid
 * \param password
 */
void wifi_init(const char *ssid, const char *password)
{
    nvs_init();
    wifi_init_event();
    wifi_init_netif();
    wifi_config(ssid, password);
    ESP_ERROR_CHECK(esp_wifi_start() );
    ESP_LOGI(TAG, "Wi-Fi station initialization finished");
    wifi_get_status(ssid);
    wifi_deinit_event();
}

/**
 * Initialize the non-volatile storage library used to store key-value pairs in
 * flash memory.
 */
void nvs_init()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS flash initialized");
}

/**
 * Initialize the event handling.
 */
void wifi_init_event()
{
    event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                               ESP_EVENT_ANY_ID,
                                               &event_handler,
                                               NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                               IP_EVENT_STA_GOT_IP,
                                               &event_handler,
                                               NULL));
}

/**
 * Initialize the network interface.
 */
void wifi_init_netif()
{
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_sta();
}

/**
 * Configure the Wi-Fi device.
 */
void wifi_config(const char *ssid, const char *password)
{
    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };

    strcpy((char *)config.sta.ssid, ssid);
    strcpy((char *)config.sta.password, password);
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &config) );
}

/**
 * Get the status of the Wi-Fi connection.
 */
void wifi_get_status(const char *ssid)
{
    EventBits_t bits = xEventGroupWaitBits(event_group,
            CONNECTED_BIT | FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
    if (bits & CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "Connection to AP SSID:%s established", ssid);
    }
    else if (bits & FAIL_BIT)
    {
        ESP_LOGI(TAG, "Connection to AP SSID:%s failed", ssid);
    } else {
        ESP_LOGE(TAG, "Unexpected event");
    }
}

/**
 * Deinitialize the event handling.
 */
void wifi_deinit_event()
{
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(event_group);
}

/**
 */
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (n_retry < MAX_RETRY)
        {
            esp_wifi_connect();
            n_retry++;
            ESP_LOGI(TAG, "Connection to AP retried");
        }
        else
        {
            xEventGroupSetBits(event_group, FAIL_BIT);
        }
        ESP_LOGI(TAG,"Connection to AP failed");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "IP obtained:" IPSTR, IP2STR(&event->ip_info.ip));
        n_retry = 0;
        xEventGroupSetBits(event_group, CONNECTED_BIT);
    }
}
