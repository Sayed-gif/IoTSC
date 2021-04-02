  
/*
 * BSD 2-Clause License
 * 
 * Copyright (c) 2021, ESILV
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file aio.h
 * @author Frédéric Fauberteau
 * @brief Implement the HTTP API of Adafruit IO.
 * 
 * This code provides some functions to send data to the Adafruit IO cloud.
 * @see https://io.adafruit.com/api/docs/
 */

#include <string.h>
#include "aio.h"

#define AIO_API_URL   "https://io.adafruit.com/api/v2"
#define TAG           "envmon:aio"
#define DATA_MAX_SIZE 128
#define URL_MAX_SIZE  128

static aio_t aio;

static void      aio_send_post_request(char *, char *, int);
static esp_err_t aio_handle_http_event(esp_http_client_event_t *);

/**
 * @brief Initialize the access to Adafruit IO.
 * @param username The Adafruit username.
 * @param key The Adafruit key.
 */
void aio_init(const char *username, const char *key)
{
    aio.username = username;
    aio.key = key;
}

/**
 * @brief Create a group of feeds.
 * @param group The name of the group.
 */
void aio_create_group(const char *group)
{
    char url[URL_MAX_SIZE];
    char data[DATA_MAX_SIZE];
    int size;

    snprintf(url, URL_MAX_SIZE, "%s/%s/groups", AIO_API_URL, aio.username);
    ESP_LOGI(TAG, "API URL: %s", url);
    size = snprintf(data, DATA_MAX_SIZE, "{\"group\": {\"name\": \"%s\"}}", group);
    aio_send_post_request(url, data, size);
}

/**
 * @brief Create a feed.
 * @param feed The name of the feed.
 * @param group_key The key of the group in which this feed is created. If NULL,
 *                  the feed is created in a Default group.
 */
void aio_create_feed(const char *feed, const char *group_key)
{
    char url[URL_MAX_SIZE];
    char data[DATA_MAX_SIZE];
    int size;

    if (group_key)
    {
        snprintf(url, URL_MAX_SIZE, "%s/%s/groups/%s/feeds", AIO_API_URL, aio.username, group_key);
    }
    else
    {
        snprintf(url, URL_MAX_SIZE, "%s/%s/feeds", AIO_API_URL, aio.username);
    }
    ESP_LOGI(TAG, "API URL: %s", url);
    size = snprintf(data, DATA_MAX_SIZE, "{\"feed\": {\"name\": \"%s\"}}", feed);
    aio_send_post_request(url, data, size);
}

/**
 * @brief Create a data.
 * @param value The value of data.
 * @param feed_key The key of the feed in which the data is created.
 */
void aio_create_data(const char *value, const char * feed_key)
{
    char url[URL_MAX_SIZE];
    char data[DATA_MAX_SIZE];
    int size;

    snprintf(url, URL_MAX_SIZE, "%s/%s/feeds/%s/data", AIO_API_URL, aio.username, feed_key);
    ESP_LOGI(TAG, "API URL: %s", url);
    size = snprintf(data, DATA_MAX_SIZE, "{\"value\": \"%s\"}", value);
    aio_send_post_request(url, data, size);
}

static void aio_send_post_request(char * url, char *data, int size)
{
    esp_http_client_handle_t client;
    esp_http_client_config_t config =
    {
        .url = url,
        .event_handler = aio_handle_http_event,
    };

    client = esp_http_client_init(&config);
    ESP_ERROR_CHECK(esp_http_client_set_method(client, HTTP_METHOD_POST));
    ESP_ERROR_CHECK(esp_http_client_set_header(client, "Content-Type", "application/json"));
    ESP_ERROR_CHECK(esp_http_client_set_header(client, "X-AIO-Key", aio.key));
    ESP_ERROR_CHECK(esp_http_client_set_post_field(client, data, size));
    ESP_ERROR_CHECK(esp_http_client_perform(client));
    ESP_ERROR_CHECK(esp_http_client_cleanup(client));
}

static esp_err_t aio_handle_http_event(esp_http_client_event_t *evt)
{
    switch(evt->event_id)
    {
        case HTTP_EVENT_ERROR:
            ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER");
            printf("%s: %s\n", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            printf("%.*s\n", evt->data_len, (char *)evt->data);
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}
