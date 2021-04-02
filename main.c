#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "mcp9700.h"
#include "vma311.h"
#include "bme680.h"
#include "aio.h"
#include "wifi.h"
#include "mqtt.h"

/**
 * These macros define characteristics of devices
 */
#define MCP9700_ADC_UNIT    ADC_UNIT_1
#define MCP9700_ADC_CHANNEL ADC_CHANNEL_4
#define VMA311_GPIO         GPIO_NUM_5
#define SPI_BUS HSPI_HOST
#define SPI_SCK_GPIO 18
#define SPI_MOSI_GPIO 23
#define SPI_MISO_GPIO 19
#define SPI_CS_GPIO 15

/**
 * BME680 initialization
 */
static bme680_sensor_t *sensor = 0;
static void bme680_init()
{
    spi_bus_init(SPI_BUS, SPI_SCK_GPIO, SPI_MISO_GPIO, SPI_MOSI_GPIO);
    // init the sensor connected to SPI_BUS with SPI_CS_GPIO as chip select.
    sensor = bme680_init_sensor(SPI_BUS, 0, SPI_CS_GPIO);
    if (sensor)
    {
        printf("Bme initialization success\n");
    }
    else
    {
        printf("Could not initialize BME680 sensor\n");
    }
}

/**
 * This function is the main function of the application.
 */
void app_main()
{
    char mcp9700_temperature[32];
    char vma311_temperature[32];
    char vma311_humidity[32];
    char bme680_temperature[32];
    char bme680_humidity[32];
    char bme680_pressure[32];
    char bme680_gas[32];
    
    //Connection to the Wi-Fi
    wifi_init("Steph","BeachPlease1799");

    //Connexion to the broker
    mqtt_init("mqtts://iot.devinci.online","st170575","%ZEdT$bW");

    //Adafruit initialization
    aio_init("PtitFlan", "aio_jhTH48vtqs5BVQm2BYHX2KGf7vx0");
    aio_create_group("envmon");
    //Creation of the feed
    aio_create_feed("mcp9700:temp", "envmon");
    aio_create_feed("vma311:temperature","envmon");
    aio_create_feed("vma311:humidity","envmon");
    aio_create_feed("bme680:temperature","envmon");
    aio_create_feed("bme680:humidity","envmon");
    aio_create_feed("bme680:pressure","envmon");
    aio_create_feed("bme680:gas_resistance","envmon");
    
    uint32_t mcp_temp;
    vma311_data_t vma311_data;
    int8_t bme680_data;

    /* Device initialization */
    mcp9700_init(MCP9700_ADC_UNIT, MCP9700_ADC_CHANNEL);
    vma311_init(VMA311_GPIO);
    bme680_init();
    uint32_t duration = bme680_get_measurement_duration(sensor);

    while(1) //Endless loop
    {
        /* Data collection */
        //MCP9700
        mcp_temp = mcp9700_get_value();
        printf("mcp9700:temp:%d\n", mcp_temp);
        printf("---------------------------------------");
        sprintf(mcp9700_temperature, "%.1f", (float)mcp9700_get_value());
        
        //VMA311
        vma311_data = vma311_get_values();
        sprintf(vma311_temperature, "%.1f", (float)vma311_data.t_int);
        sprintf(vma311_humidity, "%.1f", (float)vma311_data.rh_int);
        
        //BME680
        bme680_get_sensor_data(&bme_data,&bme);
        printf("bme680:temperature:%d\n", bme_data.temperature);
        printf("bme680:humidity:%d\n", bme_data.humidity);
        printf("bme680:pressure:%d\n", bme_data.pressure);
        printf("bme680:gas_resistance:%d\n", bme_data.gas_resistance);
        printf("---------------------------------------");
        
        if (bme680_force_measurement(sensor))
        {
            // passive waiting until measurement results are available
            vTaskDelay(duration);
            // get the results and do something with them
            
            if (bme680_get_results_float(sensor, &bmeValues))
            {
                //Sends formatted output to a string pointed to, by str
                sprintf(bme680_temperature, "%d", (int)bmeValues.temperature);
                sprintf(bme680_humidity, "%d", (int)bmeValues.humidity);
                sprintf(bme680_pressure, "%d", (int)bmeValues.pressure);
                sprintf(bme680_gas, "%d", (int)bmeValues.gas_resistance);
                
                //Publish data on Adafruit
                aio_create_data(bme680_temperature,"envmon.bme680-temperature");
                aio_create_data(bme680_humidity,"envmon.bme680-humidity");
                aio_create_data(bme680_pressure,"envmon.bme680-pressure");
                aio_create_data(bme680_gas,"envmon.bme680-gas");
              
                //Publish data on MQTT
                mqtt_publish("st170575/bme680/temp",bme680_temperature);
                mqtt_publish("st170575/bme680/hum", bme680_humidity);
                mqtt_publish("st170575/bme680/pressure", bme680_pressure);
                mqtt_publish("st170575/bme680/gas", bme680_gas);

            }
        }
         printf("bme680:temperature:%s\n", bme680_temperature);
         printf("bme680:humidity:%s\n", bme680_humidity);
         printf("bme680:pressure:%s\n", bme680_pressure);
         printf("bme680:gas:%s\n", bme680_gas);
      
        if (vma311_data.status == VMA311_OK)
        {
            printf("vma311:temperature:%d.%d\n", vma311_data.t_int, vma311_data.t_dec);
            printf("vma311:humidity:%d.%d\n", vma311_data.rh_int, vma311_data.rh_dec);
            printf("---------------------------------------");
        }
        else
        {
            printf("vma311:error\n");
        }

        //Publish on Adafruit
        aio_create_data(mcp9700_temperature, "envmon.mcp9700-temp");
        aio_create_data(vma311_temperature, "envmon.vma311-temperature");
        aio_create_data(vma311_humidity, "envmon.vma311-humidity");

        
        //Publish on MQTT
        mqtt_publish("st170575/mcp9700/temp", mcp9700_temperature);
        mqtt_publish("st170575/vma311/temperature", vma311_temperature);
        mqtt_publish("st170575/vma311/humidity", vma311_humidity);

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
