#include <stdlib.h>
#include "mcp9700.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static esp_adc_cal_characteristics_t *adc_chars; //pointer that will point to the place in memory where the ADC characterisitics will be stored
static mcp9700_t mcp9700;//create a static instance of the mcp9700 object, static means that this instance cannot be used from out of this file

void mcp9700_init(adc_unit_t unit, adc_channel_t channel)
{
    if (unit == ADC_UNIT_1) //if  ADC 1 is used
    {
        adc1_config_width(ADC_WIDTH);                  // set the width of the ADC 1 to be 12 bits
        adc1_config_channel_atten(channel, ADC_ATTEN); // configures the attenuation of the channel to give full scale voltage of 1,1V
    }
    else /* mcp9700.unit == ADC_UNIT_2 */ //if ADC 2 is used
    {
        adc2_config_channel_atten(channel, ADC_ATTEN); //same as ADC1 but the width of ADC 2 is defined automatically when there's a reading
    }
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t)); //the calloc function alllocates a place in memory
    // with all bits equal to 0, here we indicate to the pointer adc_chars, the adress of this place in memory
    esp_adc_cal_characterize(unit, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF, adc_chars); //this function stores the ADC
    //voltage curve based on the characterisitics (parameters of the function) of the ADC, at the adress pointed by adc_chars
    mcp9700.unit = unit;//we assign the parameter values to the mcp9700 structure values
    mcp9700.channel = channel;
    mcp9700.adc_chars = *adc_chars;
}

int32_t mcp9700_get_value()
{
    uint32_t adc_reading = 0;
    uint32_t voltage;
    uint32_t temperature;
    for (int i = 0; i < NO_OF_SAMPLES; i++) //here we have 64 samples
    {
        if (mcp9700.unit == ADC_UNIT_1) //if the ADC used is ADC1
        {
            adc_reading += adc1_get_raw(mcp9700.channel); //add the value of each reading to the sum
        }
        else /* mcp9700.unit == ADC_UNIT_2 */ //if the ADC used is ADC2
        {
            int raw; //the function for ADC 2 is different, it does the same as the function for ADC1 but we have to use more parameters
                     //and the value is stored in the "raw" variable
            adc2_get_raw(mcp9700.channel, ADC_WIDTH, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;                                 //compute the average value by dividing by number of samples
    voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars); //converts the average value to an average voltage value in mV
    //printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);     //prints the voltage on the serial
    vTaskDelay(pdMS_TO_TICKS(1000));                              //waits 1 sec
    voltage = voltage - 500.0;                                       // we substract the offset
    temperature = voltage / 10.0;                                           // we convert the voltage into temperature
    return temperature;
}
