/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * This example uses OTAA to join the LoRaWAN network and then sends the 
 * internal temperature sensors value up as an uplink message periodically 
 * and the first byte of any uplink messages received controls the boards
 * built-in LED.
 */

// #define REPORT_TO_STDIO 1
#define REPORT_TO_STDIO 0

#if REPORT_TO_STDIO==1
#include <stdio.h>
#endif
#include <string.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"

#include "pico/stdlib.h"
#include "pico/lorawan.h"
#include "tusb.h"

// edit with LoRaWAN Node Region and OTAA settings 
#include "config.h"

// pin configuration for SX12xx radio module
const struct lorawan_sx12xx_settings sx12xx_settings = {
    .spi = {
        .inst = spi1,
        .mosi = 11,
        .miso = 12,
        .sck  = 10,
        .nss  = 3
    },
    .reset = 15,
    .busy = 2,
    // sx127x would use dio0 pin, and sx126x dont use it 
    // .dio0  = 7,
    .dio1  = 20
};

// OTAA settings
const struct lorawan_otaa_settings otaa_settings = {
    .device_eui   = LORAWAN_DEVICE_EUI,
    .app_eui      = LORAWAN_APP_EUI,
    .app_key      = LORAWAN_APP_KEY,
    .channel_mask = LORAWAN_CHANNEL_MASK
};

// variables for receiving data
int receive_length = 0;
uint8_t receive_buffer[242];
uint8_t receive_port = 0;

// functions used in main
void internal_temperature_init();
float internal_temperature_get();
float battery_get();

void flash(int n) {
    for (int i = 0; i < n; i++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(200);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(200);
    }
}

int main( void )
{
    // initialize stdio and wait for USB CDC connect
#if REPORT_TO_STDIO==1
    stdio_init_all();
    while (!tud_cdc_connected()) {
        tight_loop_contents();
    }
    
    printf("Pico LoRaWAN - OTAA - Temperature + LED\n\n");
#else
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);    
#endif

    // initialize the LED pin and internal temperature ADC
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    internal_temperature_init();

    // uncomment next line to enable debug
    //lorawan_debug(true);

    // initialize the LoRaWAN stack
#if REPORT_TO_STDIO==1
    printf("Initilizating LoRaWAN ... ");
#else        
        flash(1);
#endif
    if (lorawan_init_otaa(&sx12xx_settings, LORAWAN_REGION, &otaa_settings) < 0) {
#if REPORT_TO_STDIO==1
        printf("failed!!!\n");
#else        
        flash(4);
#endif
        while (1) {
            tight_loop_contents();
        }
    } else {
#if REPORT_TO_STDIO==1
        printf("success!\n");
#else        
        flash(1);
#endif
    }

    // Start the join process and wait
#if REPORT_TO_STDIO==1
    printf("Joining LoRaWAN network ...");
#endif
    lorawan_join();

    while (!lorawan_is_joined()) {
        lorawan_process_timeout_ms(1000);
#if REPORT_TO_STDIO==1
        printf(".");
#else        
        flash(2);
#endif
    }
#if REPORT_TO_STDIO==1
    printf(" joined successfully!\n");
#else        
        flash(1);
#endif

    // loop forever
    while (1) {
        // get the internal temperature
        float adc_temperature_byte = internal_temperature_get();
        float adc_battery_byte = battery_get();
        char result[20];
        sprintf(result, "%.2f,%.2f", adc_temperature_byte,adc_battery_byte);
#if REPORT_TO_STDIO==1
        printf(result);
#endif
        // send the internal temperature as a (signed) byte in an unconfirmed uplink message
#if REPORT_TO_STDIO==1
        printf("\nsending internal temperature: %f 'C... ", adc_temperature_byte);
#endif
        //if (lorawan_send_unconfirmed(&adc_temperature_byte, sizeof(adc_temperature_byte), 2) < 0) {
        if (lorawan_send_unconfirmed(result, strlen(result), 2) < 0) {
#if REPORT_TO_STDIO==1
            printf("failed!!!\n");
#else        
        flash(2);
#endif
        } else {
#if REPORT_TO_STDIO==1
            printf("success!\n");
#else        
        flash(1);
#endif
        }

        // wait for up to 30 seconds for an event
        if (lorawan_process_timeout_ms(30000) == 0) {
            // check if a downlink message was received
            receive_length = lorawan_receive(receive_buffer, sizeof(receive_buffer), &receive_port);
            if (receive_length > -1) {
#if REPORT_TO_STDIO==1
                printf("received a %d byte message on port %d: ", receive_length, receive_port);
#endif
                for (int i = 0; i < receive_length; i++) {
#if REPORT_TO_STDIO==1
                    printf("%02x", receive_buffer[i]);
#endif                    
                }
#if REPORT_TO_STDIO==1
                printf("\n");
#endif                    

                // the first byte of the received message controls the on board LED
                gpio_put(PICO_DEFAULT_LED_PIN, receive_buffer[0]);
            }
        }
    }

    return 0;
}

void internal_temperature_init()
{
    adc_init();
    adc_gpio_init(29);
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
}

float internal_temperature_get()
{
    const float v_ref = 3.3;

    // select and read the ADC
    adc_select_input(4);
    uint16_t adc_raw = adc_read();

    // convert the raw ADC value to a voltage
    float adc_voltage = adc_raw * v_ref / 4095.0f;

    // convert voltage to temperature, using the formula from 
    // section 4.9.4 in the RP2040 datasheet
    //   https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf
    float adc_temperature = 27.0 - ((adc_voltage - 0.706) / 0.001721);

    return adc_temperature;
}

float battery_get()
{
    const float v_ref = 3.3;

    // select and read the ADC8
    adc_select_input(3);
    uint16_t adc_raw = adc_read();
    float conversion_factor = 3.3f * 3.0 / (1 << 12);
    // convert the raw ADC value to a voltage
    float Battery  = adc_raw * conversion_factor;

    // convert voltage to temperature, using the formula from 
    // section 4.9.4 in the RP2040 datasheet
    //   https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf

    return Battery;
}