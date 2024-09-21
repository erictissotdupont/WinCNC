/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "esp_timer.h"

/**
 * Brief:
 * This test code shows how to configure gpio and how to use gpio interrupt.
 *
 * GPIO status:
 * GPIO18: output
 * GPIO19: output
 * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
 * GPIO5:  input, pulled up, interrupt from rising edge.
 *
 * Test:
 * Connect GPIO18 with GPIO4
 * Connect GPIO19 with GPIO5
 * Generate pulses on GPIO18/19, that triggers interrupt on GPIO4/5
 *
 */

#define GPIO_OUTPUT_IO_1      8
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_1)

#define GPIO_COMM_RX          18

#define GPIO_LIMIT_ZL         41
#define GPIO_LIMIT_ZR         40
#define GPIO_LIMIT_XL         39
#define GPIO_LIMIT_XR         38
#define GPIO_LIMIT_Y          37

#define GPIO_INPUT_PIN_SEL    ((1ULL<<GPIO_COMM_RX)  |\
                               (1ULL<<GPIO_LIMIT_ZL) |\
                               (1ULL<<GPIO_LIMIT_ZR) |\
                               (1ULL<<GPIO_LIMIT_XL) |\
                               (1ULL<<GPIO_LIMIT_XR) |\
                               (1ULL<<GPIO_LIMIT_Y ))
							   
#define ESP_INTR_FLAG_DEFAULT 0

static int cnt = 0;

static int low_min = 1000;
static int low_max = 0;
static int high_min = 1000;
static int high_max = 0;

int g_state = -0;
uint32_t g_value = 0;
static xQueueHandle gpio_evt_queue = NULL;

const uint8_t crc8_table[256] = {
    0x00, 0xF7, 0xB9, 0x4E, 0x25, 0xD2, 0x9C, 0x6B,
    0x4A, 0xBD, 0xF3, 0x04, 0x6F, 0x98, 0xD6, 0x21,
    0x94, 0x63, 0x2D, 0xDA, 0xB1, 0x46, 0x08, 0xFF,
    0xDE, 0x29, 0x67, 0x90, 0xFB, 0x0C, 0x42, 0xB5,
    0x7F, 0x88, 0xC6, 0x31, 0x5A, 0xAD, 0xE3, 0x14,
    0x35, 0xC2, 0x8C, 0x7B, 0x10, 0xE7, 0xA9, 0x5E,
    0xEB, 0x1C, 0x52, 0xA5, 0xCE, 0x39, 0x77, 0x80,
    0xA1, 0x56, 0x18, 0xEF, 0x84, 0x73, 0x3D, 0xCA,
    0xFE, 0x09, 0x47, 0xB0, 0xDB, 0x2C, 0x62, 0x95,
    0xB4, 0x43, 0x0D, 0xFA, 0x91, 0x66, 0x28, 0xDF,
    0x6A, 0x9D, 0xD3, 0x24, 0x4F, 0xB8, 0xF6, 0x01,
    0x20, 0xD7, 0x99, 0x6E, 0x05, 0xF2, 0xBC, 0x4B,
    0x81, 0x76, 0x38, 0xCF, 0xA4, 0x53, 0x1D, 0xEA,
    0xCB, 0x3C, 0x72, 0x85, 0xEE, 0x19, 0x57, 0xA0,
    0x15, 0xE2, 0xAC, 0x5B, 0x30, 0xC7, 0x89, 0x7E,
    0x5F, 0xA8, 0xE6, 0x11, 0x7A, 0x8D, 0xC3, 0x34,
    0xAB, 0x5C, 0x12, 0xE5, 0x8E, 0x79, 0x37, 0xC0,
    0xE1, 0x16, 0x58, 0xAF, 0xC4, 0x33, 0x7D, 0x8A,
    0x3F, 0xC8, 0x86, 0x71, 0x1A, 0xED, 0xA3, 0x54,
    0x75, 0x82, 0xCC, 0x3B, 0x50, 0xA7, 0xE9, 0x1E,
    0xD4, 0x23, 0x6D, 0x9A, 0xF1, 0x06, 0x48, 0xBF,
    0x9E, 0x69, 0x27, 0xD0, 0xBB, 0x4C, 0x02, 0xF5,
    0x40, 0xB7, 0xF9, 0x0E, 0x65, 0x92, 0xDC, 0x2B,
    0x0A, 0xFD, 0xB3, 0x44, 0x2F, 0xD8, 0x96, 0x61,
    0x55, 0xA2, 0xEC, 0x1B, 0x70, 0x87, 0xC9, 0x3E,
    0x1F, 0xE8, 0xA6, 0x51, 0x3A, 0xCD, 0x83, 0x74,
    0xC1, 0x36, 0x78, 0x8F, 0xE4, 0x13, 0x5D, 0xAA,
    0x8B, 0x7C, 0x32, 0xC5, 0xAE, 0x59, 0x17, 0xE0,
    0x2A, 0xDD, 0x93, 0x64, 0x0F, 0xF8, 0xB6, 0x41,
    0x60, 0x97, 0xD9, 0x2E, 0x45, 0xB2, 0xFC, 0x0B,
    0xBE, 0x49, 0x07, 0xF0, 0x9B, 0x6C, 0x22, 0xD5,
    0xF4, 0x03, 0x4D, 0xBA, 0xD1, 0x26, 0x68, 0x9F
};


inline uint8_t crc8( uint8_t* pdata, unsigned int nbytes, uint8_t crc )
{
    while (nbytes-- > 0)
    {
        crc = crc8_table[(crc ^ *pdata++) & 0xff];
    }
    return crc;
}

void signal_IO_changed( )
{
    // Capture the IO state
    g_value = gpio_get_level( GPIO_LIMIT_ZL ) |
             (gpio_get_level( GPIO_LIMIT_ZR ) << 1 ) |
             (gpio_get_level( GPIO_LIMIT_XL ) << 2 ) |
             (gpio_get_level( GPIO_LIMIT_XR ) << 3 ) |
             (gpio_get_level( GPIO_LIMIT_Y )  << 4 );
             
    // Calculate the CRC on the first 3 bytes
    uint32_t crc = 0xFF;
    crc = crc8_table[((crc ^ ( g_value       )) & 0xFF) & 0xFF];
    crc = crc8_table[((crc ^ ((g_value >> 8 ))) & 0xFF) & 0xFF];
    crc = crc8_table[((crc ^ ((g_value >> 16))) & 0xFF) & 0xFF];
    // Put it in the high byte
    crc = crc << 24;
    g_value |= crc;
        
    // Signal interrupt
    g_state = 1;
    gpio_set_level(GPIO_OUTPUT_IO_1, 1 );
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
	uint64_t time = esp_timer_get_time( );
	static int64_t endOfPrev = 0;
	static uint32_t mask = 0;
    static bool bPending = false;
    
    //static uint16_t prevIO = 0;
    //uint16 IO = 0;
    
    if( gpio_num == GPIO_COMM_RX )
    {
        switch( g_state )
        {
        default:
            gpio_set_level(GPIO_OUTPUT_IO_1, 0 );
            g_state = 1;
            // fallthrough

        case 0 : // Idle
            // Ignore signal edges when idle
            break; 
            
        case 1 : // Interrupt signaled
            if( gpio_get_level(GPIO_COMM_RX) == 0 )
            {
            // Host has acked, wait for release
            g_state = 2;
            }
            break;
            
        case 2 :
            if( gpio_get_level(GPIO_COMM_RX) == 1 )
            {
            mask = 1;
            gpio_set_level(GPIO_OUTPUT_IO_1, 0 );
            g_state = 3;
            }
            break;
            
        case 3 :
            if( gpio_get_level(GPIO_COMM_RX) == 0 )
            {
                if( mask )
                {				
                    gpio_set_level(GPIO_OUTPUT_IO_1, g_value & mask );
                    mask = mask << 1;
                    g_state = 4;
                }
                else
                {
                    gpio_set_level(GPIO_OUTPUT_IO_1, 0 );                    
                    g_state = 0;
                    endOfPrev = time;
                    if( bPending )
                    {
                        xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
                        bPending = false;
                    }
                }
            }
            break;
            
        case 4 :
            if( gpio_get_level(GPIO_COMM_RX) == 1 )
            {
                g_state = 3;
            }
            break;
        }
    }
    else
    {    
        if( g_state == 0 && ( endOfPrev <= ( time - 1000 )))
        {
            signal_IO_changed( );
        }
        else
        {
            bPending = true;
        }
    }
        
	cnt++;
}

static void gpio_task_example(void* arg)
{
    while( 1 ) 
    {
        uint32_t arg;
        if(xQueueReceive(gpio_evt_queue, &arg, portMAX_DELAY)) 
        {
            do
            {
                vTaskDelay(10 / portTICK_RATE_MS);
            } while( g_state != 0 );
            
            signal_IO_changed( );
        }
    }
}

void app_main(void)
{
    gpio_config_t io_conf = {};
    
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
    
    //   O U T P U T 
    // ---------------- 
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    
    //   I N P U T S 
    // ---------------
    
    
    //   I N T E R R U P T S
    // -----------------------
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
	io_conf.pull_down_en = 1;
    gpio_config(&io_conf);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_COMM_RX, gpio_isr_handler, (void*) GPIO_COMM_RX);
    gpio_isr_handler_add(GPIO_LIMIT_ZL, gpio_isr_handler, (void*) GPIO_LIMIT_ZL);
    gpio_isr_handler_add(GPIO_LIMIT_ZR, gpio_isr_handler, (void*) GPIO_LIMIT_ZR);
    gpio_isr_handler_add(GPIO_LIMIT_XL, gpio_isr_handler, (void*) GPIO_LIMIT_XL);
    gpio_isr_handler_add(GPIO_LIMIT_XR, gpio_isr_handler, (void*) GPIO_LIMIT_XR);
    gpio_isr_handler_add(GPIO_LIMIT_Y , gpio_isr_handler, (void*) GPIO_LIMIT_Y );

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    while(1) {
        printf("%x / low %d - %d - high %d - %d \n", g_value, low_min, low_max, high_min, high_max );
        vTaskDelay(1000 / portTICK_RATE_MS);
	
    }
}
