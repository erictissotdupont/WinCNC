/* Ethernet Basic Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "Cnc.h"
#include "Network.h"
#include "LCD.h"
#include "Buttons.h"

static const char* TAG = "main";

uint64_t g_startTime = 0;
unsigned long g_Status = 0;
char g_szStatus[20] = { 0 };

void app_main(void)
{
    ESP_LOGI( TAG, "Started" );
    
    g_startTime = esp_timer_get_time( );
  
    Buttons_Init( );
  
    LCD_Init( );
  
    Wifi_Init( );
	
    while( 1 )
    {
        vTaskDelay( 10 / portTICK_PERIOD_MS);
    
        Buttons_Loop( );
    
        LCD_Refresh( );
    }
	
    // Should never get out of the main loop
    ESP_LOGE( TAG, "Stopped" );
}
