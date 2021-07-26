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
#include "sdkconfig.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "Network.h"

uint64_t g_startTime = 0;

static const char *TAG = "main";

void cnc_loop( )
{
  while( 1 )
  {    
    vTaskDelay( 100 / portTICK_PERIOD_MS);
  
  } 
}

void app_main(void)
{
	ESP_LOGI( TAG, "Started" );
  
  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      // NVS partition was truncated and needs to be erased
      // Retry nvs_flash_init
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
  }
  ESP_ERROR_CHECK( err );
  
  g_startTime = esp_timer_get_time( );

	// Initialize the network and start the tasks
	initSocketCom( );
		
	cnc_loop( );
	
	// Should never get out of the main loop
	ESP_LOGE( TAG, "Stopped" );
}
