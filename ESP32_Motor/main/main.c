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
#include "nvs_flash.h"
#include "nvs.h"
// For UART
#include "driver/uart.h"
#include "string.h"

#include "Network.h"
#include "Cnc.h"
#include "Events.h"
#include "Motor.h"
#include "Limits.h"

static const char* TAG = "main";

uint64_t g_startTime = 0;

#define UART_TXD_PIN 			(GPIO_NUM_33)
#define UART_RXD_PIN 			(GPIO_NUM_35)
#define UART_RX_BUF_SIZE 	256

char g_UARTrxData[UART_RX_BUF_SIZE];
char g_ACKchar = '0';

extern uint32_t g_CRCErrorCount;
extern uint32_t g_ErrorA;
extern uint32_t g_ErrorB;
extern uint32_t g_ErrorC;
extern uint32_t g_BadLimitData;
extern uint32_t g_limitState;

#define DEBUG_UARTx

void app_main(void)
{
	ESP_LOGI( TAG, "Started" );
  
  EventInit( );
    
  //   T I M E R 
  // --------------
  esp_timer_early_init( );
  //ESP_ERROR_CHECK( esp_timer_init( ));

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
    
  MotorInit( );
  LimitsInit( );
  NetworkInit( false );
    	
	while( 1 )
  {
    NetworkIdleTask( );
  } 
	
	// Should never get out of the main loop
	ESP_LOGE( TAG, "Stopped" );
}
