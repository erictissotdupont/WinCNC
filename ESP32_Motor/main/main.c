/* Ethernet Basic Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "Cnc.h"

#include "esp_timer.h"

#include "Events.h"
#include "Motor.h"
#include "Limits.h"
#include "WiFi.h"
#include "UDP.h"

static const char* TAG = "main";

uint64_t g_startTime = 0;

void app_main(void)
{
  esp_timer_early_init( );
  g_startTime = esp_timer_get_time( );
  
	ESP_LOGI( TAG, "Started" );
  
  Events_Init( );  
  Motor_Init( );
  Limits_Init( );
  WiFi_Init( false );
  UDP_Init( );
    	
	while( 1 )
  {
    WiFi_IdleTask( );
    UDP_IdleTask( );
  } 
	
	// Should never get out of the main loop
	ESP_LOGE( TAG, "Stopped" );
}
