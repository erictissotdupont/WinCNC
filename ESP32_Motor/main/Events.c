#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define EVENT_NEW_STATE		   BIT0
#define EVENT_CALLBACK_SET	 BIT1
#define WIFI_CONNECTED_BIT   BIT2
#define WIFI_FAIL_BIT        BIT3
#define MOTOR_IDLE_BIT       BIT4

EventGroupHandle_t g_eventGroupHandle;
static const char* TAG = "events";

int EventInit( )
{
	g_eventGroupHandle = xEventGroupCreate();  
	if( g_eventGroupHandle == NULL )
	{
		ESP_LOGE( TAG, "Failed to create event group." );
		return -1;
	}

  return 0;
}

void SignalIPConnected( )
{
  xEventGroupSetBits(g_eventGroupHandle, WIFI_CONNECTED_BIT);
}

void IRAM_ATTR SignalMotorIdleFromISR( )
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if( xEventGroupSetBitsFromISR( g_eventGroupHandle, MOTOR_IDLE_BIT, &xHigherPriorityTaskWoken ) == pdPASS )
  {
    // If xHigherPriorityTaskWoken is now set to pdTRUE then a context
    // switch should be requested.  The macro used is port specific and
    // will be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() -
    // refer to the documentation page for the port being used.
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }
}

void WaitForMotorIdle( )
{
  while(( xEventGroupWaitBits( g_eventGroupHandle, MOTOR_IDLE_BIT, pdTRUE, pdFALSE, 1000 / portTICK_PERIOD_MS ) & MOTOR_IDLE_BIT ) == 0 )
  {
    // ESP_LOGI( TAG, "Waiting for movement to complete..." ); 
  }
  ESP_LOGI( TAG, "Motors are idle" );
}