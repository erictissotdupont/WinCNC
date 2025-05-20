#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "cnc.h"

#define EVENT_NEW_STATE		   BIT0
#define EVENT_CALLBACK_SET	 BIT1
#define WIFI_CONNECTED_BIT   BIT2
#define WIFI_FAIL_BIT        BIT3
#define MOTOR_IDLE_BIT       BIT4

EventGroupHandle_t g_eventGroupHandle;
unsigned long g_State;

static const char* TAG = "events";

void SetState( unsigned long flag )
{
  g_State |= flag; 
}
void ClearState( unsigned long flag )
{
  g_State &= ~flag;
}

unsigned long GetState( )
{
  return g_State;
}

int EventInit( )
{
  uint16_t testEndianness = 0x1234;
    
	g_eventGroupHandle = xEventGroupCreate();  
	if( g_eventGroupHandle == NULL )
	{
		ESP_LOGE( TAG, "Failed to create event group." );
		return -1;
	}
  
  // Zero all the flags
  ClearState((unsigned long)-1);
  // Set the connected flag (state all zeros on the host means "disconnected")
  SetState(CNC_STATE_CONNECTED);
  
  // Test the system's endianness
  if( *((char*)&testEndianness) == 0x34 )
  { 
    SetState( CNC_STATE_LITTLE_ENDIAN );
  }
  
  // By Default, motors are idle
  xEventGroupSetBits(g_eventGroupHandle, MOTOR_IDLE_BIT);
  SetState( CNC_STATE_IDLE );

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
  SetState( CNC_STATE_IDLE );
}

void SignalMotorNotIdle( )
{
  xEventGroupClearBits( g_eventGroupHandle, MOTOR_IDLE_BIT );
  ClearState( CNC_STATE_IDLE );
}

bool IsMotorIdle( )
{
  return(( xEventGroupWaitBits( g_eventGroupHandle, MOTOR_IDLE_BIT, pdFALSE, pdFALSE, 0 ) & MOTOR_IDLE_BIT ) == MOTOR_IDLE_BIT ); 
}

void WaitForMotorIdle( )
{
  while(( xEventGroupWaitBits( g_eventGroupHandle, MOTOR_IDLE_BIT, pdTRUE, pdFALSE, 1000 / portTICK_PERIOD_MS ) & MOTOR_IDLE_BIT ) == 0 )
  {
    // ESP_LOGI( TAG, "Waiting for movement to complete..." ); 
  }
  ESP_LOGI( TAG, "Motors are idle" );
}