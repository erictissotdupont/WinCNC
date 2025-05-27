#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "cnc.h"

#define EVENT_NEW_STATE		   BIT0
#define EVENT_CALLBACK_SET	 BIT1
#define MOTOR_IDLE_BIT       BIT2

EventGroupHandle_t g_eventGroupHandle;
unsigned long g_State;
int64_t g_Debug = 0x123456789ABCDEF0;

static const char* TAG = "events";

void Events_SetState( unsigned long flag )
{
  g_State |= flag; 
}
void Events_ClearState( unsigned long flag )
{
  g_State &= ~flag;
}

void Events_SetDebug( int64_t value )
{
  g_Debug = value;
}

int64_t Events_GetDebug( )
{
  return g_Debug;
}

unsigned long Events_GetState( )
{
  return g_State;
}

int Events_Init( )
{
  uint16_t testEndianness = 0x1234;
    
	g_eventGroupHandle = xEventGroupCreate();  
	if( g_eventGroupHandle == NULL )
	{
		ESP_LOGE( TAG, "Failed to create event group." );
		return -1;
	}
  
  // Zero all the flags
  Events_ClearState((unsigned long)-1);
   
  // Test the system's endianness
  if( *((char*)&testEndianness) == 0x34 )
  { 
    Events_SetState( CNC_STATE_LITTLE_ENDIAN );
  }
  
  // By Default, motors are idle and limit sensors are not active
  xEventGroupSetBits(g_eventGroupHandle, MOTOR_IDLE_BIT);
  
  Events_SetState( CNC_STATE_CONNECTED | CNC_STATE_IDLE | CNC_STATE_LIMITS_INACTIVE );
  
  // Events_ClearState( CNC_STATE_LIMITS_INACTIVE );
  // Events_SetState( CNC_STATE_Z_CALIBRATED | CNC_STATE_X_CALIBRATED | CNC_STATE_Y_CALIBRATED );

  return 0;
}

void IRAM_ATTR Events_SignalMotorIdleFromISR( )
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
  Events_SetState( CNC_STATE_IDLE );
}

void Events_SignalMotorNotIdle( )
{
  xEventGroupClearBits( g_eventGroupHandle, MOTOR_IDLE_BIT );
  Events_ClearState( CNC_STATE_IDLE );
}

bool Events_IsMotorIdle( )
{
  return(( xEventGroupWaitBits( g_eventGroupHandle, MOTOR_IDLE_BIT, pdFALSE, pdFALSE, 0 ) & MOTOR_IDLE_BIT ) == MOTOR_IDLE_BIT ); 
}

bool Events_WaitForMotorIdle( unsigned long timeoutMs )
{
  return(( xEventGroupWaitBits( g_eventGroupHandle, MOTOR_IDLE_BIT, pdFALSE, pdFALSE, timeoutMs / portTICK_PERIOD_MS ) & MOTOR_IDLE_BIT ) == MOTOR_IDLE_BIT );
}