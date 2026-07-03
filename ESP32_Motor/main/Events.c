#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "cnc.h"
#include "Events.h"
#include "WiFi.h"

#define EVENT_NEW_STATE		   BIT0
#define EVENT_CALLBACK_SET	 BIT1
#define MOTOR_IDLE_BIT       BIT2

EventGroupHandle_t g_eventGroupHandle;
unsigned long g_State;
int64_t g_Debug = 0x123456789ABCDEF0;

static const char* TAG = "events";

inline void Events_SetState( unsigned long flag )
{
  unsigned long prevState = g_State;
  g_State |= flag; 
  if( prevState != g_State )
  {
    if( xPortCanYield( ) == pdTRUE)
    {
      WiFi_SignalStateChangeFromISR( );
    }
    else
    {
      WiFi_SignalStateChange( );
    }
  }
}

inline void Events_ClearState( unsigned long flag )
{
  unsigned long prevState = g_State;
  g_State &= ~flag;
  if( prevState != g_State )
  {
    if( xPortCanYield( ) == pdTRUE)
    {
      WiFi_SignalStateChangeFromISR( );
    }
    else
    {
      WiFi_SignalStateChange( );
    }
  }
}

inline void Events_SetDebug( int64_t value, int64_t mask )
{
  g_Debug &= ~mask;
  g_Debug |= (value & mask);
}

inline int64_t Events_GetDebug( )
{
  return g_Debug;
}

inline unsigned long Events_GetState( )
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
  
  Events_SetState( CNC_STATE_CONNECTED | CNC_STATE_IDLE );
  
  return 0;
}

inline void Events_SignalMotorIdleFromISR( )
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

inline void Events_SignalMotorNotIdle( )
{
  xEventGroupClearBits( g_eventGroupHandle, MOTOR_IDLE_BIT );
  Events_ClearState( CNC_STATE_IDLE );
}

inline bool Events_IsMotorIdle( )
{
  return(( xEventGroupGetBits( g_eventGroupHandle ) & MOTOR_IDLE_BIT ) == MOTOR_IDLE_BIT ); 
}

inline bool Events_WaitForMotorIdle( unsigned long timeoutMs )
{
  return(( xEventGroupWaitBits( g_eventGroupHandle, MOTOR_IDLE_BIT, pdFALSE, pdFALSE, timeoutMs / portTICK_PERIOD_MS ) & MOTOR_IDLE_BIT ) == MOTOR_IDLE_BIT );
}