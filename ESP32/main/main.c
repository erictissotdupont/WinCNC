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
// For UART
#include "driver/uart.h"
#include "string.h"

#include "Network.h"
#include "LCD.h"
#include "Cnc.h"

static const char* TAG = "main";

uint64_t g_startTime = 0;
unsigned long g_Status = 0;

#define UART_TXD_PIN 			(GPIO_NUM_33)
#define UART_RXD_PIN 			(GPIO_NUM_35)
#define UART_RX_BUF_SIZE 	256

char g_UARTrxData[UART_RX_BUF_SIZE];
char g_ACKchar = '0';

#define DEBUG_UARTx

void cnc_loop( )
{
  while( 1 )
  {
    //LCD_Refresh( );
    vTaskDelay( 10 / portTICK_PERIOD_MS);  
  } 
}

int UART_Command( char* szCmd )
{
  int ret;
  char rx;
  int cbCmd = strlen( szCmd ) + 1;
  bool bInaString = false;
  int rxCount = 0;
  
  // Send the command
  if(( ret = uart_write_bytes(UART_NUM_1, szCmd, cbCmd )) != cbCmd )
  {
    ESP_LOGE( TAG, "UART Write error. Out:%d = Ret:%d.", cbCmd, ret );
    return 0;
  }
        
  if(( ret = uart_wait_tx_done( UART_NUM_1, 100 / portTICK_PERIOD_MS )) != ESP_OK )
  {
    ESP_LOGE( TAG, "UART Wait TX done error (%d).", ret );
    return 0;
  }
  
  #ifdef DEBUG_UART
  ESP_LOGI( TAG, "UART TX[%d] '%s'", cbCmd, szCmd );
  #endif
        
  while( 1 )
  {
    // Reading one character at a time
    if( uart_read_bytes(UART_NUM_1, (uint8_t*)&rx, 1, 1000 / portTICK_PERIOD_MS ) != 1 )
    {
      ESP_LOGE( TAG, "UART timeout." );
      return 0;
    }
    
    //ESP_LOGI( TAG, "UART got '%c' %d %d", rx, rxCount, bInaString );
 
    if( bInaString )
    {
      if( rxCount >= sizeof(g_UARTrxData))
      {
        ESP_LOGE( TAG, "UART buffer overflow." );
        return 0;
      }
      else
      {
        g_UARTrxData[rxCount++] = rx;
        if( rx == '\0' )
        {
          break;
        }       
      }      
    }
    else if( rx == g_ACKchar )
    {
#ifdef DEBUG_UART
      ESP_LOGI( TAG, "UART ACK" );
#endif
      g_UARTrxData[0] = rx;
      return 1;
    }
    else if( rx == 'W' )
    {
#ifdef DEBUG_UART
      ESP_LOGI( TAG, "UART WAIT!" );
#endif
      continue;
    }
    else if( rx == '#' )
    {
      rxCount = 0;
      bInaString = true;
    }
    else
    {
      ESP_LOGW( TAG, "UART: Unexpected char '%c'.", rx );
    }
  }
  
  #ifdef DEBUG_UART
  ESP_LOGI( TAG, "UART RX[%d] '%s'", rxCount, g_UARTrxData );
  #endif
    
  return rxCount;
}


bool ResetCommand( )
{
  while( UART_Command( "RESET" ) != 6 || strcmp( g_UARTrxData, "Hello" ) != 0 )
  {
    ESP_LOGW( TAG, "UART not ready. Retrying reset..." );
  }
  ESP_LOGI( TAG, "Reset successful" );
  g_ACKchar = '0';
  return true;
}


bool GetAnalogCommand( unsigned long* A0, unsigned long* A1, unsigned long* A2 )
{
  bool bStatus = false;
  unsigned long a0,a1,a2;
  
  if( UART_Command( "ANALOG" ) <= 0 )
  {
    ESP_LOGE( TAG, "Get analog values failed" );
  }
  else
  {      
    if( sscanf( g_UARTrxData, "A%luB%luC%lu",
        &a0,&a1,&a2 ) != 3 )
    {
      ESP_LOGE( TAG, "Analog '%s' failed to decode.", g_UARTrxData );
    }
    else
    {  
      ESP_LOGI( TAG, "Analog : %lu,%lu,%lu",
          *A0, *A1, *A2 );
          
      if( A0 ) *A0 = a0;
      if( A1 ) *A1 = a1;
      if( A2 ) *A2 = a2;
          
      bStatus = true;
    }
  }
  
  return bStatus;
}



bool GetPositionCommand( long* pX, long* pY, long *pZ, unsigned long* pS, unsigned long *pQ )
{
  long x,y,z;
  unsigned long S,Q;
  bool bStatus = false;
  
  if( UART_Command( "POSITION" ) <= 0 )
  {
    ESP_LOGE( TAG, "Get position failed" );
  }
  else
  {      
    if( sscanf( g_UARTrxData, "X%ldY%ldZ%ldS%lxQ%lu",
        &x,&y,&z, &S, &Q ) != 5 )
    {
      ESP_LOGE( TAG, "Position '%s' failed to decode.", g_UARTrxData );
    }
    else
    {  
      ESP_LOGI( TAG, "Position : %ld,%ld,%ld S:0x%08lX Queue:%lu",
          x,y,z,S,Q );
          
      if( pX ) *pX = x;
      if( pY ) *pY = y;
      if( pZ ) *pZ = z;
      if( pS ) *pS = S;
      if( pQ ) *pQ = Q;
          
      bStatus = true;
    }
  }
  
  return bStatus;
}


bool OriginCommand( )
{
  UART_Command( "ORIGIN" );
  return true;
}

bool Calibrate_Z( )
{
  int cbRet;
  int cbCmd;
  char szCmd[16];
  bool bStatus;
  
  cbCmd = sprintf( szCmd, "CAL_Z" );
  
  bStatus = (( cbRet = UART_Command( szCmd )) == 1 && g_UARTrxData[0] == g_ACKchar ); 
  if( !bStatus )
  {
    ESP_LOGE( TAG, "UART command[%d] '%s' failed (%d,%c).", cbCmd, szCmd, cbRet, g_UARTrxData[0] );
  }

  g_ACKchar++;
  if( g_ACKchar > '9' ) g_ACKchar = '0';
  
  return true;
}


bool CheckMachineIsIdle( unsigned long seq, struct sockaddr_in* source )
{
  int retry = 0;
  bool bStatus = false;
  int s = 0;
  long x[2],y[2],z[2];
  unsigned long S,Q;
  
  do
  {
    if( GetPositionCommand( &x[s], &y[s], &z[s], &S, &Q ) == false )
    {
      vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
    else
    {      
      if( Q != 0 )
      {
        ESP_LOGW( TAG, "Queue is not empty..." );
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
        s = 0;
      }
      else
      {         
        s++;
        if( s == 1 )
        {
          // Wait one second before fetching the position again
          for( int i=0; i<(1000 / NACK_INTERVAL_MS ); i++ )
          {     
            vTaskDelay( NACK_INTERVAL_MS / portTICK_PERIOD_MS );
            sendNAK( seq, source );
          }
        }
        else if( s == 2 )
        {       
          if( x[0] == x[1] && y[0] == y[1] && z[0] == z[1] )
          {
            // Machine is not moving and queue is empty
            SetPosition( x[0], y[0], z[0] );
            bStatus = true;
          }
          else
          {
            ESP_LOGW( TAG, "Position has changed." );
          }
          s = 0;
        }          
      }
    }    
  } while( !bStatus && retry++ < 30 );
  
  return bStatus;
}


bool MoveCommand( cmd_t* pCmd )
{
  int cbRet;
  bool bStatus;
  char szCmd[80];
  int cbCmd;
  
  cbCmd = sprintf( szCmd, "@X%ldY%ldZ%ldD%luS%luC",
    pCmd->dx,
    pCmd->dy,
    pCmd->dz,
    pCmd->duration,
    pCmd->flags );
    
  uint8_t crc = crc8( (unsigned char*)szCmd+1, cbCmd-1, 0xFF );
  cbCmd += sprintf( szCmd + cbCmd, "%d", crc );
  
  bStatus = (( cbRet = UART_Command( szCmd )) == 1 && g_UARTrxData[0] == g_ACKchar ); 
  if( !bStatus )
  {
    ESP_LOGE( TAG, "UART command[%d] '%s' failed (%d,%c).", cbCmd, szCmd, cbRet, g_UARTrxData[0] );
  }
  g_ACKchar++;
  if( g_ACKchar > '9' ) g_ACKchar = '0';
  
  return bStatus;
}



void initUART(void)
{
  int ret;
  /* Gray wire on the connector. Could be used in the future for resetting the Arduino?
  const gpio_config_t io_conf = {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = GPIO_LCD_RESET_PIN_SEL,
		.pull_down_en = 0,
		.pull_up_en = 0,
	};
 
  //configure GPIO with the given settings
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  */
	
	const uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};
	
	// We won't use a buffer for sending data.
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, UART_RX_BUF_SIZE, 0, 0, NULL, 0));
	ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART_TXD_PIN, UART_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  
  // Flush the UART Rx buffer to avoid processing spurious bytes
  // from a previous command
  if(( ret = uart_flush(UART_NUM_1)) != ESP_OK )
  {
    ESP_LOGE( TAG, "UART Flush error (%d).", ret );
  }
  
  ResetCommand( );
  
  /*
  cmd_t cmd = {
    .dx = 1000,
    .dy = 1200,
    .dz = 1400,
    .duration = 10000000L,
  .flags = 0 };
  
  MoveCommand( &cmd );
  */
  
  if( !CheckMachineIsIdle( 0, NULL ))
  {
    ESP_LOGE( TAG, "Unable to retrive CNC position" );
  }
  else
  {
    g_Status |= STATUS_GOT_POSITION;
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
  
  // LCD_Refresh( );
  
	// Initialize the network and start the tasks
	initSocketCom( );

  // Initialize the UART to the ARDUINO  
  initUART( );
		
	cnc_loop( );
	
	// Should never get out of the main loop
	ESP_LOGE( TAG, "Stopped" );
}
