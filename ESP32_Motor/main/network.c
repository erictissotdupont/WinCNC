/*
 * Network.c
 *   Listener and talker socket
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "led_strip.h"

// For ESP_LOGx and ESP_ERROR_CHECK
#include "esp_log.h"
#include "esp_event.h"

// For Wi-Fi
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_smartconfig.h"

// For time (SNTP)
#include "time.h"
#include "sys/time.h"
#include "esp_system.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_sntp.h"

#include "network.h"
#include "Cnc.h"
#include "Events.h"
#include "Motor.h"

#define STORAGE_NAMESPACE       "storage"
#define STORAGE_SSID_NAME		    "SSIDName"
#define STORAGE_WIFI_PASSWORD   "WiFiPw"
#define MAX_SSID_NAME           32
#define MAX_WIFI_PASSWORD       64

#define DEBUG_UDPx

#define CONFIG_ESP_WPA3_SAE_PWE_BOTH   1
#define CONFIG_ESP_WIFI_AUTH_WPA2_PSK  1
#define CONFIG_ESP_WIFI_PW_ID          ""

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT  BIT0
#define ESPTOUCH_DONE_BIT   BIT1
#define WIFI_FAIL_BIT       BIT2
#define GOT_WIFI_SSID_PW	  BIT3
#define MSG_SENT_BIT        BIT4


static EventGroupHandle_t s_wifi_event_group;
static char g_szSSID[MAX_SSID_NAME];
static char g_szPassword[MAX_WIFI_PASSWORD];
static bool g_bSmartConfig = false;
unsigned long g_NextSeq = 0;
long g_NetworkPosition[5];

unsigned long g_nackCounter = 0;

unsigned long g_IP = 0;

int g_host_sock = -1;
struct sockaddr_in g_host_addr = { 0 }; 

#define RX_BUFFER_SIZE            (1500)
#define CMD_QUEUE_SIZE            (256)
QueueHandle_t g_cmd_queue = NULL;
static led_strip_handle_t g_led_strip = NULL;
static const char* TAG = "network";

// ----------------------------------------------------------------------------

bool SetRGBLED( uint8_t r, uint8_t g, uint8_t b )
{
	bool bRet = true;
	if( led_strip_set_pixel( g_led_strip, 0, r, g, b ) != ESP_OK ) bRet = false;
	if( led_strip_refresh(g_led_strip) != ESP_OK ) bRet = false;
	return bRet;
}

const unsigned char crc8_table[256] = {
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

unsigned char crc8( unsigned char* pt, unsigned int nbytes, unsigned char crc )
{
	while( nbytes-- > 0 )
	{
		crc = crc8_table[(crc ^ *pt++) & 0xff];
	}
	return crc;
}

bool MovementCommand( unsigned long seq, char* pt, bool bIgnoreCRC )
{
  cmd_t cmd;
  unsigned int remotePosCRC;
  
  if( sscanf( pt, CNC_CMD_PARAMS,
    &cmd.dx,
    &cmd.dy,
    &cmd.dz,
    &cmd.duration,
    &cmd.flags ) != 5 )
  {
      ESP_LOGE( TAG, "Message failed to decode" );
      SetState( CNC_STATE_COMMUNICATION_ERROR );
  }
  else 
  {
    remotePosCRC = cmd.flags & CMD_FLAGS_CRC_MASK;
    
    // Update the position from the command received so that
    // we can update calculate the CRC of the position the machine
    // should be after this command is executed
    g_NetworkPosition[0] += cmd.dx;
    g_NetworkPosition[1] += cmd.dy;
    g_NetworkPosition[2] += cmd.dz;
    g_NetworkPosition[3] = cmd.duration;
    g_NetworkPosition[4] = cmd.flags & ~CMD_FLAGS_CRC_MASK;
    
    uint8_t localPosCRC = crc8((uint8_t*)g_NetworkPosition, sizeof( g_NetworkPosition ), 0xFF );
    
    if( localPosCRC != remotePosCRC && !bIgnoreCRC )
    {
      ESP_LOGE( TAG, "Position CRC mismatch. Got %x, expected %x. (%ld,%ld,%ld)", 
        remotePosCRC, 
        localPosCRC, 
        g_NetworkPosition[0],
        g_NetworkPosition[1],
        g_NetworkPosition[2] );
        
      SetState( CNC_STATE_NETWORK_CRC_ERROR );
    }
    else if( xQueueSend( g_cmd_queue, 
                         &cmd, 
                         ( NACK_INTERVAL_MS / portTICK_PERIOD_MS )) != pdPASS )
    {
      SetState( CNC_STATE_COMMAND_QUEUE_FULL );
    }
    else
    {
      ClearState( CNC_STATE_COMMAND_QUEUE_FULL );
      return true;
    }
  }
  return false;
}

bool Calibrate( )
{
  cmd_t cmd = { 0 };
  bool bStatus = false;
 
  cmd.flags = CMD_FLAG_CALIBRATION;
  if( xQueueSend( g_cmd_queue, 
                  &cmd, 
                  1000 / portTICK_PERIOD_MS ) == pdPASS )
  {                      
    cmd.flags = CMD_CALIBRATION_COMPLETE;
    bStatus = (xQueueSend( g_cmd_queue, 
                           &cmd, 
                           1000 / portTICK_PERIOD_MS ) == pdPASS );
  }
  return bStatus;
}

int Respond( const char* rsp, unsigned long seq, unsigned int inQueue, char* outBuf )
{
  return sprintf( outBuf, "%s," CNC_POS_ACK_NAK_PARAMS,
    rsp,
    seq,
    g_NetworkPosition[0],
    g_NetworkPosition[1],
    g_NetworkPosition[2],
    GetState( ),
    inQueue );
}

int ParseMessage( char* msgbuf, int nbytes, char* outBuf )
{
  int ret = 0;
  unsigned int inQueue;

  inQueue = uxQueueMessagesWaiting( g_cmd_queue );

  //   C O M M A N D S
  // -------------------
  if( memcmp( msgbuf, CNC_CMD_HEADER ",", CNC_CMD_HEADER_LEN + 1 ) == 0 )
  {
    unsigned long seq;
    unsigned long cmdCount;
    bool bStatus = false;
    
    if( sscanf( msgbuf + CNC_CMD_HEADER_LEN + 1, CNC_CMD_HEADER_PARAMS, &seq, &cmdCount ) != 2 || 
        cmdCount > CMD_QUEUE_SIZE )
    {
      // Format error.
      ESP_LOGE( TAG, "Message header error" );
    }
    // We got a sequence # of a packet we already received ( a repeat )
    else if( seq == ( g_NextSeq - 1 ))
    {
      ESP_LOGW( TAG, "Retry of %lu - Queue:%d.", seq, inQueue );
    }
    // Got a completely out of order packet. That is not recoverable.
    else if( seq != g_NextSeq )
    {
      ESP_LOGE( TAG, "Out of sequence. Exp:%lu Got:%lu Queue:%d.", g_NextSeq, seq, inQueue );
    }
    // Queue is full
    else if(( CMD_QUEUE_SIZE - inQueue ) < cmdCount )
    {
      ESP_LOGW( TAG, "Queue is full. Got %lu commands. Queue has %d spaces.", cmdCount, CMD_QUEUE_SIZE - inQueue );
    }
    else
    {
      char *pt = msgbuf+4;
      bStatus = true;
      
      for( int i=0; i<cmdCount && bStatus; i++ )
      {
        // Find the next command separator '|'
        while( *pt != '|' && pt < (msgbuf + nbytes)) pt++;
        
        // Move to the next char which should be the start of the command
        pt++;
        if( pt >= (msgbuf + nbytes))
        {
          // Message size invalid
          ESP_LOGE( TAG, "Message too small" );
          SetState( CNC_STATE_COMMUNICATION_ERROR );
          bStatus = false;
        }
        // This is a movement command
        else if( *pt == '@' )
        {
          // Move to the start of the command parameters
          pt++;
          bStatus = MovementCommand( seq, pt, false );
          if( !bStatus )
          {
            ESP_LOGW( TAG, "Command %d failed.", i );
          }
        }
        else if( strncmp( pt, "ORIGIN", 6 ) == 0 )
        {
          // TODO : Reset the position to origin
        }           
        else if( strncmp( pt, "RST", 3 ) == 0 )
        {
          ESP_LOGW( TAG, "Reset command received." );            
        }
        else if( strncmp( pt, CNC_CMD_CALIBRATE, CNC_CMD_CALIBRATE_LEN ) == 0 )
        {
          bStatus = Calibrate( );
        }
        else
        {
          ESP_LOGE( TAG, "Invalid command" );
        }
      } // for( )        
    }
    
    if( !bStatus )
    {
      ret = Respond( CNC_NAK_HEADER, g_NextSeq, inQueue, outBuf ); 
    }
    else
    {
      ret = Respond( CNC_ACK_HEADER, seq, inQueue + cmdCount, outBuf );
      g_NextSeq++;
      MotorMoveIfIdle( );
    }
  } 
  else if( memcmp( msgbuf, CNC_INFO_HEADER, CNC_INFO_HEADER_LEN ) == 0 )
  {
    ret = sprintf( outBuf, CNC_INFO_HEADER "," CNC_INFO_PARAMS,
      CNC_PROTOCOL_VERSION,
      RX_BUFFER_SIZE,
      CMD_QUEUE_SIZE,
      1.0f / X_AXIS_RES,
      1.0f / Y_AXIS_RES,
      1.0f / Z_AXIS_RES );
  }
  else if( memcmp( msgbuf, CNC_POS_HEADER, 3 ) == 0 )
  {
    ret = Respond( CNC_POS_HEADER, g_NextSeq, inQueue, outBuf );
  }
  else
  {
    //ESP_LOGW( TAG, "Message ignored." );
  }
  return ret;
}

bool getWiFiCredentials( wifi_config_t *pWifi_Config )
{
	bool bStatus = false;
	size_t required_size = 0;
    nvs_handle_t nvs_handle;
    esp_err_t err;
	
	if( pWifi_Config == NULL )
	{
		return false;
	}

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
	{
		ESP_LOGE(TAG,"Failed to open NVS '%s' (Read)", STORAGE_NAMESPACE );
		return false;
	}

	required_size = sizeof( pWifi_Config->sta.ssid );
	err = nvs_get_blob(nvs_handle, STORAGE_SSID_NAME, pWifi_Config->sta.ssid, &required_size);
	if( err != ESP_OK )
	{
		ESP_LOGW(TAG,"Failed to get SSID (Err:%d)", err );
	}
	else
	{
		required_size = sizeof( pWifi_Config->sta.password );
		err = nvs_get_blob(nvs_handle, STORAGE_WIFI_PASSWORD, pWifi_Config->sta.password, &required_size);
		if( err != ESP_OK )
		{
			ESP_LOGW(TAG,"Failed to get Password (Err:%d)", err );
		}
		else
		{
			pWifi_Config->sta.threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD;
            pWifi_Config->sta.sae_pwe_h2e = ESP_WIFI_SAE_MODE;
            strcpy( (char*)pWifi_Config->sta.sae_h2e_identifier, H2E_IDENTIFIER );
			
			ESP_LOGI(TAG, "NVS: Read SSID='%s' Password='%s'", pWifi_Config->sta.ssid, pWifi_Config->sta.password );
			bStatus = true;
		}
  }

  // Close
  nvs_close(nvs_handle);
  return bStatus;
}

void saveWiFiCrendials( char *szSSID, char *szPassword )
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
	{
		ESP_LOGE(TAG,"Failed to open NVS '%s' (Write)", STORAGE_NAMESPACE );
		return;
	}

	err = nvs_set_blob(nvs_handle, STORAGE_SSID_NAME, szSSID, MAX_SSID_NAME );
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG,"Failed write SSID (%s)", szSSID );
	}
	else
	{
		ESP_LOGI(TAG,"NVS: Saved SSID='%s'", szSSID );
	}
	
	err = nvs_set_blob(nvs_handle, STORAGE_WIFI_PASSWORD, szPassword, MAX_WIFI_PASSWORD );
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG,"Failed to write password (%s)", szPassword );
	}
	else
	{
		ESP_LOGI(TAG,"NVS: Saved Password='%s'", szPassword );
	}

    // Commit and close
	nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
}

static void smartconfig_task(void * parm)
{
    EventBits_t uxBits;
    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_smartconfig_start(&cfg) );
    while (1) {
        uxBits = xEventGroupWaitBits(s_wifi_event_group, ESPTOUCH_DONE_BIT | GOT_WIFI_SSID_PW, true, false, portMAX_DELAY);
		if(uxBits & GOT_WIFI_SSID_PW ) {
			wifi_config_t wifi_config = {0};
			saveWiFiCrendials( g_szSSID, g_szPassword );
			getWiFiCredentials( &wifi_config );
			ESP_ERROR_CHECK( esp_wifi_disconnect() );
			ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
			esp_wifi_connect();
		}
        if(uxBits & ESPTOUCH_DONE_BIT) {
            ESP_LOGI(TAG, "smartconfig over");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }

    }
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT )
	{
		switch( event_id )
		{
		case WIFI_EVENT_STA_START :
			if( g_bSmartConfig )
			{
				xTaskCreate(smartconfig_task, "smartconfig_task", 4096, NULL, 3, NULL);
			}
			else
			{
				esp_wifi_connect();
			}
			break;
		case WIFI_EVENT_STA_DISCONNECTED :
			esp_wifi_connect();
			xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
			break;
		}
    }
	else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
	{
    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
    g_IP = ntohl( event->ip_info.ip.addr );
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);  
  } 
	else if (event_base == SC_EVENT )
	{
		switch( event_id )
		{
		case SC_EVENT_SCAN_DONE:
			ESP_LOGI(TAG, "Scan done");
			break;
		case SC_EVENT_FOUND_CHANNEL:
			ESP_LOGI(TAG, "Found channel");
			break;
		case SC_EVENT_GOT_SSID_PSWD:
			ESP_LOGI(TAG, "Got SSID and password");
			smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;	
			strcpy( g_szSSID, (char*)evt->ssid );
			strcpy( g_szPassword, (char*)evt->password );
			xEventGroupSetBits(s_wifi_event_group, GOT_WIFI_SSID_PW);
			break;
		case SC_EVENT_SEND_ACK_DONE:
			xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
			break;
		}
  }
}

static void UDP_task(void *pvParameters)
{
	bool bRun;
	static char rx_buffer[RX_BUFFER_SIZE];
	static char tx_buffer[128];
	int addr_family = 0;

	while (1) 
  {
		// Wait to be connected
		xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
	
		struct sockaddr_in dest_addr;
		dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
		dest_addr.sin_family = AF_INET;
		dest_addr.sin_port = htons(CNC_UDP_PORT);
		addr_family = AF_INET;

		g_host_sock = socket(addr_family, SOCK_DGRAM, IPPROTO_UDP);		
		if (g_host_sock < 0) 
		{
			ESP_LOGW(TAG, "Create socket failed.");
			vTaskDelay( 1000 / portTICK_PERIOD_MS );
			continue;
		}
		
		if (bind(g_host_sock, (const struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) 
		{
			ESP_LOGW(TAG, "Bind socket failed.");
			vTaskDelay( 1000 / portTICK_PERIOD_MS );
			continue;
		}

		bRun = true;
		ESP_LOGI(TAG, "Listening...");

		while (bRun)
		{
			struct sockaddr_in source_addr = { 0 };
			socklen_t socklen = sizeof(source_addr);
			int rx_len = recvfrom(g_host_sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

			if (rx_len < 0) 
			{
				bRun = false;
				ESP_LOGE(TAG, "Socket receive failed.");
				break;
			}
			else 
			{
				// Null-terminate the data we received and treat like a string
				rx_buffer[rx_len] = 0; 
			}
      
      #define RX_STRING_TRUNCATE_AT 50
      #define ELIPSYS_LEN 4

      if( rx_len > RX_STRING_TRUNCATE_AT )
      {
        char tmp[ELIPSYS_LEN];
        memcpy( tmp, rx_buffer + RX_STRING_TRUNCATE_AT - ELIPSYS_LEN, sizeof(tmp));
        strcpy( rx_buffer + RX_STRING_TRUNCATE_AT - ELIPSYS_LEN, "..." );
        ESP_LOGI(TAG, "Received %d bytes '%s' from %s", rx_len, rx_buffer, inet_ntoa(source_addr.sin_addr));
        memcpy( rx_buffer + RX_STRING_TRUNCATE_AT - ELIPSYS_LEN, tmp, sizeof(tmp));
      }
      else
      {
        ESP_LOGI(TAG, "Received %d bytes '%s' from %s", rx_len, rx_buffer, inet_ntoa(source_addr.sin_addr));
      }
      
      if( memcmp( rx_buffer, CNC_HEADER, CNC_HEADER_LEN ) == 0 )
      {			
        memset(&g_host_addr,0x00,sizeof(g_host_addr));
        g_host_addr.sin_addr.s_addr = source_addr.sin_addr.s_addr;
        g_host_addr.sin_family = AF_INET;
        g_host_addr.sin_port = htons(CNC_UDP_PORT);
        
        strcpy( tx_buffer, CNC_HEADER );
        
        int tx_len = ParseMessage( rx_buffer + CNC_HEADER_LEN, rx_len - CNC_HEADER_LEN, tx_buffer + CNC_HEADER_LEN );    
        if( tx_len > 0 )
        {
          int len = sendto( g_host_sock, tx_buffer, tx_len + CNC_HEADER_LEN, 0, (struct sockaddr *)&g_host_addr, sizeof(g_host_addr));
          if( len < 0 )
          {
            bRun = false;
            ESP_LOGE(TAG, "Socket transmit failed.");
            break;
          }
          else
          {
            xEventGroupSetBits(s_wifi_event_group, MSG_SENT_BIT);
            ESP_LOGI(TAG, "Sent:'%s' (%d) Rsp", tx_buffer, tx_len );
          }
        }
      }
		}

		if (g_host_sock != -1)
		{
			shutdown(g_host_sock, 0);
			close(g_host_sock);
		}
		ESP_LOGW(TAG, "UDP socket closed." );
	}
}

void NetworkIdleTask( )
{
  EventBits_t bits;
  
  bits = xEventGroupWaitBits(
			s_wifi_event_group,
			MSG_SENT_BIT,
			pdTRUE,
      pdFALSE,
			CNC_IDLE_POS_TIMEOUT_MS / portTICK_PERIOD_MS );
  
  if(( bits & MSG_SENT_BIT ) == 0 )
  {
    if( g_host_addr.sin_port != 0 && g_host_sock >= 0 )
    {
      char tx_buffer[100];
      int inQueue = uxQueueMessagesWaiting( g_cmd_queue );
      int tx_len = Respond( CNC_HEADER CNC_POS_HEADER, g_NextSeq, inQueue, tx_buffer );
      
      if( sendto( g_host_sock, tx_buffer, tx_len, 0, (struct sockaddr *)&g_host_addr, sizeof(g_host_addr)) <= 0 )
      {
        ESP_LOGE( TAG, "Failed to send idle POS to host" );
      }
      else
      {
        ESP_LOGI(TAG, "Sent:'%s' (%d) Idle", tx_buffer, tx_len );
      }
    }      
	}
}

int NetworkInit( bool bWiFiSetup )
{	  
  wifi_config_t wifi_config = {0};
  
  // LED
	// ---
	led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
	
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
	if( g_led_strip == NULL )
	{
		ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &g_led_strip));
		// Set all LED off to clear all pixels
		ESP_ERROR_CHECK(led_strip_clear(g_led_strip));
	}

	g_cmd_queue = xQueueCreate( CMD_QUEUE_SIZE, sizeof(cmd_t));
	if( g_cmd_queue == NULL )
	{
		ESP_LOGE( TAG, "Failed to create message queue." );
		return -1;
	}
  
	ESP_ERROR_CHECK(esp_netif_init());
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
	assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));
	ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
	
	if( !bWiFiSetup && getWiFiCredentials( &wifi_config ))
	{
		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
	}
	else
	{
		g_bSmartConfig = true;
	}
	
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "WiFiStartSTA finished.");

	int n = 0;
	EventBits_t bits;
	do
	{
		// Blink blue (SmartConfig) or green (WiFi STA)
		SetRGBLED( 0, !g_bSmartConfig && (n & 1) ? 0xFF : 0, g_bSmartConfig && (n & 1) ? 0xFF : 0 );
		n++;
		
		bits = xEventGroupWaitBits(
			s_wifi_event_group,
			WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
			pdFALSE,
			pdFALSE,
			250 / portTICK_PERIOD_MS );
				
	} while(( bits & WIFI_CONNECTED_BIT ) == 0 );

	// Turn LED solid ON blue or green
	SetRGBLED( 0, !g_bSmartConfig ? 0xFF : 0, g_bSmartConfig ? 0xFF : 0 );
	
	TaskHandle_t UDPTaskHandle = NULL;
	xTaskCreate(UDP_task, "udp_receive", 4096, NULL, 5, &UDPTaskHandle );
  		
	return 0;
}
