/*
 * WiFi.c
 *   WiFi connectivity management
*/
#include "cnc.h"

#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "led_strip.h"

// For BOOT button detection
#include "driver/gpio.h"

// For Wi-Fi
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_smartconfig.h"
#include <arpa/inet.h>
#include <sys/socket.h>

#include "UDP.h"
#include "Events.h"
#include "WiFi.h"

#define STORAGE_NAMESPACE       "storage"
#define STORAGE_SSID_NAME		    "SSIDName"
#define STORAGE_WIFI_PASSWORD   "WiFiPw"
#define MAX_SSID_NAME           32
#define MAX_WIFI_PASSWORD       64

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

static led_strip_handle_t g_led_strip;
static EventGroupHandle_t s_wifi_event_group;
static char g_szSSID[MAX_SSID_NAME];
static char g_szPassword[MAX_WIFI_PASSWORD];
static bool g_bSmartConfig;
unsigned long g_IP;
int g_host_sock = -1;
struct sockaddr_in g_host_addr; 

static const char* TAG = "WiFi";

// ----------------------------------------------------------------------------

bool WiFi_SetRGBLED( uint8_t r, uint8_t g, uint8_t b )
{
	bool bRet = true;
	if( led_strip_set_pixel( g_led_strip, 0, r, g, b ) != ESP_OK ) bRet = false;
	if( led_strip_refresh(g_led_strip) != ESP_OK ) bRet = false;
	return bRet;
}

bool WiFi_GetCredentials( wifi_config_t *pWifi_Config )
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

void WiFi_ClearCredential( )
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
  
  err = nvs_erase_all(nvs_handle);
  if (err != ESP_OK)
	{
		ESP_LOGE(TAG,"Failed to erase NVS");
	}
  else
  {
	  err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
    {
      ESP_LOGE(TAG,"Failed to commit erase NVS");
    }
  }
  
  nvs_close(nvs_handle);
}

void WiFi_SaveCredentials( char *szSSID, char *szPassword )
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

static void WiFi_SmartconfigTask(void * parm)
{
  EventBits_t uxBits;
  ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
  smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_smartconfig_start(&cfg) );
  while (1) 
  {
    uxBits = xEventGroupWaitBits(s_wifi_event_group, ESPTOUCH_DONE_BIT | GOT_WIFI_SSID_PW, true, false, portMAX_DELAY);
    if(uxBits & GOT_WIFI_SSID_PW )
    {
      wifi_config_t wifi_config = {0};
      WiFi_SaveCredentials( g_szSSID, g_szPassword );
      WiFi_GetCredentials( &wifi_config );
      ESP_ERROR_CHECK( esp_wifi_disconnect() );
      ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
      esp_wifi_connect();
    }
    if(uxBits & ESPTOUCH_DONE_BIT)
    {
      ESP_LOGI(TAG, "smartconfig over");
      esp_smartconfig_stop();
      vTaskDelete(NULL);
    }
  }
}

static void WiFi_EventHandler(void* arg,
                              esp_event_base_t event_base,
                              int32_t event_id,
                              void* event_data)
{
	if (event_base == WIFI_EVENT )
	{
		switch( event_id )
		{
		case WIFI_EVENT_STA_START :
			if( g_bSmartConfig )
			{
				xTaskCreate(WiFi_SmartconfigTask, "WiFi_SmartconfigTask", 4096, NULL, 3, NULL);
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

static void WiFi_ReceiveTask(void *pvParameters)
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
      #define ELIPSYS_STR "..."
      #define ELIPSYS_LEN 4

      if( rx_len > RX_STRING_TRUNCATE_AT )
      {
        char tmp[ELIPSYS_LEN];
        memcpy( tmp, rx_buffer + RX_STRING_TRUNCATE_AT - ELIPSYS_LEN, sizeof(tmp));
        strcpy( rx_buffer + RX_STRING_TRUNCATE_AT - ELIPSYS_LEN, ELIPSYS_STR );
        ESP_LOGI(TAG, "Received %d bytes '%s' from %s", rx_len, rx_buffer, inet_ntoa(source_addr.sin_addr));
        memcpy( rx_buffer + RX_STRING_TRUNCATE_AT - ELIPSYS_LEN, tmp, sizeof(tmp));
      }
      else
      {
        ESP_LOGI(TAG, "Received %d bytes '%s' from %s", rx_len, rx_buffer, inet_ntoa(source_addr.sin_addr));
      }
      
      int tx_len = UDP_ParseMessage( rx_buffer, rx_len, tx_buffer );    
      if( tx_len > 0 )
      {
        memset(&g_host_addr,0x00,sizeof(g_host_addr));
        g_host_addr.sin_addr.s_addr = source_addr.sin_addr.s_addr;
        g_host_addr.sin_family = AF_INET;
        g_host_addr.sin_port = htons(CNC_UDP_PORT);
        
        int len = sendto( g_host_sock, tx_buffer, tx_len, 0, (struct sockaddr *)&g_host_addr, sizeof(g_host_addr));
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

		if (g_host_sock != -1)
		{
			shutdown(g_host_sock, 0);
			close(g_host_sock);
		}
		ESP_LOGW(TAG, "UDP socket closed." );
	}
}

void WiFi_IdleTask( )
{
  EventBits_t bits;
  static int bootDownCnt = 0;
  
  if( gpio_get_level( BOOT_GPIO ) == 0 )
  {
    bootDownCnt++;
    if( bootDownCnt > 3 )
    {
      WiFi_ClearCredential( );
      WiFi_SetRGBLED( 0,0,0 );
      vTaskDelay( 1000 / portTICK_PERIOD_MS );
      while( gpio_get_level( BOOT_GPIO ) == 0 )
      {
        ESP_LOGW( TAG, "Let go the BOOT button" );
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
      }
      ESP_LOGW( TAG, "Rebooting. " );
      abort( );
    }
    else
    {      
      ESP_LOGW( TAG, "BOOT button pressed. Hold to reset WiFi and reboot (%d).", 4 - bootDownCnt );
    }
  }
  else
  {
    bootDownCnt = 0;
  }
    
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
      int tx_len = UDP_GetIdleStatus( tx_buffer );
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

int WiFi_Init( bool bWiFiSetup )
{	  
  wifi_config_t wifi_config = {0};
  
  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      // NVS partition was truncated and needs to be erased
      // Retry nvs_flash_init
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
  }
  ESP_ERROR_CHECK( err );
  
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL<<BOOT_GPIO);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  
  // Initialize onboard LED
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
                                                        &WiFi_EventHandler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &WiFi_EventHandler,
                                                        NULL,
                                                        &instance_got_ip));
	ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &WiFi_EventHandler, NULL) );
	
	if( !bWiFiSetup && WiFi_GetCredentials( &wifi_config ))
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
		WiFi_SetRGBLED( 0, !g_bSmartConfig && (n & 1) ? 0xFF : 0, g_bSmartConfig && (n & 1) ? 0xFF : 0 );
		n++;
		
		bits = xEventGroupWaitBits(
			s_wifi_event_group,
			WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
			pdFALSE,
			pdFALSE,
			LED_BLINK_RATE_MS / portTICK_PERIOD_MS );
				
	} while(( bits & WIFI_CONNECTED_BIT ) == 0 );

	// Turn LED solid ON blue or green
	WiFi_SetRGBLED( 0, !g_bSmartConfig ? 0xFF : 0, g_bSmartConfig ? 0xFF : 0 );
	
	TaskHandle_t TaskHandle = NULL;
	xTaskCreate(WiFi_ReceiveTask, "WiFi_ReceiveTask", 4096, NULL, 5, &TaskHandle );
  		
	return 0;
}
