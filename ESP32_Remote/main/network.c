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
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// For ESP_LOGx and ESP_ERROR_CHECK
#include "esp_log.h"
#include "esp_event.h"

// For Wi-Fi
#include "esp_wifi.h"

#include "Cnc.h"
#include "network.h"

// Arbitrary non-privileged port
#define BROADCAST_PORT   50042
#define DATA_PORT        50043
#define HELLO            "Cnc 2.0"

#define NETWORK_STACK_SIZE		4096

#define DEBUG_UDPx

int bRun = 1;
int bGotIP = 0;
int g_bConnected = 0;

long g_xPos;
long g_yPos;
long g_zPos;
unsigned long g_inQueue = 0;
unsigned long g_NextSeq = 0;

char myIP[IPSTRSIZE] = {0};
unsigned long iMyIP = 0;
char myMAC[MACSTRSIZE] = {0};

static const char* TAG = "network";

typedef enum {
	cncStatus_Success = 0,
  cncStatus_HeaderDecodingError = -1,
  cncStatus_MessageIsTooShort = -2,
  cncStatus_CommandDecodingError = -3,
  cncStatus_PositionCRCmismatch = -4,
  cncStatus_UnknownCommand = -5,
  cncStatus_SequenceError = -6,
} tCnCCmdStatus;

uint64_t g_timeStateTimout;

struct sockaddr_in g_hostAddr = {0};
struct sockaddr_in g_localAddr = {0};

int g_transmitSocket = 0;


#define EVENT_NEW_STATE		   BIT0
#define EVENT_CALLBACK_SET	 BIT1
#define WIFI_CONNECTED_BIT   BIT2
#define WIFI_FAIL_BIT        BIT3





EventGroupHandle_t eventGroupHandle;

TaskHandle_t receiverTaskHandle = 0;
TaskHandle_t senderTaskHandle = 0;
TaskHandle_t ackRxTaskHandle = 0;

void ResetStatus( )
{
  uint16_t testEndian = 0x1234;
  
  // By default, zero
  g_Status = 0;
  
  // Test the system's endianness
  if( *((char*)&testEndian) == 0x34 )
  { 
    g_Status |= STATUS_LITLE_ENDIAN;
  } 
}

char* getMyIP( )
{
  return myIP;
}

void ackRxTask(void* arg)
{
  char ackMsg[80];
  struct sockaddr_in rxCncAddr;
  
  ESP_LOGI( TAG, "Ack RX task started." );
  
  while(bRun)
  {
    unsigned int addrlen = sizeof(rxCncAddr);
    
    memset(&rxCncAddr,0,sizeof(rxCncAddr));
    rxCncAddr.sin_family=AF_INET;
    rxCncAddr.sin_port=htons(DATA_PORT);
    rxCncAddr.sin_addr=g_hostAddr.sin_addr;
    
#ifdef DEBUG_UDP        
    char szIP[20];
    inet_ntop(AF_INET, &rxCncAddr.sin_addr, szIP, sizeof(szIP));
    ESP_LOGI( TAG, "Waiting for ack from %s", szIP );
#endif
        
    if( recvfrom(g_transmitSocket,ackMsg,sizeof(ackMsg),0,(struct sockaddr *)&rxCncAddr,&addrlen) < 0 )
    {
      // Fatal
      ESP_LOGE( TAG, "recvfrom( ) failed." );
      return;
    }
    
    if( memcmp( ackMsg, "ACK,", 4 ) == 0 )
    {
      unsigned long seq;
      unsigned long status;
      long x,y,z;
      
      if( sscanf( ackMsg + 4, "%lu,%ld,%ld,%ld,%ld,%ld", 
        &seq, 
        &x,
        &y,
        &z,
        &g_inQueue,    
        &status ) != 6 )
      {
        ESP_LOGW( TAG, "Warning ACK format error %s", ackMsg );
      }
      else
      {
#ifdef DEBUG_UDP
        ESP_LOGI( TAG, "Got ACK. inQueue = %lu", g_inQueue );
#endif
      }
    }
  }  
}

void receiverTask(void *arg)
{
  int rxSocket, nbytes;
  unsigned int addrlen;
  u_int yes=1;
  static char msgbuf[2048]; // Static to avoid bloating the stack

  ESP_LOGI( TAG, "Receiver task started." );

  // create what looks like an ordinary UDP socket
  if ((rxSocket=socket(AF_INET,SOCK_DGRAM,0)) < 0)
  {
    ESP_LOGE( TAG, "Create RX socket failed");
    return;
  }

  // allow multiple sockets to use the same PORT number
  if (setsockopt(rxSocket,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes)) < 0)
  {
    ESP_LOGE( TAG, "Reusing ADDR failed on RX sock");
    return;
  }

  // Set up destination address
  memset(&g_localAddr,0,sizeof(g_localAddr));
  g_localAddr.sin_family=AF_INET;
  g_localAddr.sin_addr.s_addr=htonl(INADDR_ANY);
  g_localAddr.sin_port=htons(BROADCAST_PORT);
  
  // bind to receive from data this port only
  if (bind(rxSocket,(struct sockaddr *) &g_localAddr,sizeof(g_localAddr)) < 0)
  {
    ESP_LOGE( TAG, "RX socket bind failed");
    return;
  }
  
  // Now just enter the receive loop
  while (bRun) 
  {
    struct sockaddr_in cncAddr;
    char srcIP[IPSTRSIZE];

    memset(&cncAddr,0,sizeof(cncAddr));
    cncAddr.sin_family=AF_INET;
    cncAddr.sin_addr.s_addr=htonl(INADDR_ANY);
    cncAddr.sin_port=htons(BROADCAST_PORT);
    addrlen=sizeof(cncAddr);
    if ((nbytes=recvfrom(rxSocket,msgbuf,sizeof(msgbuf),0,(struct sockaddr *) &cncAddr,&addrlen)) < 0 )
    {
      // Fatal
      ESP_LOGE( TAG, "recvfrom( ) failed." );
      return;
    }
    
    inet_ntop(AF_INET, &g_hostAddr.sin_addr, srcIP, sizeof(srcIP));
    if(strcmp( srcIP, myIP )==0) 
    {
      // Ignore our own transmission
      ESP_LOGW( TAG, "Ignore our own transmission" );
      continue;
    }
    
    if( msgbuf[nbytes-1] != 0 )
    {
      msgbuf[nbytes] = 0;
    }
    
    ESP_LOGI( TAG, "Received %s", msgbuf );
       
    if( sscanf( msgbuf, "CNC,%ld,%ld,%ld,%ld,%lx",
      &g_NextSeq,
      &g_xPos,
      &g_yPos,
      &g_zPos,
      &g_Status ) != 5 )
    {
      ESP_LOGE( TAG, "Decoding failed." );  
    }
    else
    {      
      g_hostAddr.sin_addr = cncAddr.sin_addr;
      
      if( ackRxTaskHandle == 0 )
      {
        if( xTaskCreate( 
          ackRxTask, 
          "ackRx", 
          NETWORK_STACK_SIZE, 
          &cncAddr, 
          tskIDLE_PRIORITY, 
          &ackRxTaskHandle ) != pdPASS )
        {
          ESP_LOGE( TAG, "Failed to create sender task" );
        }
      }
    }
  }
  return;
}

extern int g_joyX;
extern int g_joyY;
extern int g_Green;

int absInt( int n )
{
  if( n < 0 ) return -n;
  return n;
}

void senderTask(void* arg)
{
  struct sockaddr_in cncAddr;
  u_int yes=1;
  
  ESP_LOGI( TAG, "Sender task started." );
  
  // Initislize broadcast address
  memset(&cncAddr,0,sizeof(cncAddr));
  cncAddr.sin_family=AF_INET;
  cncAddr.sin_port=htons(DATA_PORT);
  cncAddr.sin_addr=g_hostAddr.sin_addr;
  
  // Create TX UDP socket
  if ((g_transmitSocket = socket(AF_INET,SOCK_DGRAM,0)) < 0) {
    ESP_LOGE( TAG, "Failed to open TX socket" );
    return;
  }
  
  // Allow multiple sockets to use the same PORT number
  if (setsockopt(g_transmitSocket,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes)) < 0) {
    ESP_LOGE( TAG, "Reusing ADDR failed");
    return;
  }
  
  float stepX = (int)(1.0f / (128.0 * X_AXIS_RES));
  float stepY = (int)(1.0f / (128.0 * Y_AXIS_RES));
  float stepZ = (int)(1.0f / (128.0 * Z_AXIS_RES));  
  int delay_ms = 0;
 
  while (bRun) 
  {
    char cmd[64];
    long dX, dY, dZ;
    unsigned long duration_ms;
        
    *cmd = 0;
    dX = 0;
    dY = 0;
    dZ = 0;
    duration_ms = 0;
    
#define MICRO_STEP_THR   2
  
    if( g_joyX || g_joyY )
    {
      if(( absInt( g_joyX ) <= MICRO_STEP_THR && g_joyY == 0 ) || 
         ( absInt( g_joyY ) <= MICRO_STEP_THR && g_joyX == 0 ))
      {       
        dX = (int)(g_joyX * stepX);        
        dY = (int)(g_joyY * stepY);
        dZ = (int)(g_joyX * stepZ);
      
        delay_ms = 250;
        duration_ms = 50;
      }
      
      else
      { 
        if( g_joyX > 0 )
        {
          dX = (int)((g_joyX - MICRO_STEP_THR ) * stepX );
          dZ = (int)((g_joyX - MICRO_STEP_THR ) * stepZ );
        }
        else if( g_joyX < 0 )
        {
          dX = (int)((g_joyX + MICRO_STEP_THR ) * stepX );
          dZ = (int)((g_joyX + MICRO_STEP_THR ) * stepZ );
        }
          
        if( g_joyY > 0 )        
          dY = (int)((g_joyY - MICRO_STEP_THR ) * stepY );
        else if( g_joyY < 0 )
          dY = (int)((g_joyY + MICRO_STEP_THR ) * stepY );
          
        
        delay_ms = 0;
        duration_ms = 25;
      }
      
      if( !g_Green )
      {
        dZ = 0;
      }
      else
      {
        dZ = -dZ;
        dX = 0;
        dY = 0;
      }
           
      sprintf( cmd, "MAN,@%ld,%ld,%ld,%lu,0,%X", dX, dY, dZ, duration_ms * 1000L, 0 );
    }
    
    if( *cmd && g_hostAddr.sin_addr.s_addr != 0 )
    { 
      cncAddr.sin_addr=g_hostAddr.sin_addr;
      
      if ( sendto(g_transmitSocket,cmd,strlen(cmd)+1,0,(struct sockaddr *) &cncAddr, sizeof(cncAddr)) < 0) 
      {
        ESP_LOGE( TAG, "Send failed" );
      }
      else
      {                
#ifdef DEBUG_UDP
        char targetIP[20];
        inet_ntop(AF_INET, &cncAddr.sin_addr, targetIP, sizeof(targetIP));
        ESP_LOGI( TAG, "Sent : %s to %s", cmd, targetIP );
#endif                
        g_xPos += dX;
        g_yPos += dY;
        g_zPos += dZ; 
      }
      
      if( delay_ms )
      {
        vTaskDelay( delay_ms / portTICK_PERIOD_MS);
      }
      else
      {      
     
        if( g_inQueue == 0 )
        {
          vTaskDelay( (duration_ms - 5) / portTICK_PERIOD_MS);
        }
        else
        {
          vTaskDelay( (duration_ms + 5) / portTICK_PERIOD_MS);
        }
      }
    }
    else
    {
      vTaskDelay( 10 / portTICK_PERIOD_MS);
    }      
  }
  return;
}

int isIPConnected( )
{
	
	if( bGotIP )
		return 1;
	else 
		return 0;

}


static int s_retry_num = 0;
#define WIFI_CONNECT_MAXIMUM_RETRY 10

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < WIFI_CONNECT_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(eventGroupHandle, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(eventGroupHandle, WIFI_CONNECTED_BIT);
        
        bGotIP = 1;
       
        if( xTaskCreate( 
          receiverTask, 
          "receiver", 
          NETWORK_STACK_SIZE, 
          NULL, 
          tskIDLE_PRIORITY, 
          &receiverTaskHandle ) != pdPASS )
        {
          ESP_LOGE( TAG, "Failed to create receiver task" );
        }
        
        if( xTaskCreate( 
          senderTask, 
          "sender", 
          NETWORK_STACK_SIZE, 
          NULL, 
          tskIDLE_PRIORITY, 
          &senderTaskHandle ) != pdPASS )
        {
          ESP_LOGE( TAG, "Failed to create sender task" );
        }
    }
}

int Wifi_Init( void )
{	 
  eventGroupHandle = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

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

  wifi_config_t wifi_config = {
      .sta = {
          .ssid = "aet home",
          .password = "makeaguess",
          /* Setting a password implies station will connect to all security modes including WEP/WPA.
           * However these modes are deprecated and not advisable to be used. Incase your Access point
           * doesn't support WPA2, these mode can be enabled by commenting below line */
          .bssid_set = 1,
          .bssid = { 0x44,0x07,0x0b,0x03,0x08,0x00 }, // Garage AP
          
     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

          .pmf_cfg = {
              .capable = true,
              .required = false
          },
      },
  };
  
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
  ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, WIFI_BW_HT20));
  ESP_ERROR_CHECK(esp_wifi_start() );

  ESP_LOGI(TAG, "wifi_init_sta finished.");

#if 0
  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
   * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
  EventBits_t bits = xEventGroupWaitBits(eventGroupHandle,
          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
          pdFALSE,
          pdFALSE,
          portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
   * happened. */
  if (bits & WIFI_CONNECTED_BIT) 
  {
  
  }
  else if (bits & WIFI_FAIL_BIT) 
  {
    
  } 
  else 
  {
      ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }
  /* The event will not be processed after unregister */
  ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
  ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
  vEventGroupDelete(eventGroupHandle);
#endif
  		
	return 0;
}

