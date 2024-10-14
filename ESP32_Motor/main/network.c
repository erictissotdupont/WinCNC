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

// For ESP_LOGx and ESP_ERROR_CHECK
#include "esp_log.h"
#include "esp_event.h"

// For Ethernet
#ifdef USE_ETHERNET
  #include "esp_netif.h"
  #include "esp_eth.h"
#endif

// For Wi-Fi
#include "esp_wifi.h"

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

// Arbitrary non-privileged port
#define BROADCAST_PORT   50042
#define DATA_PORT        50043
#define HELLO            "Cnc 2.0"

#define NETWORK_STACK_SIZE		4096

#define DEBUG_UDPx

unsigned long comStateIdx = 0;
unsigned long currentComState = 0;

int bRun = 1;
int bGotIP = 0;

int g_bConnected = 0;

unsigned long g_NextSeq = 0;

long g_NetworkPosition[3];

long g_xPos;
long g_yPos;
long g_zPos;

unsigned long g_nackCounter = 0;

char myIP[IPSTRSIZE] = {0};
unsigned long iMyIP = 0;
unsigned long iBroadcast = 0;
char myMAC[MACSTRSIZE] = {0};

stateChangeCallback callback = NULL;

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

#define CMD_QUEUE_SIZE            ( 256 )
QueueHandle_t g_cmd_queue = NULL;

struct sockaddr_in g_broadcastAddr;
int g_transmitSocket;

TaskHandle_t receiverTaskHandle;
TaskHandle_t broadcastTaskHandle;

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

char* getMyMAC( )
{
  return myMAC;
}

static const unsigned char crc8_table[256] = {
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

void SetPosition( long x, long y, long z )
{
  g_NetworkPosition[0] = x;
  g_NetworkPosition[1] = y;
  g_NetworkPosition[2] = z;
              
  g_xPos = x;
  g_yPos = y;
  g_zPos = z;    
}


void processEvent( char* pt )
{
  
}

void sendMessage( char* msg, size_t cbMsg, struct sockaddr_in* target )
{
  if( msg[cbMsg-1] != 0 )
  {
    ESP_LOGE( TAG, "Outpbound message not zero terminated." );
    msg[cbMsg] = 0;
  }
  if( sendto( g_transmitSocket, msg, cbMsg, 0, (struct sockaddr*)target, sizeof(struct sockaddr)) < 0 )
  {
    ESP_LOGE( TAG, "Failed to send message." );
  }
  else
  {
#ifdef DEBUG_UDP
    ESP_LOGI( TAG, "Sent:'%s'", msg );
#endif
  }
}

void sendACK( unsigned long seq, tCnCCmdStatus status, int inQueue, struct sockaddr_in* target )
{
  char rspbuf[80];

  int cbRsp = sprintf( rspbuf, "ACK,%lu,%ld,%ld,%ld,%d,%d", 
    seq, 
    g_xPos,
    g_yPos,
    g_zPos,
    inQueue,    
    (int)status ) + 1;
    
  sendMessage( rspbuf, cbRsp, target );
}

void sendNAK( unsigned long seq, struct sockaddr_in* target )
{
  if( target && target->sin_addr.s_addr != 0 )
  {  
    char rspbuf[80];
    int cbRsp = sprintf( rspbuf, "NAK,%lu,%ld,%ld,%ld,%lu", 
      seq,
      g_xPos,
      g_yPos,
      g_zPos,
      g_nackCounter ) + 1;
      
    g_nackCounter++;
      
    sendMessage( rspbuf, cbRsp, target );
  }
}

tCnCCmdStatus MovementCommand( unsigned long seq, char* pt, struct sockaddr_in* source, bool bIgnoreCRC )
{
  tCnCCmdStatus status = cncStatus_Success;
  cmd_t cmd;
  unsigned int remotePosCRC;
  int nackCount = 0;
  static uint64_t timeSinceLastNAK = 0;
    
  // Pre-calculate the current position CRC so that we can compare
  // it what the command expects us to be at
  uint8_t localPosCRC = crc8( 
    (uint8_t*)g_NetworkPosition, 
    sizeof( g_NetworkPosition ), 
    0xFF );

  if( sscanf( pt, "%ld,%ld,%ld,%lu,%lu,%x",
    &cmd.dx,
    &cmd.dy,
    &cmd.dz,
    &cmd.duration,
    &cmd.flags,
    &remotePosCRC ) != 6 )
  {
      ESP_LOGE( TAG, "Message failed to decode" );
      status = cncStatus_CommandDecodingError;
  }
  else if( localPosCRC != remotePosCRC && !bIgnoreCRC )
  {
    ESP_LOGE( TAG, "Position CRC mismatch. Got %x, expected %x.", remotePosCRC, localPosCRC );
    status = cncStatus_PositionCRCmismatch;
  }
  else 
  {
    // Update the position from the command received so that
    // we can update calculate the CRC
    g_NetworkPosition[0] += cmd.dx;
    g_NetworkPosition[1] += cmd.dy;
    g_NetworkPosition[2] += cmd.dz;

    // Try to place msg into the queue. NAK until it's in queue.
    // Note that we want to nack immediately to reduce the timeout
    // on the host side.
    while( xQueueSend( g_cmd_queue, 
                       &cmd, 
                       nackCount == 0 ? 0 : ( NACK_INTERVAL_MS / portTICK_PERIOD_MS )) != pdPASS )
    {
      // The message was placed in the queue. If it has been more than
      // the nack interval or it's the first message we put in the queue,
      // send a NACK to the host.
      if( esp_timer_get_time( ) > timeSinceLastNAK )
      {
        nackCount++;
        sendNAK( seq, source );
        timeSinceLastNAK = esp_timer_get_time( ) + ( NACK_INTERVAL_MS * 1000 );
      }    
    }
  }
  return status;
}

void receiverTask(void *arg)
{
  int rxSocket, nbytes;
  int inQueue;
  unsigned int addrlen;
  u_int yes=1;
  tCnCCmdStatus status = cncStatus_Success;
  static char msgbuf[2048]; // Static to avoid bloating the stack
  struct sockaddr_in source;
  struct sockaddr_in local;

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
  memset(&local,0,sizeof(local));
  local.sin_family=AF_INET;
  local.sin_addr.s_addr=htonl(INADDR_ANY);
  local.sin_port=htons(DATA_PORT);
  
  // bind to receive from data this port only
  if (bind(rxSocket,(struct sockaddr *) &local,sizeof(local)) < 0)
  {
    ESP_LOGE( TAG, "RX socket bind failed");
    return;
  }

  // Now just enter the receive loop
  while (bRun) 
  {
    char srcIP[IPSTRSIZE];

    addrlen=sizeof(source);
    if ((nbytes=recvfrom(rxSocket,msgbuf,sizeof(msgbuf),0,(struct sockaddr *) &source,&addrlen)) < 0 )
    {
      // Fatal
      ESP_LOGE( TAG, "rcvfrom( ) failed." );
      return;
    }
    
    inet_ntop(AF_INET, &source.sin_addr, srcIP, sizeof(srcIP));
    if(strcmp( srcIP, myIP )==0) 
    {
      // Ignore our own transmission
      ESP_LOGW( TAG, "Ignore our own transmission" );
      continue;
    }
    
    if( msgbuf[nbytes-1] != 0 )
    {
      ESP_LOGE( TAG, "Inbound message not zero terminated." );
      msgbuf[nbytes] = 0;
    }
    
    
#ifdef DEBUG_UDP    
    #define MSG_TRUNK_AT  30
    #define ELIPSYS       4   // Length of "..." with the zero!
    char tmp[ELIPSYS];
    
    if( nbytes > MSG_TRUNK_AT )
    {
      memcpy( tmp, msgbuf + MSG_TRUNK_AT - ELIPSYS, ELIPSYS );
      strcpy( msgbuf + MSG_TRUNK_AT - ELIPSYS, "..." );
    }      
	
	  ESP_LOGI( TAG, "Received %d '%s'", nbytes, msgbuf );
    
    if( nbytes > MSG_TRUNK_AT )
    {
      memcpy( msgbuf + MSG_TRUNK_AT - ELIPSYS, tmp, ELIPSYS );
    }
#endif

    // This this a G Code command
    // --------------------------
    if( memcmp( msgbuf, "CMD,", 4 ) == 0 )
    {
      unsigned long seq;
      long cmdCount;
      
      if( sscanf( msgbuf+4, "%lu,%lu", &seq, &cmdCount ) != 2 || 
          cmdCount <= 0 || 
          cmdCount > 100 )
      {
        // Format error.
        ESP_LOGE( TAG, "Message header error" );
        // Ignore
      }
      else
      {
        // For the purpose of monitoring the communication, send the number
        // of message in the queue BEFORE filling it. This can help measure
        // how close to an underrun contion the CNC is.
        inQueue = uxQueueMessagesWaiting( g_cmd_queue );
        
        // We got a sequence # of a packet we already received ( a repeat )
        if( seq == ( g_NextSeq - 1 ))
        {
          // Let's ACK it. The host missed the reponse and re-sent.
          sendACK( seq, cncStatus_Success, inQueue, &source );
          
          ESP_LOGW( TAG, "Retry of %lu - Queue:%d.", 
            seq,
            inQueue );
        }
        // Got a completely out of order packet. That is not recoverable.
        else if( seq != g_NextSeq )
        {
          sendACK( seq, cncStatus_SequenceError, inQueue, &source );
          
          ESP_LOGE( TAG, "Out of sequence. Exp:%lu Got:%lu Queue:%d.", 
            g_NextSeq,
            seq,
            inQueue );
        }
        else
        {
          char *pt = msgbuf+4;
          status = cncStatus_Success;
    
          for( int i=0; i<cmdCount && status == cncStatus_Success; i++ )
          {
            // Find the next command separator '|'
            while( *pt != '|' && pt < (msgbuf + nbytes)) pt++;
            
            // Move to the next char which should be the start of the command
            pt++;
            if( pt >= (msgbuf + nbytes))
            {
              // Message size invalid
              ESP_LOGE( TAG, "Message too small" );
              status = cncStatus_MessageIsTooShort;
            }
            // This is a movement command
            else if( *pt == '@' )
            {
              // Move to the start of the command parameters
              pt++;
              
              status = MovementCommand( seq, pt, &source, false );
            }
            else if( strncmp( pt, "ORIGIN", 6 ) == 0 )
            {
              OriginCommand( );
              
              CheckMachineIsIdle( seq, &source );
              
            }           
            else if( strncmp( pt, "RST", 3 ) == 0 )
            {
              ESP_LOGW( TAG, "Reset command received." );
              
              g_nackCounter = 0;
              
            }
            else if( strncmp( pt, "POS", 3 ) == 0 )
            {
              ESP_LOGI( TAG, "Position request" );
            }
            else if( strncmp( pt, "DBG", 3 ) == 0 )
            {
              ESP_LOGI( TAG, "Debug Info" );
            }
            else if( strncmp( pt, "CAL_Z", 5 ) == 0 )
            {
              ESP_LOGI( TAG, "Calibrate Z" );
              Calibrate_Z( );
            }
            else
            {
              ESP_LOGE( TAG, "Invalid command" );
              status = cncStatus_UnknownCommand;
            }
          } // for( )
          
          // Only increment the sequence counter if the message was processed
          // successfully.
          if( status == cncStatus_Success )
          {
            g_NextSeq++;
          }
          
          // Sending the ACK once. If the message is lost, the host will 
          // timeout and re-send the command message. Since the sequence #
          // will be in the past the device will just ACK it again and
          // transfer will resume.
          sendACK( seq, status, inQueue, &source );
          
        }
      }
    }
    else if( memcmp( msgbuf, "MAN,@", 4 ) == 0 )
    {
      inQueue = uxQueueMessagesWaiting( g_cmd_queue );
      status = MovementCommand( 0, msgbuf+5, &source, true );
      sendACK( 0, status, inQueue, &source );
    }
    else if( memcmp( msgbuf, "CAL_Z", 5 ) == 0 )
    {
      ESP_LOGI( TAG, "Calibrate Z" );
      Calibrate_Z( );
    } 
    else if( memcmp( msgbuf, "NOP", 3 ) == 0 )
    {
      // Do nothing. This is to help keep the respontiveness of the joystick
    }
    else
    {
      ESP_LOGW( TAG, "Invalid header. Message ignored." );
      // Ignore
    }
  }
  return;
}

void broadcastTask(void* arg)
{
  int broadcastEnable=1;
  u_int yes=1;
  
  ESP_LOGI( TAG, "Broadcast task started." );
  
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

  if( setsockopt(g_transmitSocket, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0 ) {
    ESP_LOGE( TAG, "Enable broadcast failed");
    return;
  }
   
  while (bRun) 
  {
    cmd_t cmd;
    
    // If there is nothing to process 
    if( xQueueReceive( g_cmd_queue, &cmd, 1000 / portTICK_PERIOD_MS))
    {
      MotorMove( cmd.dx, cmd.dy, cmd.dz, cmd.duration );
    }
    else
    {
      long x,y,z;
      unsigned long S,Q;
      unsigned long A0,A1,A2;
      
      x = y = z = 0;
      S = Q = 0;
      A0 = A1 = A2 = 0;
      g_Status |= STATUS_GOT_POSITION;
      
#if 0             
      GetAnalogCommand( &A0, &A1, &A2 );
      if( !GetPositionCommand( &x, &y, &z, &S, &Q ))
      {
        ESP_LOGE( TAG, "Failed to get current position" );
        g_Status &= ~STATUS_GOT_POSITION;
      }
      else
      {          
        if( Q == 0 && uxQueueMessagesWaiting( g_cmd_queue ) == 0 )
        {
          if( g_xPos != x || g_yPos != y || g_zPos != z )
          {
            ESP_LOGW( TAG, "Position forced from %ld,%ld,%ld to %ld,%ld,%ld",
              g_xPos, g_yPos, g_zPos,
              x,y,z );
              
            g_xPos = x;
            g_yPos = y;
            g_zPos = z;
          }
          g_Status |= STATUS_GOT_POSITION;
        }           
        else
        {
          ESP_LOGW( TAG, "Queues are not empty" );
        }          
      }      
#endif   

      MotorGetPosition( &g_xPos, &g_yPos, &g_zPos );
  
      if( g_broadcastAddr.sin_addr.s_addr != 0 )
      {
        char statusmsg[64];
        
        sprintf( statusmsg, "CNC,%lu,%ld,%ld,%ld,%lx,%lx,%ld,%ld,%ld",
          g_NextSeq,
          g_xPos,
          g_yPos,
          g_zPos,
          S,
          g_Status,
          A0,
          A1,
          A2 );
          
        if ( sendto(g_transmitSocket,statusmsg,strlen(statusmsg)+1,0,(struct sockaddr *) &g_broadcastAddr, sizeof(g_broadcastAddr)) < 0) 
        {
          ESP_LOGE( TAG, "Send failed" );
        }
      }
    }    
  }
  return;
}

int getNetworkStatus( )
{
	
	if( bGotIP )
		return 0;
	else 
		return -1;

}

void time_sync_notification_cb(struct timeval *tv)
{
	sntp_sync_status_t status;
	
	if(( status = sntp_get_sync_status( )) == SNTP_SYNC_STATUS_COMPLETED )
	{
		time_t now = 0;
		struct tm timeinfo = { 0 };
		char strftime_buf[80];
		
		time(&now);
		localtime_r(&now, &timeinfo);
		strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
		ESP_LOGI(TAG, "SNTP SYNC : The current date/time in San Francisco is: %s", strftime_buf );
	}
	else
	{
		ESP_LOGE(TAG, "SNTP_SYNC : Unexpected status : %d.", status );
	}
}

#ifdef USE_ETHERNET

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
		bGotIP = 0;
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
		bGotIP = 0;
        break;
    default:
        break;
    }
}
#endif

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
  ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
  const esp_netif_ip_info_t *ip_info = &event->ip_info;

  ESP_LOGI(TAG, "Ethernet Got IP Address");
  ESP_LOGI(TAG, "~~~~~~~~~~~");
  ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
  ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
  ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
  ESP_LOGI(TAG, "~~~~~~~~~~~");
		
	//strcpy( myIP, IP2STR(&ip_info->ip));
	inet_ntop(AF_INET, &ip_info->ip, myIP, sizeof(myIP));
  
  iBroadcast = ip_info->ip.addr | ~(ip_info->netmask.addr);
  
  iMyIP = ntohl( ip_info->ip.addr );
		
	bGotIP = 1;
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
          ESP_LOGE(TAG, "AP connect failed");
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        
        SignalIPConnected( );
        
        // Set local broadcast address
        g_broadcastAddr.sin_addr.s_addr = event->ip_info.ip.addr | ~(event->ip_info.netmask.addr);
        
        bGotIP = 1;
    }
}

void socketSetStateChangeCallback( stateChangeCallback fct )
{
	ESP_LOGI( TAG, "State change callback set to %lx", (unsigned long)fct );
	callback = fct;
  //xEventGroupSetBits( eventGroupHandle, EVENT_CALLBACK_SET );
}

int initSocketCom( void )
{	
	uint8_t mac[6];
  
	g_cmd_queue = xQueueCreate( CMD_QUEUE_SIZE, sizeof(cmd_t));
	if( g_cmd_queue == NULL )
	{
		ESP_LOGE( TAG, "Failed to create message queue." );
		return -1;
	}
  
  ResetStatus( );
  
  // Initislize broadcast address
  memset(&g_broadcastAddr,0,sizeof(g_broadcastAddr));
  g_broadcastAddr.sin_family=AF_INET;
  g_broadcastAddr.sin_port=htons(BROADCAST_PORT);
  
  
  // Initialize TCP/IP network interface (should be called only once in application)
  ESP_ERROR_CHECK(esp_netif_init( ));
  
  // Create default event loop that running in background
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  // E T H E R N E T
  // ---------------
#ifdef USE_ETHERNET
  ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_ETH));
  sprintf( myMAC, "%02X:%02X:%02X:%02X:%02X:%02X",
		mac[0],mac[1],mac[2],mac[3],mac[4],mac[5] );
   
  ESP_LOGI( TAG, "MAC: %s", myMAC );
	
  esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
  esp_netif_t *eth_netif = esp_netif_new(&cfg);
  // Set default handlers to process TCP/IP stuffs
  ESP_ERROR_CHECK(esp_eth_set_default_handlers(eth_netif));
  
  // Register user defined event handers
  ESP_ERROR_CHECK(esp_event_handler_register(
    ETH_EVENT, 
    ESP_EVENT_ANY_ID, 
    &eth_event_handler, 
    NULL));
    
  ESP_ERROR_CHECK(esp_event_handler_register(
    IP_EVENT, 
    IP_EVENT_ETH_GOT_IP, 
    &got_ip_event_handler, 
    NULL));

  // Using internal PHY
  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.phy_addr = CONFIG_EXAMPLE_ETH_PHY_ADDR;
  phy_config.reset_gpio_num = CONFIG_EXAMPLE_ETH_PHY_RST_GPIO;
  mac_config.smi_mdc_gpio_num = CONFIG_EXAMPLE_ETH_MDC_GPIO;
  mac_config.smi_mdio_gpio_num = CONFIG_EXAMPLE_ETH_MDIO_GPIO;
  esp_eth_mac_t *eth_mac = esp_eth_mac_new_esp32(&mac_config);

  // IP101 Phy
  esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);

  esp_eth_config_t config = ETH_DEFAULT_CONFIG(eth_mac, phy);
  esp_eth_handle_t eth_handle = NULL;
  ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));
  /* attach Ethernet driver to TCP/IP stack */
  ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
  /* start Ethernet driver state machine */
  ESP_ERROR_CHECK(esp_eth_start(eth_handle));
#endif

  // W I - F I 
  // ---------
  esp_netif_create_default_wifi_sta( );
  
  wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
  
  ESP_ERROR_CHECK(esp_event_handler_register(
    WIFI_EVENT,
    ESP_EVENT_ANY_ID,
    &event_handler,
    NULL));
  
  ESP_ERROR_CHECK(esp_event_handler_register(
    IP_EVENT,
    IP_EVENT_STA_GOT_IP,
    &event_handler,
    NULL));

    wifi_config_t wifi_config = {
      .sta = {
        .ssid = "aet home",
        .password = "makeaguess",
        .bssid_set = 1,
        .bssid = { 0x44,0x07,0x0b,0x03,0x08,0x00 }, // Garage AP
        
        /* Setting a password implies station will connect to all security modes including WEP/WPA.
         * However these modes are deprecated and not advisable to be used. Incase your Access point
         * doesn't support WPA2, these mode can be enabled by commenting below line */
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

  if( xTaskCreate( 
		broadcastTask, 
		"broadcast", 
		NETWORK_STACK_SIZE, 
		NULL, 
		tskIDLE_PRIORITY, 
		&broadcastTaskHandle ) != pdPASS )
	{
		ESP_LOGE( TAG, "Failed to create broadcast task" );
	}
  
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
  		
	return 0;
}

