/*
 * Network.c
 *   Listener and talker socket
*/
#include "driver/gpio.h"
#include "Cnc.h"

#include "UDP.h"
#include "Events.h"
#include "Motor.h"
#include "Wifi.h"
#include "Limits.h"

#define CMD_QUEUE_SIZE                ( 256 )
#define NACK_INTERVAL_MS              ( 100L )

unsigned long g_NextSeq = 0;
long g_NetworkPosition[5];
bool g_bReboot = false;
unsigned long g_nackCounter = 0;
QueueHandle_t g_cmd_queue = NULL;

static const char* TAG = "UDP";

// ----------------------------------------------------------------------------

uint8_t UDP_GetCRCAndUpdatePosition( cmd_t *pCmd )
{
  // Update the position from the command received so that
  // we can update calculate the CRC of the position the machine
  // should be after this command is executed
  g_NetworkPosition[0] += pCmd->dx;
  g_NetworkPosition[1] += pCmd->dy;
  g_NetworkPosition[2] += pCmd->dz;
  g_NetworkPosition[3] = pCmd->duration;
  g_NetworkPosition[4] = pCmd->flags & ~CMD_FLAGS_CRC_MASK;
  
  return crc8((uint8_t*)g_NetworkPosition, sizeof( g_NetworkPosition ), 0xFF );
}

bool UDP_MovementCommand( unsigned long seq, char* pt, bool bIgnoreCRC )
{
  cmd_t cmd;
   
  if( sscanf( pt, CNC_CMD_PARAMS,
    &cmd.dx,
    &cmd.dy,
    &cmd.dz,
    &cmd.duration,
    &cmd.flags ) != 5 )
  {
      ESP_LOGE( TAG, "Message failed to decode" );
      Events_SetState( CNC_STATE_COMMUNICATION_ERROR );
  }
  else 
  {
    uint8_t remotePosCRC = cmd.flags & CMD_FLAGS_CRC_MASK;
    uint8_t localPosCRC = UDP_GetCRCAndUpdatePosition( &cmd );
    
    if( localPosCRC != remotePosCRC && !bIgnoreCRC )
    {
      ESP_LOGE( TAG, "Position CRC mismatch. Got %x, expected %x. (%ld,%ld,%ld)", 
        remotePosCRC, 
        localPosCRC, 
        g_NetworkPosition[0],
        g_NetworkPosition[1],
        g_NetworkPosition[2] );
        
      Events_SetState( CNC_STATE_NETWORK_CRC_ERROR );
    }
    else if(( Events_GetState( ) & CNC_STATE_ERROR_MASK ) != 0 )
    {
      ESP_LOGE( TAG, "Machine in error state" );
    }
    else 
    {
      if( cmd.duration > 0 )
      {       
        Motor_InitSlowStart( &cmd );
      }

      if( xQueueSend( g_cmd_queue, 
                      &cmd, 
                     ( NACK_INTERVAL_MS / portTICK_PERIOD_MS )) != pdPASS )
      {
        Events_SetState( CNC_STATE_COMMAND_QUEUE_FULL );
      }
      else
      {
        Events_ClearState( CNC_STATE_COMMAND_QUEUE_FULL );
        return true;
      }
    }
  }
  return false;
}

bool UDP_CalibrateCommand( )
{
  cmd_t cmd = { 0 };
  
  cmd.flags = CMD_FLAG_CALIBRATION;
  if( xQueueSend( g_cmd_queue, 
                  &cmd, 
                  1000 / portTICK_PERIOD_MS ) != pdPASS )
  {
    return false;
  }
  cmd.flags = CMD_CALIBRATION_COMPLETE;
  if( xQueueSend( g_cmd_queue, 
                  &cmd, 
                  1000 / portTICK_PERIOD_MS ) != pdPASS )
  {
    return false;
  }
  return true;
}

int UDP_Respond( const char* rsp, unsigned long seq, unsigned int inQueue, char* outBuf )
{
  return sprintf( outBuf, "%s," CNC_POS_ACK_NAK_PARAMS,
    rsp,
    seq,
    g_NetworkPosition[0],
    g_NetworkPosition[1],
    g_NetworkPosition[2],
    Events_GetState( ),
    inQueue,
    Events_GetDebug( ));
}

int UDP_ParseMessage( char* msgbuf, int nbytes, char* outBuf )
{
  int ret = 0;
  unsigned int inQueue;
  
  if( memcmp( msgbuf, CNC_HEADER, CNC_HEADER_LEN ) != 0 )
  {
    // Ignore messages which don't start with the expected header
    return 0;
  }
  
  // Move the pointer past the header for parsing
  msgbuf += CNC_HEADER_LEN;
  nbytes -= CNC_HEADER_LEN;
  
  // Pre-populate the output buffer with the header. This gets added at
  // the end of this function if the parser needs to return something.
  strcpy( outBuf, CNC_HEADER );
  outBuf += CNC_HEADER_LEN;
      
  inQueue = uxQueueMessagesWaiting( g_cmd_queue );

  //   C O M M A N D S
  // -------------------
  if( memcmp( msgbuf, CNC_CMD_HEADER ",", CNC_CMD_HEADER_LEN + 1 ) == 0 )
  {
    unsigned long seq;
    unsigned long cmdCount;
    bool bStatus = false;
    
    if(( sscanf( msgbuf + CNC_CMD_HEADER_LEN + 1, CNC_CMD_HEADER_PARAMS, &seq, &cmdCount ) != 2 ) || 
            ( cmdCount > CMD_QUEUE_SIZE ))
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
      Events_SetState( CNC_STATE_COMMAND_QUEUE_FULL );
    }
    else
    {
      char *pt = msgbuf+4;
      bStatus = true;
      
      Events_ClearState( CNC_STATE_COMMAND_QUEUE_FULL );
      
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
          Events_SetState( CNC_STATE_COMMUNICATION_ERROR );
          bStatus = false;
        }
        // This is a movement command
        else if( *pt == '@' )
        {
          // Move to the start of the command parameters
          pt++;
          bStatus = UDP_MovementCommand( seq, pt, false );
          if( !bStatus )
          {
            ESP_LOGW( TAG, "Command %d failed.", i );
          }
        }
        else if( strncmp( pt, "ORIGIN", 6 ) == 0 )
        {
          // TODO : Reset the position to origin
        }           
        else if( strncmp( pt, CNC_CMD_REBOOT, CNC_CMD_REBOOT_LEN ) == 0 )
        {
          ESP_LOGW( TAG, "Reboot command received." );
          g_bReboot = true;
        }
        else if( strncmp( pt, CNC_CMD_CALIBRATE, CNC_CMD_CALIBRATE_LEN ) == 0 )
        {
          if(( bStatus = UDP_CalibrateCommand( )) == false )
          {
            ESP_LOGE( TAG, "Initiating calibration failed" );
            Events_SetState( CNC_STATE_CALIBRATION_FAILED );
          }
        }
        else if( strncmp( pt, CNC_CMD_FLUSH, CNC_CMD_FLUSH_LEN ) == 0 )
        {
          xQueueReset( g_cmd_queue );
          if( Events_WaitForMotorIdle( MOTOR_IDLE_TIMEOUT_MS ))
          {
            Motor_GetPosition( &g_NetworkPosition[0], &g_NetworkPosition[1], &g_NetworkPosition[2] );
          }
          else
          {
            ESP_LOGE( TAG, "Timeout waiting motor idle" );
            Events_SetState(CNC_STATE_IDLE_TIMEOUT_ERROR);
          }
        }
        else
        {
          ESP_LOGE( TAG, "Invalid command" );
        }
      } // for( )        
    }
    
    if( !bStatus )
    {
      ret = UDP_Respond( CNC_NAK_HEADER, g_NextSeq, inQueue, outBuf ); 
    }
    else
    {
      ret = UDP_Respond( CNC_ACK_HEADER, seq, inQueue + cmdCount, outBuf );
      g_NextSeq++;
      Motor_MoveIfIdle( );
    }
  }
  else if( memcmp( msgbuf, CNC_MANUAL_HEADER, CNC_MANUAL_HEADER_LEN ) == 0 )
  {
    int x, y, z;
    if( msgbuf[CNC_MANUAL_HEADER_LEN] == '|' &&
        sscanf( &msgbuf[CNC_MANUAL_HEADER_LEN+1], CNC_MANUAL_PARAMS, &x, &y, &z ) == 3 )
    {
      //ESP_LOGI( TAG, "Manual %d,%d,%d", x, y, z );
      Motor_ManualMove( x, y, z );
    }
  }
  else if( memcmp( msgbuf, CNC_INFO_HEADER, CNC_INFO_HEADER_LEN ) == 0 )
  {
    ret = sprintf( outBuf, CNC_INFO_HEADER "," CNC_INFO_PARAMS,
      CNC_PROTOCOL_VERSION,
      RX_BUFFER_SIZE,
      CMD_QUEUE_SIZE,
      X_AXIS_RES,
      Y_AXIS_RES,
      Z_AXIS_RES,
      X_AXIS_MIN,
      Y_AXIS_MIN,
      Z_AXIS_MIN,
      X_AXIS_MAX,
      Y_AXIS_MAX,
      Z_AXIS_MAX );
  }
  else if( memcmp( msgbuf, CNC_POS_HEADER, 3 ) == 0 )
  {
    ret = UDP_Respond( CNC_POS_HEADER, g_NextSeq, inQueue, outBuf );
  }
  else
  {
    //ESP_LOGW( TAG, "Message ignored." );
  }
  if (ret > 0)
  {
    ret += CNC_HEADER_LEN;
  }
  return ret;
}

int UDP_GetIdleStatus( char* outBuf )
{
  int inQueue = uxQueueMessagesWaiting( g_cmd_queue );
  return UDP_Respond( CNC_HEADER CNC_POS_HEADER, g_NextSeq, inQueue, outBuf );
}

void UDP_IdleTask( )
{
  if( g_bReboot )
  {
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    abort( );
  }
}

void UDP_ResetPosition( )
{
  g_NetworkPosition[0] = 0;
  g_NetworkPosition[1] = 0;
  g_NetworkPosition[2] = 0;
}

int UDP_Init( )
{
  g_cmd_queue = xQueueCreate( CMD_QUEUE_SIZE, sizeof(cmd_t));
	if( g_cmd_queue == NULL )
	{
		ESP_LOGE( TAG, "Failed to create message queue." );
		return -1;
	}
  
  return 0;
}

// ----------------------------------------------------------------------------

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

inline unsigned char crc8( unsigned char* pt, unsigned int nbytes, unsigned char crc )
{
	while( nbytes-- > 0 )
	{
		crc = crc8_table[(crc ^ *pt++) & 0xff];
	}
	return crc;
}