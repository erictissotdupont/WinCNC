/*
 * sockect.c
 *   Listener and talker socket
*/
#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include "Main.h"
#include "Ws2tcpip.h"
#include "Mstcpip.h"
#include <iphlpapi.h>
#include <mmsystem.h>

#include "status.h"
#include "motor.h"
#include "gcode.h"
#include "CNC.h"

// This is how often the host will broadcast a request for info when
// no longer connected to the machine
#define UDP_BROADCAST_PERIOD_MS    1000

// This is the size of the buffer for receiving responses from the machine
#define IN_MSG_BUF_SIZE			    256

// Timeout waiting for ACK
#define COMMAND_TIMEOUT_MS		    500

#define IPSTRSIZE				     80

int bGotInfo = 0;
int bConnected = 0;

HANDLE g_hPositionMutex;
HANDLE g_hConnected;
HANDLE g_hDisconnected;
HANDLE g_hStop;
HANDLE g_hBufferFull;
HANDLE g_hBufferEmpty;

DWORD g_dwTimeLastMessageReceived;
HANDLE g_hAckReceived;
HANDLE g_hNackReceived;

SOCKET g_CNCSocket;
char g_szCNCIP[IPSTRSIZE];
struct sockaddr_in g_CncAddr;

unsigned long g_CNC_State = 0;
INT64 g_CNC_Debug;
unsigned int g_CNC_QueueSize;
unsigned int g_CNC_MsgInQueue = 0;
int g_CNC_QueueFree;

// Queue to track the movements sent to the machine but not executed yet
t3DPoint* g_CmdQueue;
unsigned int g_CmdInIndex;
unsigned int g_CmdQueueSize;

unsigned long g_TXcount = 0;
unsigned long g_RXcount = 0;
unsigned long g_RetryCount = 0;
unsigned long g_NakCount = 0;

// This is the buffer for accumulating outbound commands. Note this does not
// include the header. So the actual message will be a few bytes long.
#define OUT_MSG_BUF_SIZE		       1024
#define OUT_BUFFER_IDLE_TIMEOUT_MS      100
#define OUT_BUFFER_MUTEX_TIMEOUT_MS	   1000
int g_outCharCount;
int g_outCmdCount;
char g_outBuffer[OUT_MSG_BUF_SIZE];
HANDLE g_outBufferMutex = NULL;

// This is the message sequence counter. This is used to make sure
// all messages are received and processed only once. The machine and
// the host only increment when a message is received.
unsigned long g_msgSeq;

typedef enum {
	Endianness_Unknown = 0,
	BigEndian = 1,
	LittleEndian = 2,
} Endianness_t;

Endianness_t g_Endianness = Endianness_Unknown;

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

// Some forward declaration
tStatus postCommand(char* cmd);
int sendToCNC(char* msg, size_t cbMsg);


unsigned char crc8(unsigned char* pt, unsigned int nbytes, unsigned char crc)
{
	while (nbytes-- > 0)
	{
		crc = crc8_table[(crc ^ *pt++) & 0xff];
	}
	return crc;
}

// Returns the 8bit CRC of the 5 x 32bit long integrers in an array taking into
// account the endianness of the CNC remote processor.
//
unsigned char CNC_GetPositionCRC(long x, long y, long z, unsigned long d, unsigned long flags )
{
	long posForCRC[5];
	if (g_Endianness == LittleEndian )
	{
		posForCRC[0] = x;
		posForCRC[1] = y;
		posForCRC[2] = z;
		posForCRC[3] = d;
		posForCRC[4] = flags;
	}
	else if( g_Endianness == BigEndian )
	{
		posForCRC[0] = htonl(x);
		posForCRC[1] = htonl(y);
		posForCRC[2] = htonl(z);
		posForCRC[3] = htonl(d);
		posForCRC[4] = htonl(flags);
	}
	else
	{
		return 0;
	}
	return crc8((unsigned char*)posForCRC, sizeof(posForCRC), 0xFF);
}

// Returns the estimated postion of the machine at a given time. This position
// should ONLY be used for display. It uses the history of commands sent
// to the machine and the reported number of commands currently in the queue
// to estimate the position of the actual machine.
//
void CNC_GetDisplayPosition(t3DPoint* pPos)
{
	if (g_CNC_MsgInQueue == 0 || 
	  ((g_CNC_State & CNC_STATE_MANUAL_MODE ) == CNC_STATE_MANUAL_MODE ))
	{
		GetRealPosition(pPos);
	}
	else if( g_CmdQueue != NULL )
	{
		int oldestCmd = g_CmdInIndex - g_CNC_MsgInQueue;
		if (oldestCmd < 0) oldestCmd += g_CmdQueueSize;
		*pPos = g_CmdQueue[oldestCmd];
	}
	else
	{
		pPos->x = HUGE_VAL;
		pPos->y = HUGE_VAL;
		pPos->z = HUGE_VAL;
	}
}

// Called while a movement is being prepared from the motor position
// until it has been submitted to the machine. This is to prevent
// an idle position report to reset the host position after the CRC
// has been computed.
bool CNC_LockMachinePosition(bool bLock)
{
	if (bLock)
	{
		return (WaitForSingleObject(g_hPositionMutex, 0) == WAIT_OBJECT_0);
	}
	else
	{
		return ReleaseMutex(g_hPositionMutex);
	}
}

void CNC_GetNetworkStatusString(char* szBuffer, size_t cbBuffer)
{
	sprintf_s(szBuffer, cbBuffer, "%s - Tx:%lu - Rx:%lu - Retry:%lu - Nak:%lu - Lvl:%d %%",
		g_szCNCIP,
		g_TXcount,
		g_RXcount,
		g_RetryCount,
		g_NakCount,
		(g_CNC_QueueSize == 0) ? 0 : (100 * g_CNC_MsgInQueue) / g_CNC_QueueSize );
}

void CNC_GetStateString(char* szBuffer, size_t cbBuffer, unsigned long mask )
{
	*szBuffer = 0;
	unsigned long state = g_CNC_State & mask;

	// Error
	if (state & CNC_STATE_MOTOR_CRC_ERROR)     strcat_s(szBuffer, cbBuffer, "Motor CRC error" "\r\n");
	if (state & CNC_STATE_NETWORK_CRC_ERROR)   strcat_s(szBuffer, cbBuffer, "Network CRC error" "\r\n");
	if (state & CNC_STATE_HARD_ERROR)          strcat_s(szBuffer, cbBuffer, "Hardware error" "\r\n");
	if (state & CNC_STATE_LOGICAL_LIMIT_ERROR) strcat_s(szBuffer, cbBuffer, "Logical limit error" "\r\n");
	if (state & CNC_STATE_CALIBRATION_FAILED)  strcat_s(szBuffer, cbBuffer, "Calibration failed" "\r\n");
	if (state & CNC_STATE_COMMUNICATION_ERROR) strcat_s(szBuffer, cbBuffer, "Communication error" "\r\n");
	if (state & CNC_STATE_IDLE_TIMEOUT_ERROR)  strcat_s(szBuffer, cbBuffer, "Idle timeout error" "\r\n");
	
	// Warning
	if (state & CNC_STATE_HARD_LIMIT)          strcat_s(szBuffer, cbBuffer, "Limit Sensor" "\r\n");
	if (state & CNC_STATE_CAL_ORIGIN_ERROR)    strcat_s(szBuffer, cbBuffer, "Calibration position error" "\r\n");
	if (state & CNC_STATE_COMMAND_QUEUE_FULL)  strcat_s(szBuffer, cbBuffer, "Command queue is full" "\r\n");
	if (state & CNC_STATE_POS_SENSOR_XL)       strcat_s(szBuffer, cbBuffer, "Position sensor XL" "\r\n");
	if (state & CNC_STATE_POS_SENSOR_XR)       strcat_s(szBuffer, cbBuffer, "Position sensor XR" "\r\n");
	if (state & CNC_STATE_POS_SENSOR_ZL)       strcat_s(szBuffer, cbBuffer, "Position sensor ZL" "\r\n");
	if (state & CNC_STATE_POS_SENSOR_ZR)       strcat_s(szBuffer, cbBuffer, "Position sensor ZR" "\r\n");
	if (state & CNC_STATE_POS_SENSOR_Y)        strcat_s(szBuffer, cbBuffer, "Position sensor Y" "\r\n");

	// Information
	if (state & CNC_STATE_Z_CALIBRATED)        strcat_s(szBuffer, cbBuffer, "Z axis calibrated" "\r\n");
	if (state & CNC_STATE_Y_CALIBRATED)        strcat_s(szBuffer, cbBuffer, "Y axis calibrated" "\r\n");
	if (state & CNC_STATE_X_CALIBRATED)        strcat_s(szBuffer, cbBuffer, "X axis calibrated" "\r\n");
	if (state & CNC_STATE_CALIBRATING)         strcat_s(szBuffer, cbBuffer, "Calibrating..." "\r\n");
	if (state & CNC_STATE_MANUAL_MODE)         strcat_s(szBuffer, cbBuffer, "Manual mode" "\r\n");
	if (state & CNC_STATE_IDLE)                strcat_s(szBuffer, cbBuffer, "Idle..." "\r\n");
	//if (state & CNC_STATE_LITTLE_ENDIAN)       strcat_s(szBuffer, cbBuffer, "Little Endian" "\r\n");
	if (state & CNC_STATE_CONNECTED)
	{
		char szDebug[80];
		strcat_s(szBuffer, cbBuffer, "Connected" "\r\n");
		if (g_CNC_Debug || 1)
		{
			int i = 0;
			int n = sizeof(szDebug) - 1;
			i += sprintf_s(szDebug,   n-i, "\r\n0x%04llX", (g_CNC_Debug >> 48) & 0xFFFF);
			i += sprintf_s(szDebug+i, n-i, "-%04llX",      (g_CNC_Debug >> 32) & 0xFFFF);
			i += sprintf_s(szDebug+i, n-i, "-%04llX",      (g_CNC_Debug >> 16) & 0xFFFF);
			i += sprintf_s(szDebug+i, n-i, "-%04llX\r\n",  (g_CNC_Debug & 0xFFFF));
			strcat_s(szBuffer, cbBuffer, szDebug);
		}
	}
}

unsigned long CNC_GetState()
{
	return g_CNC_State;
}

void CNC_NotifiyMainWindow()
{
	PostMessage(GetMainWindow( ), WM_UPDATE_POSITION, 0, 0);
}

bool CheckDisconnection()
{
	// The machine will send the messages 3x slower when idle. The 10x factor allows for
	// 2 consecutive messages to be lost before the diconnected state to be declared.
	//
	if (g_dwTimeLastMessageReceived + (CNC_IDLE_POS_TIMEOUT_MS * 10) < timeGetTime())
	{
		if (bConnected)
		{
			g_CNC_State = 0;
			bGotInfo = 0;
			bConnected = false;
			ResetEvent(g_hConnected);
			SetEvent(g_hDisconnected);
			g_dwTimeLastMessageReceived = 0;
			CNC_NotifiyMainWindow();
		}
		return true;
	}
	return false;
}

void FlushOutBuffer()
{
	g_outCmdCount = 0;
	g_outCharCount = 0;
	ResetEvent(g_hBufferFull);
	SetEvent(g_hBufferEmpty);
}

int sendToCNC(char* msg, size_t cbMsg)
{
	return sendto(g_CNCSocket, msg, (int)cbMsg, 0, (SOCKADDR*)&g_CncAddr, sizeof(g_CncAddr));
}

tStatus sendAndWaitForAck(char* msg, size_t cbMsg)
{
	tStatus status = retInternalError;

	ResetEvent(g_hAckReceived);
	ResetEvent(g_hNackReceived);

	if (sendToCNC(msg,cbMsg) <= 0 )
	{
		int iResult = WSAGetLastError();
		status = retCncCommunicationError;
	}
	else
	{
		HANDLE hEvent[4];
		hEvent[0] = g_hStop;
		hEvent[1] = g_hAckReceived;
		hEvent[2] = g_hNackReceived;
		hEvent[3] = g_hDisconnected;

		switch (WaitForMultipleObjects(4, hEvent, FALSE, COMMAND_TIMEOUT_MS))
		{
		default:
			status = retInternalError;
			break;

		case WAIT_OBJECT_0: // Stop
			status = retStopRequested;
			break;

		case WAIT_TIMEOUT:
			CheckDisconnection();
			status = retCncStatusTimeout;
			break;

		case WAIT_OBJECT_0 + 1: // ACK
			g_TXcount++;
			status = retSuccess;
			break;

		case WAIT_OBJECT_0 + 2: // Nack
			if (g_CNC_State & CNC_STATE_ERROR_MASK)
			{
				status = retCncError;
			}
			else if (g_CNC_State & CNC_STATE_COMMAND_QUEUE_FULL)
			{
				// The CNC command pipe is full. Wait for a while before retrying
				Sleep(COMMAND_TIMEOUT_MS);
				status = retCncCBusy;
			}
			else
			{
				status = retUnknownErr;
			}
			break;

		case WAIT_OBJECT_0 + 3: // Disconnected
			status = retCncNotConnected;
			break;

		}
	}
	return status;
}

tStatus CNC_PostMovementCommand( long x, long y, long z, unsigned long d, unsigned long s )
{
	char str[80];
	tStatus status;

	sprintf_s(str, sizeof(str), "@" CNC_CMD_PARAMS, x, y, z, d, s);

	status = postCommand(str);

	if (status == retSuccess)
	{
		GetRealPosition(&g_CmdQueue[g_CmdInIndex]);
		g_CmdInIndex++;
		if (g_CmdInIndex >= g_CmdQueueSize) g_CmdInIndex = 0;
	}

	return status;
}

tStatus CNC_Calibrate(unsigned long axis_flags)
{
	char szCmd[64];
	sprintf_s(szCmd, sizeof(szCmd), CNC_CMD_CALIBRATE "|%X|", axis_flags);
	return postCommand(szCmd);
}

tStatus postCommand(char* cmd)
{
	HANDLE hEvent[2];
	tStatus status = retSuccess;
	
	if (cmd == NULL)
	{
		return retInvalidParam;
	}
	
	int cl = strlen(cmd);
	if (cl > OUT_MSG_BUF_SIZE)
	{
		return retInvalidParam;
	}

	// Wait for the output buffer to be available or the stop event
	hEvent[0] = g_hStop;
	hEvent[1] = g_outBufferMutex;

	switch (WaitForMultipleObjects(2, hEvent, FALSE, INFINITE))
	{
	default:
		status = retInternalError;
		break;

	case WAIT_OBJECT_0 :
		status = retStopRequested;
		break;

	case WAIT_OBJECT_0 + 1:
		// First, check if the outbout buffer is full
		if (cl + g_outCharCount + 2 >= sizeof(g_outBuffer))
		{
			// Signal the sender thread to flush the outbound buffer
			SetEvent(g_hBufferFull);
			ResetEvent(g_hBufferEmpty);

			// Release access to the buffer
			ReleaseMutex(g_outBufferMutex);

			hEvent[0] = g_hStop;
			hEvent[1] = g_hBufferEmpty;
			switch (WaitForMultipleObjects(2, hEvent, FALSE, INFINITE))
			{
			default:
				status = retInternalError;
				break;

			case WAIT_OBJECT_0 :
				status = retStopRequested;
				break;

			case WAIT_OBJECT_0 + 1 : // Buffer is now empty
				// Re-acquire the mutex. This should be instantaneous since
				// the buffer is now empty. Timeout should never occur.
				if (WaitForSingleObject(g_outBufferMutex, OUT_BUFFER_MUTEX_TIMEOUT_MS) != WAIT_OBJECT_0)
				{
					status = retBufferMutexTimeout;
				}
				break;
			}
		}

		if (status == retSuccess)
		{
			strcpy_s(&g_outBuffer[g_outCharCount], sizeof(g_outBuffer) - g_outCharCount, cmd);
			g_outCharCount += cl;
			g_outBuffer[g_outCharCount++] = '|';
			g_outBuffer[g_outCharCount] = '\0';
			g_outCmdCount++;

			ReleaseMutex(g_outBufferMutex);
		}
		break;
	}

	if (status == retStopRequested )
	{
		FlushOutBuffer();
	}

	return status;
}

void CNC_Reboot()
{
	char msg[64];
	int msgLen = sprintf_s(msg, sizeof(msg), CNC_HEADER CNC_CMD_HEADER "," CNC_CMD_HEADER_PARAMS "|%s|",
		g_msgSeq,
		1,
		CNC_CMD_REBOOT);

	sendAndWaitForAck(msg, msgLen);
}

void CNC_SendManualUpdate(int x, int y, int z)
{
	char msg[100];
	int len = sprintf_s(msg, sizeof(msg), CNC_HEADER CNC_MANUAL_HEADER "|" CNC_MANUAL_PARAMS "|", x, y, z);

	sendToCNC(msg, len);
}

void CNC_ForceStop( )
{
	SetEvent(g_hStop);

	if (WaitForSingleObject(g_hBufferEmpty, INFINITE) == WAIT_OBJECT_0)
	{
		char msg[64];
		int msgLen = sprintf_s(msg, sizeof(msg), CNC_HEADER CNC_CMD_HEADER "," CNC_CMD_HEADER_PARAMS "|%s|",
			g_msgSeq,
			1,
			CNC_CMD_FLUSH );

		sendAndWaitForAck(msg, msgLen);
	}
}

void CNC_Resume()
{
	ResetEvent(g_hStop);
}

DWORD senderThread(PVOID pParam)
{
	HANDLE hEvent[3];
	char msg[OUT_MSG_BUF_SIZE + 64]; // Extra space is of the header
	tStatus ret = retSuccess;

	hEvent[0] = g_hBufferFull;
	hEvent[1] = g_hConnected;

	while( 1 )
	{
		if (bConnected)
		{
			hEvent[0] = g_hBufferFull;
			WaitForMultipleObjects( 1, hEvent, FALSE, OUT_BUFFER_IDLE_TIMEOUT_MS );
		}
		else
		{
			hEvent[0] = g_hConnected;
			WaitForMultipleObjects( 1, hEvent, FALSE, INFINITE );
		}
		
		if( CheckDisconnection( ))
		{
			continue;
		}

		if (g_CNC_QueueFree < g_outCmdCount)
		{
			continue;
		}

		if (g_CNC_State & CNC_STATE_ERROR_MASK)
		{
			FlushOutBuffer();
			continue;
		}

		if (WaitForSingleObject(g_outBufferMutex, OUT_BUFFER_MUTEX_TIMEOUT_MS ) != WAIT_OBJECT_0)
		{
			// Deal with the buffer mutex timeout
		}
		else
		{
			char dbg[80];
			if (g_outCmdCount == 0)
			{
				// Nothing to send... idle
			}
			else
			{
				int cbHeader = sprintf_s(msg, sizeof(msg), CNC_HEADER CNC_CMD_HEADER "," CNC_CMD_HEADER_PARAMS "|", g_msgSeq, g_outCmdCount);
				memcpy(msg + cbHeader, g_outBuffer, g_outCharCount + 1);

				ret = sendAndWaitForAck(msg, cbHeader + g_outCharCount + 1);

				if (ret == retCncCBusy || 
					ret == retCncStatusTimeout ||
					ret == retCncCommunicationError )
				{
					// Keep trying
					g_RetryCount++;
					sprintf_s(dbg, sizeof(dbg), ">>> Resending msg %d.\r\n", g_msgSeq);
				}
				else
				{
					sprintf_s(dbg, sizeof(dbg), "Flushing msg %d\r\n", g_msgSeq);
					g_msgSeq++;
					FlushOutBuffer();
				}
				OutputDebugStringA(dbg);

			}
			ReleaseMutex(g_outBufferMutex);
		}
	}
	return 0;
}

void DecodeMessage(const char* msg, int cnt)
{
	bool bPos = false;
	bool bAck = false;
	bool bNak = false;

	g_dwTimeLastMessageReceived = timeGetTime();

	if (strncmp(msg, CNC_INFO_HEADER ",", CNC_INFO_HEADER_LEN) == 0)
	{
		int g_CNCversion;
		int g_rxBufferSize;
		float g_xRes, g_yRes, g_zRes;
		long g_xMin, g_xMax;
		long g_yMin, g_yMax;
		long g_zMin, g_zMax;

		if (sscanf_s(msg + CNC_INFO_HEADER_LEN + 1, CNC_INFO_PARAMS,
			&g_CNCversion,
			&g_rxBufferSize,
			&g_CNC_QueueSize,
			&g_xRes,
			&g_yRes,
			&g_zRes,
			&g_xMin,
			&g_yMin,
			&g_zMin,
			&g_xMax,
			&g_yMax,
			&g_zMax) != 12)
		{
			OutputDebugStringA(__FUNCTION__"::INFO format error.");
		}
		else
		{
			char szOut[100];
			int cbOut;

			InitMotorAxis(0, 1.0f / g_xRes ); // X
			InitMotorAxis(1, 1.0f / g_yRes ); // Y
			InitMotorAxis(2, 1.0f / g_zRes ); // Z

			bGotInfo = 1;

			cbOut = sprintf_s(szOut, sizeof(szOut), "%s", CNC_HEADER CNC_POS_HEADER);

			sendToCNC(szOut, cbOut);
		}
	}
	else if(( bPos = (strncmp(msg, CNC_POS_HEADER ",", CNC_POS_ACK_NAK_HEADER_LEN+1) == 0)) ||
		    ( bAck = (strncmp(msg, CNC_ACK_HEADER ",", CNC_POS_ACK_NAK_HEADER_LEN+1) == 0)) ||
		    ( bNak = (strncmp(msg, CNC_NAK_HEADER ",", CNC_POS_ACK_NAK_HEADER_LEN+1) == 0)))
	{
		unsigned long seq;
		long x,y,z;
		unsigned long state;
		unsigned int inQueue;
		INT64 debug;

		if (sscanf_s(msg + CNC_POS_ACK_NAK_HEADER_LEN + 1, CNC_POS_ACK_NAK_PARAMS,
			&seq,
			&x,
			&y,
			&z,
			&state,
			&inQueue,
			&debug) != 7)
		{
			OutputDebugStringA(__FUNCTION__"::Format error.");
		}
		else
		{
			char dbg[80] = { 0 };

			g_CNC_Debug = debug;
			g_CNC_MsgInQueue = inQueue;
			g_CNC_QueueFree = g_CNC_QueueSize - inQueue;
			g_CNC_State = state;
			g_RXcount++;
		
			if (bPos)
			{
				if (!bGotInfo)
				{
					char szMsg[80];
					int len = sprintf_s(szMsg, sizeof(szMsg), "%s", CNC_HEADER CNC_INFO_HEADER);
					sendToCNC(szMsg, len);
				}
				else
				{
					if (!bConnected)
					{
						bConnected = true;
						ResetEvent(g_hDisconnected);
						SetEvent(g_hConnected);
					}

					if (CNC_LockMachinePosition(true))
					{
						if ((g_CNC_State & (CNC_STATE_MANUAL_MODE | CNC_STATE_CONNECTED)) == (CNC_STATE_MANUAL_MODE | CNC_STATE_CONNECTED))
						{
							ResetMachinePosition(x, y, z);
						}
						else if ((inQueue == 0) && 
						        (g_outCmdCount == 0) &&
						       ((g_CNC_State & (CNC_STATE_IDLE | CNC_STATE_CONNECTED )) == (CNC_STATE_IDLE | CNC_STATE_CONNECTED)))
						{
							g_msgSeq = seq;
							g_Endianness = ((g_CNC_State & CNC_STATE_LITTLE_ENDIAN) != 0 ) ? LittleEndian : BigEndian;
							ResetMachinePosition(x, y, z);

							// Add one slot just in case
							if (g_CNC_QueueSize + 1 != g_CmdQueueSize)
							{
								if (g_CmdQueue) free(g_CmdQueue);
								g_CmdQueue = (t3DPoint*)malloc(sizeof(t3DPoint) * g_CNC_QueueSize);
								g_CmdQueueSize = g_CNC_QueueSize + 1;
							}
						}
						CNC_LockMachinePosition(false);
					}
				}
				sprintf_s(dbg, sizeof(dbg), "Got POS. In queue %d. State:%X\r\n", inQueue, state );
			}
			else if (bAck)
			{
				if (!bConnected)
				{
					// Got an ACK while we're not connected. Ignore?
					sprintf_s(dbg, sizeof(dbg), "Got ACK while not connected for command %d\r\n", seq);
				}
				else if (seq != g_msgSeq)
				{
					sprintf_s(dbg, sizeof(dbg),
						__FUNCTION__"::Out of sequence ACK. Got %lu, expected %lu.\r\n",
						seq, g_msgSeq);
				}
				else if (seq == g_msgSeq)
				{
					SetEvent(g_hAckReceived);
					sprintf_s(dbg, sizeof(dbg), "Got ACK for command %d\r\n", seq);
				}
			}
			else if (bNak)
			{
				if (seq == (g_msgSeq + 1))
				{
					SetEvent(g_hAckReceived);
					sprintf_s(dbg, sizeof(dbg), "Got NAK for previous seq (%d). Treat it like an ACK.\r\n", seq );
				}
				else if( seq == g_msgSeq )
				{
					g_NakCount++;
					SetEvent(g_hNackReceived);
					sprintf_s(dbg, sizeof(dbg), "Got NAK for command %d\r\n", seq);
				}
				else
				{
					sprintf_s(dbg, sizeof(dbg), "Got unexpected NAK: %d local Seq: %d\r\n", seq, g_msgSeq );
				}
			}
			OutputDebugStringA(dbg);
			CNC_NotifiyMainWindow();
		}
	}
}

DWORD __stdcall listenerThread(PVOID pParam)
{
	int cnt;
	struct sockaddr_in listenAddr;
	char msg[IN_MSG_BUF_SIZE];
	char hostName[256];
	struct hostent* host_entry;

	// Retrieve hostname
	if (gethostname(hostName, sizeof(hostName)) < 0)
	{
		//IOBoardSetLastError(IOBoard_UnableToGetHostName);
		return 0;
	}

	// Retrieve host IP addresses
	host_entry = gethostbyname(hostName);
	if (host_entry == NULL)
	{
		//IOBoardSetLastError(IOBoard_UnableToGetHostAddress);
		return 0;
	}

	memset(&listenAddr, 0x00, sizeof(listenAddr));
	listenAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	listenAddr.sin_family = AF_INET;
	listenAddr.sin_port = htons(CNC_UDP_PORT);

	if (bind(g_CNCSocket, (SOCKADDR*)&listenAddr, sizeof(listenAddr)) != 0)
	{
		//IOBoardSetLastError(IOBoard_BindingFailed, WSAGetLastError());
		return 0;
	}

	memset(&g_CncAddr, 0x00, sizeof(g_CncAddr));

	while (1)
	{
		struct sockaddr_in sourceAddr;
		int addrSize = sizeof(sourceAddr);

		if ((cnt = recvfrom(g_CNCSocket, msg, sizeof(msg) - 1, 0, (SOCKADDR*)&sourceAddr, &addrSize)) <= 0)
		{
			//IOBoardSetLastError(IOBoard_ReceivingFailed, WSAGetLastError());
			return 0;
		}

		// Zero terminate the buffer
		msg[cnt] = 0;

		// Filter out messages sent by outselves
		bool bOwnMessage = false;
		struct in_addr** addr_list = (struct in_addr**)host_entry->h_addr_list;
		for (int i = 0; addr_list[i] != NULL; i++)
		{
			if (memcmp(&sourceAddr.sin_addr, addr_list[i], sizeof(sourceAddr.sin_addr)) == 0)
			{
				bOwnMessage = true;
				break;
			}
		}

		if (bOwnMessage) continue;

		// Check if the message starts with the expected header. Ignore everything else
		if (memcmp(msg, CNC_HEADER, CNC_HEADER_LEN) == 0)
		{
			RtlIpv4AddressToStringA(&g_CncAddr.sin_addr, g_szCNCIP);
			
			g_CncAddr.sin_addr = sourceAddr.sin_addr;
			g_CncAddr.sin_family = AF_INET;
			g_CncAddr.sin_port = htons(CNC_UDP_PORT);

			DecodeMessage(msg + CNC_HEADER_LEN, cnt - CNC_HEADER_LEN );
		}
	}
}


#define MAX_BROADCAST_ADDR    30

DWORD __stdcall broadcasterThread(PVOID pParam)
{
	struct sockaddr_in Addr;
	int idleCount = 0;
	DWORD BroadcastAddr[MAX_BROADCAST_ADDR];
	int nBroadcastAddr = 0;
	ULONG lRet;
	ULONG cbAddr = 0;
	IP_ADAPTER_ADDRESSES* pAddresses = NULL;

	HANDLE g_hForceProbingEvent = CreateEvent(NULL, FALSE, TRUE, NULL);

	// First, get the size of the buffer needed to store the network adapters info (gets put in cbAddr)
	// Second, allocate memory for this size
	// Third, call the function again to receive the network adapter info
	// If any of those fail, use the generic broadcast address but if there is more than one it's likely
	// that we won't find our board.
	//
	if (((lRet = GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_DNS_SERVER, NULL, NULL, &cbAddr)) != ERROR_BUFFER_OVERFLOW) ||
		((pAddresses = (IP_ADAPTER_ADDRESSES*)malloc(cbAddr)) == NULL) ||
		((lRet = GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_DNS_SERVER, NULL, pAddresses, &cbAddr)) != ERROR_SUCCESS))
	{
		// This should probably surface an error as it will only when if only one network adapter is active
		BroadcastAddr[0] = 0xFFFFFFFF;
		nBroadcastAddr = 1;
		if (pAddresses != NULL) free(pAddresses);
	}
	else
	{
		// Point to the first adapter info
		IP_ADAPTER_ADDRESSES* pCurAddr = pAddresses;
		while (pCurAddr)
		{
			// If the adapter type is Ethernet or Wifi and it's up and running
			if ((pCurAddr->IfType == IF_TYPE_ETHERNET_CSMACD ||
				pCurAddr->IfType == IF_TYPE_IEEE80211) &&
				pCurAddr->OperStatus == IfOperStatusUp)
			{
				ULONG mask;
				sockaddr_in* pAddr = (sockaddr_in*)pCurAddr->FirstUnicastAddress->Address.lpSockaddr;
				ULONG addr = pAddr->sin_addr.S_un.S_addr;

				// Get the subnet mask
				ConvertLengthToIpv4Mask(pCurAddr->FirstUnicastAddress->OnLinkPrefixLength, &mask);

				// Generate the broadcast address from it
				BroadcastAddr[nBroadcastAddr++] = addr & mask | ~mask;
			}
			// Traverse the link list
			pCurAddr = pCurAddr->Next;
		}
		free(pAddresses);
	}

	g_CNCSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (g_CNCSocket == INVALID_SOCKET)
	{
		//IOBoardSetLastError(IOBoard_CreateSocketFailed, WSAGetLastError());
		return 0;
	}

	BOOL bFlag = 1;
	if (setsockopt(g_CNCSocket, SOL_SOCKET, SO_BROADCAST, (const char*)&bFlag, sizeof(bFlag)) < 0)
	{
		//IOBoardSetLastError(IOBOard_SetSocketOptFailed, WSAGetLastError());
		return 0;
	}

	if (setsockopt(g_CNCSocket, SOL_SOCKET, SO_REUSEADDR, (const char*)&bFlag, sizeof(bFlag)) < 0) {
		//IOBoardSetLastError(IOBOard_SetSocketOptFailed, WSAGetLastError());
		return 0;
	}

	memset(&Addr, 0x00, sizeof(Addr));
	Addr.sin_family = AF_INET;
	Addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
	Addr.sin_port = htons(CNC_UDP_PORT);

	HANDLE g_hListenerThread = CreateThread(NULL, 0, listenerThread, NULL, 0, NULL);

	// Wait for the listener thread to listen. If an error occurs
	// the thread will stop and the event be signalled
	if (g_hListenerThread == NULL || WaitForSingleObject(g_hListenerThread, 100 ) != WAIT_TIMEOUT)
	{
		// An error should have been set by the listener thread
		return 0;
	}

	while (1)
	{
		char msg[80];

		// Look for new IO boards every 1 seconds. Note that event is created
		// as already in the signaled state so the first time the loop runs, 
		// this does not wait 
		WaitForSingleObject(g_hForceProbingEvent, UDP_BROADCAST_PERIOD_MS );

		if (bConnected) continue;

		for (int i = 0; i < nBroadcastAddr; i++)
		{
			Addr.sin_addr.s_addr = BroadcastAddr[i];

			sprintf_s(msg, sizeof(msg), CNC_HEADER CNC_INFO_HEADER);

			if (sendto(g_CNCSocket, msg, (int)strlen(msg), 0, (struct sockaddr*)&Addr, sizeof(Addr)) < 0)
			{
				// IOBoardSetLastError(IOBoard_SendToFailed, WSAGetLastError());
				continue;
			}
		}
	}

	closesocket(g_CNCSocket);
	return 0;
}


int CNC_InitNetworkCom( )
{
  DWORD threadId;
  int iResult;
  WSADATA wsaData;

  memset(&wsaData, 0x00, sizeof(wsaData));
  if ((iResult = WSAStartup(0x0202, &wsaData)) != NO_ERROR) 
  {
	  return 1;
  }  

  g_outCmdCount = 0;
  g_outCharCount = 0;
  g_outBufferMutex = CreateMutex(NULL, FALSE, NULL);

  g_hConnected = CreateEvent(NULL, TRUE, FALSE, NULL);
  g_hDisconnected  = CreateEvent(NULL, TRUE, FALSE, NULL);
  g_hStop = CreateEvent(NULL, TRUE, FALSE, NULL);
  
  g_hBufferFull    = CreateEvent(NULL, FALSE, FALSE, NULL);
  g_hBufferEmpty   = CreateEvent(NULL, FALSE, FALSE, NULL);
  g_hAckReceived   = CreateEvent(NULL, FALSE, FALSE, NULL);
  g_hNackReceived  = CreateEvent(NULL, FALSE, FALSE, NULL);

  g_hPositionMutex = CreateMutex(NULL, FALSE, NULL);

  CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)broadcasterThread, NULL, 0, &threadId);
  CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)senderThread, NULL, 0, &threadId);

  CNC_NotifiyMainWindow( );
  return 0;
}

