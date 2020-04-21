
#include "CNC.h"
#include "status.h"
#include "geometry.h"
#include "motor.h"
#include "keyboard.h"
#include "socket.h"

#define MAX_DURATION_IN_PIPE	  1000 // 1 second
#define TIMEPIPESIZE			  256
#define POLL_RATE				  30 // ms

#define COMMAND_RESET_ORIGIN   "O\n"
#define COMMAND_CLEAR_ERROR    "C\n"
#define COMMAND_GET_POSITION   "P\n"
#define COMMAND_GET_DEBUG	   "D\n"


HANDLE exportFile = NULL;
tAxis XMotor,YMotor,ZMotor;
tAxis* pMotor[] = {&XMotor,&YMotor,&ZMotor};
tSpindle Spindle;

typedef struct
{
	long x;
	long y;
	long z;
	unsigned long duration;
} tCommandInPipe;

int timePipeInIdx, timePipeOutIdx;
tCommandInPipe inPipe[TIMEPIPESIZE];
unsigned long timeInPipe;
long xInPipe;
long yInPipe;
long zInPipe;

tStatus(*g_pSimulation)(t3DPoint, long, long, long, long, long) = NULL;

int getCountOfCommandsInPipe()
{
	int c = timePipeInIdx - timePipeOutIdx;
	if (c < 0) c = c + TIMEPIPESIZE;
	return c;
}

unsigned long getDurationOfCommandsInPipe()
{
	return timeInPipe;
}


void getDistanceInPipe(long* x, long* y, long* z)
{
	*x = xInPipe;
	*y = yInPipe;
	*z = zInPipe;
}

bool isPipeAvailable(int cmdLen)
{
	if (getDurationOfCommandsInPipe() > MAX_DURATION_IN_PIPE) return false;
	
	// Add function to check if the out buffer is full

	return true;
}


void addCommandToPipe( long x, long y, long z, unsigned long d )
{
	tCommandInPipe *pt = &inPipe[timePipeInIdx++];
	if (timePipeInIdx >= TIMEPIPESIZE) timePipeInIdx = 0;

	pt->duration = d;
	pt->x = x;
	pt->y = y;
	pt->z = z;

	timeInPipe += d;
	xInPipe += x;
	yInPipe += y;
	zInPipe += z;
}

void removeCommandFromPipe( )
{
	tCommandInPipe *pt = &inPipe[timePipeOutIdx++];
	if (timePipeOutIdx >= TIMEPIPESIZE) timePipeOutIdx = 0;
	timeInPipe -= pt->duration;
	xInPipe -= pt->x;
	yInPipe -= pt->y;
	zInPipe -= pt->z;
	if (pt->x || pt->y || pt->z)
	{
		printf("move\r\n");
	}
}

void setExportFile( HANDLE file )
{
	exportFile = file;
}

void setSimulationMode(tStatus(*callback)(t3DPoint, long, long, long, long, long))
{
	g_pSimulation = callback;
}

void resetCompensation( )
{
  XMotor.cutComp = 0;
  YMotor.cutComp = 0;
  ZMotor.cutComp = 0;
}

void getCompensation( double* x, double* y, double* z )
{
  if( x ) *x = XMotor.cutComp;
  if( y ) *y = YMotor.cutComp;
  if( z ) *z = ZMotor.cutComp;
}

void stepToPos(long x, long y, long z, t3DPoint* P)
{
	P->x = x * XMotor.scale;
	P->y = y * YMotor.scale;
	P->z = z * ZMotor.scale;
}

void getCurPos( t3DPoint* P )
{
	stepToPos(XMotor.step, YMotor.step, ZMotor.step, P);
}

void getRawStepPos( int* x, int* y, int* z )
{
  if( x ) *x = XMotor.step;
  if( y ) *y = YMotor.step;
  if( z ) *z = ZMotor.step;
}

void addCompensation( double x, double y, double z )
{
  XMotor.cutComp += x;
  YMotor.cutComp += y;
  ZMotor.cutComp += z;
}

void initAxis( int a, double scale )
{
  tAxis* pA = pMotor[a];
  pA->step = 0;
  pA->scale = scale;
  pA->cutComp = 0.0;
}

void resetMotorPosition( long x, long y, long z )
{
  XMotor.step = x;
  YMotor.step = y;
  ZMotor.step = z;
}

double getLargestStep( )
{
  return maxOf3( XMotor.scale, YMotor.scale, ZMotor.scale );
}

double getSmalestStep( )
{
  return minOf3( XMotor.scale, YMotor.scale, ZMotor.scale );
}

double getMaxDistanceError( )
{
  t3DPoint oneStep;
  oneStep.x = XMotor.scale;
  oneStep.y = YMotor.scale;
  oneStep.z = ZMotor.scale;
  return vector3DLength(oneStep);
}

long calculateMove( tAxis* A, double target )
{
  double delta = target - A->step * A->scale;
  long step = (long)(delta / A->scale);
  A->step += step;
  return step;
}

int setSpindleState( int state )
{
  if( Spindle.nextState != state )
  {
    Spindle.nextState = state;
    return 1;
  }
  return 0;
}

long getSpindleState( )
{
  if( Spindle.currentState == Spindle.nextState ) return -1;
  Spindle.currentState = Spindle.nextState;
  return( Spindle.nextState == 3 );
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

tStatus doMove( void(*posAtStep)(t3DPoint*,int,int,void*), int stepCount, double duration, void* pArg )
{
  int i;
  long x,y,z,d,s;
  t3DPoint Ideal;
  char str[ 100 ];
  char tmp[ 30 ];
  tStatus status = retNoOutputFound;
 
  if (stepCount == 0)
  {
	  return retUnknownErr;
  }

  // Split the duration of the whole move for each step
  duration = duration / stepCount;
  // Convert that in uS for the CNC
  d = (long)(duration * 1000);
 
  for( i=1; i<=stepCount; i++ )
  {
    // Get the position we should be at for step i of stepCount
    posAtStep( &Ideal, i, stepCount, pArg );

    x = calculateMove( &XMotor, Ideal.x );
    y = calculateMove( &YMotor, Ideal.y );
    z = calculateMove( &ZMotor, Ideal.z );
    s = getSpindleState( );

	if (g_pSimulation)
	{
		status = g_pSimulation(Ideal, x, y, z, d, s);
	}
	else
	{
		strcpy_s(str, sizeof(str), "@");
		if (x) { sprintf_s(tmp, sizeof(tmp), "X%ld", x); strcat_s(str, sizeof(str), tmp); }
		if (y) { sprintf_s(tmp, sizeof(tmp), "Y%ld", y); strcat_s(str, sizeof(str), tmp); }
		if (z) { sprintf_s(tmp, sizeof(tmp), "Z%ld", z); strcat_s(str, sizeof(str), tmp); }
		if (d) { sprintf_s(tmp, sizeof(tmp), "D%ld", d); strcat_s(str, sizeof(str), tmp); }
		if (s >= 0) { sprintf_s(tmp, sizeof(tmp), "S%ld", s); strcat_s(str, sizeof(str), tmp); }

		// Not an empty command
		if (strlen(str) > 1)
		{
			// Add a 8 bit CRC on the parameters of the command. This protects 
			// everything after the '@' until the 'C'.
			strcat_s(str, sizeof(str), "C");
			sprintf_s(tmp, sizeof(tmp), "%d", crc8((unsigned char*)(str + 1), strlen(str) - 1, 0xFF));
			strcat_s(str, sizeof(str), tmp);

			// End the command with a line return
			strcat_s(str, sizeof(str), "\n");

			// If the duration is more than 10sec there is a problem
			if (duration > 5000000.0)
			{
				return retUnknownErr;
			}

			// Push a preset # of seconds worth of commands in the pipe
			// Make sure the # of commands tracked doesn't overflow the buffer
			int timeout = 10000 / POLL_RATE;
			unsigned long pipeDuration;
			unsigned long pipeCount;
			while (((((pipeDuration = getDurationOfCommandsInPipe( )) > MAX_DURATION_IN_PIPE) ||
				     ((pipeCount = getCountOfCommandsInPipe( )) >= TIMEPIPESIZE))) && timeout >= 0 )
			{
				Sleep(POLL_RATE);
				timeout--;
			}
			if (timeout < 0)
			{
				return retBufferBusyTimeout;
			}

			status = sendCommand( str );

			if (status == retSuccess)
			{
				addCommandToPipe(x, y, z, (unsigned long)duration);
			}
		}
		else
		{
			// Empty command. Shouldn't happen. Just skip over.
			status = retSuccess;
		}
	}

    if( exportFile )
    {
	  // if( fwrite( str, 1, strlen( str ), exportFile ) > 0 && status == retCncNotConnected )
	  if (WriteFile(exportFile, str, strlen(str), NULL, NULL) && status == retCncNotConnected)
      {
        status = retSuccess;
      }
    }
  }

  CheckStatus(false);

  return status;
}

tStatus ResetCNCPosition( )
{
	tStatus ret = retSuccess;
	if (g_pSimulation == NULL)
	{
		ret = sendCommand( COMMAND_RESET_ORIGIN );
		if (ret == retSuccess) addCommandToPipe(0, 0, 0, 10);
	}
	return ret;
}

tStatus ClearCNCError()
{
	tStatus ret = retSuccess;
	if (g_pSimulation == NULL)
	{
		ret = sendCommand( COMMAND_CLEAR_ERROR );
		if (ret == retSuccess) addCommandToPipe(0, 0, 0, 10);
	}
	return ret;
}

tStatus CheckStatus( BOOL bWait )
{
	static DWORD count = 0;
	static DWORD lastCheck = 0;
	tStatus ret = retSuccess;

	//if (bWait == false) return retSuccess;

	if (g_pSimulation == NULL)
	{
		DWORD now = GetTickCount();
		if( bWait || (( getDurationOfCommandsInPipe( ) == 0 ) && (( now - lastCheck ) > 200 )))
		{
			lastCheck = now;
			if((( count & 0x01 ) != 0 ) || bWait )
			{
				ret = sendCommand( COMMAND_GET_POSITION );
				if (ret == retSuccess) addCommandToPipe(0, 0, 0, 10);
				if (ret == retSuccess && bWait)
				{
					// Give one more second that the duration of commands peninding in
					// the pipe
					ret = waitForStatus( getDurationOfCommandsInPipe( ) + 1000 );
				}
			}
			else if(( count & 0x01 ) == 0 )
			{
				ret = sendCommand( COMMAND_GET_DEBUG );
				if (ret == retSuccess) addCommandToPipe(0, 0, 0, 10);
			}
			count++;
		}
		else ret = retBusy;
	}
	return ret;
}

void OnAcknowledge(PVOID pParam)
{
	removeCommandFromPipe();
}

void motorInit()
{
	Spindle.currentState = 0;
	Spindle.nextState = 0;

	timePipeInIdx = 0;
	timePipeOutIdx = 0;
	timeInPipe = 0;
	xInPipe = 0;
	yInPipe = 0;
	zInPipe = 0;
	memset(inPipe, 0, sizeof(inPipe));

	registerSocketCallback(CNC_ACKNOWLEDGE, OnAcknowledge);
}
