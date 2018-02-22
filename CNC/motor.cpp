
#include "CNC.h"
#include "status.h"
#include "geometry.h"
#include "motor.h"
#include "keyboard.h"
#include "socket.h"

#define MAX_CMD_IN_PIPE	  1024 // 1 second
#define TIMEPIPESIZE	  (MAX_CMD_IN_PIPE/16)
#define POLL_RATE		  30 // ms

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
	if (getDurationOfCommandsInPipe() > MAX_CMD_IN_PIPE) return false;
	
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

		if (strcmp(str, "@") != 0)
		{
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
			while (((((pipeDuration = getDurationOfCommandsInPipe( )) > MAX_CMD_IN_PIPE) ||
				     ((pipeCount = getCountOfCommandsInPipe( )) >= TIMEPIPESIZE))) && timeout >= 0 )
			{
				Sleep(POLL_RATE);
				timeout--;
			}
			if (timeout <= 0)
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
