
#include "CNC.h"
#include "winsock.h"
#include "status.h"
#include "geometry.h"
#include "motor.h"
#include "keyboard.h"
#include "socket.h"
#include "gcode.h"

#define TIMEPIPESIZE			  256
#define POLL_RATE				  30 // ms

#define COMMAND_RESET_ORIGIN   "RST"
#define COMMAND_GET_POSITION   "POS"
#define COMMAND_GET_DEBUG	   "DBG"
#define COMMAND_CALIBRATE      "CALIBRATE"

HANDLE exportFile = NULL;
tAxis XMotor,YMotor,ZMotor;
t3DPoint g_TheoricalPosition = { 0.0, 0.0, 0.0 };
tAxis* pMotor[] = {&XMotor,&YMotor,&ZMotor};
tSpindle Spindle;
tStatus(*g_pSimulation)(t3DPoint, t3DPoint, long) = NULL;
unsigned int g_CmdLast;
t3DPoint* g_CmdQ;
int g_CmdQueueSize;

void setExportFile( HANDLE file )
{
	exportFile = file;
}

void setSimulationMode(tStatus(*callback)(t3DPoint, t3DPoint, long))
{
	g_pSimulation = callback;
}

void stepToPos(long x, long y, long z, t3DPoint* P)
{
	P->x = x * XMotor.scale;
	P->y = y * YMotor.scale;
	P->z = z * ZMotor.scale;
}

void getPhysicalPosition( t3DPoint* P )
{
	stepToPos(XMotor.step, YMotor.step, ZMotor.step, P);
}

void getTheoricalPos( t3DPoint* R )
{
	*R = g_TheoricalPosition;
}

void updateTheoricalPosition(double X, double Y, double Z)
{
	g_TheoricalPosition.x += X;
	g_TheoricalPosition.y += Y;
	g_TheoricalPosition.z += Z;
}

void initAxis( int a, double scale )
{
  tAxis* pA = pMotor[a];
  pA->scale = scale;
}

void resetMotorPosition( long x, long y, long z, int cmdQueueSize )
{
  XMotor.step = x;
  YMotor.step = y;
  ZMotor.step = z;

  g_TheoricalPosition.x = x * XMotor.scale;
  g_TheoricalPosition.y = y * YMotor.scale;
  g_TheoricalPosition.z = z * ZMotor.scale;

  // Add one slot just in case
  cmdQueueSize++;
  if (cmdQueueSize != g_CmdQueueSize)
  {
	  if (g_CmdQ) free(g_CmdQ);
	  g_CmdQ = (t3DPoint*)malloc(sizeof(t3DPoint) * cmdQueueSize);
	  g_CmdQueueSize = cmdQueueSize;
  }
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
  Spindle.currentState = Spindle.nextState;
  return Spindle.currentState;
}

void GetMachinePosition(t3DPoint* pPos)
{
	int inQueue = GetInQueueCount();
	if (inQueue == 0 || g_CmdQ )
	{
		getPhysicalPosition(pPos);
	}
	else
	{
		int oldestCmd = g_CmdLast - inQueue;
		if (oldestCmd < 0) oldestCmd += g_CmdQueueSize;
		*pPos = g_CmdQ[oldestCmd];
	}
}

tStatus doMove( void(*posAtStep)(t3DPoint*,int,int,void*), int stepCount, double duration, void* pArg )
{
  int i;
  long x, y, z;
  unsigned long d, s;
  t3DPoint End;
  char str[100] = { 0 };
  tStatus status = retNoOutputFound;
 
  if (stepCount <= 0)
  {
	  return retUnknownErr;
  }

  // Split the duration of the whole move for each step
  duration = duration / stepCount;
  // Convert that in uS for the CNC
  d = (unsigned long)(duration * 1000);

  LockMachinePosition(true);
 
  for( i=1; i<=stepCount; i++ )
  {
    // Get the position we should be at for step i of stepCount
    posAtStep( &End, i, stepCount, pArg );

	x = calculateMove( &XMotor, End.x );
    y = calculateMove( &YMotor, End.y );
    z = calculateMove( &ZMotor, End.z );
	s = ( getSpindleState() == 3 ) ? CMD_FLAG_SPINDLE_ON : 0;

	if (g_pSimulation)
	{
		t3DPoint Start;
		getPhysicalPosition(&Start);
		status = g_pSimulation(Start, End, d );
	}
	else
	{
		// Calculate the CRC of the position at the end of the movement
        // so that the machine can check if its position and distance 
		// corresponds to what the host wants. Includes duration and
		// flags
		s = s | GetPosCRC(XMotor.step, YMotor.step, ZMotor.step, d, s );

		sprintf_s(str, sizeof(str), "@" CNC_CMD_PARAMS, x, y, z, d, s );

		status = postCommand( str );

		if (status != retSuccess)
		{
			break;
		}
		else
		{
			getPhysicalPosition(&g_CmdQ[g_CmdLast]);
			g_CmdLast++;
			if (g_CmdLast >= g_CmdQueueSize) g_CmdLast = 0;
		}
	}

    if( exportFile && *str )
    {
	  // if( fwrite( str, 1, strlen( str ), exportFile ) > 0 && status == retCncNotConnected )
	  if (WriteFile(exportFile, str, strlen(str), NULL, NULL) && status == retCncNotConnected)
      {
        status = retSuccess;
      }
    }
  }

  LockMachinePosition(false);

  return status;
}

tStatus ResetCNCPosition( )
{
	// TODO
	return retNotImplemented;
}

tStatus ClearCNCError()
{
	// TODO
	return retNotImplemented;
}

void motorInit()
{
	Spindle.currentState = 0;
	Spindle.nextState = 0;
}
