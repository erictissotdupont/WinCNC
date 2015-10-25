
#include "CNC.h"

#include "status.h"
#include "geometry.h"
#include "motor.h"
#include "keyboard.h"
#include "socket.h"

HANDLE exportFile = NULL;
tAxis XMotor,YMotor,ZMotor;
tAxis* pMotor[] = {&XMotor,&YMotor,&ZMotor};
tSpindle Spindle;

tStatus(*g_pSimulation)(t3DPoint, long, long, long, long, long) = NULL;

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

void initSpindle( )
{
  Spindle.currentState = 0;
  Spindle.nextState = 0;
}

void resetMotorPosition( )
{
  XMotor.step = 0;
  YMotor.step = 0;
  ZMotor.step = 0;
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

  duration = duration / stepCount;
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
			status = sendCommand(str, (long)duration);
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

  return status;
}


tStatus CheckStatus()
{
	tStatus ret;
	ret = sendCommand( "S\n", 1000);
	if (ret == retSuccess)
	{
		ret = waitForStatus( );
	}
	return ret;
}
