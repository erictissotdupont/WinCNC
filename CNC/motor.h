
#include "geometry.h"

typedef struct _tAxis {
  long step;
  double scale;
} tAxis;

typedef struct _tSpindle {
  int currentState;
  int nextState;
} tSpindle;

void motorInit( );
void setExportFile( HANDLE file );
void setSimulationMode(tStatus(*callback)(t3DPoint, t3DPoint, long));

void initAxis( int a, double scale );

void stepToPos(long x, long y, long z, t3DPoint* P);

void getPhysicalPosition( t3DPoint* P );
void getTheoricalPos(t3DPoint* R);
void updateTheoricalPosition(double X, double Y, double Z);

void resetMotorPosition( long x, long y, long z, int queueSize );

double getLargestStep( );
double getSmalestStep( );
double getMaxDistanceError( );

void GetMachinePosition(t3DPoint* pPos);
tStatus doMove( void(*posAtStep)(t3DPoint*,int,int,void*), int stepCount, double duration, void* pArg );

int setSpindleState( int state );

tStatus ResetCNCPosition( );
tStatus ClearCNCError( );