
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
void getCurPos( t3DPoint* P );
void getRawStepPos( long* x, long* y, long* z );
void setRawStepPos(long x, long y, long z);

void resetMotorPosition( long x, long y, long z );

double getLargestStep( );
double getSmalestStep( );
double getMaxDistanceError( );

tStatus doMove( void(*posAtStep)(t3DPoint*,int,int,void*), int stepCount, double duration, void* pArg );

int setSpindleState( int state );

tStatus ResetCNCPosition( );
tStatus ClearCNCError( );