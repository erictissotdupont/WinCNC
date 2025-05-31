
#include "geometry.h"

typedef struct _tAxis {
  long step;
  double scale;
} tAxis;

typedef struct _tSpindle {
  int currentState;
  int nextState;
} tSpindle;

void MotorInit( );
void SetMotorSimulationMode(tStatus(*callback)(t3DPoint, t3DPoint, long));

void InitMotorAxis( int a, double scale );

void GetRealPosition( t3DPoint* P );
void GetTheoricalPosition(t3DPoint* R);
void UpdateTheoricalPosition(double X, double Y, double Z);

void ResetMachinePosition( long x, long y, long z );

double GetMotorSmalestStep( );
double GetMaxMotorDistanceError( );

tStatus MotorDoTheMode( void(*posAtStep)(t3DPoint*,int,int,void*), int stepCount, double duration, void* pArg );

int SetMachineSpindleState( int state );
