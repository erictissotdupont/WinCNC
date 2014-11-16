
#include <unistd.h>
#include <sys/stat.h> 
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include <string.h>


#define PI           3.14159265359
#define IPM_TO_IPMS  60000
#define NEAR_ZERO    0.000000001

#define MAX_FEED     200.0
#define MIN_FEED     0.1

typedef struct _tAxis {
  int step;
  int dir;
  int fStep;
  int fDir;
  int swap;
  double scale;
  double cutComp;
} tAxis;

typedef struct _tPoint {
  double x;
  double y;
  double z;
} tPoint;

typedef struct _tMotion {
  double X;
  double Y;
  double Z;
  double I;
  double J;
  double K;
  double P;
  double b1,b2;
  int motion;
} tMotion;


FILE *simOutput = NULL;

tAxis XMotor,YMotor,ZMotor;

// Compensated
tMotion CompBfrdB = {0,0,0,0,0,0,0,0,0}; // Buffered move
tMotion CompBfrdC = {0,0,0,0,0,0,0,0,0}; // Buffered circle
tMotion CompBfrdZ = {0,0,0,0,0,0,0,0,0}; // Buffered Z only move

double feedSpeed = 30; // Inches per minute

double cutterRadius = 0;

int set_gpio_func(unsigned int pin, unsigned int out ) 
{
  char cmd[100];
  sprintf( cmd, "echo %d > /sys/class/gpio/export", pin );
  if( system( cmd ) < 0 ) return -1;
  sprintf( cmd, "echo %s > /sys/class/gpio/gpio%d/direction", out ? "out" : "in", pin );
  if( system( cmd ) < 0 ) return -1;
  sprintf( cmd, "/sys/class/gpio/gpio%d/value", pin );
  return open( cmd, out ? O_WRONLY : O_RDONLY );
}

double getTimeMs( int reset )
{
  double ret;
  static struct timespec start;
  struct timespec now;
  if( reset )
  {
    clock_gettime(CLOCK_MONOTONIC, &start);
    ret = 0.0;
  }
  else
  {
    clock_gettime(CLOCK_MONOTONIC, &now);
    ret = ( now.tv_nsec - start.tv_nsec ) / 1000000.0;
    ret = ret + ( now.tv_sec - start.tv_sec ) * 1000.0;
  }
  return ret;
}


int set_gpio( int fIO, int val) 
{
  return write( fIO, val ? "1" : "0", 1 );
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

void getCurPos( tPoint* P )
{
  P->x = XMotor.step * XMotor.scale;
  P->y = YMotor.step * YMotor.scale;
  P->z = ZMotor.step * ZMotor.scale;
}

void getCompPos( tPoint* P )
{
  double cX,cY,cZ;
  getCompensation( &cX,&cY,&cZ );
  printf( " Compensation : %.3f,%.3f,%.3f\n", cX,cY,cZ );
  getCurPos( P );
  P->x -= cX;
  P->y -= cY;
  P->z -= cZ;

  if( CompBfrdB.motion )
  {
    P->x += CompBfrdB.X;
    P->y += CompBfrdB.Y;
    P->z += CompBfrdB.Z;
    printf( " Buffered compensated move : %.3f,%.3f,%.3f\n", CompBfrdB.X,CompBfrdB.Y,CompBfrdB.Z );
  }
  if( CompBfrdC.motion )
  {
    P->x += CompBfrdC.X;
    P->y += CompBfrdC.Y;
    P->z += CompBfrdC.Z;
    printf( " Buffered compensated circle : %.3f,%.3f,%.3f\n", CompBfrdC.X,CompBfrdC.Y,CompBfrdC.Z );
  }
  if( CompBfrdZ.motion )
  {
    P->x += CompBfrdZ.X;
    P->y += CompBfrdZ.Y;
    P->z += CompBfrdZ.Z;
    printf( " Buffered compensated Z : %.3f,%.3f,%.3f\n", CompBfrdZ.X,CompBfrdZ.Y,CompBfrdZ.Z );
  }
}

void addCompensation( double x, double y, double z )
{
  XMotor.cutComp += x;
  YMotor.cutComp += y;
  ZMotor.cutComp += z;
}

void setDir( tAxis* pA, int dir )
{
  if( pA->dir != dir )
  {
    pA->dir = dir;
    set_gpio( pA->fDir, ( dir * pA->swap ) < 0 );
  }
}

void step( tAxis* pA )
{
  if( pA->dir > 0 ) 
    pA->step++; 
  else 
    pA->step--;

  if( !simOutput ) set_gpio( pA->fStep, pA->step & 1 );
}

void initAxis( tAxis* pA, int stepIO, int dirIO, double scale, int swap )
{
  pA->fStep = set_gpio_func(stepIO, 1);
  pA->fDir = set_gpio_func(dirIO, 1);
  set_gpio( pA->fDir, 0 );
  set_gpio( pA->fStep, 0 );
  pA->dir = 0;
  pA->step = 0;
  pA->scale = scale;
  pA->swap = swap;
  pA->cutComp = 0;
}


void resetAxisPosition( tAxis* pA )
{
  pA->step = 0;
}

double minOf3( double a, double b, double c )
{
  if( a < b && a < c ) return a;
  if( b < c ) return b;
  return c;
}

double minOf2( double a, double b )
{
  if( a < b ) return a;
  return b;
}

double vector3DLength( tPoint V )
{
  return sqrt( V.x*V.x + V.y*V.y + V.z*V.z );
}

double vectorLength( double U1, double U2 )
{
  return sqrt( U1*U1 + U2*U2 );
}

double distance3D( tPoint A, tPoint B )
{
  tPoint V;
  V.x = A.x - B.x;
  V.y = A.y - B.y;
  V.z = A.z - B.z;
  return vector3DLength( V ); 
}


void axisMoveCalc( tAxis* A, double ideal, int* flag, int mask )
{
  double delta = ideal - A->step * A->scale;
  if( delta >= A->scale )
  { 
    setDir( A, 1 );
    *flag |= mask;
    if( delta >= 2 * A->scale ) *flag |= 8;
  }
  else if( delta <= -A->scale )
  {
    setDir( A, -1 ); 
    *flag |= mask;
    if( delta <= -2 * A->scale ) *flag |= 8;
  }
}


void doMove( void(*posAtStep)(tPoint*,int,int,void*), int stepCount, double duration, void* pArg )
{
  int i;
  int flag;
  tPoint Ideal;

  // Debug stuff
  int tooSlow = 0;
  int overflow = 0;
  double minSleep = 10000000.0;
  double maxSleep = 0.0;
  
  // Reset the time origin
  getTimeMs( 1 );
 
  for( i=1; i<=stepCount; i++ )
  {
    flag = 0;

    // Get the position we should be at for step i of stepCount
    posAtStep( &Ideal, i, stepCount, pArg );
    
    // For each axis, check if a movement is needed. Note that the
    // direction change happens in here
    axisMoveCalc( &XMotor, Ideal.x, &flag, 1 );
    axisMoveCalc( &YMotor, Ideal.y, &flag, 2 );
    axisMoveCalc( &ZMotor, Ideal.z, &flag, 4 );

    // If a movement is needed, do it. Otherwise, move on to 
    // the next step position
    if( flag & 7 )
    {
      // If simulation mode is enabled, don't do the wait
      if( !simOutput )
      {
        // Calculate how much time to wait (constant speed)
        double timeToWait = ( duration * i / stepCount ) - getTimeMs( 0 );
        
        // If we're already late
        if( timeToWait < 0 )
        {
          // Keep track for debug but still sleep for a minimal amount of time 
          // to avoid missing the step in the controller.
          tooSlow++;
          timeToWait = 0.01;
        };
        // Do the sleep
        usleep( timeToWait * 1000 );

        // For debug
        if( timeToWait > maxSleep ) maxSleep = timeToWait;
        else if( timeToWait < minSleep ) minSleep = timeToWait;
      }
     
      // This is where we actually move the motors
      if( flag & 1 ) step( &XMotor );
      if( flag & 2 ) step( &YMotor );
      if( flag & 4 ) step( &ZMotor );

      // For debug. This means the we should have moved to steps on one 
      // of the axis
      if( flag & 8 ) { overflow++; }

      // This logs the position of the tool in a text file
      if( simOutput )
      {
        static tPoint lastLogPos = {0,0,0};
        tPoint pos;

        getCurPos( &pos );
        if( distance3D( pos, lastLogPos ) >= 0.01 )
        {
          char strPos[80];
          sprintf( strPos, "%.3f\t%.3f\t%.3f\n", pos.x, pos.y, pos.z );
          fwrite( strPos, 1, strlen( strPos ), simOutput );
          lastLogPos = pos;
        }
      }
    }
  }

  // Makes sure we get all the trace of this movement if program is interrupted
  if( simOutput )
  {
    fflush( simOutput );
  }

  printf( "T:%.0fms Step:%d Sleep(Avg:%.2f,Min:%.2f,Max:%.2f) - SLOW:%d - OVLF:%d\n", 
    duration,
    stepCount,
    duration/stepCount,
    minSleep, maxSleep,
    tooSlow, 
    overflow );
}

// ---------------------- ARCS ------------------------------------

typedef struct {
  tPoint center;
  tPoint start;
  tPoint end;
  double arcAngle;
  double arcT;
  double aRadius;
  double bRadius;
  double rotAngle;
} tArcInfo;

double angleVector( double U1, double U2, double V1, double V2 )
{
  double a,b,r;
  a = U1*V1 + U2*V2;
  b = vectorLength(U1,U2) * vectorLength(V1,V2);
  r = acos( a/b ); 
  //printf( "angleVector:(%.3f,%.3f)|(%.3f,%.3f)=%.3f (a:%.3f b:%.3f)\n", U1,U2,V1,V2,r*180.0/PI,a,b );
  return r;
}

double dotProduct( double U1, double U2, double V1, double V2 )
{
  return U1*V1+U2*V2;
}

void rotateInXYPlane( tPoint* P, tPoint C, double a )
{
  tPoint R;
  // Translate axis to orign at C
  R.x = P->x - C.x;
  R.y = P->y - C.y;
  // Rotate in XY plane
  P->x = R.x*cos(a) - R.y*sin(a);
  P->y = R.x*sin(a) + R.y*cos(a);
  // Move back the axis origin
  P->x = P->x + C.x;
  P->y = P->y + C.y;
}

void getArcPosStepAt( tPoint* P, int s, int total, void* pArg )
{
  tArcInfo* pInfo = (tArcInfo*)pArg;
  double t = PI + ( pInfo->arcT * s / total );

  P->x = pInfo->center.x + pInfo->aRadius * cos( t );
  P->y = pInfo->center.y + pInfo->bRadius * sin( t );
  P->z = pInfo->start.z;

  rotateInXYPlane( P, pInfo->center, pInfo->rotAngle );

  P->x = pInfo->start.x + P->x;
  P->y = pInfo->start.y + P->y;
}

#define ARC_LENGTH_APPROX_STEPS	100

double lengthOfArc( tArcInfo* pInfo )
{
  int i;
  tPoint a,b;
  double l = 0;

  getArcPosStepAt( &a, 0, ARC_LENGTH_APPROX_STEPS, pInfo );
  for( i=1;i<=ARC_LENGTH_APPROX_STEPS;i++)
  {
    getArcPosStepAt( &b, i, ARC_LENGTH_APPROX_STEPS, pInfo );
    l = l + distance3D( a, b );
    a = b;
  }
  return l;
}

void arcInXYPlane( double X, double Y, double Z, double I, double J, double P )
{
  double l;
  tArcInfo info;
  int stepCount;
  double duration;

  // Debug
  double err;
  tPoint start;
  tPoint end;

  // Start and end of the movement
  getCurPos( &info.start );

  info.end.x = info.start.x + X;
  info.end.y = info.start.y + Y;
  info.end.z = info.start.z + Z;  

  // The center of the ellipse
  info.center.x = I;
  info.center.y = J;
  info.center.z = info.start.z;

  // First radius is distance from start to center (angle start is PI)
  info.aRadius = vectorLength( I,J );

  // The angle of the arc to be drawn
  info.arcAngle = angleVector( -I, -J, X-I, Y-J );

  // The parameter of the ellipse equation isn't the same as the angle (except on axis)
  info.arcT = acos( vectorLength( X-I, Y-J ) * cos( info.arcAngle ) / info.aRadius );

  // Second radio is projection of endpoint on perpendicular to first radius
  info.bRadius = vectorLength( X-I, Y-J ) * sin( info.arcAngle ) / sin( info.arcT );

  // If P is negative, direction is CW, positive then it's CCW
  // Also adds the # of turns (P MUST be a integer)
  // TODO : generate an error if P has a factional part.
  if( P < 0 )
    info.arcT = -info.arcT + ( 2 * PI * ( P + 1.0 ));
  else
    info.arcT = info.arcT + ( 2 * PI * ( P - 1.0 ));

  // The rotation of the ellipse (relative to the X axis)
  info.rotAngle = atan2( J, I );

  // Length of the arc
  l = lengthOfArc( &info );

  printf( " Ctr:%.3f,%.3f,%.3f - R:%.3f,%.3f - L:%.3f - arc:%.3f t:%.3f - Rot:%.4f\n", 
    info.center.x,
    info.center.y,
    info.center.z,
    info.aRadius,
    info.bRadius,
    l,
    info.arcAngle * 180.0 / PI,
    info.arcT * 180.0 / PI,
    info.rotAngle * 180.0 / PI );

  stepCount = l / minOf3( XMotor.scale, YMotor.scale, ZMotor.scale );
  duration = l / feedSpeed * IPM_TO_IPMS;

  getArcPosStepAt( &start, 0, 100, &info );
  err = distance3D( start, info.start );
  if( err > NEAR_ZERO ) printf( "ERROR on starting point of %f.\n",err );
  
  getArcPosStepAt( &end, 100, 100, &info );
  err = distance3D( end, info.end );
  if( err > NEAR_ZERO ) printf( "ERROR on ending point of %f.\n",err );
   
  doMove( getArcPosStepAt, stepCount, duration, &info );
  
}

// --------------------- LINEAR ----------------------------
typedef struct {
  double x,y,z;
  tPoint Origin;
} linearRelInfo;

void getLinearRelStepAt( tPoint* P, int s, int total, void* pArg )
{
  linearRelInfo* pInfo = (linearRelInfo*)pArg;
  P->x = pInfo->Origin.x + pInfo->x * s / total;
  P->y = pInfo->Origin.y + pInfo->y * s / total;
  P->z = pInfo->Origin.z + pInfo->z * s / total;
}

void linearRel( double x, double y, double z )
{ 
  linearRelInfo info;
  double l;
  int stepCount;
  double duration;

  l = sqrt(x*x+y*y+z*z);
  stepCount = 3 * l / minOf3( XMotor.scale, YMotor.scale, ZMotor.scale );
  duration = l / feedSpeed * IPM_TO_IPMS;

  info.x = x;
  info.y = y;
  info.z = z;
  getCurPos( &info.Origin );

  doMove( getLinearRelStepAt, stepCount, duration, &info );

}

// --------------------- CIRCLE IN X-Y PLANE -----------------------

typedef struct {
  tPoint center;
  tPoint origin;
  double radius;
} tCircleInfo;

void getCircleRelStepAt( tPoint* P, int s, int total, void* pArg )
{
  double arc = PI * 2 * s / total;
  tCircleInfo* pInfo = (tCircleInfo*)pArg;

  P->x = pInfo->origin.x + pInfo->center.x + pInfo->radius * cos( arc );
  P->y = pInfo->origin.y + pInfo->center.y + pInfo->radius * sin( arc );
  P->z = pInfo->origin.z;
}

void circle( double radius )
{ 
  tCircleInfo info;
  double l = 2 * PI * radius;
  int stepCount = l / minOf3( XMotor.scale, YMotor.scale, ZMotor.scale );
  double duration = l / feedSpeed * IPM_TO_IPMS;

  getCurPos( &info.origin );
  info.origin.x -= radius;
  info.radius = radius;

  doMove( getCircleRelStepAt, stepCount, duration, &info );
}

// ---------------------- Cutter Compensation -----------------------------

#define STATE_IDLE       0
#define STATE_LEAD       1
#define STATE_CONTINUOUS 2

int toolCompensation( tMotion* pM, int mode, int loop )
{
  tMotion temp;
  static int state = STATE_IDLE;
  //static double sum1;
  //static double sum2;
  static int lastLoop;
  
  // No compensation
  if( mode == 40 )
  {
    // and no previous compensation ending
    if( state == STATE_IDLE )
    {
      // Returns true only if it's the first time we're here
      return (loop==0);
    }
    else
    {
      // Flush out compensated movement
      if(loop == 0)
      {
        CompBfrdB.X += CompBfrdB.b1;
        CompBfrdB.Y += CompBfrdB.b2;
        addCompensation( CompBfrdB.b1, CompBfrdB.b2, 0 );
        temp = *pM;
        *pM = CompBfrdB;
        CompBfrdB = temp;
        lastLoop = 1;
        return 1; 
      }
      else if( CompBfrdZ.motion != 0 )
      {
        *pM = CompBfrdZ;
        CompBfrdZ.motion = 0;
        lastLoop = 2;
        return 1;
      }
      else if(loop == lastLoop)
      {
        double compX, compY;
        // And execute the next non compensated (if any) less
        // the overall compensation in this motion
        getCompensation( &compX, &compY, 0 );
        printf( "Cutter Compensation. Removing %.3f,%.3f\n", compX, compY );
        CompBfrdB.X -= compX;
        CompBfrdB.Y -= compY;
        resetCompensation( );
        *pM = CompBfrdB;
        CompBfrdB.motion = 0;
        state = STATE_IDLE;
        return 1;
      }
      return 0;
    }
  }
  else if( CompBfrdC.motion != 0 )
  {
    *pM = CompBfrdC;
    CompBfrdC.motion = 0;
    return 1;
  }
  else if(( mode == 41 || mode == 42 ) && loop == 0 )
  {
    double angle;
    double backCut = 0;
    double l;
    double dir = ( mode == 41 ) ? 1 : -1;

    // Move in the tool axis gets buffered and executed
    // after the current move can be evaluated relative to
    // the next move (otherwise it will be out of order)
    if( pM->X == 0 && pM->Y == 0 )
    {
       CompBfrdZ = *pM;
       return 0;
    }
 
    if( state == STATE_IDLE )
    {
      CompBfrdB = *pM;
      state = STATE_LEAD;
      return 0;
    }
    else if( state == STATE_LEAD )
    {
      // Perpendicular offset
      l = vectorLength( pM->X, pM->Y ) * dir;
      CompBfrdB.b1 = -pM->Y * cutterRadius / l;
      CompBfrdB.b2 = pM->X * cutterRadius / l;
      state = STATE_CONTINUOUS;
    }
    else
    {
      if( dir > 0 )
        angle = atan2( CompBfrdB.X, CompBfrdB.Y ) - atan2( pM->X, pM->Y );
      else
        angle = atan2( pM->X, pM->Y ) - atan2( CompBfrdB.X, CompBfrdB.Y );

      if( angle < 0 ) angle += 2 * PI;

      // printf( "Cutter Compensation. Angle between moves is %.1f.\n", angle * 180 / PI );

      // If opening between the 2 moves is a closed angle (less than 180 degrees)
      // tool needs to stop on the bisect to avoid cutting into the next
      // move path. This is by how much less along the direction of the cut
      if( angle < PI )
      {
        backCut = cutterRadius * tan( angle / 2 );
        l = vectorLength( CompBfrdB.X, CompBfrdB.Y );          
        CompBfrdB.b1 -= CompBfrdB.X * backCut / l;
        CompBfrdB.b2 -= CompBfrdB.Y * backCut / l;

        l = vectorLength( pM->X, pM->Y );          
        pM->b1 -= pM->X * backCut / l;
        pM->b2 -= pM->Y * backCut / l;
      }
      else
      {
        l = vectorLength( CompBfrdB.X, CompBfrdB.Y ) * dir;
        CompBfrdC.I = CompBfrdB.Y * cutterRadius / l;
        CompBfrdC.J = -CompBfrdB.X * cutterRadius / l;
        CompBfrdC.K = 0;

        l = vectorLength( pM->X, pM->Y ) * dir;
        CompBfrdC.X = -pM->Y * cutterRadius / l + CompBfrdC.I;
        CompBfrdC.Y = pM->X * cutterRadius / l + CompBfrdC.J;
        CompBfrdC.Z = 0;
        CompBfrdC.P = 1;
        CompBfrdC.motion = ( mode == 41 ) ? 2 : 3;

        addCompensation( CompBfrdC.X, CompBfrdC.Y, 0 );
      }
    }

    CompBfrdB.X += CompBfrdB.b1;
    CompBfrdB.Y += CompBfrdB.b2;

    addCompensation( CompBfrdB.b1, CompBfrdB.b2, 0 );

    // Save current move in buffer and return the previous one in pM
    temp = *pM;
    *pM = CompBfrdB;
    CompBfrdB = temp;

    return 1;
  }
  else if( CompBfrdZ.motion != 0 )
  {
    *pM = CompBfrdZ;
    CompBfrdZ.motion = 0;
    return 1;
  }

  return 0;
}




// ------------------------- G CODE PARSER ------------------------

const int modal0[] = { 0,1,2,3,38,82,84,85,86,87,88,89, -1 }; // Motion
const int modal1[] = { 17,18,19, -1 };                        // Plane selection
const int modal2[] = { 90,91, -1 };                           // Distance mode
const int modal3[] = { 93,94, -1 };                           // Feed rate mode
const int modal4[] = { 20,21, -1 };                           // Units
const int modal5[] = { 40,41,42, -1 };                        // Cutter radius compensation
const int modal6[] = { 43,49, -1 };                           // Tool length offset
const int modal7[] = { 98,99, -1 };                           // Return mode in canned cycles
const int modal8[] = { 54,55,56,57,58,59, -1 };               // Coordinates system selection
const int modal9[] = { 61,64, -1 };                           // Path control mode

const int nonModal[] = { 4,10,28,30,53,92, -1 };              // Non modal group

#define MODALGRPCNT 10
const int* modalGroup[MODALGRPCNT] = {modal0,modal1,modal2,modal3,modal4,modal5,modal6,modal7,modal8,modal9};


int parseCmd( char* cmd )
{
  static int activeCmd[MODALGRPCNT] = { 0, 17, 91, 93, 20, 40, 43, 98, 54, 61 }; // Default active commands
  char *pt;
  int i,j,n;
  double val;
  tMotion M;
  int loop;
  int gotWhat = 0;
  static tPoint theoricalPos = {0,0,0};
  tPoint actualPos;
  tPoint compPos;

  M.X = 0;
  M.Y = 0;
  M.Z = 0;
  M.I = 0;
  M.J = 0;
  M.K = 0;
  M.P = 1;
  M.b1 = 0;
  M.b2 = 0;
  M.motion = 0;
  
  // Debug
  int cmdInGroup[MODALGRPCNT];
  memset( cmdInGroup, 0x00, sizeof( cmdInGroup ));

  printf( "parseCmd( %s )\n", cmd );

  if(( pt = strstr( cmd, "X" )) != NULL )
  {
    if( sscanf( pt+1, "%lf", &val ) != 1 ) return -1; 
    M.X = val;
    gotWhat |= 1;
  }
  if(( pt = strstr( cmd, "I" )) != NULL )
  {
    if( sscanf( pt+1, "%lf", &val ) != 1 ) return -1; 
    M.I = val;
    gotWhat |= 2;
  }
  if(( pt = strstr( cmd, "Y" )) != NULL )
  {
    if( sscanf( pt+1, "%lf", &val ) != 1 ) return -1; 
    M.Y = val;
    gotWhat |= 4;
  }
  if(( pt = strstr( cmd, "J" )) != NULL )
  {
    if( sscanf( pt+1, "%lf", &val ) != 1 ) return -1; 
    M.J = val;
    gotWhat |= 8;
  }
  if(( pt = strstr( cmd, "Z" )) != NULL )
  {
    if( sscanf( pt+1, "%lf", &val ) != 1 ) return -1; 
    M.Z = val;
    gotWhat |= 16;
  }
  if(( pt = strstr( cmd, "K" )) != NULL )
  {
    if( sscanf( pt+1, "%lf", &val ) != 1 ) return -1; 
    M.K = val;
    gotWhat |= 32;
  }
  if(( pt = strstr( cmd, "D" )) != NULL )
  {
    if( sscanf( pt+1, "%lf", &val ) != 1 ) return -1; 
    cutterRadius = val;
    gotWhat |= 64;
  }
  if(( pt = strstr( cmd, "F" )) != NULL )
  {
    if( sscanf( pt+1, "%lf", &val ) != 1 ) return -1; 
    if( val > 0 )
    {
      if( val > MAX_FEED ) 
      {
        feedSpeed = MAX_FEED; 
        printf( "WRN: MAX feed limit %f IMP.\n", MAX_FEED );
      }
      else if( val < MIN_FEED )
      {
        feedSpeed = MIN_FEED;
        printf( "WRN: MIN feed limit %f IMP.\n", MIN_FEED );
      }
      else feedSpeed = val;
    }
    else return -1;
  }

  pt = cmd;
  while(( pt = strstr( pt, "G" )) != NULL )
  {
    pt++;
    if( sscanf( pt, "%d", &n ) != 1 ) return -1;

    switch( n )
    {
    case 10 :
      resetAxisPosition( &XMotor );
      resetAxisPosition( &YMotor );
      resetAxisPosition( &ZMotor );
      theoricalPos.x = 0;
      theoricalPos.y = 0;
      theoricalPos.z = 0;
      break;

    default :
      for( j=0; j<MODALGRPCNT; j++ )
      {
        for(i=0; modalGroup[j][i]>=0; i++ )
        {
          //printf( "[%d,%d]=%d\n", j,i, modalGroup[j][i] );
          if( n == modalGroup[j][i] )
          {
            cmdInGroup[j]++;
            // There can only be one command of each modal group per
            if( cmdInGroup[j] > 1 ) return -1;
            // Remember the currently active for this group
            activeCmd[j] = n;
          }
        }
      }
    }
  }
  if(( pt = strstr( cmd, "P" )) != NULL )
  {
    if( sscanf( pt+1, "%lf", &val ) != 1 ) return -1; 
    M.P = val;
    gotWhat |= 128;
  }

  getCompPos( &compPos ); // To be worked on

  //compPos = theoricalPos;   // So that cutter comp works
  //getCurPos( &compPos );  // So that precision remains

  // If active distance mode is absolute
  if( activeCmd[2] == 90 )
  {
    // Make coordinates relative and update the theorical position
    if( gotWhat & 1 )
    {
      theoricalPos.x = M.X;
      M.X = M.X - compPos.x;
    }
    if( gotWhat & 4 )
    {
      theoricalPos.y = M.Y;
      M.Y = M.Y - compPos.y;
    }
    if( gotWhat & 16 )
    {
      theoricalPos.z = M.Z;
      M.Z = M.Z - compPos.z;
    }
  }
  else if( activeCmd[2] == 91 )
  {
    // Update the new theorical tool position
    theoricalPos.x += M.X;
    theoricalPos.y += M.Y;
    theoricalPos.z += M.Z;
  }
  else
  {
    printf ("ERROR : Invalid distance mode.\n"); 
    return -1; 
  }

  // From this point on, everything is in relative motion

  loop = 0;
  M.motion = activeCmd[0];

  while( toolCompensation( &M, activeCmd[5], loop++ ))
  {
    // Depending on the active command in the motion group
    switch( M.motion )
    {
      case 0 : // Rapid positioning
      case 1 : // Linear coordinated feed speed
        printf( "G1-Linear movement at feed speed (%.3f,%.3f,%.3f)\n", M.X,M.Y,M.Z );
        linearRel( M.X,M.Y,M.Z );
        break;

      case 2 : // Arc in CW direction
        printf( "G2-Arc CW movement at feed speed(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)\n", M.X,M.Y,M.Z,M.I,M.J,-M.P );
        arcInXYPlane( M.X,M.Y,M.Z,M.I,M.J,-M.P );
        break;

      case 3 : // Arc in CCW direction
        printf( "G3-Arc CCW movement at feed speed(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)\n", M.X,M.Y,M.Z,M.I,M.J,M.P );
        arcInXYPlane( M.X,M.Y,M.Z,M.I,M.J,M.P );
        break;
    }
  }

  getCurPos( &actualPos );
  getCompPos( &compPos );

  printf( "Theor.:%.4f,%.4f,%.4f\nActual:%.4f,%.4f,%.4f\nCompsd:%.4f,%.4f,%.4f\n",
    theoricalPos.x,
    theoricalPos.y,
    theoricalPos.z,
    actualPos.x,
    actualPos.y,
    actualPos.z,
    compPos.x,
    compPos.y,
    compPos.z );

  if( distance3D( theoricalPos, compPos ) > 0.001 ) printf( "################ ERROR #################\n" );
  
  return 0;
}


char* parseLine( char* cmd )
{
  char* pt;
  pt = strstr( cmd, "\n" );
  if( pt )
  {
    *pt = 0;
    pt++;
  }
  
  //printf( "Parsing '%s'\n", cmd );

  if( strstr( cmd, "EXEC " ) == cmd )
  {
    FILE *fp;
    char *fileName = cmd+5;
    char* buffer = (char*)malloc( 5000 );  // 5000 character per line

    fp = fopen( fileName, "r");
    if( fp == NULL ) { printf( "Opening command file '%s' failed.\n", fileName ); return NULL; }

    while( fgets( buffer, 5000, fp ))
    {
      parseLine( buffer );
    }

    printf( "Done.\n" );
    fclose( fp );
    free( buffer );
  }
  else if( strstr( cmd, "SIM " ) == cmd )
  { 
    char *fileName = cmd+4;
    simOutput = fopen( fileName, "w");
  }
  else if( strstr( cmd, "QUIT" ) == cmd )
  {
    if( simOutput ) fclose( simOutput );
    pt = NULL;
  }
  else
  {
    if( parseCmd( cmd ) < 0 ) printf ("Invalid command\n");
  }

  return pt;
}



int main()
{
  char cmd[100];
 
  pthread_t thId = pthread_self();
  pthread_attr_t thAttr;
  int policy = 0;
  int max_prio_for_policy = 0;

  pthread_attr_init(&thAttr);
  pthread_attr_getschedpolicy(&thAttr, &policy);
  max_prio_for_policy = sched_get_priority_max(policy);
  pthread_setschedprio(thId, max_prio_for_policy);
  pthread_attr_destroy(&thAttr);

  initAxis( &XMotor,  7,  8, 0.0004375,  1 ); // 1/16 step - 400 steps - 2.8in per turn
  initAxis( &YMotor,  9, 10, 0.0004375,  1 );
  initAxis( &ZMotor, 11, 25, 0.0003125, -1 ); // 1/4 step - 400 steps - 0.5in per turn

  startKeyInput( );

  sleep( 30 );

  do
  {
    printf( "CMD>" );
  } while( parseLine( fgets(cmd, sizeof(cmd), stdin) ));

  return 0;
}

