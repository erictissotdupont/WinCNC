#include "CNC.h"

#include "status.h"
#include "geometry.h"
#include "gcode.h"
#include "motor.h"

#define MAX_FEED     200.0
#define MIN_FEED     0.5

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

t3DPoint theoricalPos = {0,0,0};

// Cutter compensation buffered moves
tMotion CompBfrdB = {0,0,0,0,0,0,0,0,0,0}; // Buffered move
tMotion CompBfrdC = {0,0,0,0,0,0,0,0,0,0}; // Buffered circle
tMotion CompBfrdZ = {0,0,0,0,0,0,0,0,0,0}; // Buffered Z only move

double feedSpeed = 30; // Inches per minute
double cutterRadius = 0;
int spindle = 0;

void getCompPos( t3DPoint* P )
{
  double cX,cY,cZ;
  
  getCompensation( &cX,&cY,&cZ );
  // printf( " Compensation : %.3f,%.3f,%.3f\n", cX,cY,cZ );
  
  getCurPos( P );
  P->x -= cX;
  P->y -= cY;
  P->z -= cZ;

  if( CompBfrdB.motion )
  {
    P->x += CompBfrdB.X;
    P->y += CompBfrdB.Y;
    P->z += CompBfrdB.Z;
    // printf( " Buffered compensated move : %.3f,%.3f,%.3f\n", CompBfrdB.X,CompBfrdB.Y,CompBfrdB.Z );
  }
  if( CompBfrdC.motion )
  {
    P->x += CompBfrdC.X;
    P->y += CompBfrdC.Y;
    P->z += CompBfrdC.Z;
    // printf( " Buffered compensated circle : %.3f,%.3f,%.3f\n", CompBfrdC.X,CompBfrdC.Y,CompBfrdC.Z );
  }
  if( CompBfrdZ.motion )
  {
    P->x += CompBfrdZ.X;
    P->y += CompBfrdZ.Y;
    P->z += CompBfrdZ.Z;
    // printf( " Buffered compensated Z : %.3f,%.3f,%.3f\n", CompBfrdZ.X,CompBfrdZ.Y,CompBfrdZ.Z );
  }
}

// ---------------------- ARCS ------------------------------------

typedef struct {
  t3DPoint center;
  t3DPoint start;
  t3DPoint end;
  double arcAngle;
  double arcT;
  double aRadius;
  double bRadius;
  double rotAngle;
  double z;
} tArcInfo;


void getArcPosStepAt( t3DPoint* P, int s, int total, void* pArg )
{
  tArcInfo* pInfo = (tArcInfo*)pArg;
  double t = PI + ( pInfo->arcT * s / total );

  P->x = pInfo->center.x + pInfo->aRadius * cos( t );
  P->y = pInfo->center.y + pInfo->bRadius * sin( t );
  P->z = pInfo->start.z + pInfo->z * s / total;

  rotateInXYPlane( P, pInfo->center, pInfo->rotAngle );

  P->x = pInfo->start.x + P->x;
  P->y = pInfo->start.y + P->y;
}


// No formula to calculate the length of an elipse. Approximation
// is good enough for our purpose.
#define ARC_LENGTH_APPROX_STEPS	100

double lengthOfArc( tArcInfo* pInfo )
{
  int i;
  t3DPoint a,b;
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

tStatus arcInXYPlane( double X, double Y, double Z, double I, double J, double P )
{
  double l;
  tArcInfo info;
  long stepCount;
  double duration;

  // Debug
  double err;
  t3DPoint start;
  t3DPoint end;

  // Start and end of the movement
  getCurPos( &info.start );

  // If any z motion
  info.z = Z;

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
  // Also adds the # of turns (P MUST be a integer and non zero)
  // TODO : generate an error if P has a factional part.
  if( P < 0 )
    info.arcT = -info.arcT + ( 2 * PI * ( P + 1.0 ));
  else 
    info.arcT = info.arcT + ( 2 * PI * ( P - 1.0 ));

  // The rotation of the ellipse (relative to the X axis)
  info.rotAngle = atan2( J, I );

  // Length of the arc
  l = lengthOfArc( &info );

  stepCount = 1 + (long)( l / ( getSmalestStep( ) * 100 ));
  duration = l / feedSpeed * IPM_TO_IPMS;

  printf( " Ctr:%.3f,%.3f,%.3f - R:%.3f,%.3f - L:%.3f/%d - arc:%.3f t:%.3f - Rot:%.4f\n", 
    info.center.x,
    info.center.y,
    info.center.z,
    info.aRadius,
    info.bRadius,
    l,stepCount,
    info.arcAngle * 180.0 / PI,
    info.arcT * 180.0 / PI,
    info.rotAngle * 180.0 / PI );

  getArcPosStepAt( &start, 0, 100, &info );
  err = distance3D( start, info.start );
  if( err > NEAR_ZERO ) printf( "ERROR on starting point of %f.\n",err );
  
  getArcPosStepAt( &end, 100, 100, &info );
  err = distance3D( end, info.end );
  if( err > NEAR_ZERO )
  {
    printf( "ERROR on ending point of %f.\n",err );
    printf( "Expected : %.3f %.3f %.3f. Got %.3f %.3f %.3f\n", 
      info.end.x, info.end.y, info.end.z,
      end.x, end.y, end.z );
  }
   
  return doMove( getArcPosStepAt, stepCount, duration, &info );
}

// ------------------------ LINEAR - G1 --------------------------
typedef struct {
  double x,y,z;
  t3DPoint Origin;
} linearRelInfo;

void getLinearRelStepAt( t3DPoint* P, int s, int total, void* pArg )
{
  linearRelInfo* pInfo = (linearRelInfo*)pArg;
  P->x = pInfo->Origin.x + pInfo->x * s / total;
  P->y = pInfo->Origin.y + pInfo->y * s / total;
  P->z = pInfo->Origin.z + pInfo->z * s / total;
}

tStatus linearRel( double x, double y, double z )
{ 
  linearRelInfo info;
  double l;
  double duration;
  long steps;

  l = sqrt(x*x+y*y+z*z);
  duration = ( l / feedSpeed ) * IPM_TO_IPMS;

  info.x = x;
  info.y = y;
  info.z = z;
  getCurPos( &info.Origin );

  // Make sure we don't send single move command taking longer than 1sec
  steps = 1 + (long)( duration / 1000 );

  // printf( "Length=%.2f - Feed=%.2f - Duration=%.2f - Steps=%d\n", l, feedSpeed, duration,steps );

  return doMove( getLinearRelStepAt, steps, duration, &info );
}


// -------------------- Rapid Positioning - G0 ---------------------------
typedef struct {
  t3DPoint Origin;
  double x,y,z;
} rapidPosInfo;

void getrapidPosStepAt( t3DPoint* P, int s, int total, void* pArg )
{
  rapidPosInfo* pInfo = (rapidPosInfo*)pArg;
  P->x = pInfo->Origin.x + pInfo->x * s / total;
  P->y = pInfo->Origin.y + pInfo->y * s / total;
  P->z = pInfo->Origin.z + pInfo->z * s / total;
}

tStatus rapidPosRel( double x, double y, double z )
{ 
  rapidPosInfo info;

  getCurPos( &info.Origin );
  info.x = x;
  info.y = y;
  info.z = z;

  return doMove( getrapidPosStepAt, 1, 0, &info );
}

// --------------------------- Dwell - G4 --------------------------------
void getDwellPosStepAt( t3DPoint* P, int s, int total, void* pArg )
{
  getCurPos( P );
}

tStatus dwell( long t )
{
  return doMove( getDwellPosStepAt, 1, t, NULL );
}

// ------------------------ Cutter Compensation -----------------------------

#define STATE_IDLE       0
#define STATE_LEAD       1
#define STATE_CONTINUOUS 2

int doCutterCompensation( tMotion* pM, int mode, int loop )
{
  tMotion temp;
  static int state = STATE_IDLE;
  //static double sum1;
  //static double sum2;
  static int lastLoop;
  
  // If no cutter compensation
  if( mode == 40 )
  {
    // and previous statis not cutter compensation
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
        if( CompBfrdB.motion == 0 )
        {
          printf( "ERROR : No motion to get out of tool compensation.\n" );
          exit( 1 );
        }
        getCompensation( &compX, &compY, 0 );
        // printf( "Cutter Compensation. Removing %.3f,%.3f\n", compX, compY );
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
    else if( state == STATE_CONTINUOUS )
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
        CompBfrdC.motion = ( mode == 410 ) ? 20 : 30;

        addCompensation( CompBfrdC.X, CompBfrdC.Y, 0 );
      }
	}
	else
	{
		printf("ERROR: Invalid compensation state (%d)\n", state);
		exit(1);
	}

	CompBfrdB.X += CompBfrdB.b1;
	CompBfrdB.Y += CompBfrdB.b2;

	addCompensation(CompBfrdB.b1, CompBfrdB.b2, 0);

	// Save current move in buffer and return the previous one in pM
	temp = *pM;
	*pM = CompBfrdB;
	CompBfrdB = temp;

	return 1;
  }
  else if (CompBfrdZ.motion != 0)
  {
	  *pM = CompBfrdZ;
	  CompBfrdZ.motion = 0;
	  return 1;
  }

  return 0;
}


// ------------------------- G CODE PARSER ------------------------

#define CMD_G0		  0
#define CMD_G1		 10
#define CMD_G2		 20
#define CMD_G3		 30
#define CMD_G4		 40

#define CMD_G10		100
#define CMD_G30		300
#define CMD_G90		900
#define CMD_G91		910


const int modal0[] = { 0, 10, 20, 30, 380, 820, 840, 850, 860, 870, 880, 890, -1 }; // Motion
const int modal1[] = { 170, 180, 190, -1 };                      // Plane selection
const int modal2[] = { 900, 910, -1 };                           // Distance mode
const int modal3[] = { 930, 940, -1 };                           // Feed rate mode
const int modal4[] = { 200, 210, -1 };                           // Units
const int modal5[] = { 400, 410, 420, -1 };                      // Cutter radius compensation
const int modal6[] = { 430, 490, -1 };                           // Tool length offset
const int modal7[] = { 980, 990, -1 };                           // Return mode in canned cycles
const int modal8[] = { 540, 550, 560, 570, 580, 590, -1 };       // Coordinates system selection
const int modal9[] = { 610, 640, -1 };                           // Path control mode

typedef enum groupId { 
	MG_MOTION = 0,  
	MG_PLANE = 1,
	MG_DISTANCE_MODE = 2,
	MG_FEEDRATE_MODE = 3,
	MG_UNITS = 4,
	MG_CUTTER_RADIUS_COMP = 5,
	MG_TOOL_LENGTH_OFFSET = 6,
	MG_RTN_MODE = 7,
	MG_COORDINATES_SELECTION = 8,
	MG_PATH_CTRL_MODE = 9,
	MG_COUNT };

const int* modalGroup[MG_COUNT] = { modal0, modal1, modal2, modal3, modal4, modal5, modal6, modal7, modal8, modal9 };

#define GOT_X          0x0001
#define GOT_X_OFFSET   0x0002
#define GOT_Y          0x0004
#define GOT_Y_OFFSET   0x0008
#define GOT_Z          0x0010
#define GOT_Z_OFFSET   0x0020
#define GOT_DIMENSION  0x0040
#define GOT_TURN_COUNT 0x0080
#define GOT_SPINDLE    0x0100

void showDistanceInfo()
{
	t3DPoint actualPos;
	t3DPoint compPos;
	int x, y, z;

	printf("Distance information\n");

	getRawStepPos(&x, &y, &z);
	printf(" Raw Motor Step: X:%d Y:%d Z:%d \n", x, y, z);

	getCompPos(&compPos);
	getCurPos(&actualPos);

	printf(" Current Error: %.5f (Max err.:%.5f)\n",
		distance3D(theoricalPos, compPos), getMaxDistanceError());

	printf(" Theor.:%.4f,%.4f,%.4f\n", theoricalPos.x, theoricalPos.y, theoricalPos.z);
	printf(" Actual:%.4f,%.4f,%.4f\n", actualPos.x, actualPos.y, actualPos.z);
	printf(" Compsd:%.4f,%.4f,%.4f\n", compPos.x, compPos.y, compPos.z);
}


tStatus doGcode(char* cmd)
{
	static int activeCmd[MG_COUNT] = { 0, 170, 910, 930, 200, 400, 430, 980, 540, 610 }; // Default active commands
	char *pt;
	int i, j, n;
	double val;
	tMotion M = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	int loop;
	long gotWhat = 0;
	t3DPoint compPos;
	tStatus ret = retUnknownErr;

	// Debug
	int cmdInGroup[MG_COUNT];

	memset(cmdInGroup, 0x00, sizeof(cmdInGroup));

	// Ignore characters within brackets: Mach3 comments
	if((pt = strchr(cmd, '(')) != NULL )
	{
		char* cp = strchr(pt + 1, ')');
		if(cp)
		{
			while (++pt < cp) { *pt = ' '; }
		}
		else return retSyntaxError;
	}

	// Ignore what's right of semicolon: Siemens Comment
	if ((pt = strchr(cmd, ';')) != NULL)
	{
		while (*pt != 0) { *pt = ' '; pt++; }
	}


	if ((pt = strchr(cmd, 'X' )) != NULL )
	{
		if( sscanf_s( pt+1, "%lf", &val ) != 1 ) return retInvalidParam; 
		M.X = val;
		gotWhat |= GOT_X;
	}
	if(( pt = strchr( cmd, 'I' )) != NULL )
	{
		if( sscanf_s( pt+1, "%lf", &val ) != 1 ) return retInvalidParam; 
		M.I = val;
		gotWhat |= GOT_X_OFFSET;
	}
	if(( pt = strchr( cmd, 'Y' )) != NULL )
	{
		if( sscanf_s( pt+1, "%lf", &val ) != 1 ) return retInvalidParam; 
		M.Y = val;
		gotWhat |= GOT_Y;
	}
	if(( pt = strchr( cmd, 'J' )) != NULL )
	{
		if( sscanf_s( pt+1, "%lf", &val ) != 1 ) return retInvalidParam; 
		M.J = val;
		gotWhat |= GOT_Y_OFFSET;
	}
	if ((pt = strchr(cmd, 'Z' )) != NULL )
	{
		if( sscanf_s( pt+1, "%lf", &val ) != 1 ) return retInvalidParam; 
		M.Z = val;
		gotWhat |= GOT_Z;
	}
	if(( pt = strchr( cmd, 'K' )) != NULL )
	{
		if( sscanf_s( pt+1, "%lf", &val ) != 1 ) return retInvalidParam; 
		M.K = val;
		gotWhat |= GOT_Z_OFFSET;
	}
	if ((pt = strchr(cmd, 'D' )) != NULL )
	{
		if( sscanf_s( pt+1, "%lf", &val ) != 1 ) return retInvalidParam; 
		cutterRadius = val;
		gotWhat |= GOT_DIMENSION;
	}
	if ((pt = strchr(cmd, 'P' )) != NULL )
	{
		if( sscanf_s( pt+1, "%lf", &val ) != 1 ) return retInvalidParam; 
		M.P = val;
		gotWhat |= GOT_TURN_COUNT;
	}

  if(( pt = strchr( cmd, 'F' )) != NULL )
  {
    if( sscanf_s( pt+1, "%lf", &val ) != 1 ) return retInvalidParam; 
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
    else return retInvalidParam;
  }

  if(( pt = strchr( cmd, 'M' )) != NULL )
  {
    if( sscanf_s( pt+1, "%d", &i ) != 1 ) return retInvalidParam; 
    switch( i )
    {
      case 0 :
      case 1 :
	  case 2 :
        if( setSpindleState( 0 )) gotWhat |= GOT_SPINDLE;
        break;
      case 3 :
        if( setSpindleState( 3 )) gotWhat |= GOT_SPINDLE;
        break;
      default:
        return retInvalidParam;
    } 
  }
 
  pt = cmd;
  while(( pt = strchr( pt, 'G' )) != NULL )
  {
    pt++;
    if( sscanf_s( pt, "%lf", &val ) != 1 ) return retInvalidParam;
	n = (int)round(val*10);

    switch( n )
    {
    case CMD_G4 : // G4 : Dwell
      printf( "Dwell( %.3f sec).\n", M.P );
      dwell( (long)(M.P * 1000));
      break;

    case CMD_G10 : // G10 : Reset home position to current
      // Use the actual compensensated position to reset the theorical position
      // in order not to loose the sub-step residual error
      getCompPos( &compPos );
      theoricalPos.x -= compPos.x;
      theoricalPos.y -= compPos.y;
      theoricalPos.z -= compPos.z;

      resetMotorPosition( );
      break;

    case CMD_G30 : // G30 : Go back home
      printf( "G30-Return home (%.3f,%.3f,%.3f) at feed speed (%.1f)\n", 
        -theoricalPos.x,-theoricalPos.y,-theoricalPos.z, feedSpeed );
      // Move first in X-Y plane and then Z axis
      ret = linearRel( -theoricalPos.x, -theoricalPos.y, 0 );
      if( ret == 0 ) ret = linearRel( 0, 0, -theoricalPos.z );
      if( ret < 0 )
      {
        // Interrupted. Set the current theorical pos to the actual pos.
        getCurPos( &theoricalPos );
      }
      else
      {
        // We're home!!!
        theoricalPos.x = 0;
        theoricalPos.y = 0;
        theoricalPos.z = 0;
      }
      break;

    default :
	  for (j = 0; j<MG_COUNT; j++)
      {
        for(i=0; modalGroup[j][i]>=0; i++ )
        {
          //printf( "[%d,%d]=%d\n", j,i, modalGroup[j][i] );
          
		  if( n == modalGroup[j][i] )
          {
            cmdInGroup[j]++;
            // There can only be one command of each modal group per
            if( cmdInGroup[j] > 1 ) return retSyntaxError;
            // Remember the currently active for this group
            activeCmd[j] = n;
          }
        }
      }
    }
  }

  // Nothing actionalble on this line (no movement)
  if( gotWhat == 0 ) return retSuccess;

  if(( gotWhat & GOT_TURN_COUNT ) == 0 ) M.P = 1.0;

  // This gets the actual position minus the cutter compensation offset
  // and all moves buffered by the cutter compensation algorithm
  getCompPos( &compPos );

  // Checks if the distance between the current position and the theorical
  // position is less than the length of one motor step in all directions
  if( distance3D( theoricalPos, compPos ) > getMaxDistanceError( )) 
  {
    printf( "################ LOCATION ERROR ###############\n" );
    showDistanceInfo( );
  }
  
  //compPos = theoricalPos;   // So that cutter comp works
  //getCurPos( &compPos );  // So that precision remains

  // If active distance mode is absolute (G90)
  if (activeCmd[MG_DISTANCE_MODE] == CMD_G90)
  {
    // Update the theorical position for each Axis we got a new value and make
    // the coordinates relative to current position
    if( gotWhat & GOT_X )
    {
      theoricalPos.x = M.X;
      M.X = M.X - compPos.x;
    }
    if( gotWhat & GOT_Y )
    {
      theoricalPos.y = M.Y;
      M.Y = M.Y - compPos.y;
    }
    if( gotWhat & GOT_Z )
    {
      theoricalPos.z = M.Z;
      M.Z = M.Z - compPos.z;
    }
  }
  // If activce distance mode is relative (G91)
  else if (activeCmd[MG_DISTANCE_MODE] == CMD_G91)
  {
    // Update the new theorical tool position
    theoricalPos.x += M.X;
    theoricalPos.y += M.Y;
    theoricalPos.z += M.Z;
    // Calculate the relative motion as the distance between the current compensated position
    // and the new theorical position so that sub-step positioning error doesn't accumulate.
    M.X = theoricalPos.x - compPos.x;
    M.Y = theoricalPos.y - compPos.y;
    M.Z = theoricalPos.z - compPos.z;
  }
  else
  {
    printf ("ERROR : Invalid distance mode.\n"); 
    return retSyntaxError; 
  }

  // From this point on, everything is relative coordinates
  loop = 0;
  M.motion = activeCmd[MG_MOTION];
  ret = retSuccess;

  while (doCutterCompensation(&M, (activeCmd[MG_CUTTER_RADIUS_COMP] / 10), loop++))
  {
    // Depending on the active command in the motion group
    switch( M.motion )
    {
    case CMD_G0 : // Rapid positioning
      // printf( "G0-Rapid positioning (%.3f,%.3f,%.3f)\n", M.X,M.Y,M.Z );
      ret = rapidPosRel( M.X,M.Y,M.Z );
      break;

    case CMD_G1 : // Linear coordinated feed speed
      // printf( "G1-Linear movement (%.3f,%.3f,%.3f) at feed speed (%.1f)\n", M.X,M.Y,M.Z, feedSpeed );
      ret = linearRel( M.X,M.Y,M.Z );
      break;

    case CMD_G2 : // Arc in CW direction
      // printf( "G2-Arc CW movement (%.3f,%.3f,%.3f,%.3f,%.3f,%.3f) at feed speed (%.1f)\n", M.X,M.Y,M.Z,M.I,M.J,-M.P, feedSpeed );
      ret = arcInXYPlane( M.X,M.Y,M.Z,M.I,M.J,-M.P );
      break;

    case CMD_G3 : // Arc in CCW direction
      // printf( "G3-Arc CCW movement (%.3f,%.3f,%.3f,%.3f,%.3f,%.3f) at feed speed (%.1f)\n", M.X,M.Y,M.Z,M.I,M.J,M.P, feedSpeed );
      ret = arcInXYPlane( M.X,M.Y,M.Z,M.I,M.J,M.P );
      break;
    }
  }

  if( ret != retSuccess )
  {
    // Something went wrong during the motion. Reset the theorical position to
    // the current compensated position because theorical position is already
    // set at the end the movement.
    getCurPos( &theoricalPos );
 
    // Reset cutter compensation (if any)
    doGcode( "G40" );
  }

  return ret;
}

