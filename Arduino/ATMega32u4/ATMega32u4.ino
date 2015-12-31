#include "motor.h"
#include "helper.h"
#include "LCD.h"
#include "CNC.h"

// ------------------------------------------------------------------------

// StepPin, DirPin, EndPin, Direction
Motor X( 13, 12, A5,  1 );  // 13, 12, A5, 1
Motor Y( 11, 10, A4,  1 );  // 11, 10, A4, 1
Motor Z(  3,  2, A3,  1 );  //  3,  2, A3, 1

unsigned int commandCount = 0;
unsigned int error = 0;

#define MAX_DEBUG 4
int g_debug[MAX_DEBUG];

// This calculates time spent in the execution of motor step move routine
// and places the average loop time in uS in debug A and number of loops of 
// the last move in debug B status.
//#define TEST_TIME

// ------------------------------------------------------------------------
// Move the tool position by the specified # of steps by x,y,z
// Duration of the motion determines the speed. d is in microseconds
//
void Move( long x, long y, long z, long d )
{
  unsigned long s,tSpent;
  unsigned long t = 0;
  long tX,tY,tZ;
  
  // If in error state, no move
  if( error )
  {
    return;
  }
  
  // No movement means dwell
  if( x==0 && y==0 && z==0 )
  {
    delay( d / 1000 );
    return;
  }
  
  // This calculates the interval between steps for each axis and returns the time
  // to wait until the first move needs to occur (-1 if no move necessary).
  tSpent = micros();
  tX = X.InitMove( x, d );
  tY = Y.InitMove( y, d );
  tZ = Z.InitMove( z, d ); 

#ifdef TEST_TIME  
  long count = 0;
  long start = micros();
#endif
  
  do
  {
    // Finds which of the non negative wait time is the shortest
    // 4.2 uS
    s = minOf3( tX, tY, tZ );
              
    // Accumulate the sleep time to out global time variable
    t = t + s;

    // Remove the sleep time from each axis
    tX = tX - s;
    tY = tY - s;
    tZ = tZ - s;
    
#ifdef TEST_TIME 
    s = 0;
    count++;
#endif
 
    // If we have at least 150uS to wait   
    if( s > 150 )
    {
      // Refresh the screen (if enough time)
      LCD_UpdateTask( s - 150 );
    }
    
    // Calculate how much time we spent for processing
    tSpent = micros() - tSpent;
    
    // If we still have time to wait
    if( s > tSpent )
    {
      s = s - tSpent;      
      // When sleeping more than 16388 uS, documentation says don't use delayMicroseconds.
      if( s < 16000 )
      {
        if( s ) delayMicroseconds( s );
      }
      else 
      {
        delay( s / 1000 );
        delayMicroseconds( s % 1000 );
      }
    }

    // For each axis where the time to move has been reached, move.
     tSpent = micros();
    if( tX == 0 ) X.Move( );
    if( tY == 0 ) Y.Move( );    
    if( tZ == 0 ) Z.Move( );
    
    // Get the duration to the next move (based on t) or
    // a negative value if the axis has eached the end position.
    if( tX == 0 ) tX = X.TimeToNextMove( t );
    if( tY == 0 ) tY = Y.TimeToNextMove( t );
    if( tZ == 0 ) tZ = Z.TimeToNextMove( t );
    
  } while(( tX > 0 ) || ( tY > 0 ) || ( tZ > 0 ));
  
#ifdef TEST_TIME
  long finish = micros();
  g_debug[0] = (finish-start)/count;
  g_debug[1] = count;
#endif

  // Check proper distance was achieved for debug purpose.
  /*
  if( X.Remain( ) != 0 || Y.Remain( ) != 0 || Z.Remain( ) != 0 )
  {
    DebugPulse( 5 );
  }
  */
}

void Decode( char c )
{
  static long x = 0;
  static long y = 0;
  static long z = 0;
  static long d = 0;   /* Duration in micro seconds. 0 means max speed (G0) */
  static long s = -1;  /* Spindle state : -1 = No change, 0-1 = OFF-ON */
  static long i = 0;   /* Packet index */
  static int sign = 0;
  static int SOF = 0;
  static long *pt = NULL;
  static long nextIndex = 0;
  static int reset = 3;
  
  if( reset == 0 )
  {   
    if( c == '@' )
    {
      if( SOF == 0 ) SOF = 1;
      else
      {
        error |= ERROR_SYNTAX;
      }
    }
    else if( SOF )
    {
      if( c == 'X' ) { pt = &x; sign = 1; }
      else if( c == 'Y' ) { pt = &y; sign = 1; }
      else if( c == 'Z' ) { pt = &z; sign = 1; }
      else if( c == 'D' ) { pt = &d; sign = 1; }
      else if( c == 'S' ) { pt = &s; sign = 1; s = 0; }
      else if( c == '-' ) { sign = -1; }
      else if( c >= '0' && c <= '9' )
      {
        if( pt ) *pt = *pt * 10 + ( c - '0' ) * sign;
        else error |= ERROR_NUMBER;
      }    
      else if( c == '\n' )
      {        
        Serial1.write( error ? 'E' : 'O' );
        
        if( s >= 0 )
        {
          digitalWrite(TOOL_ON_REPLAY, s ? HIGH : LOW);
        }
 
        if( x != 0 || y != 0 || z != 0 || d != 0 )
        {
          commandCount += 1;
          Move( x,y,z,d );
        }
            
        x=0;
        y=0;
        z=0;
        d=0;
        i=0;
        s = -1;
        pt = NULL;
        SOF = 0;
        nextIndex++;
      }
      else
      {
        error |= ERROR_SYNTAX;
      }
    }
    else if( c == 'R' )
    {
       reset = 2;
       Serial.write( 'O' ); 
    }
    else if( c == 'S' )
    {
      Serial1.print( "X" );
      Serial1.print( X.GetPos( ));
      Serial1.print( "Y" );
      Serial1.print( Y.GetPos( ));
      Serial1.print( "Z" );
      Serial1.print( Z.GetPos( ));
      Serial1.print( "S" );
      Serial1.print( error );
      
      Serial1.print( "A" );
      Serial1.print( g_debug[0] );
      Serial1.print( "B" );
      Serial1.print( g_debug[1] );
      Serial1.print( "C" );
      Serial1.print( g_debug[2] );
      Serial1.print( "D" );
      Serial1.print( g_debug[3] );
      
      Serial1.print( "\n" );
    }
    else if( c != 0 && c != '\n' )
    {
       Serial.write( 'E' ); 
    }
  }
  else
  {    
    if( c == 'R' && reset == 3 ) reset = 2;
    else if( c == 'S' && reset == 2 ) reset = 1;
    else if( c == 'T' && reset == 1 )
    {
      Serial1.print( "HLO\n" );
      reset = 0;
      error = 0;
      x=0;
      y=0;
      z=0;
      d=0;
      s=-1;
      pt = NULL;
      X.Reset( );
      Y.Reset( );
      Z.Reset( );
      SOF = 0;
      nextIndex = 0;
    }
    else
    {     
      reset = 3;
    }
  }
}

// ------------------------------------------------------------------------

void setup() {
  int i;
  
  error = 0;
  for( i=0; i<MAX_DEBUG; i++ ) g_debug[i] = 0;
  
  delay( 250 );
  
  LCD_Init( );
  LCD_Clear( );
  delay( 250 );
  
  Serial1.begin(500000);
  
  pinMode(TOOL_ON_REPLAY, OUTPUT);
  digitalWrite(TOOL_ON_REPLAY, LOW);

  X.Reset( );
  Y.Reset( );
  Z.Reset( );
}

// ------------------------------------------------------------------------

void loop() 
{
  while(Serial1.available( ) > 0 )
  {
    Decode(Serial1.read( ));
  }
  // Refresh the LCD outside of the move loop
  LCD_UpdateTask( 1000 );
  // Scan the keyboard
  LCD_ButtonTask( );
  if( error )
  {
    LCD_SetStatus( "ERR", 0 );
  }
  else
  {
    LCD_SetStatus( X.IsAtTheEnd( ) ? "X" : " ", 0 );
    LCD_SetStatus( Y.IsAtTheEnd( ) ? "Y" : " ", 1 );
    LCD_SetStatus( Z.IsAtTheEnd( ) ? "Z" : " ", 2 );
  }
}

