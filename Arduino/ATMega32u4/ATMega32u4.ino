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

unsigned long g_tSpent;

// ------------------------------------------------------------------------
// Move the tool position by the specified # of steps by x,y,z
// Duration of the motion determines the speed. d is in microseconds
//
void Move( long x, long y, long z, long d )
{
  long s;
  unsigned int count = 0;
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
  tX = X.InitMove( x, d );
  tY = Y.InitMove( y, d );
  tZ = Z.InitMove( z, d ); 
  
#ifdef TEST_TIME
  long start = micros();
  g_debug[2] = start-g_tSpent;
#endif
  
  do
  {
    // Finds which of the non negative wait time is the shortest
    s = minOf3( tX, tY, tZ );
              
    // Remove the sleep time from each axis
    tX = tX - s;
    tY = tY - s;
    tZ = tZ - s;
    
#ifdef TEST_TIME 
    // This skip the delay and allow to mesure the processing time of the move
    // routine (calculated at the end)
    s = 0;
#endif
 
    // If we have at least 150uS to wait AND it's not the first loop
    if( s > 150 && count )
    {
      // Refresh the screen (if enough time)
      LCD_UpdateTask( s - 150 );
    }
    ++count;
    
    // Calculate how much time we spent processing
    g_tSpent = micros() - g_tSpent;
    
    // If we still have time to wait
    if( s > g_tSpent )
    {
      s = s - g_tSpent;      
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
#ifndef TEST_TIME
    // Counts how many times this move was late for making a motot pulse
    else g_debug[3]++;
#endif

    // For each axis where the time to move has been reached, move.
    // Get the duration to the next move (based on t) or
    // a negative value if the axis has eached the end position.
    g_tSpent = micros();
    if( tX == 0 ) tX = X.Move( );
    if( tY == 0 ) tY = Y.Move( );    
    if( tZ == 0 ) tZ = Z.Move( );
    
  } while(( tX > 0 ) || ( tY > 0 ) || ( tZ > 0 ));
  
#ifdef TEST_TIME
  long finish = micros();
  g_debug[0] = (finish-start)/count;
  g_debug[1] = count;
#endif

}

void Decode( char c )
{
  static long x = 0;
  static long y = 0;
  static long z = 0;
  static long d = 0;   /* Duration in micro seconds. 0 means max speed (G0) */
  static long s = -1;  /* Spindle state : -1 = No change, 0-1 = OFF-ON */
  static int sign = 0;
  static int SOF = 0;
  static long *pt = NULL;
  static long nextIndex = 0;
  static int reset = 3;
  
  if( reset == 0 )
  {
    // START OF FRAME
    // -------------- 
    if( c == '@' )
    {
      if( SOF == 0 )
      {
        // Save the time we received the first character of a frame so that we can take the time to 
        // receive it into account in the delay until the first move pulse. Between 250yuS to 450uS
        // depending on the # of arguments passed to the move function.
        g_tSpent = micros( );
        SOF = 1;
      }
      else
      {
        error |= ERROR_SYNTAX;
      }
    }
    // COMMAND FRAME
    // -------------
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
    // RESET
    // -----
    else if( c == 'R' )
    {
       reset = 2;
       Serial.write( 'O' ); 
    }
    // STATUS
    // ------
    else if( c == 'S' )
    {
      Serial1.write( 'X' ); 
      Serial1.print( X.GetPos( ));
      Serial1.write( 'Y' );
      Serial1.print( Y.GetPos( ));
      Serial1.write( 'Z' );
      Serial1.print( Z.GetPos( ));
      Serial1.write( 'S' );
      Serial1.print( error );
      Serial1.write( '\n' );
    }
    // DEBUG
    // -----
    else if( c == 'D' )
    {
      int i;      
      for( i=0; i<MAX_DEBUG; i++ )
      {      
        Serial1.write( 'a' + i );
        Serial1.print( g_debug[i] );
      }     
      Serial1.write( '\n' );

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

