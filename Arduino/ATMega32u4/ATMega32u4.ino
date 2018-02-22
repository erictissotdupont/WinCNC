#include "motor.h"
#include "helper.h"
#include "LCD.h"
#include "CNC.h"

// ------------------------------------------------------------------------
// GLOBALS
// -------
// Instantiation and configuration of the stepper motor controlers.
// (StepPin, DirPin, EndPin, Direction)
//
Motor X( 13, 12, A5,  1 );  // 13, 12, A5, 1
Motor Y( 11, 10, A4,  1 );  // 11, 10, A4, 1
Motor Z(  3,  2, A3,  1 );  //  3,  2, A3, 1

unsigned int g_error = 0;
int g_debug[MAX_DEBUG];
unsigned long g_tSpent;

tMove g_fifo[MAX_FIFO_MOVE+1];
int g_fifoIn;
int g_fifoOut;

// This calculates time spent in the execution of motor step move routine
// and places the average loop time in uS in debug A and number of loops of 
// the last move in debug B status.
//#define TEST_TIME

inline void SendStatus( )
{
  Serial1.write( 'X' ); 
  Serial1.print( X.GetPos( ));
  Serial1.write( 'Y' );
  Serial1.print( Y.GetPos( ));
  Serial1.write( 'Z' );
  Serial1.print( Z.GetPos( ));
  Serial1.write( 'S' );
  Serial1.print( g_error );
  Serial1.write( '\n' );
}

inline unsigned int fifoCount( )
{
  if( g_fifoIn >= g_fifoOut )
    return g_fifoIn - g_fifoOut;
  else
    return g_fifoIn - g_fifoOut + MAX_FIFO_MOVE + 1;
}

bool Decode( )
{
  static int sign = 0;
  static long *pt = NULL;
  static tCommState state = COMM_IDLE;
  char c;

  c = Serial1.read( );

  if( c == -1 )
  {
    // Nothing to decode on UART, return false
    return false;
  }
  else
  {   
    if( state == COMM_IN_FRAME )
    {
      if( c == 'X' ) { pt = &g_fifo[g_fifoIn].x; sign = 1; }
      else if( c == 'Y' ) { pt = &g_fifo[g_fifoIn].y; sign = 1; }
      else if( c == 'Z' ) { pt = &g_fifo[g_fifoIn].z; sign = 1; }
      else if( c == 'D' ) { pt = &g_fifo[g_fifoIn].d; sign = 1; }
      else if( c == 'S' ) { pt = &g_fifo[g_fifoIn].s; sign = 1; *pt = 0; }
      else if( c == '-' ) { sign = -1; }
      else if( c >= '0' && c <= '9' )
      {
        if( pt )
        { 
          *pt = *pt * 10 + ( c - '0' ) * sign;
        }
        else
        { 
          g_error |= ERROR_NUMBER;
        }
      }    
      else if( c == '\n' )
      {
        if( g_error )
        {      
          Serial1.write( 'E' );
        }
        else
        {
          ++g_fifoIn;
          if( g_fifoIn > MAX_FIFO_MOVE ) g_fifoIn = 0;

          if( fifoCount( ) < MAX_FIFO_MOVE ) Serial1.write( 'O' );
        }
                      
        g_fifo[g_fifoIn].x = 0;
        g_fifo[g_fifoIn].y = 0;
        g_fifo[g_fifoIn].z = 0;
        g_fifo[g_fifoIn].d = 0;
        g_fifo[g_fifoIn].s = -1;
        pt = NULL;
        state = COMM_CONNECTED;
      }
      else
      {
        // This is a bit strict but if we receive any other symbol than
        // what should be in frame then we enter an error state.
        g_error |= ERROR_SYNTAX;
        g_debug[1] = c;
      }  
    }
    else if( state == COMM_CONNECTED )
    {
      // START OF FRAME
      // -------------- 
      if( c == '@' )
      {
        state = COMM_IN_FRAME;
      }
      // ORIGIN
      // ------
      else if( c == 'O' )
      {
        X.Reset( );
        Y.Reset( );
        Z.Reset( );
        SendStatus( );
      }
      // CLEAR
      // -----
      else if( c == 'C' )
      {
        g_error = 0;
        SendStatus( );
      }    
      // RESET
      // -----
      else if( c == 'R' )
      {
        state = COMM_RESET_1;
      }
      // POSITION
      // --------
      else if( c == 'P' )
      {
        SendStatus( );
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
      else if ( c == '\n' )
      {
        // Ignore new lines symbols
      }
      else
      {
        // This symbol is not valid in this state
        g_error |= ERROR_COMM;
        g_debug[0] = c;
      }    
    }
    else if( state == COMM_IDLE )
    {
      if( c == 'R' ) state = COMM_RESET_1;
    }
    else if( state == COMM_RESET_1 )
    {
      if( c == 'S' ) 
        state = COMM_RESET_2;
      else
        state = COMM_IDLE;
    }
    else if( state == COMM_RESET_2 )
    {
      if( c == 'T' )
      {
        Serial1.print( "HLO\n" );
        g_fifoIn = 0;
        g_fifoOut = 0;
        g_fifo[g_fifoIn].x = 0;
        g_fifo[g_fifoIn].y = 0;
        g_fifo[g_fifoIn].z = 0;
        g_fifo[g_fifoIn].d = 0;
        g_fifo[g_fifoIn].s = -1;
        pt = NULL;
        state = COMM_CONNECTED;
      }
      else
      {
        state = COMM_IDLE;
      }
    }
    return true;
  }
}

// ------------------------------------------------------------------------

void setup() {
  
  g_error = 0;
  for( int i=0; i<MAX_DEBUG; i++ ) g_debug[i] = 0;
  
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

  g_fifoIn = 0;
  g_fifoOut = 0;
}


void Move( )
{
  // While there are commands in the FIFO
  while( g_fifoIn != g_fifoOut )
  {
    // Update the state of the spindle if needed
    if( g_fifo[g_fifoOut].s >= 0 )
    {
      digitalWrite(TOOL_ON_REPLAY, g_fifo[g_fifoOut].s ? HIGH : LOW);
    }

    // Perform the move
    Move( g_fifo[g_fifoOut].x,
          g_fifo[g_fifoOut].y,
          g_fifo[g_fifoOut].z,
          g_fifo[g_fifoOut].d );

    // If the FIFO was full, send the ACK to the AR9331
    if( fifoCount( ) == MAX_FIFO_MOVE ) Serial1.write( 'O' );
    // Move the FIFO out pointer
    if( ++g_fifoOut > MAX_FIFO_MOVE ) g_fifoOut = 0;
  }
}

// Move the tool position by the specified # of steps for x,y,z directions.
// Duration of the motion determines the speed. d is in microseconds
void Move( long x, long y, long z, long d )
{
  long s;
  unsigned int count = 0;
  long tX,tY,tZ;
  
  // If in error state, dot not move
  if( g_error != 0 )
  {
    return;
  }
  
  // No movement means dwell (GCode "P")
  if( x==0 && y==0 && z==0 )
  {
    // d is in microseconds
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
    // Finds which of the non negative wait times is the shortest.
    s = minOf3( tX, tY, tZ );
              
    // Remove the wait time from each axis. At least one should become zero.
    tX = tX - s;
    tY = tY - s;
    tZ = tZ - s;
    
#ifdef TEST_TIME 
    // This skip the delay and allow to mesure the processing time of the move
    // routine (calculated at the end)
    s = 0;
#endif
 
    // If we have at least 150uS to wait, perform background
    // update tasks
    if( s > 150 )
    {
      // Alternate between LCD and UART
      if( count++ & 1 )
      {
        // Refresh the LCD screen (if enough time)
        LCD_UpdateTask( s - 150 );
      }
      else
      {
        // Read from UART and decode (one char at a time)
        Decode( );
      }
    }
    
    // Calculate how much time we spent processing since the last move pulse
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
    // Counts how many times move was late for making a motor pulse
    else g_debug[3]++;
#endif

    // For each axis where the time to move has been reached, move.
    // Save the duration to the next move or a negative value if the
    // axis has eached the end position.
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

// ------------------------------------------------------------------------

void loop() 
{
  // Process UART commands (until buffer is empty)
  while( Decode( ));

  // Perform the move
  Move( );

  // Refresh the LCD outside of the move loop
  LCD_UpdateTask( 1000 );
  
  // Scan the keyboard
  LCD_ButtonTask( );
  
  if( g_error )
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

