#include "CNC.h"
#include "helper.h"
#include "motor.h"
#include "LCD.h"
#include "UART.h"

unsigned int g_error;
int g_debug[MAX_DEBUG];

#define RAMP_TIME     400000L
#define MIN_SPEED     60L
#define MAX_SPEED     180L // 360L
#define IMP_TO_NS     24000L

#define MIN_STEP      ( IMP_TO_NS / MIN_SPEED )
#define MAX_STEP      ( IMP_TO_NS / MAX_SPEED )

#define AVG_STEP      (( MIN_STEP + MAX_STEP ) / 2 )
#define STEP_RANGE    ( MIN_STEP - MAX_STEP )
#define STEP_ACC      (( 256 * STEP_RANGE ) / (( RAMP_TIME / AVG_STEP ) - 1 ))

#define MAX_BACKSTEPS 2000

// Instantiation and configuration of the stepper motor controlers.
// (StepPin, DirPin, EndPin, Direction)
//
Motor X( 13, 12, A5,  1, ERROR_LIMIT_X );  // 13, 12, A5, 1
Motor Y( 11, 10, A4,  1, ERROR_LIMIT_Y );  // 11, 10, A4, 1
Motor Z(  3,  2, A3,  1, ERROR_LIMIT_Z );  //  3,  2, A3, 1

Motor Z1(SDA,SCL, A3,  1, ERROR_LIMIT_Z );  //  3,  2, A3, 1

uint32_t g_tSpent;     // Tracks time since last move to calculated accurate step delay
                       // Also used in LCD.cpp to track time spent updating the LCD.

Motor::Motor( uint8_t sp, uint8_t dp, uint8_t ep, int8_t sw, uint32_t lf )
{
  stepPin = sp;
  dirPin = dp;
  endPin = ep;
  swap = sw;
  limitFlag = lf;
}

void Motor::Reset( )
{
  curPos = 0;
  curDir = 0;
  toggle = 0;
  moveLength = 0;
  moveStep = 0;
  moveDuration = 0;

  pinMode( stepPin, OUTPUT );
  digitalWrite(stepPin, LOW );
  pinMode( dirPin, OUTPUT );
  digitalWrite(dirPin, LOW );
#ifdef TEST_LIMITS
  pinMode( endPin, INPUT );
#endif
}

bool Motor::IsAtTheEnd( )
{
#ifdef TEST_LIMITS
  return(( digitalRead( endPin ) == HIGH ) ? true : false );
#else
  return false;
#endif
}

void Motor::SetDirection( int8_t d )
{
  curDir = d;
  digitalWrite( dirPin, d < 0 ? LOW : HIGH );
}

long Motor::InitMove( int32_t s, uint32_t t )
{
  int32_t d = swap;
  
  // Backward direction
  if( s < 0 ) { 
    d = -swap;
    s = -s; 
  }
  
  moveStep = 0;
  moveLength = s;
  moveDuration = t;
  
  if( s > 0 )
  {
    // Are we changing direction
    if( curDir != d )
    {
      SetDirection( d );
      if( toggle )
      { 
        // Very important. This is to avoid missing a step when changing direction
        // when the stepper PIN is high. Toggle in order to make sure the motor
        // controler does not miss a step. 
        digitalWrite( stepPin, LOW );
        delay( 1 );
        digitalWrite( stepPin, HIGH );
        delay( 1 );
      }
      else delay( 2 );
    }    
    // Linear motion (G1)
    if( moveDuration )
    {
      stepDuration = t / s;
    }
    // Rapid positionning (G0)
    else
    {
      stepDecrement = 0; 
      stepDuration = IMP_TO_NS / MIN_SPEED;
      decelDist = moveLength;      
    }
  }
  else 
  {
    stepDuration = -1;
    // Return negative value to indicate no motion
    decelDist = 0;    
  }
  return stepDuration;
}

long Motor::Move( )
{
  // Test the end of rail sensor
#ifdef TEST_LIMITS
  while( digitalRead( endPin ) == HIGH )
  {
    int32_t bs = MAX_BACKSTEPS;

    // Check if this is not a glitch on the switch caused by
    // vibrations
    delay( 1 );
    if( digitalRead( endPin ) == LOW ) break;
    delay( 5 );
    if( digitalRead( endPin ) == LOW ) break;
    delay( 10 );
    if( digitalRead( endPin ) == LOW ) break;
 
    // Wait for 200ms
    delay( 200 );
    // Reverse the direction    
    SetDirection( -curDir );
    // And backout slowly
    while( bs > 0 && digitalRead( endPin ) == HIGH )
    {
      delay( 2 );
      digitalWrite( stepPin, bs & 1 ? LOW : HIGH );
      bs--;
    }
    if( bs > 200 ) bs = 200;
    while( bs > 0 )
    {
      delay( 2 );
      digitalWrite( stepPin, bs & 1 ? LOW : HIGH );
      bs--;
    }
    
    // Turn OFF the tool
    digitalWrite(TOOL_ON_REPLAY, LOW);
    
    g_error |= limitFlag;
    // Reset the move distance to force the stopping
    moveLength = 0;
    return -1;
  }
#endif
  
  curPos = curPos + curDir;
  if( toggle )
  {
    toggle = 0;
    digitalWrite( stepPin, LOW );
  }
  else
  {
    toggle = 1;
    digitalWrite( stepPin, HIGH );
  }
  
  // Move is complete. Return negarive value to indicate this.
  if( ++moveStep >= moveLength ) return -1;
  
  // Rapid positioning motion (G0)
  if( moveDuration == 0 )
  {
    if( moveStep > decelDist )
    {
      // Deceleration phase
      stepDecrement += STEP_ACC;
      stepDuration += (stepDecrement >> 8);
      stepDecrement &= 0xFF;
    }
    else if( stepDuration > MAX_STEP )
    {
      // Acceleration phase
      stepDecrement += STEP_ACC;
      stepDuration -= (stepDecrement >> 8);
      stepDecrement &= 0xFF;
      decelDist = moveLength - moveStep;        
    }
  }
  
  return stepDuration; 
}

long Motor::GetPos( )
{
  return curPos;
}

long Motor::FakeMove( int32_t s )
{
  curPos += s;
}

// Move the tool position by the specified # of steps for x,y,z directions.
// Duration of the motion determines the speed. d is in microseconds
void Motor_Move( int32_t x, int32_t y, int32_t z, uint32_t d )
{
  uint32_t s;
  uint32_t count = 0;
  int32_t tX,tY,tZ;

  // DebugPrint( "Move %d,%d,%d in %d)\n", x,y,z,d );

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
  uint32_t start = micros();
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
        UART_Task( );
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
  uint32_t finish = micros();
  g_debug[0] = (finish-start)/count;
  g_debug[1] = count;
#endif

}

void Motor_Init( )
{
  g_error = 0;
  for( int i=0; i<MAX_DEBUG; i++ ) g_debug[i] = 0;
  
  pinMode(TOOL_ON_REPLAY, OUTPUT);
  digitalWrite(TOOL_ON_REPLAY, LOW);
  
  X.Reset( );
  Y.Reset( );
  Z.Reset( );
}

