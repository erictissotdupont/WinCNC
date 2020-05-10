#include "CNC.h"
#include "helper.h"
#include "motor.h"
#include "LCD.h"
#include "UART.h"

unsigned int g_error;
int g_debug[MAX_DEBUG];

#define RAMP_SHIFT    19 // 19=524ms
#define RAMP_TIME     (1L << RAMP_SHIFT)
#define MIN_SPEED     30L
#define MAX_SPEED     250L
#define NO_MOVE_TIME  0xFFFFFFFF
// Speed is in inch by minute, hence the 30M micro seconds
#define SPEED_TO_HALF_STEP( s ) ( 30000000L / ( stepByInch * s ))

// Calculate the ramp for G0 accel / decel phases 
#define HALF_STEP_FROM_RAMP( t ) ( minSpeedHalfStep - ((( minSpeedHalfStep - maxSpeedHalfStep ) * t ) >> RAMP_SHIFT))

#define MAX_BACKSTEPS  2000

// Instantiation and configuration of the stepper motor controlers.
//-----------------------------------------------------------------
//      Step,Dir,End,Swap,    LimitFlag,      StepByInch
Motor X ( 11, 10, -1,  0, ERROR_LIMIT_X, 1.0f/X_AXIS_RES );
Motor Y ( 13, 12, -1,  1, ERROR_LIMIT_Y, 1.0f/Y_AXIS_RES );
Motor ZL( A3, A2, -1,  1, ERROR_LIMIT_Z, 1.0f/Z_AXIS_RES );
Motor ZR( A5, A4, -1,  1, ERROR_LIMIT_Z, 1.0f/Z_AXIS_RES );

unsigned long g_MoveStart;  // Time when the current move was started (uS)

// For debug purpose, measures the time spent sleeping
unsigned long g_timeSleepingUs = 0;
unsigned long g_missedStepCount = 0;
long g_maxErrorAllowed;

Motor::Motor( int sp, int dp, int ep, int rd, unsigned int lf, unsigned long sbi )
{
  stepPin = sp;
  dirPin = dp;
  endPin = ep;
  reverseDir = rd;
  limitFlag = lf;
  stepByInch = sbi;
  minSpeedHalfStep = SPEED_TO_HALF_STEP( MIN_SPEED );
  maxSpeedHalfStep = SPEED_TO_HALF_STEP( MAX_SPEED );
}

void Motor::Reset( )
{
  curPos = 0;
  curDir = 0;
  moveLength = 0;
  moveStep = 0;
  moveDuration = 0;
  toggle = 0;
  g_missedStepCount = 0;

  pinMode( stepPin, OUTPUT );
  digitalWrite(stepPin, HIGH );
  pinMode( dirPin, OUTPUT );
  digitalWrite(dirPin, LOW );
  if( endPin >= 0 ) {
    pinMode( endPin, INPUT );
  }
}

bool Motor::IsAtTheEnd( )
{
  if( endPin >= 0 ) 
    return(( digitalRead( endPin ) == HIGH ) ? true : false );
  else
    return false;
}

void Motor::SetDirection( int d )
{
  if( !reverseDir )
  {
    digitalWrite( dirPin, d < 0 ? LOW : HIGH );
  }
  else
  {
    digitalWrite( dirPin, d < 0 ? HIGH : LOW );
  }
  
  if( curDir != d )
  {
    curDir = d;
    // Give some time for the controller to capture the new
    // direction pin level. Otherwise the next move may
    // step in the wrong direction for its first steps.
    delayMicroseconds( 100 );
  }
}

unsigned long Motor::InitMove( long s, long t )
{
  long ret = -1;
  int d = 1;
  
  // Backward direction
  if( s < 0 ) 
  { 
    d = -1;
    s = -s; 
  }
  
  moveStep = 0;
  moveLength = s;
  halfStepCount = s * 2;
  moveDuration = t;
  toggle = 0;
  
  if( s > 0 )
  {
    SetDirection( d );
    // Linear motion (G1)
    if( moveDuration )
    {
      // Divide the step duration by half because the ::Move( )
      // function is called for every half period
      halfStepDuration = t / halfStepCount;
      halfStepModulo = t % halfStepCount;
      halfStepAcc = halfStepModulo;
    }
    // Rapid positionning (G0)
    else
    {
      halfStepDuration = minSpeedHalfStep;
      decelDist = moveLength;
      decelStart = 0;
    }
  }
  else 
  {
    return NO_MOVE_TIME;
  }
  nextHalfStepTime = halfStepDuration;
  return halfStepDuration;
}

inline unsigned long timeSinceMoveStarted( )
{
  unsigned long now = micros( );
  // Calculate time since the start of the move
  if( g_MoveStart <= now )
    return( now - g_MoveStart );
  else
    // Rollover of the 32bit microsecond timer (every 71min)
    return( NO_MOVE_TIME - g_MoveStart + now );
}

unsigned long Motor::Move( )
{

#if 0
  // Test the end of rail sensor
  while( digitalRead( endPin ) == HIGH )
  {
    long bs = MAX_BACKSTEPS;
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
    digitalWrite( stepPin, LOW );
    return -1;
  }
#endif

  // Toggle the step input of the controller
  if( !toggle )
  {
    // First half. Set the step signal high
    toggle = 1;
    digitalWrite( stepPin, HIGH );
  }
  else
  {
    // Second half. Set the step signal low
    toggle = 0;
    digitalWrite( stepPin, LOW );
    // Count the step and update the position
    moveStep++;
    curPos = curPos + curDir;
  }

  // Move is complete.
  if( moveStep >= moveLength )
  {
    return NO_MOVE_TIME;
  }
  // Movement with duration means linear motion (G1).
  else if( moveDuration != 0 )
  {
    nextHalfStepTime += halfStepDuration;
    halfStepAcc += halfStepModulo;
    if( halfStepAcc >= halfStepCount )
    {
      halfStepAcc -= halfStepCount;
      nextHalfStepTime++;
    }
  }
  // Zero duration means rapid positioning motion (G0).
  else
  {
    // Time since movement started
    long t = timeSinceMoveStarted( );
    
    // Deceleration phase
    if( moveStep >= decelDist )
    {
      if( decelStart == 0 )
      {
        // Capture the time when the deceleration started
        decelStart = t;
      }
      // Time spent decelerating 
      t = t - decelStart;
      if( t < decelTime )
      {
        // Expected time decelerating
        t = decelTime - t;
      }
      else
      {
        t = 0;
      }
      halfStepDuration = HALF_STEP_FROM_RAMP( t );
    }
    // Acceleration phase
    else if( t <= RAMP_TIME )
    {
      halfStepDuration = HALF_STEP_FROM_RAMP( t );
      
      // Save the distance and time when deceleration should end
      // If the movement is so short that full speed can't be
      // reached, the previous test will pass and deceleration
      // will start before the movement has reached full speed.
      decelDist = moveLength - moveStep;
      decelTime = t; 
    }
    nextHalfStepTime += halfStepDuration;
  }  
  return nextHalfStepTime; 
}

long Motor::GetPos( )
{
  return curPos;
}

long Motor::FakeMove( long s )
{
  curPos += s;
}

inline void WaitTillItsTime( unsigned long timeToMove )
{
  long s;
  static int count = 0;

  // How much time until the move?
  s = timeToMove - timeSinceMoveStarted( );

  // Are we late?
  if( s <= 0 )
  {
    // How badly (allowed error depends on how many directions are
    // moving at the same time in this motion).
    if( s < g_maxErrorAllowed ) g_missedStepCount++;
    return;
  }

  // This is to calculate the CPU load
  // (time spent doing background task and sleeping vs actual movement)
  g_timeSleepingUs += s;
  
  // LCD and UART tasks are designed to never take longer than 25uS
  // Verify this by running with "DISPLAY_TASK_TIME" enabled.
  while( s > 30 )
  {
    // Alternate between LCD and UART
    if( count++ & 1 )
    {
      // Refresh the LCD screen (if enough time)
      // Most LCD refresh commands take 30uS or less
      LCD_UpdateTask( s - 5 );
    }
    else
    {
      // Read from UART and decode (one char at a time)
      // Except for the G10 (Reset) command, all the UART processing is done
      // in less than 20uS
      UART_Task( );
    }
    
    // Check how much time left until the next movement
    s = timeToMove - timeSinceMoveStarted( );
  }

  // Do we still need to wait for a (short) while?
  if( s > 0 )
  {
    delayMicroseconds( s );
  }
  else
  {
    // Shouldnt be late here...
    if( s < 0 ) g_missedStepCount++;
  }
}

// Measures the Min/Max/Average of the Move( ) function
//#define MEASURE_MOVE

// Move the tool position by the specified # of steps for x,y,z directions.
// Duration of the motion determines the speed. d is in microseconds
void Motor_Move( long x, long y, long z, long d )
{
#ifdef MEASURE_MOVE
  long maxMoveTime = 0;
  long minMoveTime = 10000;
  long avgMoveTime = 0;
  long moveCount = 0;
#endif
  unsigned long prev;
  unsigned long tX,tY,tZ;
  
  // If in error state, dot not move
  if( g_error != 0 )
  {
    return;
  }

  // Capture when the motion started (in uS)
  g_MoveStart = micros();
  
  // No movement means dwell (GCode "P")
  if( x==0 && y==0 && z==0 )
  {
    // Just wait...
    WaitTillItsTime( d );
    return;
  }

  // This is to verify that pulses to the motors are made with
  // a reasonnable precision. The error allowed increases when
  // the number of axis moving at the same time increases.
  g_maxErrorAllowed = -3;
  if( x ) g_maxErrorAllowed -= 5;
  if( y ) g_maxErrorAllowed -= 5;
  // Z is special because it moves 2 motors at the same time
  if( z ) g_maxErrorAllowed -= 10;

  // This calculates the interval between steps for each axis and returns the time
  // to wait until the first move needs to occur (0 if no move necessary).
  tX = X.InitMove( x, d );
  tY = Y.InitMove( y, d );
  tZ = ZL.InitMove( z, d );
       ZR.InitMove( z, d );

  do
  {
    // Check which axis is the next one to be stepped
    if(( tX < tY ) && ( tX < tZ ))
    {
      // X is next move
      WaitTillItsTime( tX );
#ifdef MEASURE_MOVE
      unsigned long t = micros( );
#endif
      noInterrupts( );
      prev = tX + 3;
      tX = X.Move( );
      if( tY < prev ) tY = Y.Move( );
      if( tZ < prev ) { tZ = ZL.Move( ); ZR.Move( ); }
      interrupts( );
#ifdef MEASURE_MOVE
      t = micros( ) - t;
      if( t > maxMoveTime ) maxMoveTime = t;
      if( t < minMoveTime ) minMoveTime = t;
      avgMoveTime += t;
      moveCount++;
#endif
    }
    else if( tY < tZ )
    {
      // Y is next move
      WaitTillItsTime( tY );
      noInterrupts( );
      prev = tY + 3;
      tY = Y.Move( );
      if( tX < prev ) tX = X.Move( );
      if( tZ < prev ) { tZ = ZL.Move( ); ZR.Move( ); }
      interrupts( );
    }
    else if( tZ != NO_MOVE_TIME )
    {
      // Z is next move
      WaitTillItsTime( tZ );
      noInterrupts( );
      prev = tZ + 5;
      tZ = ZL.Move( );
      ZR.Move( );
      if( tX < prev ) tX = X.Move( );
      if( tY < prev ) tY = Y.Move( );
      interrupts( ); 
    }
    else
    {
      // No movement, leave
      break;
    }
  } while( 1 );
  
#ifdef MEASURE_MOVE
  char str[10];
  sprintf( str, "%d %d %d", minMoveTime, maxMoveTime, avgMoveTime / moveCount );
  LCD_SetStatus( str, 0 );
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
  ZL.Reset( );
  ZR.Reset( );
}
