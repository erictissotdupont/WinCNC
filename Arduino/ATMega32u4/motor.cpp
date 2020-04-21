#include "CNC.h"
#include "helper.h"
#include "motor.h"
#include "LCD.h"
#include "UART.h"

unsigned int g_error;
int g_debug[MAX_DEBUG];

#define RAMP_SHIFT    20 // 19=524ms
#define RAMP_TIME     (1L << RAMP_SHIFT)
#define MIN_SPEED     30L
#define MAX_SPEED     500L

// Speed is in inch by minute, hence the 30M micro seconds
#define SPEED_TO_HALF_STEP( s ) ( 30000000L / ( stepByInch * s ))

// Calculate the ramp for G0 accel / decel phases 
#define HALF_STEP_FROM_RAMP( t ) ( minSpeedHalfStep - ((( minSpeedHalfStep - maxSpeedHalfStep ) * t ) >> RAMP_SHIFT))

#define MAX_BACKSTEPS  2000

// Instantiation and configuration of the stepper motor controlers.
//-----------------------------------------------------------------
//      Step,Dir,End,Swap,    LimitFlag,      StepByInch
Motor X ( 11, 10, -1,  1, ERROR_LIMIT_X, 1.0f/X_AXIS_RES );
Motor Y ( 13, 12, -1,  1, ERROR_LIMIT_Y, 1.0f/Y_AXIS_RES );
Motor ZL( A3, A2, -1,  1, ERROR_LIMIT_Z, 1.0f/Z_AXIS_RES );
Motor ZR( A5, A4, -1,  1, ERROR_LIMIT_Z, 1.0f/Z_AXIS_RES );

unsigned long g_tSpent;     // Tracks time since last move to calculated accurate step delay
                            // Also used in LCD.cpp to track time spent updating the LCD.

unsigned long g_MoveStart;  // Time when the current move was started (uS)


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

long Motor::InitMove( long s, long t )
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
      halfStepDuration = t / ( moveLength * 2 );
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
    // Return negative value to indicate no motion
    halfStepDuration = -1;
  }
  
  return halfStepDuration;
}

long Motor::Move( )
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
    // Return negative value to indicate this.
    halfStepDuration = -1;
  }
  // Zero duration means rapid positioning motion (G0).
  else if( moveDuration == 0 )
  {
    // Time since movement started
    long t = micros( ) - g_MoveStart;
    
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
  }
  return halfStepDuration; 
}

long Motor::GetPos( )
{
  return curPos;
}

long Motor::FakeMove( long s )
{
  curPos += s;
}

// Move the tool position by the specified # of steps for x,y,z directions.
// Duration of the motion determines the speed. d is in microseconds
void Motor_Move( long x, long y, long z, long d )
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
    if( d < 1000 )
    {
      delayMicroseconds( d );
    }
    else
    {
      delay( d / 1000 );
    }
    return;
  }

  // Capture when the motion started (in uS)
  g_MoveStart = micros();
  
  // This calculates the interval between steps for each axis and returns the time
  // to wait until the first move needs to occur (-1 if no move necessary).
  tX = X.InitMove( x, d );
  tY = Y.InitMove( y, d );
  tZ = ZL.InitMove( z, d );
       ZR.InitMove( z, d );
  
#ifdef TEST_TIME
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
    if( tZ == 0 ) { tZ = ZL.Move( ); ZR.Move( ); }
    
  } while(( tX > 0 ) || ( tY > 0 ) || ( tZ > 0 ));
  
#ifdef TEST_TIME
  long finish = micros();
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
  ZL.Reset( );
  ZR.Reset( );
}
