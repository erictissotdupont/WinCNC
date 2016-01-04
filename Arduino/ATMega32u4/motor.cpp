#include <Arduino.h>
#include "CNC.h"
#include "helper.h"
#include "motor.h"

extern unsigned int g_error;
extern int g_debug[MAX_DEBUG];

#define RAMP_TIME     400000L
#define MIN_SPEED     60L
#define MAX_SPEED     360L
#define IMP_TO_NS     24000L

#define MIN_STEP      ( IMP_TO_NS / MIN_SPEED )
#define MAX_STEP      ( IMP_TO_NS / MAX_SPEED )

#define AVG_STEP      (( MIN_STEP + MAX_STEP ) / 2 )
#define STEP_RANGE    ( MIN_STEP - MAX_STEP )
#define STEP_ACC      (( 256 * STEP_RANGE ) / (( RAMP_TIME / AVG_STEP ) - 1 ))

#define MAX_BACKSTEPS 2000

Motor::Motor( int sp, int dp, int ep, int s )
{
  stepPin = sp;
  dirPin = dp;
  endPin = ep;
  swap = s;
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
  if( endPin >= 0 ) {
    pinMode( endPin, INPUT );
  }
}

bool Motor::IsAtTheEnd( )
{
  return(( digitalRead( endPin ) == HIGH ) ? true : false );
}

void Motor::SetDirection( int d )
{
  curDir = d;
  digitalWrite( dirPin, d < 0 ? LOW : HIGH );
}

long Motor::InitMove( long s, long t )
{
  long ret = -1;
  int d = swap;
  
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
    
    g_error |= ERROR_LIMIT;
    // Reset the move distance to force the stopping
    moveLength = 0;
    return -1;
  }
  
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

long Motor::FakeMove( long s )
{
  curPos += s;
}

