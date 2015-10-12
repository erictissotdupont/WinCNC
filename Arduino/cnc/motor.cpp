#include <Arduino.h>
#include "helper.h"
#include "motor.h"

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

bool Motor::IsEnd( )
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
  int d = swap;
  if( s < 0 ) { 
    d = -swap; 
    s = -s; 
  }
  
  moveStep = 0;
  moveLength = s;
  moveDuration = t;
  
  decelTime = 0;
  decelDist = moveLength;
  
  if( s > 0 )
  {
    stepDuration = t / s;  
    if( curDir != d )
    {
      SetDirection( d );
      if( toggle )
      { 
        digitalWrite( stepPin, LOW );
        delay( 1 );
        digitalWrite( stepPin, HIGH );
        delay( 1 );
      }
      else delay( 2 );
    }
  }
  else
  {
    stepDuration = 0;
  }
  
  return TimeToNextMove( 0 );
}

#define RAMP_TIME    200000
#define MIN_SPEED    20
#define MAX_SPEED    600
#define IMP_TO_NS    24000


long Motor::TimeToNextMove( long t )
{
  if( moveStep >= moveLength ) return -1;
  
  if( moveDuration > 0 )
  {
    long w = ( moveStep + 1 ) * stepDuration;
    if( t >= w ) DebugPulse( 6 );
    return w - t;
  }
  else
  {
    double f;
    
    if( moveStep > decelDist )
    {
      f = decelTime + stepDuration - t;
      f = f / (double)RAMP_TIME;
    }
    else if( t < RAMP_TIME )
    {
      f = t;
      f = f / (double)RAMP_TIME;
      
      decelTime = t;
      decelDist = moveLength - moveStep;        
      stepDuration = t;    
    }
    else
    {
      decelTime = t;
      f = 1.0;
    }
    
    return IMP_TO_NS / ( MIN_SPEED + ( MAX_SPEED - MIN_SPEED ) * f );
  }
}

long Motor::Move( long t )
{
  if( endPin > 0 && digitalRead( endPin ) == HIGH )
  {
    int maxBackSteps = 1000;
    
    // Wait for 200ms
    delay( 500 );
    // Reverse the direction    
    SetDirection( -curDir );
    // And backout slowly
    while( maxBackSteps > 0 && digitalRead( endPin ) == HIGH )
    {
      delay( 5 );
      digitalWrite( stepPin, maxBackSteps & 1 ? LOW : HIGH );
      maxBackSteps--;
    }    
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

  moveStep++;
  return TimeToNextMove( t );
}

long Motor::Remain( )
{
  return moveLength - moveStep;
}

long Motor::Done( )
{
  return moveStep; 
}

long Motor::GetPos( )
{
  return curPos;
}

long Motor::FakeMove( long s )
{
  curPos += s;
}

