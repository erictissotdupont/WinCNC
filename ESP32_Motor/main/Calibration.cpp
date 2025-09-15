
extern "C" {
  #include "CNC.h"

  #include "driver/gpio.h"
  #include "driver/gptimer.h"
  #include "esp_timer.h"

  #include "UDP.h"  
  #include "Events.h"
  #include "Motor.h"
}
#include "Motor.hpp"

#define CAL_STEP_SPEED   1000
#define CAL_PAUSE        50000
#define CAL_STALL        100
#define CAL_ERROR_TRLD   3

extern uint32_t g_limitState;

void Motor::CalibrateStart( uint64_t now, long max_step, unsigned long state_flag )
{
  cal_state = 1;
  cal_cycle = 1;
  cal_state_flag = state_flag;
  cal_max_step = max_step;
  nextStepTime = now + 1000;
  cal_checkCurPos = Events_GetState( ) & state_flag;
  Events_ClearState( state_flag );
}

void Motor::CalibrationComplete( )
{
  // Done! We're calibrated
  cal_state = 0;
  if( cal_checkCurPos )
  {
    if ((curPos < -CAL_ERROR_TRLD) || (curPos > CAL_ERROR_TRLD))
    {
      Events_SetDebug( (cal_state_flag << 16) | ( curPos & 0xFFFF ));
      Events_SetState( CNC_STATE_CAL_ORIGIN_ERROR );
    }
  }
  curPos = 0;
  Events_SetState( cal_state_flag );
}

void Motor::CalibrateTask( uint64_t now )
{
  unsigned int p;
 
  switch( cal_state )
  {
  default:
    cal_state = 0; // Idle
    nextStepTime = NO_STEP_TIME;
    break;

  case 1 : // Started
    if( GetLimit( ) != 0 )
    {
      // Already at the end, move away so that we can detect the edge
      SetDirection( cal_away );
      cal_state = 2;
      cal_count = stepByInch / 16; // Move 1/16 in away
    }
    else
    {
      SetDirection( cal_toward );
      cal_state = 4;
    }
    nextStepTime = now + CAL_PAUSE;
    break;
    
  case 2 : // Move until away from the ensor plus some
    p = GetLimit( );
    if( p != 0 || cal_count != 0 )
    {
      Pulse( 1 );
      cal_state = 3;
      if( p == 0 ) cal_count--;
      nextStepTime = now + STEP_PULSE_US;
    }
    else
    {
      SetDirection( cal_toward );
      cal_state = 4;
      nextStepTime = now + CAL_PAUSE;
    }
    break;
    
  case 3 :
    Pulse( 0 );
    nextStepTime = now + CAL_STEP_SPEED;
    cal_state = 2;
    break;

  case 4 : // Move towards the sensors and calibrate
    if( GetLimit( ) == 0 )
    {
      Pulse( 1 );
      cal_state = 5;
      nextStepTime = now + STEP_PULSE_US;
      cal_stall = CAL_STALL;
    }
    else
    {
      if( cal_stall )
      {
        // Stall here for a while to make sure the limit sensor is
        // fully triggered.
        nextStepTime = now + CAL_STEP_SPEED; 
        cal_stall--; 
      }
      else
      {
        // First cycle was the approach. Now back out and come back
        // slower in order to stop at a precise location from the 
        // limit sensor.
        if( cal_cycle == 1 )
        {
          // This will slow down 8 times compared to the approach speed
          cal_cycle = 5;
          cal_state = 1;
          nextStepTime = now + STEP_PULSE_US;
        }
        else
        {
          CalibrationComplete( );
        }
      }
    }
    break;
    
  case 5 :
    Pulse( 0 );
    if(( --cal_max_step < 0 ) || ( Events_GetState( ) & CNC_STATE_RECOVERABLE_ERROR_MASK ) != 0 )
    {
      Events_SetState( CNC_STATE_CALIBRATION_FAILED );
      cal_state = 0;
      nextStepTime = NO_STEP_TIME;
    }
    else
    {
      nextStepTime = now + ( CAL_STEP_SPEED << (cal_cycle - 1));
      cal_state = 4;
    }
    break;
  }
}

void DualMotor::CalibrateTask( uint64_t now )
{
  unsigned int p;
    
  switch( cal_state )
  {
  default:
    cal_state = 0; // Idle
    nextStepTime = NO_STEP_TIME;
    break;

  case 1 : // Started
    if( GetLimit( ) != 0 )
    {
      // If either side is already at the end, move away
      // so that we can detect the edge
      SetDirection( cal_away ); // Down
      cal_state = 2;
      cal_count = abs(cal_offset) + stepByInch / 16; // Move 1/16in away
    }
    else
    {
      SetDirection( cal_toward ); // Up
      cal_state = 4;
      cal_count = cal_offset;
      cal_delta = 0;
    }
    nextStepTime = now + CAL_PAUSE;
    break;
    
  case 2 : // Move until both sided are away from the end plus some
    p = GetLimit( );
    if( p != 0 || cal_count != 0 )
    {
      Pulse( 1 );
      cal_state = 3;
      if( p == 0 ) cal_count--;
      nextStepTime = now + STEP_PULSE_US;
    }
    else
    {
      SetDirection( cal_toward ); // Up
      cal_state = 4;
      nextStepTime = now + CAL_PAUSE;
      cal_count = cal_offset;
      cal_delta = 0;
    }
    break;
    
  case 3 :
    Pulse( 0 );
    nextStepTime = now + CAL_STEP_SPEED;
    cal_state = 2;
    break;

  case 4 : // Move towards the sensors and calibrate
    p = GetLimit( );
    if( p != 3 || cal_count != 0 )
    {
      // Left not activated.
      if(( p & 1 ) == 0 || cal_count > 0 )
      {
        if(( p & 1 ) == 1 && cal_count > 0 ) cal_count--;
        if(( p & 2 ) == 2 ) cal_delta++;
        PulseLeft( 1 );
      }
      // Right not activated
      if(( p & 2 ) == 0 || cal_count < 0 )
      {
        if(( p & 2 ) == 2 && cal_count < 0 ) cal_count++;
        if(( p & 1 ) == 1 ) cal_delta--;
        PulseRight( 1 );
      }
      nextStepTime = now + STEP_PULSE_US;
      cal_state = 5;
      cal_stall = 100;
    }
    else
    {
      if( cal_stall )
      {
        nextStepTime = now + CAL_STEP_SPEED; 
        cal_stall--; 
      }
      else
      {
        if( cal_cycle == 1 )
        {
          cal_cycle = 3;
          cal_state = 1;
          nextStepTime = now + CAL_PAUSE;
        }
        else
        {
          cal_delta = cal_delta + cal_offset;
          
          int margin = (( motorFlags & CALIBRATION_OFFSET_INTERIOR ) == 0 ) ? 2 : 4;
          
          if( abs(cal_delta) < margin )
          {
            CalibrationComplete( );
          }
          else
          {
            Events_SetDebug( cal_delta );
            
            // If the position sensors are outside of the motor axis the
            // calibration algorithm removes the slanting automatically
            // because when one side stops and the other continues the 
            // sensor which first hits the moves away and triggers again
            if(( motorFlags & CALIBRATION_OFFSET_INTERIOR ) == 0 )
            {
              // For this confifguration, just go again
              cal_state = 1;
            }
            else
            {
              // cal_R : Ratio between the distance between the mootor axis and the sensor snd the totsl distance          
           
              if( cal_delta < 0 )
              {
                cal_dL = (-cal_delta) * (((1.0f / cal_R ) - 1.0f) / (2.0f - cal_R ));
                cal_dR = 0;
              }
              else
              {
                cal_dR = cal_delta * (((1.0f / cal_R ) - 1.0f) / (2.0f - cal_R ));
                cal_dL = 0;
              }

              SetDirection( cal_away );
              cal_state = 6;
            }

            nextStepTime = now + CAL_PAUSE; 
          }
        }
      }
    }
    break;
    
  case 5 :
    PulseLeft( 0 );
    PulseRight( 0 );
    if(( --cal_max_step < 0 ) || ( Events_GetState( ) & CNC_STATE_RECOVERABLE_ERROR_MASK ) != 0 )
    {
      Events_SetState( CNC_STATE_CALIBRATION_FAILED );
      cal_state = 0;
      nextStepTime = NO_STEP_TIME;
    }
    else
    {
      nextStepTime = now + CAL_STEP_SPEED * ( 1 << ( cal_cycle - 1));
      cal_state = 4;
    }
    break;
    
  case 6 : // Correction of "d1" which is away from sensor on the opposite side which was furthest
    nextStepTime = now + STEP_PULSE_US;
    if( cal_dR > 0 )
    {
      PulseRight( 1 );
      cal_dR--;
      cal_state = 7;
    }
    else if( cal_dL > 0 )
    {
      PulseLeft( 1 );
      cal_dL--;
      cal_state = 7;
    }
    else
    {
      SetDirection( cal_toward );
      nextStepTime = now + CAL_PAUSE;
      cal_state = 8;
    }
    break;
    
  case 7 :
    PulseLeft( 0 );
    PulseRight( 0 );
    nextStepTime = now + CAL_STEP_SPEED;
    cal_state = 6;
    break;
    
  case 8 : // Correction of "d2" which is towards the sensor on the same side which was furthest
    nextStepTime = now + STEP_PULSE_US;
    if( cal_dR < 0 )
    {
      PulseRight( 1 );
      cal_dR++;
      cal_state = 9;
    }
    else if( cal_dL < 0 )
    {
      PulseLeft( 1 );
      cal_dL++;
      cal_state = 9;
    }
    else
    {
      SetDirection( cal_away );
      nextStepTime = now + CAL_PAUSE;
      cal_state = 10;
      cal_count = abs(cal_offset) + stepByInch / 4; // Move 1/4in away
    }
    break;
    
  case 9 :
    PulseLeft( 0 );
    PulseRight( 0 );
    nextStepTime = now + CAL_STEP_SPEED;
    cal_state = 8;
    break;
    
  case 10 :
    if( cal_count > 0 )
    {
      cal_count--;
      Pulse( 1 );
      nextStepTime = now + STEP_PULSE_US;
      cal_state = 11;
    }
    else
    {
      // Let's start over. The process should end when cal_delta is small enough
      cal_state = 1;
      //cal_cycle++;
    }
    break;
    
  case 11 :
    Pulse( 0 );
    nextStepTime = now + CAL_STEP_SPEED;
    cal_state = 10;
    break; 
  }
}