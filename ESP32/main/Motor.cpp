
#include <stdlib.h>

#include "CNC.h"
#include "motor.h"


uint32_t g_error = WARNING_CALIBRATION;
uint32_t g_limitState = 0;
int g_debug[MAX_DEBUG];

#define RAMP_SHIFT    19 // 19=524ms
#define RAMP_TIME     (1L << RAMP_SHIFT)
#define MIN_SPEED     15L
#define MAX_SPEED     150L
// Calculate the speed ramp for G0 accel / decel phases 
#define STEP_FROM_RAMP( Min, Max, t ) ( Min - ((( Min - Max ) * t ) >> RAMP_SHIFT))

// Speed is in inch by minute, hence the 60M micro seconds
#define SPEED_TO_STEP( sbi, s ) ( 60000000L / ((sbi) * (s)))

#define NO_STEP_TIME  0xFFFFFFFF

// Instantiation and configuration of the stepper motor controlers.
//-----------------------------------------------------------------
//                   Step IO,  Direction IO,  EndMsk,  Configuration flags,               StepByInch           Calibration
DualMotor X ( MOTOR_X_L_STEP, MOTOR_X_L_DIR,  0x0004,
              MOTOR_X_R_STEP, MOTOR_X_R_DIR,  0x0008,  ERROR_LIMIT_X |
                                                       CALIBRATION_REVERSED | 
                                                       REDUCED_RAPID_POSITIONING_SPEED,   1.0f/X_AXIS_RES,     0.009f/X_AXIS_RES, 0.001f );

Motor     Y ( MOTOR_Y_STEP,    MOTOR_Y_DIR,   0x0010,  ERROR_LIMIT_Y |
                                                       CALIBRATION_REVERSED |
                                                       DIRECTION_REVERSED,                1.0f/Y_AXIS_RES );

DualMotor Z ( MOTOR_Z_L_STEP,  MOTOR_Z_L_DIR, 0x0001,
              MOTOR_Z_R_STEP,  MOTOR_Z_R_DIR, 0x0002,  ERROR_LIMIT_Z | 
                                                       REDUCED_RAPID_POSITIONING_SPEED,   1.0f/Z_AXIS_RES,    0.22197f / Z_AXIS_RES, 0.0277f * 2.0f );


unsigned long g_MoveStart = 0;  // Time when the current move was started (uS)

// For debug purpose, measures the time spent on idle task to estimate the
// CPU load of the system (displayed on LCD screen).
unsigned long g_timeIdleUs = 0;

unsigned long g_missedStepCount = 0;


// ############################# MAKE IT BUILD #####################################

uint32_t reg;

#define OUTPUT 0
#define LOW 0
#define HIGH 1

uint32_t* digitalSetRegister( uint32_t mask ) { return &reg; }
uint32_t* digitalClrRegister( uint32_t mask ) { return &reg; }
uint32_t digitalPinMask( uint32_t mask ) { return 0; }
void pinMode( int pin, int mode ) { };
void digitalWrite( int pin, int state ) { };
uint32_t micros( ) { return 0; }
void delayMicroseconds( uint32_t ) { };
void interrupts( ) { };
void noInterrupts( ) { };
void UART_Task( ) { };
bool Limit_Task( uint32_t t ) { return false; };
void AddMoveToFifo( long x, long y, long z, long t, long s );

// #################################################################################

Motor::Motor( int sp, int dp, uint32_t em, unsigned long flags, unsigned long sbi )
{
  stepPin = sp;
  dirPin = dp;
  endMask = em;
  reverseDir = flags & DIRECTION_REVERSED;
  limitFlag = flags & ERROR_FLAG_MASK;
  stepByInch = sbi;

  if( flags & CALIBRATION_REVERSED )
  {
    cal_toward = -1;
    cal_away = 1;
  }
  else
  {
    cal_toward = 1;
    cal_away = -1;
  }
  
  if(( flags & REDUCED_RAPID_POSITIONING_SPEED ) == 0 )
  {
    // Normal speed
    minSpeedStep = SPEED_TO_STEP( stepByInch, MIN_SPEED );
    maxSpeedStep = SPEED_TO_STEP( stepByInch, MAX_SPEED );
  }
  else
  {
    // Reduced speed for axis with weak motors or lots of intertia
    minSpeedStep = SPEED_TO_STEP( stepByInch, MIN_SPEED ) * 2;
    maxSpeedStep = SPEED_TO_STEP( stepByInch, MAX_SPEED ) * 4;
  }

  stepSetReg = digitalSetRegister( sp );
  stepClrReg = digitalClrRegister( sp );
  stepPinMask = digitalPinMask( sp );

  endPinDebounceTime = 0;
  endDetectionState = 0;
}

DualMotor::DualMotor( int sp, int dp, uint32_t em, int sp2, int dp2, uint32_t em2, unsigned long flags, unsigned long sbi, long cof, float R )
: Motor( sp, dp, em, flags, sbi )
{
  stepPin2 = sp2;
  dirPin2 = dp2;
  endMask2 = em2;
  cal_offset = cof;
  cal_R = R;
  cal_cycle = 1;

  step2SetReg = digitalSetRegister( sp2 );
  step2ClrReg = digitalClrRegister( sp2 );
  step2PinMask = digitalPinMask( sp2 );
}

void Motor::Reset( )
{
  curPos = 0;
  curDir = 0;
  moveLength = 0;
  moveStep = 0;
  moveDuration = 0;

  accManual = 0;
  nextSlowStep = 0;

  pinMode( stepPin, OUTPUT );
  digitalWrite(stepPin, LOW );
  pinMode( dirPin, OUTPUT );
  digitalWrite(dirPin, LOW );
}

void DualMotor::Reset( )
{
  pinMode( stepPin2, OUTPUT );
  digitalWrite(stepPin2, LOW );
  pinMode( dirPin2, OUTPUT );
  digitalWrite(dirPin2, LOW );
  Motor::Reset( );
}

#define AVG_SAMPLE_COUNT  100

int Motor::GetLimit( )
{
  return ( g_limitState & endMask ) ? 1 : 0;
}

int DualMotor::GetLimit( )
{
  int ret = 0;
  if( g_limitState & endMask ) ret |= 1;
  if( g_limitState & endMask2 ) ret |= 2;
  return ret;
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

void DualMotor::SetDirection( int d )
{
  if( !reverseDir )
  {
    digitalWrite( dirPin2, d < 0 ? LOW : HIGH );
  }
  else
  {
    digitalWrite( dirPin2, d < 0 ? HIGH : LOW );
  }
  Motor::SetDirection( d );
}

unsigned long Motor::InitMove( long s, long t )
{
  int d = 1;

  // The previous move might have left this pin HIGH
  *stepClrReg = stepPinMask;
  //digitalWrite( stepPin, LOW );

/*
  if( s == 0 && manual )
  {
    if( manual > -100 && manual < 100 )
    {
      unsigned long now = micros( );
      if( now > nextSlowStep )
      {
        nextSlowStep = now + 100000;
        accManual += manual;
        if( accManual < -100 || accManual > 100 )
        {
          if( accManual < 0 ) s = -1; else s = 1;
          accManual = 0;
        }
      }
    }
    else
    {
      if( manual > 0 ) s = ( manual - 100 ); else s = manual + 100;
    }
    t = 100000;
  }
*/  
  // Backward direction
  if( s < 0 ) 
  { 
    d = -1;
    s = -s; 
  }

  // If we do not return "NO_STEP_TIME" the move will
  // occur no matter what.
  
  moveLength = s;
  moveDuration = t;
  nextStepTime = 0;
  
  if( s > 0 )
  {
    SetDirection( d );
    // Linear motion (G1)
    if( moveDuration )
    {
      // Calculate the step duration in uS with 32:32bit precision
      stepDuration = t / moveLength;
      stepModulo = t % moveLength;
      stepAcc = 0;
    }
    // Rapid positionning (G0)
    else
    {
      stepDuration = minSpeedStep;
      decelDist = moveLength;
      decelStart = 0;
    }
    moveStep = 1;
  }
  else 
  {
    moveStep = 0;
    nextStepTime = NO_STEP_TIME;
    return NO_STEP_TIME;
  }
  
  currentStepTime = stepDuration;
  return stepDuration;
}

inline unsigned long timeSinceMoveStarted( )
{
  unsigned long now = micros( );
  // Calculate time since the start of the move
  if( g_MoveStart <= now )
    return( now - g_MoveStart );
  else
    // Rollover of the 32bit microsecond timer (every 71min)
    return( NO_STEP_TIME - g_MoveStart + now );
}

void Motor::PrepareNextStep( )
{
  // Have we prepared yet?
  if( nextStepTime == 0 )
  {        
    // Move is complete.
    if( moveStep >= moveLength )
    {
      curPos =  curPos + ( curDir * moveStep );
      moveStep = 0;
      nextStepTime = NO_STEP_TIME;
    }
    // Movement with duration means linear motion (G1).
    else if( moveDuration != 0 )
    {      
      nextStepTime = currentStepTime + stepDuration;
      stepAcc += stepModulo;
      if( stepAcc >= moveLength )
      {
        stepAcc -= moveLength;
        nextStepTime++;
      }
      currentStepTime = nextStepTime;
      moveStep++;
    }
    // Zero duration means rapid positioning motion (G0).
    else
    {
      // Time since movement started
      long t = timeSinceMoveStarted( );
      
      // Deceleration phase. Checking for deceleration first handles
      // the case where the G0 movement is so short that there is not
      // enough distance to reach full speed. As the acceleration phase
      // increases the decelDistance, this deceleration will take over
      // when the movement reaches mid point.
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
        stepDuration = STEP_FROM_RAMP( minSpeedStep, maxSpeedStep, t );
      }
      // Acceleration phase
      else if( t <= RAMP_TIME )
      {
        stepDuration = STEP_FROM_RAMP( minSpeedStep, maxSpeedStep, t );
        
        // Save the distance and time when deceleration should end
        // If the movement is so short that full speed can't be
        // reached, the previous test will pass and deceleration
        // will start before the movement has reached full speed.
        decelDist = moveLength - moveStep;
        decelTime = t;
      }
      // In the constant speed phase, just update the time for the
      // next half step.
      nextStepTime = currentStepTime + stepDuration;
      currentStepTime = nextStepTime;
      moveStep++;
    }

    // Clear the signal last to get the longest HIGH pulse possible
    // TB67S109AFTG Toshiba datasheet requires minimum CLK pulse width
    // 0.3uS (page 26). Withe the current code running on SAMD21, 
    // this pulse is at least 3uS long.
    *stepClrReg = stepPinMask;
  }
}

void DualMotor::PrepareNextStep( )
{
  // Have we prepared yet?
  if( nextStepTime == 0 )
  {
    Motor::PrepareNextStep( );
    // Clear the pin last to make sure the pulse lasts for long enough
    *step2ClrReg = step2PinMask;    
  }
}

#ifdef MEASURE_MOVE
long maxLateTime;
long maxEarlyTime;
long avgStepTime;
long shortestStep;
long avgStepCounter;
#endif

inline bool WaitTillItsTime( unsigned long t )
{
  long s;
  int count = 0;
  bool interruptsEnabled = false;

  // First, disable interrupts so that the first time measurements can be
  // accurate if it turns out that we're late or there is less than 10uS
  // until the next time to step.
  noInterrupts( );
  
  do
  {
    // How much time until the next step?
    s = t - timeSinceMoveStarted( );

    // Are we late...?
    if( s <= 0 )
    {
      // ... then get out and step it already!
      break;
    }
    else if( s < 10 )
    {
      // If we have less than 10uS, spin in a tight loop until it's time
      // without interrupts enabled to avoid jitter. Do not do this for 
      // longer than 10uS because UART bytes can come 20uS (500kbaud) 
      // appart. If interrupts are masked for longer UART may receive
      // another byte before the ISR got the time to grab it.
      // On the other hand, if this is too short, we have the risk of
      // going into interrupt and miss the time. (7uS seem to be the
      // threshold).
         
      // If we get here after spending time running the background tasks
      // interrupts will be enabled. Disabled them again so that the next
      // time measurement is accurate.      
      if( interruptsEnabled ) 
      {
        interruptsEnabled = false;
        noInterrupts( );
      }
      continue;
    }
    else if( s < 30 )
    {
      // If we have less than 30uS, just "prepare" for next steps if we
      // didn't get the chance to do it previously

      // Re-enable interrupts first since this is not time critical
      if( !interruptsEnabled )
      {
        interruptsEnabled = true;
        interrupts( );
      }

      // More efficient check if this got done already or not
      if( count == 0 )
      { 
        X.PrepareNextStep( );
        Y.PrepareNextStep( );
        Z.PrepareNextStep( );
        count = 1;
      }
    }
    else
    {
      // We have "plenty" of time. Perform background tasks such as processing
      // the bytes received via the UART, refresh the LCD screen and prepare
      // the next move times.
      
      if( !interruptsEnabled )
      {
        interruptsEnabled = true;
        interrupts( );
      }
        
      if( count == 0 )
      {
        // This is to calculate the CPU load. Only do this once.
        // (time spent doing background task and sleeping vs actual movement)
        g_timeIdleUs += s;

        // Do this first and once only         
        X.PrepareNextStep( );
        Y.PrepareNextStep( );
        Z.PrepareNextStep( );

        count = 1;
      }
      else
      {   
        // Read from UART and decode (one char at a time)
        // Except for the G10 (Reset) command, all the UART processing is done
        // in less than 20uS
        UART_Task( );
      }
    }
  } while( 1 );
  
  // Time to move
  return true;
}

void Motor::Step( )
{
  *stepSetReg = stepPinMask;
  delayMicroseconds(100);
  *stepClrReg = stepPinMask;
}

void DualMotor::StepL( )
{
  *stepSetReg = stepPinMask;
  delayMicroseconds(100);
  *stepClrReg = stepPinMask;
}

void DualMotor::StepR( )
{
  *step2SetReg = step2PinMask;
  delayMicroseconds(100);
  *step2ClrReg = step2PinMask;
}


void Motor::Step( unsigned long& t )
{
#ifdef MEASURE_MOVE
  long s = t - timeSinceMoveStarted( );
#endif

  //digitalWrite( stepPin, HIGH );
  *stepSetReg = stepPinMask;

  // Interrupts get disabled when returning from WaitTillItsTime( )
  // Turn them back ON now that we're out of the time critical section.
  interrupts( );
   
  // In some rare occasions (when all axis had to pulse with less than 30uS
  // in between steps), this didn't give enough time to prepare the next step.
  if( nextStepTime == 0 )
  {
    PrepareNextStep( );
  }
  // Return the next step time...
  t = nextStepTime;
  // ... and clear the time so that it will be "prepared" as soon as there is time
  nextStepTime = 0;

#ifdef MEASURE_MOVE
  if( s < maxLateTime ) maxLateTime = s;
  if( s > maxEarlyTime ) maxEarlyTime = s;
  avgStepTime += s;
  avgStepCounter++;
#endif
}

void DualMotor::Step( unsigned long& t )
{
  //digitalWrite( stepPin2, HIGH );
  *step2SetReg = step2PinMask;
  Motor::Step( t );
}


void Motor::Manual( int dir )
{
  manual = dir;
}

long Motor::GetPos( )
{
  // During a movement, curPos doesn't get updated until the
  // end to same on the operations done in the time critical
  // sections of the code. Add "moveStep" to account for that.
  return curPos + (curDir * moveStep);
}

long Motor::FakeMove( long s )
{
  curPos += s;
  return curPos;
}

void Motor::CalibrateStart( )
{
  cal_state = 1;
}

void DualMotor::CalibrateStart( )
{
  cal_state = 1;
  cal_cycle = 1;
}

#define CAL_STEP_SPEED   1000
#define CAL_PAUSE        50000
#define CAL_STALL        100

bool Motor::CalibrateTask( )
{
  unsigned int p;
  uint32_t now = micros( );

  switch( cal_state )
  {
    default:
      cal_state = 0;
    case 0 : // Idle
      break;

    case 1 : // Started
      if( GetLimit( ) != 0 )
      {
        // Already at the end, move away so that we can detect the edge
        SetDirection( cal_away );
        cal_state = 2;
        cal_count = stepByInch / 4; // Move 1/4in away
      }
      else
      {
        SetDirection( cal_toward );
        cal_state = 3;
      }
      cal_time = now + CAL_PAUSE;
      break;

    case 2 : // Move until away from the ensor plus some
      if( now >= cal_time )
      {
        p = GetLimit( );
        if( p != 0 || cal_count != 0 )
        {
          Step( );
          if( p == 0 ) cal_count--;
          cal_time = now + CAL_STEP_SPEED;
        }
        else
        {
          SetDirection( cal_toward );
          cal_state = 3;
          cal_time = now + CAL_PAUSE;
        }
      }
      break;

    case 3 : // Move towards the sensors and calibrate
      if( now >= cal_time )
      {
        p = GetLimit( );
        if( p == 0 )
        {
          Step( );
          cal_time = now + CAL_STEP_SPEED;
          cal_stall = CAL_STALL;
        }
        else
        {
          if( cal_stall )
          {
             cal_time = now + CAL_STEP_SPEED; 
             cal_stall--; 
          }
          else
          {
#if 1
            // Done! We're calibrated!
            cal_state = 0;
#else
            SetDirection( cal_away );
            cal_state = 4;
            cal_count = stepByInch * 0.5; // Move away 1/2 inch from the reference position
            cal_time = now + CAL_PAUSE;
#endif
          }
        }
      }
      break;

    case 4 : // Move to the origin position (from the reference)
      if( now >= cal_time )
      {
        if( cal_count >= 0 )
        {
          Step( );
          cal_count--;
          cal_time = now + CAL_STEP_SPEED;
        }
        else
        {
          SetDirection( cal_toward );
          cal_time = now + CAL_PAUSE;
          cal_state = 5;
          cal_count = 0;
        }
      }
      break;

    case 5 :
      if( now >= cal_time )
      {
        p = GetLimit( );
        if( p == 0 )
        {
          cal_count++;
          Step( );
          cal_time = now + CAL_STEP_SPEED;
        } 
        else
        {
          SetDirection( cal_away ); // Down
          cal_state = 1;
          cal_count = stepByInch * 0.5; // Move away 3 inch from the reference position
          cal_time = now + CAL_PAUSE;

          // Serial.printf("S:%lu \n", cal_count );
        }
      }
      break;
  }
  // Return true only if calibration is ongoing
  return ( cal_state != 0 );
}

bool DualMotor::CalibrateTask( )
{
  unsigned int p;
  uint32_t now = micros( );

  switch( cal_state )
  {
    default:
      cal_state = 0;
      __attribute__ ((fallthrough));
    case 0 : // Idle
      cal_cycle = 1;
      break;

    case 1 : // Started
      if( GetLimit( ) != 0 )
      {
        // If either side is already at the end, move away
        // so that we can detect the edge
        SetDirection( cal_away ); // Down
        cal_state = 2;
        cal_count = abs(cal_offset) + stepByInch / 4; // Move 1/4in away
      }
      else
      {
        SetDirection( cal_toward ); // Up
        cal_state = 3;
        cal_count = cal_offset;
        cal_delta = 0;
      }
      cal_time = now + CAL_PAUSE;
      break;

    case 2 : // Move until both sided are away from the end plus some
      if( now >= cal_time )
      {
        p = GetLimit( );
        if( p != 0 || cal_count != 0 )
        {
          StepL( );
          StepR( );
          if( p == 0 ) cal_count--;
          cal_time = now + CAL_STEP_SPEED;
        }
        else
        {
          SetDirection( cal_toward ); // Up
          cal_state = 3;
          cal_time = now + CAL_PAUSE;
          cal_count = cal_offset;
          cal_delta = 0;
        }
      }
      break;

    case 3 : // Move towards the sensors and calibrate
      if( now >= cal_time )
      {
        p = GetLimit( );
        if( p != 3 || cal_count != 0 )
        {
          if(( p & 1 ) == 0 || cal_count > 0 )
          {
            if(( p & 1 ) == 1 && cal_count > 0 ) cal_count--;
            if(( p & 2 ) == 2 ) cal_delta++;
            StepL( );
          } 
          if(( p & 2 ) == 0 || cal_count < 0 )
          {
            if(( p & 2 ) == 2 && cal_count < 0 ) cal_count++;
            if(( p & 1 ) == 1 ) cal_delta--;
            StepR( );
          }
          cal_time = now + CAL_STEP_SPEED * cal_cycle;
          cal_stall = 100;
        }
        else
        {
          if( cal_stall )
          {
             cal_time = now + CAL_STEP_SPEED; 
             cal_stall--; 
          }
          else
          {
            cal_delta = cal_delta + cal_offset;

            // Calculate as if Left axis was slanted further away
            cal_dR = abs(cal_delta) * (cal_R - 1.0f) / (2.0f - ( 1.0f / cal_R )); // Positive (correction is to move away on the right side)
            cal_dL = (abs(cal_delta) * cal_R) - cal_dR; // Negative (correction is to move toward on the left side)

            //if( cal_dR == 0 && cal_dL == 0 )
            if( abs(cal_delta) < 2 )
            {
              // Done! We're calibrated
              cal_state = 0;
            }
            else
            {              
              if( cal_delta < 0 ) // Right was actually further away
              {
                // Then swap the axis corrections
                long tmp = cal_dR;
                cal_dR = cal_dL;
                cal_dL = tmp;
              }

              // Serial.printf("D:%ld dL:%ld dR:%ld\n", cal_delta, cal_dL, cal_dR );

              SetDirection( cal_away );
              cal_state = 4;
              cal_time = now + CAL_PAUSE;
            }
          }
        }
      }
      break;

    case 4 : // Correction of "d1" which is away from sensor on the opposite side which was furthest
      if( now >= cal_time )
      {
        cal_time = now + CAL_STEP_SPEED;
        if( cal_dR > 0 )
        {
          StepR( );
          cal_dR--;
        }
        else if( cal_dL > 0 )
        {
          StepL( );
          cal_dL--;
        }
        else
        {
          SetDirection( cal_toward );
          cal_time = now + CAL_PAUSE;
          cal_state = 5;
        }
      }
      break;

    case 5 : // Correction of "d2" which is towards the sensor on the same side which was furthest
      if( now >= cal_time )
      {
        cal_time = now + CAL_STEP_SPEED;
        if( cal_dR < 0 )
        {
          StepR( );
          cal_dR++;
        }
        else if( cal_dL < 0 )
        {
          StepL( );
          cal_dL++;
        }
        else
        {
          SetDirection( cal_away );
          cal_time = now + CAL_PAUSE;
          cal_state = 6;
          cal_count = abs(cal_offset) + stepByInch / 4; // Move 1/4in away
        }
      }
      break;

    case 6 :
      if( now >= cal_time )
      {
        if( cal_count > 0 )
        {
          cal_count--;
          StepL( );
          StepR( );
          cal_time = now + CAL_STEP_SPEED;
        }
        else
        {
          // Let's start over. The process should end when cal_delta is small enough
          cal_state = 1;
          cal_cycle++;
        }
      }
      break;
  }
  // Return true only if calibration is ongoing
  return ( cal_state != 0 );
}

// Move the tool position by the specified # of steps for x,y,z directions.
// Duration of the motion determines the speed. d is in microseconds
void Motor_Move( long x, long y, long z, long d )
{
  unsigned long tX,tY,tZ;
  
  // If in error state, dot not move
  if( g_error & ERROR_FLAG_MASK )
  {
    return;
  }

  if( d == -1 ) // Calibration!
  {
    bool calibrated;
    X.CalibrateStart( );
    Y.CalibrateStart( );
    Z.CalibrateStart( );
    do
    {
      calibrated = true;

      if( X.CalibrateTask( )) calibrated = false;
      if( Y.CalibrateTask( )) calibrated = false;
      if( Z.CalibrateTask( )) calibrated = false;
    
      Limit_Task( micros( ));

       UART_Task( );

    } while( !calibrated );

    AddMoveToFifo( 6.0 / X_AXIS_RES, 6.0 / Y_AXIS_RES, -4.0 / Z_AXIS_RES, 0, 0 );
    //AddMoveToFifo( 0, 0, -4.0 / Z_AXIS_RES, 0, 0 );

  }

  if( g_MoveStart == 0 )
  {
    // Capture when the motion started (in uS). Do this as early as possible
    // to avoid jitter in consecutive linear motion
    g_MoveStart = micros();
  }

#ifdef MEASURE_MOVE
  maxLateTime = 0;
  maxEarlyTime = 0;
  avgStepTime = 0;
  avgStepCounter = 0;
#endif

  // This calculates the interval between steps for each axis and returns the time
  // to the first step needs to occur ( NO_STEP_TIME if no move necessary).
  tX = X.InitMove( x, d );
  tY = Y.InitMove( y, d );
  tZ = Z.InitMove( z, d );
 
  // No movement means dwell (GCode "P")
  if( x==0 && y==0 && z==0 )
  {
    if( d != 0 )
    {
      while( g_MoveStart + d >= micros( ))
      {
        UART_Task( );
      }     
      
      // Just wait...
      //WaitTillItsTime( d );
      g_MoveStart += d;
    }
    return;
  }

  //Serial.printf("Start Move %ld,%ld,%ld\n", x,y,z );

  do
  {
    // Check which axis is the next one to be stepped
    if(( tX < tY ) && ( tX < tZ ))
    {
      // X is next step
      if( WaitTillItsTime( tX )) X.Step( tX );
    }
    else if( tY < tZ )
    {
      // Y is next step
      if( WaitTillItsTime( tY )) Y.Step( tY );
    }
    else if( tZ != NO_STEP_TIME )
    {
      // Z is next step
      if( WaitTillItsTime( tZ )) Z.Step( tZ );
    }
    else
    {
      // No more steps, move is done.
      break;
    }
  } while( 1 );

  if( d != 0 )
  {
    g_MoveStart += d;
  }
  else
  {
    g_MoveStart = 0;
  }
  
#ifdef MEASURE_MOVE
  char str[10];
  sprintf( str, "%d %d %d ", maxLateTime, maxEarlyTime, avgStepTime / avgStepCounter );
  LCD_SetStatus( str, 0 );
#endif

}


void Motor_Init( )
{  
  g_missedStepCount = 0;
  
  for( int i=0; i<MAX_DEBUG; i++ )
  {
    g_debug[i] = 0;
  }
  
  pinMode(TOOL_ON_REPLAY, OUTPUT);
  digitalWrite(TOOL_ON_REPLAY, LOW);
  
  X.Reset( );
  Y.Reset( );
  Z.Reset( );
}

inline unsigned int filterPin( )
{
  return (g_limitState & 3);
}

#define CALIB_Z_OFFSET (0.2545f / Z_AXIS_RES)

void UART_delay( unsigned long us )
{
    uint32_t waitUntil = micros( ) + us;
    do {
      while( Limit_Task( micros( )));
    }
    while( micros( ) <= waitUntil );
}

