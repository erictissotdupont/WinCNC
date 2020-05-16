#include "CNC.h"
#include "helper.h"
#include "motor.h"
#include "LCD.h"
#include "UART.h"

unsigned int g_error;
int g_debug[MAX_DEBUG];

#define RAMP_SHIFT    19 // 19=524ms
#define RAMP_TIME     (1L << RAMP_SHIFT)
#define MIN_SPEED     15L
#define MAX_SPEED     250L
// Calculate the speed ramp for G0 accel / decel phases 
#define STEP_FROM_RAMP( Min, Max, t ) ( Min - ((( Min - Max ) * t ) >> RAMP_SHIFT))

// Speed is in inch by minute, hence the 60M micro seconds
#define SPEED_TO_STEP( sbi, s ) ( 60000000L / ( sbi * s ))

#define NO_STEP_TIME  0xFFFFFFFF

// Instantiation and configuration of the stepper motor controlers.
//-----------------------------------------------------------------
//           Step,Dir,End,Swap,    LimitFlag,      StepByInch
Motor     X ( 11, 10, -1,  0, ERROR_LIMIT_X, 1.0f/X_AXIS_RES );
Motor     Y ( 13, 12, -1,  1, ERROR_LIMIT_Y, 1.0f/Y_AXIS_RES );
DualMotor Z ( A3, A2, -1,
              A5, A4, -1,  1, ERROR_LIMIT_Z | REDUCED_RAPID_POSITIONING_SPEED,
                                             1.0f/Z_AXIS_RES );



unsigned long g_MoveStart;  // Time when the current move was started (uS)

// For debug purpose, measures the time spent on idle task to estimate the
// CPU load of the system (displayed on LCD screen).
unsigned long g_timeIdleUs = 0;

unsigned long g_missedStepCount = 0;

Motor::Motor( int sp, int dp, int ep, int rd, unsigned long flags, unsigned long sbi )
{
  stepPin = sp;
  dirPin = dp;
  endPin = ep;
  reverseDir = rd;
  limitFlag = flags & ERROR_FLAG_MASK;
  stepByInch = sbi;
  
  if(( flags & REDUCED_RAPID_POSITIONING_SPEED ) == 0 )
  {
    minSpeedStep = SPEED_TO_STEP( stepByInch, MIN_SPEED );
    maxSpeedStep = SPEED_TO_STEP( stepByInch, MAX_SPEED );
  }
  else
  {
    minSpeedStep = SPEED_TO_STEP( stepByInch, MIN_SPEED / 2 );
    maxSpeedStep = SPEED_TO_STEP( stepByInch, MAX_SPEED / 2 );
  }

  stepSetReg = digitalSetRegister( sp );
  stepClrReg = digitalClrRegister( sp );
  stepPinMask = digitalPinMask( sp );
}

DualMotor::DualMotor( int sp, int dp, int ep, int sp2, int dp2, int ep2, int s, unsigned long flags, unsigned long sbi )
: Motor( sp, dp, ep, s, flags, sbi )
{
  stepPin2 = sp2;
  dirPin2 = dp2;
  endPin2 = ep2;

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

  pinMode( stepPin, OUTPUT );
  digitalWrite(stepPin, LOW );
  pinMode( dirPin, OUTPUT );
  digitalWrite(dirPin, LOW );
  if( endPin >= 0 ) {
    pinMode( endPin, INPUT );
  }
}

void DualMotor::Reset( )
{
  pinMode( stepPin2, OUTPUT );
  digitalWrite(stepPin2, LOW );
  pinMode( dirPin2, OUTPUT );
  digitalWrite(dirPin2, LOW );
  Motor::Reset( );
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
  long ret = -1;
  int d = 1;

  // The previous move might have left this pin HIGH
  *stepClrReg = stepPinMask;
  //digitalWrite( stepPin, LOW );
  
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
      stepAcc += stepModulo;
      if( stepAcc >= moveLength )
      {
        stepAcc -= moveLength;
        nextStepTime++;
      }
      nextStepTime = currentStepTime + stepDuration;
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
    // TB6600HG datasheet requires minimum CLK pulse width of 2.2uS
    // Withe the current code running on SAMD21, this pulse is 5uS.
    *stepClrReg = stepPinMask;
  }
}

void DualMotor::PrepareNextStep( )
{
  // Have we prepared yet?
  if( nextStepTime == 0 )
  {
    Motor::PrepareNextStep( );
    // Clear the pin last to make sure the pulse lasts for
    // at least 
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

inline void WaitTillItsTime( unsigned long t )
{
  long s;
  int count = 0;
  bool interruptsEnabled = false;

  // First, disable interrupts so that the first time measurements can be
  // accurate if it turns out that we're late or there is less than 10uS until
  // the next time to move
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
      }
      else
      {   
        // Alternate between UART and LCD tasks (UART first)
        if( count & 1 )
        {
          // Read from UART and decode (one char at a time)
          // Except for the G10 (Reset) command, all the UART processing is done
          // in less than 20uS
          UART_Task( );
        }
        else
        {
          // Refresh the LCD screen (if enough time)
          // Most LCD refresh commands take 30uS or less
          LCD_UpdateTask( );       
        }
      }
      count++;
    }
  } while( 1 );  
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
}

// Move the tool position by the specified # of steps for x,y,z directions.
// Duration of the motion determines the speed. d is in microseconds
void Motor_Move( long x, long y, long z, long d )
{
  unsigned long tX,tY,tZ;
  
  // If in error state, dot not move
  if( g_error != 0 )
  {
    return;
  }

  // Capture when the motion started (in uS). Do this as early as possible
  // to avoid jitter in consecutive linear motion
  g_MoveStart = micros();

#ifdef MEASURE_MOVE
  maxLateTime = 0;
  maxEarlyTime = 0;
  avgStepTime = 0;
  avgStepCounter = 0;
#endif
  
  // No movement means dwell (GCode "P")
  if( x==0 && y==0 && z==0 )
  {
    // Just wait...
    WaitTillItsTime( d );
    return;
  }

  // This calculates the interval between steps for each axis and returns the time
  // to the first step needs to occur ( NO_STEP_TIME if no move necessary).
  tX = X.InitMove( x, d );
  tY = Y.InitMove( y, d );
  tZ = Z.InitMove( z, d );

  do
  {
    // Check which axis is the next one to be stepped
    if(( tX < tY ) && ( tX < tZ ))
    {
      // X is next step
      WaitTillItsTime( tX );
      X.Step( tX );
    }
    else if( tY < tZ )
    {
      // Y is next step
      WaitTillItsTime( tY );
      Y.Step( tY );
    }
    else if( tZ != NO_STEP_TIME )
    {
      // Z is next step
      WaitTillItsTime( tZ );
      Z.Step( tZ );
    }
    else
    {
      // No more steps, move is done.
      break;
    }
  } while( 1 );
  
#ifdef MEASURE_MOVE
  char str[10];
  sprintf( str, "%d %d %d ", maxLateTime, maxEarlyTime, avgStepTime / avgStepCounter );
  LCD_SetStatus( str, 0 );
#endif

}

void Motor_Init( )
{
  g_error = 0;
  g_missedStepCount = 0;
  
  for( int i=0; i<MAX_DEBUG; i++ ) g_debug[i] = 0;
  
  pinMode(TOOL_ON_REPLAY, OUTPUT);
  digitalWrite(TOOL_ON_REPLAY, LOW);
  
  X.Reset( );
  Y.Reset( );
  Z.Reset( );
}
