
extern "C" {
  #include <stdlib.h>
  #include <cstring>

  #include "freertos/FreeRTOS.h"
  #include "driver/gpio.h"
  #include "driver/gptimer.h"
  #include "esp_log.h"

  #include "CNC.h"
}

uint32_t g_error = WARNING_CALIBRATION;
uint32_t g_limitState = 0;
int g_debug[MAX_DEBUG];

#define STEP_PULSE_US 100 // Duration of the motor step pulse
#define RAMP_SHIFT    19 // 19=524ms
#define RAMP_TIME     (1L << RAMP_SHIFT)
#define MIN_SPEED     15L
#define MAX_SPEED     150L
// Calculate the speed ramp for G0 accel / decel phases 
#define STEP_FROM_RAMP( Min, Max, t ) ( Min - ((( Min - Max ) * t ) >> RAMP_SHIFT))
// Speed is in inch by minute, hence the 60M micro seconds
#define SPEED_TO_STEP( sbi, s ) ( 60000000L / ((sbi) * (s)))
#define NO_STEP_TIME  (-1ULL)

class Motor {
public : 
  Motor( gpio_num_t sp, gpio_num_t dp, uint32_t em, unsigned long flags, unsigned long sbi );
  void Reset( );
  
  // This is virtual to make sure the derived class
  // implementation gets called
  virtual void SetDirection( int d );
  
  uint64_t InitMove( long s, unsigned long t, uint64_t now );
  uint64_t GetNextStepTime( );
  void PrepareNextStep( uint64_t now );
  
  void Step( );
  
  long GetPos( );
  int GetLimit( );
  void CalibrateStart( );
  bool CalibrateTask( );
  
protected :
  long curPos;                // Current axis position in steps

  gpio_num_t stepPin;         // GPIO for stepping
  gpio_num_t dirPin;          // GPIO for direction
  int reverseDir;             // Reverse the motor direction
  uint32_t endMask;           // Bitmask for limit detection
  unsigned int limitFlag;     // Flags to set when limit is reached
    
  long curDir;                // Current movement direction (+/- 1)
  unsigned long moveLength;   // Movement total length in steps
  unsigned long moveStep;     // Steps performed in movement
  unsigned long moveDuration; // Expected total duration of the movement
  long stepDuration;          // Duration of a half step
  long stepModulo;            // The remainder of the division
  uint64_t nextStepTime;      // Time when the next half step should be made
  bool stepLevel;
//uint64_t currentStepTime;   // Time when the current step is happening
  long stepAcc;               // The fractional error accumulator

  // Rapid positionning (G0)
  long decelDist;             // Step in movement when deceleration starts
  unsigned long decelTime;    // Duration of the deceleration phase 
  unsigned long decelStart;   // Time when the deceleration has started
  unsigned long minSpeedStep; // Duration of a step at G0 min speed 
  unsigned long maxSpeedStep; // Same for max speed
  unsigned long stepByInch;   // Number of steps for one inch (approx)

  // Calibration
  int cal_state;
  long cal_count;
  long cal_stall;
  uint32_t cal_time;
  int cal_toward;
  int cal_away;

};

class DualMotor : public Motor 
{
public:
  DualMotor( gpio_num_t sp, gpio_num_t dp, uint32_t em, gpio_num_t sp2, gpio_num_t dp2, uint32_t em2, unsigned long flags, unsigned long sbi, long cof, float R );
  
private:
  gpio_num_t stepPin2;               // GPIO for stepping 2nd motor
  gpio_num_t dirPin2;                // GPIO for direction of 2nd motor
  uint32_t endMask2;          // Bitmask for limit detection for 2nd motor

  // Calibration
  long cal_offset;            // Position difference between the L and R calibration positions (in steps)
  long cal_delta;             // Difference of steps required between the L and R motor to reach each sensor
  long cal_dL, cal_dR;        // Number of steps to correct the slanting of the axis prior to calibration for L and R motors 
  float cal_R;                // The ratio betweem the position of the sensors and the motors. Used to correct the slanting effect.
  int cal_cycle;              // Number of calibration cycles. Echh cycle slows down to increase precision

public:
  void Step( );
  void StepL( );
  void StepR( );
  void Reset( );
  void SetDirection( int d );
  int GetLimit( );
  void CalibrateStart( );
  bool CalibrateTask( );
};

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


Motor *g_pNextMotorToStep = NULL;
gptimer_handle_t g_motorTimer = NULL;
uint64_t g_MoveStart = 0;  // Time when the current move was started (uS)

// For debug purpose, measures the time spent on idle task to estimate the
// CPU load of the system (displayed on LCD screen).
unsigned long g_timeIdleUs = 0;

unsigned long g_missedStepCount = 0;

static const char* TAG = "motor";



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

Motor::Motor( gpio_num_t sp, gpio_num_t dp, uint32_t em, unsigned long flags, unsigned long sbi )
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
  
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT; // GPIO_MODE_OUTPUT_OD
  io_conf.pin_bit_mask = (1ULL<<sp) | (1ULL<<dp);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
    
}

DualMotor::DualMotor( gpio_num_t sp, gpio_num_t dp, uint32_t em, gpio_num_t sp2, gpio_num_t dp2, uint32_t em2, unsigned long flags, unsigned long sbi, long cof, float R )
: Motor( sp, dp, em, flags, sbi )
{
  stepPin2 = sp2;
  dirPin2 = dp2;
  endMask2 = em2;
  cal_offset = cof;
  cal_R = R;
  cal_cycle = 1;
}

void Motor::Reset( )
{
  curPos = 0;
  curDir = 0;
  moveLength = 0;
  moveStep = 0;
  moveDuration = 0;
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
    gpio_set_level( dirPin, d < 0 ? LOW : HIGH );
    //digitalWrite( dirPin, d < 0 ? LOW : HIGH );
  }
  else
  {
    gpio_set_level( dirPin, d < 0 ? HIGH : LOW );
    //digitalWrite( dirPin, d < 0 ? HIGH : LOW );
  }
  
  if( curDir != d )
  {
    curDir = d;
  }
}

void DualMotor::SetDirection( int d )
{
  if( !reverseDir )
  {
    gpio_set_level( dirPin2, d < 0 ? LOW : HIGH );
    //digitalWrite( dirPin2, d < 0 ? LOW : HIGH );
  }
  else
  {
    gpio_set_level( dirPin2, d < 0 ? HIGH : LOW );
    //digitalWrite( dirPin2, d < 0 ? HIGH : LOW );
  }
  Motor::SetDirection( d );
}

uint64_t Motor::InitMove( long s, unsigned long t, uint64_t now )
{
  int d = 1;
  
  if( stepLevel == HIGH )
  {
    ESP_LOGE( TAG, "Previous move did not clear the pulse" );
    assert(false);
  }

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
  
  nextStepTime = stepDuration + now;
  return stepDuration;
}

inline uint64_t IRAM_ATTR Motor::GetNextStepTime( )
{
    return nextStepTime;
}

void Motor::PrepareNextStep( uint64_t now )
{
  if( stepLevel == HIGH )
  {
    nextStepTime += STEP_PULSE_US;
  }
  else
  {
    // Move is complete.
    if( moveStep >= moveLength )
    {
      moveStep = 0;
      nextStepTime = NO_STEP_TIME;
    }
    // Movement with duration means linear motion (G1).
    else if( moveDuration != 0 )
    {      
      nextStepTime += stepDuration - STEP_PULSE_US;
      stepAcc += stepModulo;
      if( stepAcc >= moveLength )
      {
        stepAcc -= moveLength;
        nextStepTime++;
      }
      moveStep++;
    }
    // Zero duration means rapid positioning motion (G0).
    else
    {      
      uint64_t t = now - g_MoveStart;
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
      nextStepTime += stepDuration - STEP_PULSE_US;
      moveStep++;
    }
  }
}

void IRAM_ATTR Motor::Step( )
{
  if( stepLevel == LOW )
  {
    gpio_set_level( stepPin, HIGH );
    stepLevel = HIGH;
    curPos += curDir;
  }
  else
  {
    gpio_set_level( stepPin, LOW );
    stepLevel = LOW;
  }
}

void IRAM_ATTR DualMotor::Step( )
{
  Motor::Step( );
  gpio_set_level( stepPin2, stepLevel );
}

void DualMotor::StepL( )
{
  /*
  *stepSetReg = stepPinMask;
  delayMicroseconds(100);
  *stepClrReg = stepPinMask;
  */
}

void DualMotor::StepR( )
{
  /*
  *step2SetReg = step2PinMask;
  delayMicroseconds(100);
  *step2ClrReg = step2PinMask;
  */
}

long Motor::GetPos( )
{
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

/*
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
*/

extern "C" {
  
  #include "Events.h"
  
  extern QueueHandle_t g_cmd_queue;
  
  static void IRAM_ATTR PrepareNextStep( uint64_t now )
  {
    // Check which axis is the next one to be stepped
    if(( X.GetNextStepTime( ) < Y.GetNextStepTime( )) && ( X.GetNextStepTime( ) < Z.GetNextStepTime( )))
    {
      g_pNextMotorToStep = &X;
    }
    else if( Y.GetNextStepTime( ) < Z.GetNextStepTime( ))
    {
      g_pNextMotorToStep = &Y;
    }
    else if( Z.GetNextStepTime( ) != NO_STEP_TIME )
    {
      g_pNextMotorToStep = &Z;
    }
    else
    {
      cmd_t cmd;
      BaseType_t xTaskWokenByReceive = pdFALSE;
      
      // If this was called from idle, there has to be a movement to make
      assert( now != 0 );

      // Pull next move command from the queue
      if( xQueueReceiveFromISR( g_cmd_queue, &cmd, &xTaskWokenByReceive ))
      {
        // Start the next move using the curent time as the starting point (now)
        g_MoveStart = now;
        X.InitMove( cmd.dx, cmd.duration, now );
        Y.InitMove( cmd.dy, cmd.duration, now );
        Z.InitMove( cmd.dz, cmd.duration, now );
        PrepareNextStep( now );
      }
      else
      {
        // No more movements to perform, stop the timer, reset
        // the counter and go idle    
        ESP_ERROR_CHECK(gptimer_stop(g_motorTimer));
        ESP_ERROR_CHECK(gptimer_set_raw_count(g_motorTimer,0));
        g_pNextMotorToStep = NULL;
        SignalMotorIdleFromISR( );
      }
    }

    if( g_pNextMotorToStep )
    {
      gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = g_pNextMotorToStep->GetNextStepTime( ),
        .reload_count = 0,
        .flags = 0,
      };

      gptimer_set_alarm_action(g_motorTimer, &alarm_config1);

      // If this was started from idle,
      if( now == 0 )
      {
        ESP_ERROR_CHECK(gptimer_start(g_motorTimer));
      }
    }
  }  

  static bool IRAM_ATTR timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
  {
    g_pNextMotorToStep->Step( );
    g_pNextMotorToStep->PrepareNextStep( edata->alarm_value );
    PrepareNextStep( edata->alarm_value );
    return false;
  }
  
  void MotorGetPosition( long *pX, long *pY, long *pZ )
  {
    *pX = X.GetPos( );
    *pY = Y.GetPos( );
    *pZ = Z.GetPos( );
  }
  
  // Move the tool position by the specified # of steps for x,y,z directions.
  // Duration of the motion determines the speed. d is in microseconds
  void MotorMove( long x, long y, long z, long d )
  { 
    g_MoveStart = 0;

    // This calculates the interval between steps for each axis and returns the time
    // to the first step needs to occur ( NO_STEP_TIME if no move necessary).
    X.InitMove( x, d, 0 );
    Y.InitMove( y, d, 0 );
    Z.InitMove( z, d, 0 );
    
    PrepareNextStep( 0 );
    
    WaitForMotorIdle( );
       
       
   /*
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

    } while( 1 );

    if( d != 0 )
    {
      g_MoveStart += d;
    }
    else
    {
      g_MoveStart = 0;
    }
    */
  }
  
  void MotorInit( )
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
    
    gptimer_config_t timer_config;
    
    memset( &timer_config, 0x00, sizeof(timer_config));
    timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = 1000000; // 1MHz, 1 tick=1us

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &g_motorTimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_motorTimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(g_motorTimer));
    
  }
}
