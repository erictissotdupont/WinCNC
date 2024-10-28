
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
extern uint32_t g_limitState;
int g_debug[MAX_DEBUG];

#define STEP_PULSE_US     100 // Duration of the motor step pulse
#define RAMP_SHIFT        19 // 19=524ms
#define RAMP_TIME         (1L << RAMP_SHIFT)
#define MIN_SPEED         15L
#define MAX_SPEED         150L
#define NO_STEP_TIME      (-1ULL)

// Calculate the speed ramp for G0 accel / decel phases 
#define STEP_FROM_RAMP( Min, Max, t ) ( Min - ((( Min - Max ) * t ) >> RAMP_SHIFT))
// Speed is in inch by minute, hence the 60M micro seconds
#define SPEED_TO_STEP( sbi, s ) ( 60000000L / ((sbi) * (s)))

class Motor {
public : 
  Motor( gpio_num_t sp, gpio_num_t dp, uint32_t em, unsigned long flags, unsigned long sbi );
  
  // Virtual is to make sure the derived class
  // implementation gets called
  virtual bool SetDirection( int d );
  virtual void Pulse( bool on );
  virtual int GetLimit( );
  virtual void CalibrateStart( uint64_t now );
  virtual void CalibrateTask( uint64_t now );
  
  // Those functions are not to overloaded
  void Reset( );
  long GetPos( );
  uint64_t InitMove( long s, unsigned long t, uint64_t now );
  uint64_t GetNextStepTime( );
  void PrepareNextStep( uint64_t now );
  
protected :
  long curPos;                // Current axis position in steps

  gpio_num_t stepPin;         // GPIO for stepping
  bool pulseLevel;            // Save the current state of the pulse
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
  bool dirLevel;
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
  int cal_toward;
  int cal_away;
  int cal_cycle;              // Number of calibration cycles. Echh cycle slows down to increase precision

};

class DualMotor : public Motor 
{
public:
  DualMotor( gpio_num_t sp, gpio_num_t dp, uint32_t em, gpio_num_t sp2, gpio_num_t dp2, uint32_t em2, unsigned long flags, unsigned long sbi, long cof, float R );
  
private:
  gpio_num_t stepPin2;        // GPIO for stepping 2nd motor
  gpio_num_t dirPin2;         // GPIO for direction of 2nd motor
  uint32_t endMask2;          // Bitmask for limit detection for 2nd motor

  // Calibration
  long cal_offset;            // Position difference between the L and R calibration positions (in steps)
  long cal_delta;             // Difference of steps required between the L and R motor to reach each sensor
  long cal_dL, cal_dR;        // Number of steps to correct the slanting of the axis prior to calibration for L and R motors 
  float cal_R;                // The ratio betweem the position of the sensors and the motors. Used to correct the slanting effect.


public:
  virtual void Pulse( bool on ) override;
  void PulseLeft( bool on );
  void PulseRight( bool on );
  virtual bool SetDirection( int d )override;
  virtual int GetLimit( ) override;
  virtual void CalibrateTask( uint64_t now ) override;

};

// Instantiation and configuration of the stepper motor controlers.
//-----------------------------------------------------------------
//                   Step IO,  Direction IO,  EndMsk,  Configuration flags,               StepByInch           Calibration
DualMotor X ( MOTOR_X_L_STEP, MOTOR_X_L_DIR,  0x0004,
              MOTOR_X_R_STEP, MOTOR_X_R_DIR,  0x0008,  ERROR_LIMIT_X |
                                                       CALIBRATION_REVERSED |
                                                       DIRECTION_REVERSED |
                                                       REDUCED_RAPID_POSITIONING_SPEED,   1.0f/X_AXIS_RES,     0.009f/X_AXIS_RES, 0.001f );

Motor     Y ( MOTOR_Y_STEP,    MOTOR_Y_DIR,   0x0010,  ERROR_LIMIT_Y |
                                                       CALIBRATION_REVERSED,              1.0f/Y_AXIS_RES );

DualMotor Z ( MOTOR_Z_L_STEP,  MOTOR_Z_L_DIR, 0x0001,
              MOTOR_Z_R_STEP,  MOTOR_Z_R_DIR, 0x0002,  ERROR_LIMIT_Z |
                                                       DIRECTION_REVERSED |
                                                       REDUCED_RAPID_POSITIONING_SPEED,   1.0f/Z_AXIS_RES,    0.22197f / Z_AXIS_RES, 0.0277f * 2.0f );


static Motor *g_pNextMotorToStep = NULL;
static gptimer_handle_t g_motorTimer = NULL;
static uint64_t g_MoveStart = 0;  // Time when the current move was started (uS)
static const char* TAG = "motor";

Motor::Motor( gpio_num_t sp, gpio_num_t dp, uint32_t em, unsigned long flags, unsigned long sbi )
{
  stepPin = sp;
  dirPin = dp;
  endMask = em;
  reverseDir = flags & DIRECTION_REVERSED;
  limitFlag = flags & ERROR_FLAG_MASK;
  stepByInch = sbi;
  nextStepTime = NO_STEP_TIME;

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
  io_conf.mode = GPIO_MODE_OUTPUT_OD; // GPIO_MODE_OUTPUT_OD; // GPIO_MODE_OUTPUT; // GPIO_MODE_OUTPUT_OD
  io_conf.pin_bit_mask = (1ULL<<stepPin) | (1ULL<<dirPin);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  
  ESP_ERROR_CHECK(gpio_set_level( stepPin, OD_OPEN ));
  pulseLevel = 0;
  
  ESP_ERROR_CHECK(gpio_set_level( dirPin, OD_OPEN ));
  dirLevel = OD_OPEN;
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
  
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT_OD; // GPIO_MODE_OUTPUT_OD; // GPIO_MODE_OUTPUT; // GPIO_MODE_OUTPUT_OD
  io_conf.pin_bit_mask = (1ULL<<stepPin2) | (1ULL<<dirPin2);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  
  ESP_ERROR_CHECK(gpio_set_level( stepPin2, OD_OPEN ));
  ESP_ERROR_CHECK(gpio_set_level( dirPin2, dirLevel ));
}

void Motor::Reset( )
{
  curPos = 0;
  curDir = 0;
  moveLength = 0;
  moveStep = 0;
  moveDuration = 0;
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

bool Motor::SetDirection( int d )
{
  if( curDir != d )
  {
    curDir = d;
    if( reverseDir ) d = -d;
    dirLevel = d < 0 ? OD_CLOSED : OD_OPEN;
    gpio_set_level( dirPin, dirLevel );
    return true;
  }
  return false;
}

bool DualMotor::SetDirection( int d )
{
  if( Motor::SetDirection( d ))
  {
    gpio_set_level( dirPin2, dirLevel );
    return true;
  }
  return false;  
}

uint64_t Motor::InitMove( long s, unsigned long t, uint64_t now )
{
  int d = 1;
  
  if( pulseLevel != 0 )
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
  if( pulseLevel == 0 )
  {
    Pulse( 1 ); 
    nextStepTime += STEP_PULSE_US;
  }
  else
  {
    Pulse( 0 );
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

void IRAM_ATTR Motor::Pulse( bool on )
{
  if( on )
  {
    gpio_set_level( stepPin, OD_CLOSED );
    curPos += curDir;
  }
  else
  {
    gpio_set_level( stepPin, OD_OPEN );
  }
  pulseLevel = on;
}

void IRAM_ATTR DualMotor::Pulse( bool on )
{
  Motor::Pulse( on );
  gpio_set_level( stepPin2, on ? OD_CLOSED : OD_OPEN );
}

void DualMotor::PulseLeft( bool on )
{
  // Call the parent call so that only the left motor gets the pulse
  Motor::Pulse( on );
}

void DualMotor::PulseRight( bool on )
{
  gpio_set_level( stepPin2, on ? OD_CLOSED : OD_OPEN );
}

long Motor::GetPos( )
{
  return curPos;
}

void Motor::CalibrateStart( uint64_t now )
{
  cal_state = 1;
  cal_cycle = 1;
  nextStepTime = now + 1000;
}

#define CAL_STEP_SPEED   1000
#define CAL_PAUSE        50000
#define CAL_STALL        100

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
          // First cycle was the approach. Now back out and come bacl
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
            // Done! We're calibrated!
            cal_state = 0;
          }
        }
      }
      break;
      
    case 5 :
      Pulse( 0 );
      nextStepTime = now + ( CAL_STEP_SPEED << (cal_cycle - 1));
      cal_state = 4;
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
        if(( p & 1 ) == 0 || cal_count > 0 )
        {
          if(( p & 1 ) == 1 && cal_count > 0 ) cal_count--;
          if(( p & 2 ) == 2 ) cal_delta++;
          PulseLeft( 1 );
        } 
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
              cal_state = 6;
              nextStepTime = now + CAL_PAUSE;
            }
          }
        }
      }
      break;
      
    case 5 :
      PulseLeft( 0 );
      PulseRight( 0 );
      nextStepTime = now + CAL_STEP_SPEED * ( 1 << ( cal_cycle - 1));
      cal_state = 4;
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
  #include "Motor.h"
  
  extern QueueHandle_t g_cmd_queue;
    
  static void MotorMove( cmd_t *pCmd, uint64_t now );
  
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
           
      // Pull next move command from the queue
      if( xQueueReceiveFromISR( g_cmd_queue, &cmd, &xTaskWokenByReceive ))
      {
        MotorMove( &cmd, now );
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
  
  static bool IRAM_ATTR calibration_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
  {
    if( g_pNextMotorToStep )
    {
      g_pNextMotorToStep->CalibrateTask( edata->alarm_value );
    }
    PrepareNextStep( edata->alarm_value );    
    return false; // No need to yield
  }

  static bool IRAM_ATTR movement_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
  {
    if( g_pNextMotorToStep )
    {
      g_pNextMotorToStep->PrepareNextStep( edata->alarm_value );
    }
    PrepareNextStep( edata->alarm_value );
    return false; // No need to yield
  }
  
  // Move the tool position by the specified # of steps for x,y,z directions.
  // Duration of the motion determines the speed. d is in microseconds
  static void IRAM_ATTR MotorMove( cmd_t *pCmd, uint64_t now )
  { 
    g_MoveStart = now;
    
    gpio_set_level( TOOL_ON_RELAY, pCmd->flags & CMD_FLAG_SPINDLE_ON );
      
    // Is this a movement command
    if( pCmd->dx != 0 || pCmd->dy != 0 || pCmd->dz != 0 )
    {
      // Movements
      gptimer_event_callbacks_t cbs = {
        .on_alarm = movement_timer_callback,
      };
      ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_motorTimer, &cbs, NULL));
        
      // This calculates the interval between steps for each axis and returns the time
      // to the first step needs to occur ( NO_STEP_TIME if no move necessary).
      X.InitMove( pCmd->dx, pCmd->duration, now );
      Y.InitMove( pCmd->dy, pCmd->duration, now );
      Z.InitMove( pCmd->dz, pCmd->duration, now );
      PrepareNextStep( now );
    }
    else
    {
      // No Movement means "dwell"
      if( pCmd->duration > 0 )
      {
        // Check that the dwelve time is at least the duration of a step pulse.
        if( pCmd->duration < STEP_PULSE_US ) pCmd->duration = STEP_PULSE_US;
        
        X.InitMove( 0, 0, now );
        Y.InitMove( 0, 0, now );
        Z.InitMove( 0, 0, now );
        g_pNextMotorToStep = NULL; 
        
        gptimer_alarm_config_t alarm_config1 = {
          .alarm_count = now + pCmd->duration,
          .reload_count = 0,
          .flags = 0,
        };

        gptimer_event_callbacks_t cbs = {
          .on_alarm = movement_timer_callback,
        };
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_motorTimer, &cbs, NULL));
        gptimer_set_alarm_action(g_motorTimer, &alarm_config1);
        
        // If this was started from idle,
        if( now == 0 )
        {
          ESP_ERROR_CHECK(gptimer_start(g_motorTimer));
        }
      }
      else if( pCmd->flags & CMD_FLAG_CALIBRATION )
      {
        // Calibration
        gptimer_event_callbacks_t cbs = {
          .on_alarm = calibration_timer_callback,
        };
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_motorTimer, &cbs, NULL));
              
        X.CalibrateStart( now );
        Y.CalibrateStart( now );
        Z.CalibrateStart( now );
        
        PrepareNextStep( now );
      }
      else
      {
        cmd_t cmd;
        BaseType_t xTaskWokenByReceive = pdFALSE;
        
                     
        // Pull next move command from the queue
        if( xQueueReceiveFromISR( g_cmd_queue, &cmd, &xTaskWokenByReceive ))
        {
          MotorMove( &cmd, now );
        }
      }
    }
  }
  
  void MotorMoveFromIdle( cmd_t *pCmd )
  {
    MotorMove( pCmd, 0 );
    WaitForMotorIdle( );
  }
  
  void MotorGetPosition( long *pX, long *pY, long *pZ )
  {
    *pX = X.GetPos( );
    *pY = Y.GetPos( );
    *pZ = Z.GetPos( );
  }
       
       
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
  
  void MotorInit( )
  {          
    for( int i=0; i<MAX_DEBUG; i++ )
    {
      g_debug[i] = 0;
    }
    
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<TOOL_ON_RELAY | 1ULL<<MOTOR_ENABLE);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    ESP_ERROR_CHECK(gpio_set_level( TOOL_ON_RELAY, LOW ));
    ESP_ERROR_CHECK(gpio_set_level( MOTOR_ENABLE, HIGH ));

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
        .on_alarm = movement_timer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_motorTimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(g_motorTimer));
    
  }
}
