
extern "C"
{
#include "CNC.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "math.h"

#include "UDP.h"
#include "Events.h"
#include "Motor.h"
#include "Limits.h"
}
#include "Motor.hpp"

// Instantiation and configuration of the stepper motor controlers.
//-----------------------------------------------------------------
//                   Step IO,  Direction IO,  EndMsk,  Configuration flags,               StepByInch           L/R axis offset
DualMotor X(MOTOR_X_L_STEP, MOTOR_X_L_DIR, XL_LIM,
            MOTOR_X_R_STEP, MOTOR_X_R_DIR, XR_LIM, 
            CALIBRATION_REVERSED | REDUCED_RAPID_POSITIONING_SPEED, X_AXIS_RES, 
            0.0091f * X_AXIS_RES, 0.0f);

    Motor Y(MOTOR_Y_STEP, MOTOR_Y_DIR, Y_LIM, 
            CALIBRATION_REVERSED, Y_AXIS_RES);

DualMotor Z(MOTOR_Z_L_STEP, MOTOR_Z_L_DIR, ZL_LIM,
            MOTOR_Z_R_STEP, MOTOR_Z_R_DIR, ZR_LIM, 
            DIRECTION_REVERSED | CALIBRATION_OFFSET_INTERIOR | REDUCED_RAPID_POSITIONING_SPEED, 
            Z_AXIS_RES, 
            0.2231f * Z_AXIS_RES, 36.0f);

#define SLOW_FACTOR_STEP_COUNT    2
#define SLOW_FACTOR_RES           4
#define SLOW_FACTOR_ONE          (1 << SLOW_FACTOR_RES)

static int g_slowFactor = SLOW_FACTOR_ONE;
static long g_slowStepCount = 0;
static bool g_bSlowStartEnabled = false;

static bool g_bMotorEnabled = false;
static Motor *g_pNextMotorToStep = NULL;
static gptimer_handle_t g_motorTimer = NULL;
static bool g_bTimerRunning = false;
static enum { UNDEFINED,
              MOVEMENT,
              CALIBRATION } g_TimerMode = UNDEFINED;

extern QueueHandle_t g_cmd_queue;

static const char *TAG = "motor";

Motor::Motor(gpio_num_t sp, gpio_num_t dp, uint32_t em, unsigned long flags, unsigned long sbi)
{
  stepPin = sp;
  dirPin = dp;
  endMask = em;
  reverseDir = flags & DIRECTION_REVERSED;
  stepByInch = sbi;
  nextStepTime = NO_STEP_TIME;

  motorFlags = flags;

  if (flags & CALIBRATION_REVERSED)
  {
    cal_toward = -1;
    cal_away = 1;
  }
  else
  {
    cal_toward = 1;
    cal_away = -1;
  }

  if ((flags & REDUCED_RAPID_POSITIONING_SPEED) == 0)
  {
    // Normal speed
    minSpeedStep = SPEED_TO_STEP(stepByInch, MIN_SPEED);
    maxSpeedStep = SPEED_TO_STEP(stepByInch, MAX_SPEED);
  }
  else
  {
    // Reduced speed for axis with weak motors or lots of intertia
    minSpeedStep = SPEED_TO_STEP(stepByInch, MIN_SPEED) * 2;
    maxSpeedStep = SPEED_TO_STEP(stepByInch, MAX_SPEED) * 1.25;
  }

  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT_OD; // GPIO_MODE_OUTPUT_OD; // GPIO_MODE_OUTPUT; // GPIO_MODE_OUTPUT_OD
  io_conf.pin_bit_mask = (1ULL << stepPin) | (1ULL << dirPin);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));

  ESP_ERROR_CHECK(gpio_set_level(stepPin, OD_OPEN));
  pulseLevel = 0;

  ESP_ERROR_CHECK(gpio_set_level(dirPin, OD_OPEN));
  dirLevel = OD_OPEN;
}

DualMotor::DualMotor(gpio_num_t sp, gpio_num_t dp, uint32_t em, gpio_num_t sp2, gpio_num_t dp2, uint32_t em2, unsigned long flags, unsigned long sbi, long cof, float R)
    : Motor(sp, dp, em, flags, sbi)
{
  stepPin2 = sp2;
  dirPin2 = dp2;
  endMask2 = em2;
  cal_offset = cof;

  if (R != 0.0f && R != 2.0f)
  {
    cal_R = round((2.0f - R) / ((1.0f / R) - 1.0f));
  }
  else
  {
    // Avoid division by zero in the calibration algorithm
    cal_R = 1;
  }

  cal_cycle = 1;

  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT_OD; // GPIO_MODE_OUTPUT_OD; // GPIO_MODE_OUTPUT; // GPIO_MODE_OUTPUT_OD
  io_conf.pin_bit_mask = (1ULL << stepPin2) | (1ULL << dirPin2);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));

  ESP_ERROR_CHECK(gpio_set_level(stepPin2, OD_OPEN));
  ESP_ERROR_CHECK(gpio_set_level(dirPin2, dirLevel));
}

void Motor::Reset()
{
  curPos = 0;
  curDir = 0;
  moveLength = 0;
  moveStep = 0;
  moveDuration = 0;
}

void Motor::SetIdle()
{
  moveStartTime = 0;
  lastPulseTime = 0;
  previousSpeed = 0;
}

int Motor::GetLimit()
{
  return (GetLimitState() & endMask) ? 1 : 0;
}

int DualMotor::GetLimit()
{
  int ret = 0;
  if (GetLimitState() & endMask)
    ret |= 1;
  if (GetLimitState() & endMask2)
    ret |= 2;
  return ret;
}

bool IRAM_ATTR Motor::SetDirection(int d)
{
  if (curDir != d)
  {
    curDir = d;
    if (reverseDir)
      d = -d;
    dirLevel = d < 0 ? OD_CLOSED : OD_OPEN;
    gpio_set_level(dirPin, dirLevel);
    return true;
  }
  return false;
}

bool IRAM_ATTR DualMotor::SetDirection(int d)
{
  if (Motor::SetDirection(d))
  {
    gpio_set_level(dirPin2, dirLevel);
    return true;
  }
  return false;
}

long __inline Motor::GetStepDuration( )
{
  if( g_bSlowStartEnabled )
  {
    return (( stepDuration * g_slowFactor ) >> SLOW_FACTOR_RES);
  }
  else
  {
    return stepDuration;
  }
}

void IRAM_ATTR Motor::InitMove(long s, unsigned long t, uint64_t now)
{
  int d = 1;

  moveStartTime = now;
  hasQueueHasBeenPeeked = false;

  if (pulseLevel != 0)
  {
    ESP_LOGE(TAG, "Previous move did not clear the pulse");
    assert(false);
  }

  // Backward direction
  if (s < 0)
  {
    d = -1;
    s = -s;
  }

  // If we do not return "NO_STEP_TIME" the move will
  // occur no matter what.

  moveLength = s;
  moveDuration = t;
  nextStepTime = 0;

  if (s > 0)
  {
    SetDirection(d);
    // Linear motion (G1)
    if (moveDuration != 0)
    {
      // Calculate the step duration
      stepDuration = t / moveLength - STEP_PULSE_US;
      
      if (stepDuration < (long)(STEP_PULSE_US))
      {
        ESP_LOGE(TAG, "Step duration too small: %ld", stepDuration);
        assert(false);
      }
    }
    // Rapid positionning (G0)
    else
    {
      lastPulseTime = now;
      stepDuration = minSpeedStep;
      decelStart = 0;
    }
    decelDist = moveLength;
    moveStep = 0;
  }
  else
  {
    moveStep = 0;
    nextStepTime = NO_STEP_TIME;
    return;
  }

  nextStepTime = GetStepDuration( ) + lastPulseTime;
  if (nextStepTime < now)
  {
    nextStepTime = nextStepTime - lastPulseTime + now;
  }
}

inline uint64_t IRAM_ATTR Motor::GetNextStepTime()
{
  return nextStepTime;
}

void IRAM_ATTR Motor::MovementTask(uint64_t now)
{
  if (pulseLevel == 0)
  {
    Pulse(1);
    nextStepTime += STEP_PULSE_US;
  }
  else
  {
    lastPulseTime = now;

    Pulse(0);
    moveStep++;

    // Move is complete.
    if (moveStep >= moveLength)
    {
      nextStepTime = NO_STEP_TIME;
    }
    // Movement with duration means linear motion (G1).
    else if (moveDuration != 0)
    {
      
      if (isTheLongMove)
      {
        if( moveStep >= decelDist )
        {
          cmd_t cmd;
          if( !hasQueueHasBeenPeeked )
          {
            hasQueueHasBeenPeeked = true;
            if( xQueuePeekFromISR( g_cmd_queue, &cmd ) == pdPASS )
            {
              if(( cmd.flags & CMD_SLOW_START_ENABLED ) == 0 )
              {
                // Cancel the if the next command is not a slow start command.
                g_bSlowStartEnabled = false;
              }
            }
            else
            {
              // Force the slow start if this is the last command
              g_bSlowStartEnabled = true;
            }
          }

          g_slowStepCount--;
          if (g_slowStepCount <= 0)
          {
            g_slowStepCount = SLOW_FACTOR_STEP_COUNT;
            g_slowFactor++;
          }
        }
        else if( g_slowFactor > SLOW_FACTOR_ONE)
        {
          g_slowStepCount--;
          if (g_slowStepCount <= 0)
          {
            g_slowStepCount = SLOW_FACTOR_STEP_COUNT;
            g_slowFactor--;
            if( g_slowFactor == SLOW_FACTOR_ONE )
            {
              decelDist = moveLength - moveStep;
            }
          }
        }
      }

      nextStepTime += GetStepDuration( );
    }
    // Zero duration means rapid positioning motion (G0).
    else
    {
      uint64_t t = lastPulseTime - moveStartTime;
      // Deceleration phase. Checking for deceleration first handles
      // the case where the G0 movement is so short that there is not
      // enough distance to reach full speed. As the acceleration phase
      // increases the decelDistance, this deceleration will take over
      // when the movement reaches mid point.
      if (moveStep >= decelDist)
      {
        if (decelStart == 0)
        {
          // Capture the time when the deceleration started
          decelStart = t;
        }
        // Time spent decelerating
        t = t - decelStart;
        if (t < decelTime)
        {
          // Expected time decelerating
          t = decelTime - t;
        }
        else
        {
          t = 0;
        }
        stepDuration = STEP_FROM_RAMP(minSpeedStep, maxSpeedStep, t);
      }
      // Acceleration phase
      else if (t <= RAMP_TIME)
      {
        stepDuration = STEP_FROM_RAMP(minSpeedStep, maxSpeedStep, t);

        // Save the distance and time when deceleration should end
        // If the movement is so short that full speed can't be
        // reached, the previous test will pass and deceleration
        // will start before the movement has reached full speed.
        decelDist = moveLength - moveStep;
        decelTime = t;
      }
      // In the constant speed phase, just update the time for the
      // next half step.
      nextStepTime += stepDuration;
    }
  }
}

void IRAM_ATTR Motor::Pulse(bool on)
{
  if (on)
  {
    gpio_set_level(stepPin, OD_CLOSED);
    curPos += curDir;
  }
  else
  {
    gpio_set_level(stepPin, OD_OPEN);
  }
  pulseLevel = on;
}

void IRAM_ATTR DualMotor::Pulse(bool on)
{
  Motor::Pulse(on);
  gpio_set_level(stepPin2, on ? OD_CLOSED : OD_OPEN);
}

void IRAM_ATTR DualMotor::PulseLeft(bool on)
{
  // Call the parent call so that only the left motor gets the pulse
  Motor::Pulse(on);
}

void IRAM_ATTR DualMotor::PulseRight(bool on)
{
  gpio_set_level(stepPin2, on ? OD_CLOSED : OD_OPEN);
}

long Motor::GetPos()
{
  return curPos;
}

extern "C"
{
  static uint64_t g_ManUpdateTimeout;
  int g_ManX, g_ManY, g_ManZ;

  void IRAM_ATTR Motor_PrepareNextStep( uint64_t now )
  {
    // Check which axis is the next one to be stepped
    if ((X.GetNextStepTime() < Y.GetNextStepTime()) && (X.GetNextStepTime() < Z.GetNextStepTime()))
    {
      g_pNextMotorToStep = &X;
    }
    else if (Y.GetNextStepTime() < Z.GetNextStepTime())
    {
      g_pNextMotorToStep = &Y;
    }
    else if (Z.GetNextStepTime() != NO_STEP_TIME)
    {
      g_pNextMotorToStep = &Z;
    }
    else
    {
      cmd_t cmd;
      BaseType_t xTaskWokenByReceive = pdFALSE;

      // Pull next move command from the queue
      if (xQueueReceiveFromISR(g_cmd_queue, &cmd, &xTaskWokenByReceive))
      {
        Motor_PrepareNextCommand(&cmd, now);

        // The above function will call this function again which will set
        // the timer. Not returning here causes the timer to be initialized
        // twice and cause jerkiness in back to back movements.
        return;
      }
      else
      {
        // No more movements to perform, stop the timer, reset
        // the counter and go idle
        ESP_ERROR_CHECK(gptimer_stop(g_motorTimer));
        g_bTimerRunning = false;
        ESP_ERROR_CHECK(gptimer_set_raw_count(g_motorTimer, 0));
        g_pNextMotorToStep = NULL;
        Events_SignalMotorIdleFromISR();
        X.SetIdle();
        Y.SetIdle();
        Z.SetIdle();
      }
    }

    if (g_pNextMotorToStep)
    {
      gptimer_alarm_config_t alarm_config1 = {
          .alarm_count = g_pNextMotorToStep->GetNextStepTime(),
          .reload_count = 0,
          .flags = 0,
      };

      gptimer_set_alarm_action(g_motorTimer, &alarm_config1);

      // If this was started from idle,
      if (!g_bTimerRunning)
      {
        ESP_ERROR_CHECK(gptimer_start(g_motorTimer));
        g_bTimerRunning = true;
      }
    }
  }

  bool IRAM_ATTR Motor_CalibrationTimerCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
  {
    if (g_pNextMotorToStep)
    {
      g_pNextMotorToStep->CalibrateTask(edata->alarm_value);
    }
    Motor_PrepareNextStep(edata->alarm_value);
    return false; // No need to yield
  }

  bool IRAM_ATTR Motor_MovementTimerCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
  {
    if (g_pNextMotorToStep)
    {
      g_pNextMotorToStep->MovementTask(edata->alarm_value);
    }
    Motor_PrepareNextStep(edata->alarm_value);
    return false; // No need to yield
  }

  // Move the tool position by the specified # of steps for x,y,z directions.
  // Duration of the motion determines the speed. d is in microseconds
  void IRAM_ATTR Motor_PrepareNextCommand(cmd_t *pCmd, uint64_t now)
  {
    gpio_set_level(TOOL_ON_RELAY, pCmd->flags & CMD_FLAG_SPINDLE_ON);

    // Is this a movement command
    if (pCmd->dx != 0 || pCmd->dy != 0 || pCmd->dz != 0)
    {
      long newPos[5];
      newPos[0] = X.GetPos() + pCmd->dx;
      newPos[1] = Y.GetPos() + pCmd->dy;
      newPos[2] = Z.GetPos() + pCmd->dz;
      newPos[3] = pCmd->duration;
      newPos[4] = pCmd->flags & ~CMD_FLAGS_EXCLUDED_FROM_CRC_MASK;

      uint8_t newCRC = crc8((unsigned char *)newPos, sizeof(newPos), 0xFF);

      if (newCRC != (pCmd->flags & CMD_FLAGS_CRC_MASK))
      {
        Events_SetState(CNC_STATE_MOTOR_CRC_ERROR);
      }
      else if (((Events_GetState() & CNC_STATE_ALL_CALIBRATED) == CNC_STATE_ALL_CALIBRATED) &&
               ((newPos[0] < X_AXIS_MIN) || (newPos[0] > X_AXIS_MAX) ||
                (newPos[1] < Y_AXIS_MIN) || (newPos[1] > Y_AXIS_MAX) ||
                (newPos[2] < Z_AXIS_MIN) || (newPos[2] > Z_AXIS_MAX)))
      {
        Events_SetState(CNC_STATE_LOGICAL_LIMIT_ERROR);
      }
      else
      {
        if (g_TimerMode != MOVEMENT)
        {
          // Movements
          g_TimerMode = MOVEMENT;

          gptimer_event_callbacks_t cbs = {
              .on_alarm = Motor_MovementTimerCallback,
          };
          ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_motorTimer, &cbs, NULL));
        }

        g_slowFactor = (pCmd->flags & CMD_SLOW_START_MASK) >> SLOW_FACTOR_FLAG_SHIFT;
        g_slowStepCount = SLOW_FACTOR_STEP_COUNT;
        g_bSlowStartEnabled = ( pCmd->flags & CMD_SLOW_START_ENABLED ) != 0;

        X.isTheLongMove = false;
        Y.isTheLongMove = false;
        Z.isTheLongMove = false;

        if( ABS(pCmd->dx) > ABS(pCmd->dy) && ABS(pCmd->dx) > ABS(pCmd->dz))
          X.isTheLongMove = true;
        else if( ABS(pCmd->dy) > ABS(pCmd->dz))
          Y.isTheLongMove = true;
        else  
          Z.isTheLongMove = true;

        // This calculates the interval between steps for each axis and returns the time
        // to the first step needs to occur ( NO_STEP_TIME if no move necessary).
        X.InitMove(pCmd->dx, pCmd->duration, now );
        Y.InitMove(pCmd->dy, pCmd->duration, now );
        Z.InitMove(pCmd->dz, pCmd->duration, now );
        Motor_PrepareNextStep(now);

        if (pCmd->flags & CMD_FLAG_MANUAL_MOVE)
        {
          Motor_PrepareManualMove();
        }
      }
    }
    else
    {
      // No Movement with non zero duration means "dwell"
      if (pCmd->duration != 0)
      {
        // Check that the dwelve time is at least the duration of a step pulse.
        if (pCmd->duration < STEP_PULSE_US)
          pCmd->duration = STEP_PULSE_US;

        X.InitMove(0, 0, now);
        Y.InitMove(0, 0, now);
        Z.InitMove(0, 0, now);
        g_pNextMotorToStep = NULL;

        gptimer_alarm_config_t alarm_config1 = {
            .alarm_count = now + pCmd->duration,
            .reload_count = 0,
            .flags = 0,
        };

        if (g_TimerMode != MOVEMENT)
        {
          g_TimerMode = MOVEMENT;

          gptimer_event_callbacks_t cbs = {
              .on_alarm = Motor_MovementTimerCallback,
          };
          ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_motorTimer, &cbs, NULL));
        }
        gptimer_set_alarm_action(g_motorTimer, &alarm_config1);

        // If this was started from idle,
        if (!g_bTimerRunning)
        {
          ESP_ERROR_CHECK(gptimer_start(g_motorTimer));
          g_bTimerRunning = true;
        }
      }
      else if (pCmd->flags & CMD_FLAG_CALIBRATION)
      {
        if (g_TimerMode != CALIBRATION)
        {
          // Calibration
          g_TimerMode = CALIBRATION;

          gptimer_event_callbacks_t cbs = {
              .on_alarm = Motor_CalibrationTimerCallback,
          };
          ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_motorTimer, &cbs, NULL));
        }

        Events_SetState(CNC_STATE_CALIBRATING);

        X.CalibrateStart(now, X_AXIS_LENGTH, CNC_STATE_X_CALIBRATED);
        Y.CalibrateStart(now, Y_AXIS_LENGTH, CNC_STATE_Y_CALIBRATED);
        Z.CalibrateStart(now, Z_AXIS_LENGTH, CNC_STATE_Z_CALIBRATED);

        Motor_PrepareNextStep(now);
      }
      else // No movement commands
      {
        cmd_t cmd;
        BaseType_t xTaskWokenByReceive = pdFALSE;

        if (pCmd->flags & CMD_CALIBRATION_COMPLETE)
        {
          UDP_ResetPosition();
          Events_ClearState(CNC_STATE_CALIBRATING);
        }

        // Pull next move command from the queue
        if (xQueueReceiveFromISR(g_cmd_queue, &cmd, &xTaskWokenByReceive))
        {
          Motor_PrepareNextCommand(&cmd, now);
        }
        else
        {
          // No more movements to perform, stop the timer, reset
          ESP_ERROR_CHECK(gptimer_stop(g_motorTimer));
          g_bTimerRunning = false;
          ESP_ERROR_CHECK(gptimer_set_raw_count(g_motorTimer, 0));
          g_pNextMotorToStep = NULL;

          Events_SignalMotorIdleFromISR();

          X.SetIdle();
          Y.SetIdle();
          Z.SetIdle();
        }
      }
    }
  }

  void Motor_Enable(bool on)
  {
    if (on && g_bMotorEnabled == false)
    {
      ESP_LOGW(TAG, "Enabling the montor controllers.");
      g_bMotorEnabled = true;
      gpio_set_level(MOTOR_ENABLE, HIGH);

      // Sleep for 100ms to allow the motors to energize
      // before any movement is attempted.
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    else if (!on && g_bMotorEnabled == true)
    {
      g_bMotorEnabled = false;
      gpio_set_level(MOTOR_ENABLE, LOW);
    }
  }

  void Motor_MoveIfIdle()
  {
    cmd_t cmd;
    if (Events_IsMotorIdle())
    {
      if (xQueueReceive(g_cmd_queue, &cmd, 0) == pdTRUE)
      {
        // ESP_LOGW( TAG, "Limit 0x%lx, State 0x%lx", GetLimitState( ), Events_GetState( ));

        if ((Events_GetState() & (CNC_STATE_ERROR_MASK | CNC_STATE_HARD_LIMIT)) == 0)
        {
          Motor_Enable(true);
        }

        Events_SignalMotorNotIdle( );
        Motor_PrepareNextCommand( &cmd,0 );
      }
    }
  }

#define MANUAL_STEP (0.0078125f) // 1/128 inch steps
#define MAN_SPEED_IPS (0.125f)
#define MAN_STEP_US (1000000.0f * MANUAL_STEP / MAN_SPEED_IPS)
#define MAN_CONT_THRLD (4)
#define MANUAL_MAX_SPEED (8)
#define MANUAL_SLOW_STEP_US (500000)

  void IRAM_ATTR Motor_GetManualInterval(int speed, long *pAxis, cmd_t *pMoveCmd, long steps)
  {
    int dir = 1;
    cmd_t dwellCmd = {.dx = 0, .dy = 0, .dz = 0, .duration = 0, .flags = 0};
    BaseType_t xTaskWokenByReceive = pdFALSE;

    if (speed < 0)
    {
      speed = -speed;
      dir = -1;
    }
    if (speed > MANUAL_MAX_SPEED)
    {
      speed = MANUAL_MAX_SPEED;
    }
    if (speed <= MAN_CONT_THRLD)
    {
      dwellCmd.duration = MANUAL_SLOW_STEP_US / (1 << (speed - 1));
      xQueueSendFromISR(g_cmd_queue, &dwellCmd, &xTaskWokenByReceive);

      pMoveCmd->duration = 0;
      *pAxis = steps * dir;
    }
    else
    {
      pMoveCmd->duration = MAN_STEP_US;
      *pAxis = steps * dir * (speed - MAN_CONT_THRLD);
    }
  }

  void IRAM_ATTR Motor_PrepareManualMove()
  {
    cmd_t cmd = {.dx = 0, .dy = 0, .dz = 0, .duration = 0, .flags = 0};
    BaseType_t xTaskWokenByReceive = pdFALSE;

    if (esp_timer_get_time() > g_ManUpdateTimeout)
    {
      g_ManX = 0;
      g_ManY = 0;
      g_ManZ = 0;
    }

    if ((g_ManX > g_ManY && g_ManX > g_ManZ) ||
        (g_ManX < g_ManY && g_ManX < g_ManZ))
    {
      Motor_GetManualInterval(g_ManX, &cmd.dx, &cmd, MANUAL_STEP * X_AXIS_RES);
    }
    else if ((g_ManY > g_ManZ || g_ManY < g_ManZ) &&
             (g_ManY != 0))
    {
      Motor_GetManualInterval(g_ManY, &cmd.dy, &cmd, MANUAL_STEP * Y_AXIS_RES);
    }
    else if (g_ManZ != 0)
    {
      Motor_GetManualInterval(g_ManZ, &cmd.dz, &cmd, MANUAL_STEP * Z_AXIS_RES);
    }
    else
    {
      // No more move
      Events_ClearState(CNC_STATE_MANUAL_MODE);
      return;
    }

    cmd.flags = CMD_FLAG_MANUAL_MOVE;
    cmd.flags |= UDP_GetCRCAndUpdatePosition(&cmd);
    xQueueSendFromISR(g_cmd_queue, &cmd, &xTaskWokenByReceive);
  }

  void Motor_ManualMove(int dX, int dY, int dZ)
  {
    g_ManX = dX;
    g_ManY = dY;
    g_ManZ = dZ;

    // Set the time when continuation of this manual move should stop
    // Controller must send refresh messages continuously
    g_ManUpdateTimeout = esp_timer_get_time() + 1000000;

    if (Events_IsMotorIdle())
    {
      Events_SetState(CNC_STATE_MANUAL_MODE);
      Motor_PrepareManualMove();
      Motor_MoveIfIdle();
    }
  }

  void Motor_GetPosition(long *pX, long *pY, long *pZ)
  {
    *pX = X.GetPos();
    *pY = Y.GetPos();
    *pZ = Z.GetPos();
  }

  void Motor_Init()
  {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << TOOL_ON_RELAY | 1ULL << MOTOR_ENABLE);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Tool off by default
    ESP_ERROR_CHECK(gpio_set_level(TOOL_ON_RELAY, LOW));

    // Only enable the motors pulse if there is no error condition
    if ((Events_GetState() & CNC_STATE_ERROR_MASK) == 0)
    {
      Motor_Enable(true);
    }

    X.Reset();
    Y.Reset();
    Z.Reset();

    gptimer_config_t timer_config;

    memset(&timer_config, 0x00, sizeof(timer_config));
    timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = 1000000; // 1MHz, 1 tick=1us

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &g_motorTimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = Motor_MovementTimerCallback,
    };
    g_TimerMode = MOVEMENT;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_motorTimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(g_motorTimer));
  }

  void Motor_InitSlowStart(cmd_t *pCmd)
  {
    pCmd->flags &= ~CMD_SLOW_START_MASK;
    pCmd->flags |= SLOW_FACTOR_ONE << SLOW_FACTOR_FLAG_SHIFT;

    X.CheckSlowStart( pCmd->dx, pCmd );
    Y.CheckSlowStart( pCmd->dy, pCmd );
    Z.CheckSlowStart( pCmd->dz, pCmd );
  }
}

#define SPEED_THRESHOLD 0.5f  // (steps / milliseconds

void Motor::CheckSlowStart( long d, cmd_t *pCmd )
{
  if( d == 0 )
  {
    previousSpeed = 0.0f;
  }
  else
  {
    // Get the current slow factor for the move command
    long currentSlowFactor = (pCmd->flags & CMD_SLOW_START_MASK) >> SLOW_FACTOR_FLAG_SHIFT;
    // Calculate the absolute (always positive) stepp duration
    long stepTime = (long)pCmd->duration / ABS(d);
    // Calculate the speed in steps per second
    float speed = 1000000.0f / (float)stepTime;
    // Calculate the factor this move should apply to start at the safe startup speed
    long newSlowFactor = ( minSpeedStep * SLOW_FACTOR_ONE ) / stepTime; 

    // If that factor is greater than the current slow factor, update the move command
    // to use that new slow factor
    if( newSlowFactor > currentSlowFactor )
    {
      pCmd->flags &= ~(CMD_SLOW_START_MASK | CMD_SLOW_START_ENABLED);
      pCmd->flags |= newSlowFactor << SLOW_FACTOR_FLAG_SHIFT;
    }
    
    // If the change in speed is an acceleration or deceleration greater than 1000 steps 
    // per second, enable the slow start for this move command
    if((( speed - previousSpeed ) > 1000.0f ) ||
       (( speed - previousSpeed ) < -1000.0f ))
    {      
      pCmd->flags |= CMD_SLOW_START_ENABLED;
    }

    previousSpeed = speed;
  }
}