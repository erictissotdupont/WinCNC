#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "driver/gpio.h"

#include "cnc.h"
#include "events.h"
#include "Motor.h"

void static Limits_GPIO_ISR(void* arg);
uint32_t g_limitState;

inline uint32_t GetLimitState( )
{
  return g_limitState;
}

#define TEST_LIMIT( IO, EVT, MASK ) \
  if( gpio_get_level( IO )) { g_limitState &= ~MASK; Events_ClearState( EVT ); } else \
                            { g_limitState |=  MASK; Events_SetState( EVT ); }
							
static void IRAM_ATTR Limits_GPIO_ISR(void* arg)
{  
  // Read all the limit sensors and update the state
  TEST_LIMIT( SOFT_LIMIT_ZL, CNC_STATE_POS_SENSOR_ZL, ZL_LIM )
  TEST_LIMIT( SOFT_LIMIT_ZR, CNC_STATE_POS_SENSOR_ZR, ZR_LIM )
  TEST_LIMIT( SOFT_LIMIT_Y,  CNC_STATE_POS_SENSOR_Y,  Y_LIM )
  TEST_LIMIT( SOFT_LIMIT_XL, CNC_STATE_POS_SENSOR_XL, XL_LIM )
  TEST_LIMIT( SOFT_LIMIT_XR, CNC_STATE_POS_SENSOR_XR, XR_LIM )

  // Update the hard limit state. The positive logic is reversed
  // as this pin is connected to the opto-coupler which is ON
  // when no limit switch is open (triggered).
  if( !gpio_get_level( HARD_LIMIT_SWITCH )) 
  { 
    g_limitState &= ~HARD_LIM;
    Events_ClearState( CNC_STATE_HARD_LIMIT ); 
  } 
  else
  { 
    g_limitState |= HARD_LIM; 
    Events_SetState( CNC_STATE_HARD_LIMIT ); 
  }

  // Update the motor disabled state. The positive logic is reversed
  // as the pin is connected to the open drain of the pulse detector
  if( !gpio_get_level( MOTOR_DISABLED))
  {
    Motor_Enable( false );
    Events_ClearState( CNC_STATE_ALL_CALIBRATED );
    Events_SetState( CNC_STATE_HARD_ERROR);
  }
  else
  {
    Events_ClearState( CNC_STATE_HARD_ERROR);
  }
}

#define LIMIT_MASK  ((1ULL << SOFT_LIMIT_XL)|\
                     (1ULL << SOFT_LIMIT_XR)|\
                     (1ULL << SOFT_LIMIT_Y) |\
                     (1ULL << SOFT_LIMIT_ZL)|\
                     (1ULL << SOFT_LIMIT_ZR)|\
					           (1ULL << HARD_LIMIT_SWITCH)|\
                     (1ULL << MOTOR_DISABLED))
					 
void Limits_Init( )
{  
  gpio_config_t io_conf = {};
  
  // INPUT (ISR)
  // -----------  
  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = LIMIT_MASK;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(gpio_isr_handler_add(SOFT_LIMIT_XL, Limits_GPIO_ISR, (void*)NULL));
  ESP_ERROR_CHECK(gpio_isr_handler_add(SOFT_LIMIT_XR, Limits_GPIO_ISR, (void*)NULL));
  ESP_ERROR_CHECK(gpio_isr_handler_add(SOFT_LIMIT_Y, Limits_GPIO_ISR, (void*)NULL));
  ESP_ERROR_CHECK(gpio_isr_handler_add(SOFT_LIMIT_ZL, Limits_GPIO_ISR, (void*)NULL));
  ESP_ERROR_CHECK(gpio_isr_handler_add(SOFT_LIMIT_ZR, Limits_GPIO_ISR, (void*)NULL));
  ESP_ERROR_CHECK(gpio_isr_handler_add(HARD_LIMIT_SWITCH, Limits_GPIO_ISR, (void*)NULL));
  ESP_ERROR_CHECK(gpio_isr_handler_add(MOTOR_DISABLED, Limits_GPIO_ISR, (void*)NULL));
    
  g_limitState = 0;
  // Get the initial state of the limit sensors and switch
  Limits_GPIO_ISR( NULL );
  
  /*
  // TIMER
  // -----
  gptimer_config_t timer_config = { 0 };
  
  timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  timer_config.direction = GPTIMER_COUNT_UP;
  timer_config.resolution_hz = 1000000; // 1MHz, 1 tick=1us

  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &g_limitsTimer));

  gptimer_event_callbacks_t cbs = {
      .on_alarm = Limits_TimerCallback,
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_limitsTimer, &cbs, NULL));
  ESP_ERROR_CHECK(gptimer_enable(g_limitsTimer));
  */
  
}