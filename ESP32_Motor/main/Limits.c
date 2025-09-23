#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"

#include "cnc.h"
#include "events.h"

#define LHP                            50    // Limit clock half period in uS

#define LIMIT_PULL_UP_TIMEOUT       20000
#define LIMIT_CONSECUTIVE_READ_LOW      4

static gptimer_handle_t g_limitsTimer = NULL;

void static Limits_GPIO_ISR(void* arg);
uint32_t g_limitState;

bool IRAM_ATTR Limits_TimerCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
  uint64_t nextT = 0;

  if( nextT )
  {
    gptimer_alarm_config_t alarm_config = { 0 };
    alarm_config.alarm_count = edata->alarm_value + nextT;
    gptimer_set_alarm_action(g_limitsTimer, &alarm_config);
  }
  else
  {
    ESP_ERROR_CHECK(gptimer_stop(g_limitsTimer));
    ESP_ERROR_CHECK(gptimer_set_raw_count(g_limitsTimer,0));
  }
  
  // No need to yield
  return false;
}

#define CNC_STATE_POS_SENSOR_XL           0x00010000L // The left side position sensor for the X axis is triggered
#define CNC_STATE_POS_SENSOR_XR           0x00008000L // Same for the right side X axis.
#define CNC_STATE_POS_SENSOR_ZL           0x00004000L // Same for the left side Z axis
#define CNC_STATE_POS_SENSOR_ZR           0x00002000L // Same for the right side Z axis.
#define CNC_STATE_POS_SENSOR_Y            0x00001000L // The Y axis position sensor is triggered

#define TEST_LIMIT( IO, EVT, MASK ) \
  if( gpio_get_level( IO )) { g_limitState &= ~MASK; Events_ClearState( EVT ); } else \
                            { g_limitState |=  MASK; Events_SetState( EVT ); }
							
#define XL_LIM  0x0001
#define XR_LIM  0x0002
#define Y_LIM   0x0004
#define ZL_LIM  0x0008
#define ZR_LIM  0x0010

static void IRAM_ATTR Limits_GPIO_ISR(void* arg)
{  
  TEST_LIMIT( LIMIT_ZL, CNC_STATE_POS_SENSOR_ZL, ZL_LIM )
  TEST_LIMIT( LIMIT_ZR, CNC_STATE_POS_SENSOR_ZR, ZR_LIM )
  TEST_LIMIT( LIMIT_Y,  CNC_STATE_POS_SENSOR_Y,  Y_LIM )
  TEST_LIMIT( LIMIT_XL, CNC_STATE_POS_SENSOR_XL, XL_LIM )
  TEST_LIMIT( LIMIT_XR, CNC_STATE_POS_SENSOR_XR, XR_LIM )
}

#define LIMIT_MASK  ((1ULL << LIMIT_XL)|\
                     (1ULL << LIMIT_XR)|\
                     (1ULL << LIMIT_Y) |\
                     (1ULL << LIMIT_ZL)|\
                     (1ULL << LIMIT_ZR)|\
					 (1ULL << LIMIT_SWITCH))
					 
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
  ESP_ERROR_CHECK(gpio_isr_handler_add(LIMIT_XL, Limits_GPIO_ISR, (void*)NULL));
  ESP_ERROR_CHECK(gpio_isr_handler_add(LIMIT_XR, Limits_GPIO_ISR, (void*)NULL));
  ESP_ERROR_CHECK(gpio_isr_handler_add(LIMIT_Y, Limits_GPIO_ISR, (void*)NULL));
  ESP_ERROR_CHECK(gpio_isr_handler_add(LIMIT_ZL, Limits_GPIO_ISR, (void*)NULL));
  ESP_ERROR_CHECK(gpio_isr_handler_add(LIMIT_ZR, Limits_GPIO_ISR, (void*)NULL));
  ESP_ERROR_CHECK(gpio_isr_handler_add(LIMIT_SWITCH, Limits_GPIO_ISR, (void*)NULL));
  
  Events_ClearState( CNC_STATE_LIMITS_INACTIVE );
  
  g_limitState = 0;
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