#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"

#include "cnc.h"

#define LHP                   50    // Limit clock half period in uS

static gptimer_handle_t g_limitsTimer = NULL;
static int g_state = 0;
uint32_t g_limitState = 0;
uint32_t g_CRCErrorCount = 0;
uint32_t g_ErrorA = 0;
uint32_t g_ErrorB = 0;
uint32_t g_ErrorC = 0;
uint32_t g_BadLimitData = 0;

extern unsigned char crc8_table[256];

static void gpio_isr_handler(void* arg);

bool IRAM_ATTR limits_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
  uint64_t nextT = 0;
  static uint32_t mask = 0;
  static uint32_t data = 0;

  switch( g_state )
  {
    default:
      // Invalid state. Reset to zero and force the output clock to zero
      g_state = 0;
      gpio_set_level( LIMIT_OUT, LOW );
      break;

    case 1: // Debounce interrupt g_state by checking signal stays low for half a period
      gpio_set_level( LIMIT_OUT, LOW );
      if( gpio_get_level( LIMIT_IN ) == HIGH )
      {
        // Input signal didn't stay low, stop the timer and go back to idle g_state 
        g_state = 0;
        g_ErrorA++;
      }
      else
      {  
        g_state = 2;
        nextT = LHP;
      }
      break;

    case 2: // Wait for half a period and check the limit sensor has released the interrupt g_state
      if( gpio_get_level( LIMIT_IN ) == LOW )
      {
        // Interrupt g_state didn't clear, stop the timer and go back to idle
        g_state = 0;
        g_ErrorB++;
      }
      else
      {
        mask = 1;
        data = 0;
        gpio_set_level( LIMIT_OUT, HIGH );
        g_state = 3;
        nextT = LHP;
      }
      break;

    case 3:
      gpio_set_level( LIMIT_OUT, LOW );
      g_state = 4;
      nextT = LHP;
      break;

    case 4:
      if( gpio_get_level( LIMIT_IN ) == LOW )
      {
        data = data | mask;
      }
      gpio_set_level( LIMIT_OUT, HIGH );

      mask = mask << 1;
      if( mask )
      {
        // More bits incoming. Keep clocking and receiving data
        g_state = 3;
      }
      else
      {
        uint8_t crc = 0xFF;
        crc = crc8_table[ ( crc ^ ( data       )) & 0xFF ];
        crc = crc8_table[ ( crc ^ ( data >> 8  )) & 0xFF ];
        crc = crc8_table[ ( crc ^ ( data >> 16 )) & 0xFF ];
        
        if( crc == (( data >> 24 ) & 0xFF ))
        {
          g_limitState = data;
          // Serial.printf("Limit %lx\n", data );

          // Got the correct CRC. Clock one more to let the limit sensor
          // that we're okay.
          g_state = 5;
        }
        else
        {
          // Skip the extra clock cycle and go back to idle. The sensor
          // will timeout and retry
          g_state = 7;
          g_CRCErrorCount++;
          g_BadLimitData = data;
        }          
      }
      nextT = LHP;
      break;

    case 5:
      gpio_set_level( LIMIT_OUT, LOW ); 
      nextT = LHP;
      g_state = 6;
      break;
    case 6:
      gpio_set_level( LIMIT_OUT, HIGH );
      nextT = LHP;
      g_state = 7;
      break;
    case 7:
      gpio_set_level( LIMIT_OUT, LOW );
      g_state = 0;      
      break;
  }

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
    gpio_isr_handler_add(LIMIT_IN, gpio_isr_handler, (void*)NULL);
  }
  
  // No need to yield
  return false;
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
  assert( g_state == 0 );
  
  gptimer_alarm_config_t alarm_config = { 0 };
  
  if( gpio_get_level( LIMIT_IN ) == LOW )
  {    
    g_state = 1;
    // Acknowledge the interrupt g_state by driving the output low
    gpio_set_level( LIMIT_OUT, HIGH );
    // Start the timer to generate the clock and read the data from 
    // the limit peripherial device
    alarm_config.alarm_count = LHP;
    gptimer_set_alarm_action(g_limitsTimer, &alarm_config);
    gptimer_start(g_limitsTimer);
    
    gpio_isr_handler_remove( LIMIT_IN );
  }
  else
  {
    g_ErrorC++;
  }
}


void LimitsInit( )
{
  
  // OUTPUT
  // ------  
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL<<LIMIT_OUT);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));

  ESP_ERROR_CHECK(gpio_set_level( LIMIT_OUT, LOW ));
  
  // INPUT (ISR)
  // -----------  
  io_conf.intr_type = GPIO_INTR_NEGEDGE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL<<LIMIT_IN);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(gpio_isr_handler_add(LIMIT_IN, gpio_isr_handler, (void*)NULL));
  
  // TIMER
  // -----
  gptimer_config_t timer_config = { 0 };
  
  timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  timer_config.direction = GPTIMER_COUNT_UP;
  timer_config.resolution_hz = 1000000; // 1MHz, 1 tick=1us

  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &g_limitsTimer));

  gptimer_event_callbacks_t cbs = {
      .on_alarm = limits_timer_callback,
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_limitsTimer, &cbs, NULL));
  ESP_ERROR_CHECK(gptimer_enable(g_limitsTimer));
}