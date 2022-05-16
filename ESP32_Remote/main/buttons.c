
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include "hal/adc_types.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "nvs.h"

static const char* TAG = "Buttons";

#define ADC_BATTERY         ADC1_CHANNEL_0
#define ADC_X_AXIS          ADC1_CHANNEL_1
#define ADC_Y_AXIS          ADC1_CHANNEL_2

#define IO_RED_BUTTON       (5)
#define GPIO_RED_SEL        (1ULL<<IO_RED_BUTTON)

#define IO_GREEN_BUTTON     (4)
#define GPIO_GREEN_SEL      (1ULL<<IO_GREEN_BUTTON)

#define BUTTON_CNT          (2)

#define BUTTON_DEBOUCE_US   (30*1000) // 30ms

//ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
  #define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
  #define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
  #define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
  #define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif


#define ADC_ATTENUATION       ADC_ATTEN_DB_6
#define ADC_AVG               8
#define INPUT_CNT             3


typedef struct analog_input_t {
  int idx;                                        // ADC channel index
  adc1_channel_t channel;                         // ADC channel channel identifier
  int offset;                                     // Zero offset
  esp_adc_cal_characteristics_t characteristic;   // Factory calibration data
  int rawIdx;                                     // Index in the raw sample table
  int raw[ADC_AVG];                               // Circular buffer of raw samples
  int rawSum;                                     // Current sum of the samples in the raw buffer
  unsigned long count;                            // Number of samples

  int rawMin;
  int rawCenter;
  int rawMax;
  int stepCount;
  int curStep;
  
} analog_input_t;

typedef struct button_t {
  
  int gpio_num;
  int state;
  uint64_t timeDown;
  
} button_t;


static button_t button[BUTTON_CNT] = {
  { .gpio_num = IO_GREEN_BUTTON,
    .state = -1,
    .timeDown = 0,
  },
  { .gpio_num = IO_RED_BUTTON,
    .state = -1,
    .timeDown = 0,
  },
};

static analog_input_t input[INPUT_CNT] = {
  { .idx = 0, 
    .channel = ADC_BATTERY,
    .offset = 60,
    .characteristic = {0},
    .rawIdx = 0,
    .raw = {0},
    .rawSum = 0,
  },
  { .idx = 1, 
    .channel = ADC_X_AXIS,
    .offset = 0,
    .characteristic = {0},
    .rawIdx = 0,
    .raw = {0},
    .rawSum = 0, 
    
    .rawMin = 20,
    .rawCenter = 450,
    .rawMax = 850,
    .stepCount = 9,
    .curStep = 0,
  },
  { .idx = 2, 
    .channel = ADC_Y_AXIS,
    .offset = 0,
    .characteristic = {0},
    .rawIdx = 0,
    .raw = {0},
    .rawSum = 0,

    .rawMin = 20,
    .rawCenter = 450,
    .rawMax = 850,
    .stepCount = 9,
    .curStep = 0,    
  },
};

bool g_joyCalibrated = false;

int GetRawInputAverage( analog_input_t* pInput )
{
  int avg = ( pInput->rawSum / ADC_AVG );
  return esp_adc_cal_raw_to_voltage(avg, &pInput->characteristic) - pInput->offset;
}

int GetRawInput( int idx )
{
  return GetRawInputAverage( &input[idx] );
}

#define HYST                (10)

int GetStep( analog_input_t* pInput )
{
  if( !g_joyCalibrated ) return 0;
  
  int raw = GetRawInputAverage( pInput );
  int prevStep;
  do
  {
    int thrDown = pInput->rawCenter;
    int thrUp = pInput->rawCenter;
    
    prevStep = pInput->curStep;
    
    if( pInput->curStep < 0 ) // Negative (Left)
    {
      thrUp -= (-pInput->curStep * (pInput->rawCenter - pInput->rawMin)) / pInput->stepCount - HYST;
      thrDown -= ((1 - pInput->curStep) * (pInput->rawCenter - pInput->rawMin)) / pInput->stepCount + HYST;
    }
    else if( pInput->curStep > 0 ) // Positive (Right)
    {
      thrDown += (pInput->curStep * (pInput->rawMax - pInput->rawCenter)) / pInput->stepCount - HYST;
      thrUp += ((pInput->curStep + 1 ) * (pInput->rawMax - pInput->rawCenter)) / pInput->stepCount + HYST;
    }
    else // Zero (center)
    {
      thrDown -= (pInput->rawCenter - pInput->rawMin) / pInput->stepCount - HYST;
      thrUp += (pInput->rawMax - pInput->rawCenter) / pInput->stepCount + HYST;
    }
    if( raw > thrUp ) pInput->curStep++;
    else if( raw < thrDown ) pInput->curStep--;
    
    if( pInput->curStep >= pInput->stepCount ) pInput->curStep = pInput->stepCount-1;
    if( pInput->curStep <= -pInput->stepCount ) pInput->curStep = 1 - pInput->stepCount;
  }
  
  //ESP_LOGI( TAG, "Raw %d - Step %d - Down %d - Up %d", raw, pInput->curStep, thrDown, thrUp );
  
  while( pInput->curStep != prevStep );
  return pInput->curStep;
}

int GetButtonState( button_t *pButton )
{
  return pButton->state;
}

void CalibrateStep( int step )
{
  switch( step )
  {
  case 0 :
    input[1].rawCenter = GetRawInputAverage( &input[1] );
    input[2].rawCenter = GetRawInputAverage( &input[2] );
    break;
  case 1 :
    input[1].rawMin = GetRawInputAverage( &input[1] );  // Down
    break;
  case 2 :
    input[1].rawMax = GetRawInputAverage( &input[1] );  // Up
    break;
  case 3 :
    input[2].rawMin = GetRawInputAverage( &input[2] );  // Right
    break;
  case 4 :
    input[2].rawMax = GetRawInputAverage( &input[2] );  // Left
    break;    
  }
}

void CalibrationSave( )
{
  nvs_handle_t handle;
  esp_err_t err;
  
  err = nvs_open( "calibration", NVS_READWRITE, &handle );
  if( err == ESP_OK )
  {
    nvs_set_i32(handle, "center_x", input[1].rawCenter );
    nvs_set_i32(handle, "center_y", input[2].rawCenter );
    nvs_set_i32(handle, "down",     input[1].rawMin );
    nvs_set_i32(handle, "up",       input[1].rawMax );
    nvs_set_i32(handle, "right",    input[2].rawMin );
    nvs_set_i32(handle, "left",     input[2].rawMax );
    nvs_close( handle );
  }
  g_joyCalibrated = true;
}

void Buttons_Init( )
{
  esp_err_t ret;
  bool cali_enable = false;
  
    // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      // NVS partition was truncated and needs to be erased
      // Retry nvs_flash_init
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
  }
  ESP_ERROR_CHECK( err );
  
  nvs_handle_t handle;
  
  err = nvs_open( "calibration", NVS_READWRITE, &handle );
  if( err == ESP_OK )
  {
    int tmp;
    ESP_LOGI( TAG, "NVS Open( )");
    
    g_joyCalibrated = true;
    if( nvs_get_i32(handle, "center_x", &tmp ) == ESP_OK ) input[1].rawCenter = tmp; else g_joyCalibrated = false;
    if( nvs_get_i32(handle, "center_y", &tmp ) == ESP_OK ) input[2].rawCenter = tmp; else g_joyCalibrated = false;
    if( nvs_get_i32(handle, "down",     &tmp ) == ESP_OK ) input[1].rawMin = tmp; else g_joyCalibrated = false;
    if( nvs_get_i32(handle, "up",       &tmp ) == ESP_OK ) input[1].rawMax = tmp; else g_joyCalibrated = false;
    if( nvs_get_i32(handle, "right",    &tmp ) == ESP_OK ) input[2].rawMin = tmp; else g_joyCalibrated = false;
    if( nvs_get_i32(handle, "left",     &tmp ) == ESP_OK ) input[2].rawMax = tmp; else g_joyCalibrated = false;
    
    nvs_close( handle );
  }
  
  adc1_config_width(ADC_WIDTH_BIT_13);
  
  ret = esp_adc_cal_check_efuse(ADC_CALI_SCHEME);
  if (ret == ESP_ERR_NOT_SUPPORTED) {
    ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
  } else if (ret == ESP_ERR_INVALID_VERSION) {
    ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
  } else if (ret == ESP_OK) {
    cali_enable = true;
    for( int i=0; i<INPUT_CNT;i++ )
    {
      esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTENUATION, ADC_WIDTH_BIT_13, input[i].idx, &input[i].characteristic);
      adc1_config_channel_atten(input[i].channel, ADC_ATTENUATION);
    }
  } else {
    ESP_LOGE(TAG, "Invalid arg");
  }

  /* Configure Button GPIO */
  gpio_config_t io_conf = {
    .intr_type = GPIO_INTR_DISABLE, //disable interrupt
    .mode = GPIO_MODE_INPUT, //set as input mode
    .pin_bit_mask = GPIO_RED_SEL | GPIO_GREEN_SEL,
    .pull_down_en = 0, //disable pull-down mode
    .pull_up_en = 0, //disable pull-up mode
  };
  //configure GPIO with the given settings
  gpio_config(&io_conf);

}

void SampleInput( analog_input_t* pInput )
{
  pInput->count++;
  // For the battery ADC, only sample once every 64 since the battery 
  // voltage does not changed than often
  if( pInput->count >= ADC_AVG && pInput->idx == 0 && (pInput->count % 64) != 0 ) 
  {
    return;
  }
  
  pInput->raw[pInput->rawIdx] = adc1_get_raw(pInput->channel);
  pInput->rawSum += pInput->raw[pInput->rawIdx];
  pInput->rawIdx++;
  if( pInput->rawIdx >= ADC_AVG ) pInput->rawIdx = 0;
  pInput->rawSum -= pInput->raw[pInput->rawIdx];
}

void SampleButton( button_t *pButton, uint64_t now )
{
  int sample = gpio_get_level( pButton->gpio_num );
  
  if( sample == 0 ) // Pressed
  {
    if( pButton->timeDown == 0 )
    {
      pButton->timeDown = now;
    }
    else
    {
      if( pButton->timeDown + BUTTON_DEBOUCE_US < now )
      {
        if( pButton->state != 1 )
        {
          pButton->state = 1;
          ESP_LOGI( TAG, "Button %d DOWN", pButton->gpio_num );
        }
      }
    }
  }
  else
  {
    if( pButton->state != 0 )
    {
      pButton->state = 0;
      pButton->timeDown = 0;
      ESP_LOGI( TAG, "Button %d UP", pButton->gpio_num );
    }
  }
}

extern int g_joyX;
extern int g_joyY;
extern float g_Vbatt;
extern int g_Red;
extern int g_Green;

void Buttons_Loop( )
{
  uint64_t now = esp_timer_get_time( );
  
  for( int i=0; i<INPUT_CNT;i++ )
  {
    SampleInput( &input[i] );
  }
  
  for( int i=0; i<BUTTON_CNT;i++ )
  {
    SampleButton( &button[i], now );
  }
  
  g_joyX  = GetStep( &input[1] );
  g_joyY  = GetStep( &input[2] );
  g_Vbatt = GetRawInputAverage( &input[0] ) / 224.8f;

  g_Green = GetButtonState( &button[0] );  
  g_Red = GetButtonState( &button[1] );
  
}