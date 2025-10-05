#include "..\..\CNC_Protocol.h"
#include "freertos/FreeRTOS.h"
// For ESP_LOGx and ESP_ERROR_CHECK
#include "esp_log.h"
#include "string.h"

//   A X I S   
// -----------
// Axis resolution in inches per step 
#define Z_AXIS_RES           2538.572f
#define X_AXIS_RES           2033.132f
#define Y_AXIS_RES           2033.132f

// Maximum positions per axis in steps
#define X_AXIS_MIN           ((long)(   0.00f * X_AXIS_RES))
#define X_AXIS_MAX           ((long)( 48.875f * X_AXIS_RES))
#define Y_AXIS_MIN           ((long)(   0.00f * Y_AXIS_RES))
#define Y_AXIS_MAX           ((long)( 33.125f * Y_AXIS_RES))
#define Z_AXIS_MIN           ((long)( -7.625f * Z_AXIS_RES))
#define Z_AXIS_MAX           ((long)(   0.00f * Z_AXIS_RES))

// Axis length in steps
#define X_AXIS_LENGTH        (X_AXIS_MAX-X_AXIS_MIN)
#define Y_AXIS_LENGTH        (Y_AXIS_MAX-Y_AXIS_MIN)
#define Z_AXIS_LENGTH        (Z_AXIS_MAX-Z_AXIS_MIN)

//-----------------------------------------------------------------------------
//                            G P I O   M A P P I N G 
//-----------------------------------------------------------------------------
#define MOTOR_X_L_DIR        (gpio_num_t)4
#define MOTOR_X_L_STEP       (gpio_num_t)5
#define MOTOR_X_R_DIR        (gpio_num_t)6
#define MOTOR_X_R_STEP       (gpio_num_t)7
#define MOTOR_Z_L_DIR        (gpio_num_t)15
#define MOTOR_Z_L_STEP       (gpio_num_t)16
#define MOTOR_Z_R_DIR        (gpio_num_t)17
#define MOTOR_Z_R_STEP       (gpio_num_t)9  // Used to be 18. Conflicted with onboard RGB LED
#define MOTOR_Y_DIR          (gpio_num_t)8
#define MOTOR_Y_STEP         (gpio_num_t)3

#define TOOL_ON_RELAY        (gpio_num_t)11
#define MOTOR_ENABLE         (gpio_num_t)12

#ifdef CONFIG_IDF_TARGET_ESP32S2
  #define BLINK_GPIO         (gpio_num_t)18
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  #define BLINK_GPIO         (gpio_num_t)48
#else
  #error "This code is only for ESP32-S2 or ESP32-S3"
#endif

#define BOOT_GPIO            (gpio_num_t)0

#define SOFT_LIMIT_XR             (gpio_num_t)35 // Warning : Connection board hardware bug. Green / Green 
                                            // white on the RJ45 socket are swapped. XR and XL are swapped.
#define SOFT_LIMIT_XL             (gpio_num_t)36 // See above
#define SOFT_LIMIT_Y              (gpio_num_t)37 // Y limit
#define SOFT_LIMIT_ZL             (gpio_num_t)38 // Z left limit
#define SOFT_LIMIT_ZR             (gpio_num_t)39 // Z right limit
#define HARD_LIMIT_SWITCH         (gpio_num_t)40 // The direct input from the limit switch (not used for now)
#define MOTOR_DISABLED            (gpio_num_t)41 // The motor drivers are disabled when this input is LOW


// For code compatibility with Arduino
#define LOW                  0
#define HIGH                 1
#define OD_OPEN              HIGH
#define OD_CLOSED            LOW

// During WiFi setup
#define LED_BLINK_RATE_MS    250

typedef struct _cmd_t
{
  long dx;
  long dy;
  long dz;
  unsigned long duration;
  unsigned long flags;
} cmd_t;
