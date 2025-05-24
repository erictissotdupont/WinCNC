
//  A X I S    R E S O L U T I O N
//  ------------------------------- 
#define Z_AXIS_RES            0.0003925f
#define X_AXIS_RES            0.00049213f
#define Y_AXIS_RES            0.00049213f

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
#define LIMIT_IN             (gpio_num_t)13
#define LIMIT_OUT            (gpio_num_t)14
#define BLINK_GPIO           (gpio_num_t)18
#define BOOT_GPIO            (gpio_num_t)0

// For code compatibility with Arduino
#define LOW                  0
#define HIGH                 1
#define OD_OPEN              HIGH
#define OD_CLOSED            LOW

// During WiFi setup
#define LED_BLINK_RATE_MS    250

#include "..\..\CNC_Protocol.h"
#include "freertos/FreeRTOS.h"
// For ESP_LOGx and ESP_ERROR_CHECK
#include "esp_log.h"
#include "string.h"

typedef struct _cmd_t
{
  long dx;
  long dy;
  long dz;
  unsigned long duration;
  unsigned long flags;
} cmd_t;
