
#define I2C_LCD

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

// For UART
#include "driver/i2c.h"
#include "string.h"

#include "Cnc.h"
#include "Network.h"
#include "LCD.h"
#include "Buttons.h"

#define DEC_TO_STR( pt, n, dec ) { *pt = '0' + (n / dec); pt++; n = n % dec; }
#define STATUS_SIZE 10

void LCD_Send4bit( uint8_t value );
void LCD_Send( uint8_t value, int8_t mode );
void LCD_Command( uint8_t cmd );
void LCD_Write( uint8_t value );
void LCD_SetCursor( uint8_t col, uint8_t row);


#define I2C_MASTER_SDA_IO      (GPIO_NUM_8)
#define I2C_MASTER_SCL_IO      (GPIO_NUM_9)
#define LCD_I2C_ADDRESS        0x27
#define LCD_COL                20
#define LCD_LINE               4
#define I2C_MASTER_FREQ_HZ     400000
#define I2C_MASTER_TIMEOUT_MS  1000
#define I2C_MASTER_NUM         0

#define delay(t)               vTaskDelay( t / portTICK_PERIOD_MS)

extern long g_xPos;
extern long g_yPos;
extern long g_zPos;
extern unsigned long g_Status;
extern char g_szStatus[20];

bool g_bI2Cinitialized = false;

int g_joyX = 0;
int g_joyY = 0;
float g_Vbatt = 0;
int g_Red;
int g_Green;

static esp_err_t TransmitLCD( uint8_t* data, unsigned int count )
{
    int ret;
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, LCD_I2C_ADDRESS, data, count, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ret;
}

#define SPACE  (char)(' ')

typedef enum {
  
  LCD_Idle,
  LCD_Menu,
  LCD_Calibration,
  
} LCD_State_t;

LCD_State_t g_LCD_State = LCD_Idle;

int g_CalStep = 0;
int g_CalStepButtonRelease = 0;

#define CAL_STEP_COUNT 5

const char* g_szCalStepString[CAL_STEP_COUNT] = {
  "CENTER",
  "DOWN",  
  "UP",
  "RIGHT",
  "LEFT"
};

void LCD_Refresh( void )
{
  char LCD_buffer[LCD_LINE][LCD_COL+21];
  static char prevBuf[LCD_LINE][LCD_COL+21];
  
  if( g_bI2Cinitialized == false )
  {
    memset( prevBuf, SPACE, sizeof( prevBuf ));
    LCD_Init( );
    if( g_bI2Cinitialized == false )
    {
      delay( 100 );
      return;
    }
  }
  
  if( g_Green && g_Red )
  {
    if( g_LCD_State != LCD_Calibration )
    {
      g_LCD_State = LCD_Calibration;
      g_CalStep = 0;
    }
  }
  else if( g_Red )
  {
    g_LCD_State = LCD_Idle;
  }
  
  memset( LCD_buffer, SPACE, sizeof( LCD_buffer ));
  
  switch (g_LCD_State)
  {
  case LCD_Menu :
  
    break; 
    
  case LCD_Calibration :
  
    if( g_Green == 0 )
    {
      g_CalStepButtonRelease = 1;
      if( g_CalStep == CAL_STEP_COUNT )
      {
        CalibrationSave( );
        g_LCD_State = LCD_Idle;
      }
    }
    else
    {
      if( g_CalStepButtonRelease )
      {
        CalibrateStep( g_CalStep );
        g_CalStepButtonRelease = 0;
        if( g_CalStep < CAL_STEP_COUNT )
        {
          g_CalStep++;
        }
      }
    }  
    
    if( g_CalStep < CAL_STEP_COUNT )
    {      
      LCD_buffer[0][sprintf( LCD_buffer[0], "Step %d %d,%d", g_CalStep + 1, GetRawInput(1), GetRawInput(2) )] = SPACE;
      LCD_buffer[2][sprintf( LCD_buffer[2], "Joystick %s then", g_szCalStepString[g_CalStep] )] = SPACE;
      LCD_buffer[3][sprintf( LCD_buffer[3], "press GREEN" )] = SPACE;
    }    
    break;
 
  case LCD_Idle :
    if(( g_Status & STATUS_GOT_POSITION ) == 0 )
    {
      int n;
      char tmp[LCD_COL+1];
      
      n = sprintf( tmp, "%.1f", g_Vbatt );
      memcpy( LCD_buffer[3]+LCD_COL-n, tmp, n);
      
      LCD_buffer[3][9] = g_Red ? 'R' :'-';
      LCD_buffer[3][10] = g_Green ? 'G' : '-';
      
      if( isIPConnected( ))
        memcpy( LCD_buffer[1]+1+(20-9)/2, "Connected", 9 );
      else
        memcpy( LCD_buffer[1]+1+(20-13)/2, "Connecting...", 13 );
      
      LCD_buffer[3][sprintf( LCD_buffer[3], "%d,%d", g_joyY, g_joyX)] = SPACE;
                 
    }
    else
    {
      int n;
      char tmp[LCD_COL+1];
      float x = g_xPos * X_AXIS_RES;    
      n = sprintf( tmp, "X:%.3f", x );
      memcpy( &LCD_buffer[0][0], tmp, n );
      
      float y = g_yPos * Y_AXIS_RES;    
      n = sprintf( tmp, "Y:%.3f  ", y );
      memcpy( &LCD_buffer[1][0], tmp, n );
      
      float z = g_zPos * Z_AXIS_RES;    
      n = sprintf( tmp, "Z:%.3f", z );
      memcpy( &LCD_buffer[0][10], tmp, n );
      
      memcpy( LCD_buffer[3], g_szStatus, strlen(g_szStatus));
      
      n = sprintf( tmp, "%.1fV", g_Vbatt );
      memcpy( LCD_buffer[3]+LCD_COL-n, tmp, n);
      
#if 0
      int i;
      i = 10 + g_joyX;
      if( i < 0 ) i = 0;
      if( i > 19 ) i = 19; 
      LCD_buffer[2][ i ] = '*';
      
      i = 10 + g_joyY;
      if( i < 0 ) i = 0;
      if( i > 19  ) i = 19;
      LCD_buffer[3][ i ] = '*';
#endif      

      static int wiggle = 0;
      static uint64_t timeToNextWiggle = 0;
      uint64_t now = esp_timer_get_time( );
      if( now > timeToNextWiggle )
      {       
        wiggle++;
        timeToNextWiggle = now + 250000;
      }
      LCD_buffer[0][LCD_COL-1] = wiggle & 7;
      
    }
    break;
  }
  
  
  for( int l = 0; l<LCD_LINE; l++)
  {
    int c = 0;
    while( c < LCD_COL )
    {
      int n = 0;
      while((LCD_buffer[l][c+n] != prevBuf[l][c+n]) && 
            ((c+n) < LCD_COL))
      {
        n++;
      }
      if( n )
      {
        LCD_SetCursor( c, l );
        for( int i=0; i<n; i++ )
        {
          LCD_Write( LCD_buffer[l][c+i] );
          prevBuf[l][c+i] = LCD_buffer[l][c+i];
        }
      }
      c = c + n + 1;
    }
  }
}


const char CUSTOM_CHAR_SET[] = {
 0x0C, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06,
 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00,
 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 
 0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

 
void LCD_Init( )
{
  uint8_t cmd;
  uint8_t _displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
  //uint8_t _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  uint8_t _displaymode = LCD_ENTRYRIGHT | LCD_ENTRYSHIFTINCREMENT;
  uint8_t _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;

  
  int i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_IO,         // select GPIO specific to your project
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = I2C_MASTER_SCL_IO,         // select GPIO specific to your project
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,  // select frequency specific to your project
    // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
};

  ESP_LOGW( "LCD", "Initializing" );

  i2c_param_config(i2c_master_port, &conf);

  i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
  
  g_bI2Cinitialized = true;
  
  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way before 4.5V so we'll wait 50
  delay(50);

  // Now we pull both RS and R/W low to begin commands
  cmd = 0;
  TransmitLCD( &cmd, 1 );

  //expanderWrite(_backlightval); // reset expanderand turn backlight off (Bit 8 =1)
  
  delay(100);

  //put the LCD into 4 bit mode
  // this is according to the hitachi HD44780 datasheet
  // figure 24, pg 46

  // we start in 8bit mode, try to set 4 bit mode
  //write4bits( 0x03 << 4 );
  LCD_Send4bit( 0x03 << 4 );
  delay(5); // wait min 4.1ms
  
  // second try
  //write4bits( 0x03 << 4 );
  LCD_Send4bit( 0x03 << 4 );
  delay(5); // wait min 4.1ms

  // third go!
  //write4bits( 0x03 << 4 );
  LCD_Send4bit( 0x03 << 4 );
  delay(1);

  // finally, set to 4-bit interface
  // write4bits(0x02 << 4);
  LCD_Send4bit( 0x02 << 4 );

  // set # lines, font size, etc.
  //command(LCD_FUNCTIONSET | _displayfunction);
  LCD_Command( LCD_FUNCTIONSET | _displayfunction );
  
    // Initialize to default text direction (for roman languages)
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  // command(LCD_ENTRYMODESET | _displaymode);
  LCD_Command( LCD_ENTRYMODESET | _displaymode );

  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  //display();
  LCD_Command( LCD_DISPLAYCONTROL | _displaycontrol );
  
  delay(5);
  LCD_Command( LCD_SETCGRAMADDR );
  for( int i=0; i<sizeof(CUSTOM_CHAR_SET); i++)
  {
    LCD_Write(CUSTOM_CHAR_SET[i]);
  }
  
  delay(5);
  
  // clear it off
  //clear();
  LCD_Command( LCD_CLEARDISPLAY ); // clear display, set cursor position to zero
  delay(5);       // this command takes a long time!

  delay( 50 );
  
  if( g_bI2Cinitialized )
  {    
    // To get the splash screen displayed
    LCD_Refresh( );
  }
}

/******************************************************************/

void LCD_Send4bit( uint8_t value )
{
  uint8_t tmp[3];
  tmp[0] = value;
  tmp[1] = value | En;
  tmp[2] = value;
  if( TransmitLCD( tmp, 3 ) != ESP_OK )
  {
    g_bI2Cinitialized = false;
  }
}

void LCD_Send( uint8_t value, int8_t mode )
{
  uint8_t highnib = (value&0xf0) | mode | LCD_BACKLIGHT;
  uint8_t lownib= ((value<<4)&0xf0) | mode | LCD_BACKLIGHT;
  LCD_Send4bit( highnib );
  LCD_Send4bit( lownib );  
}

void LCD_Command( uint8_t cmd )
{
  LCD_Send( cmd, 0 );
}

void LCD_Write( uint8_t value )
{
  LCD_Send( value, Rs );
}

void LCD_SetCursor( uint8_t col, uint8_t row)
{
  uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  LCD_Command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}
