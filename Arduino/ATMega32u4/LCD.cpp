#include "LCD.h"
#include "CNC.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"
#include "motor.h"

extern Motor X;
extern Motor Y;
extern Motor Z;
extern void Move(long,long,long,long);
extern unsigned int commandCount;
extern unsigned int error;
extern unsigned int g_duration;

#define LCD_D0  4
#define LCD_D1  5
#define LCD_D2  6
#define LCD_D3  7
#define LCD_RS  8
#define LCD_EN  9
#define LCD_BTN A0

void LCD_Init( )
{
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_EN, OUTPUT);
  pinMode(LCD_BTN, INPUT);
 
  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
  delayMicroseconds(50000); 
  // Now we pull both RS and R/W low to begin commands
  digitalWrite(LCD_RS, LOW);
  digitalWrite(LCD_EN, LOW);
   
  //put the LCD into 4 bit mode
 
  // this is according to the hitachi HD44780 datasheet
  // figure 24, pg 46
  pinMode(LCD_D0 , OUTPUT);
  pinMode(LCD_D1 , OUTPUT);
  pinMode(LCD_D2 , OUTPUT);
  pinMode(LCD_D3 , OUTPUT);

  // we start in 8bit mode, try to set 4 bit mode
  LCD_send_low(0x03);
  delayMicroseconds(4500); // wait min 4.1ms

  // second try
  LCD_send_low(0x03);
  delayMicroseconds(4500); // wait min 4.1ms
    
  // third go!
  LCD_send_low(0x03); 
  delayMicroseconds(150);

  // finally, set to 4-bit interface
  LCD_send_low(0x02);
  delayMicroseconds(5000);
 
  // finally, set # lines, font size, etc.
  LCD_command( LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS );
  delayMicroseconds(5000);
  
  LCD_command( LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF );
}

/********** high level commands, for the user! */
void LCD_Clear( void )
{
  LCD_command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  delayMicroseconds(5000);  // this command takes a long time!
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void LCD_CreateChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  LCD_command(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    LCD_write(charmap[i]);
  }
}

inline void LCD_command(uint8_t value) 
{
  LCD_send(value, LOW);
}

void LCD_setCursor(uint8_t col, uint8_t row)
{
  if( row != 0 ) row  = 0x40;
  LCD_send(LCD_SETDDRAMADDR | (col + row ), LOW );
}

void LCD_write(uint8_t value) {
  LCD_send(value, HIGH);
}

void LCD_send(uint8_t value, uint8_t mode) {
  LCD_send_high( value, mode );
  delayMicroseconds(500); 
  LCD_send_low( value );
  delayMicroseconds(500); 
}

// #####################################################################################
void LCD_setCursor_high(uint8_t col, uint8_t row)
{
  if( row != 0 ) row  = 0x40;
  LCD_send_high(LCD_SETDDRAMADDR | (col + row ), LOW );
}

void LCD_setCursor_low(uint8_t col, uint8_t row)
{
  if( row != 0 ) row  = 0x40;
  LCD_send_low(LCD_SETDDRAMADDR | (col + row ));
}

void LCD_write_high(uint8_t value) {
  LCD_send_high(value, HIGH);
}

void LCD_write_low(uint8_t value) {
  LCD_send_low(value);
}

/************ low level data pushing commands **********/

// write either command or data, with automatic 4/8-bit selection
void LCD_send_high(uint8_t value, uint8_t mode) {
  digitalWrite(LCD_RS, mode);
  digitalWrite(LCD_D0, value & 0x10);
  digitalWrite(LCD_D1, value & 0x20);
  digitalWrite(LCD_D2, value & 0x40);
  digitalWrite(LCD_D3, value & 0x80);
  LCD_pulseEnable();
}

void LCD_send_low( uint8_t value) {
  digitalWrite(LCD_D0, value & 0x01);
  digitalWrite(LCD_D1, value & 0x02);
  digitalWrite(LCD_D2, value & 0x04);
  digitalWrite(LCD_D3, value & 0x08);
  LCD_pulseEnable();
}

inline void LCD_pulseEnable(void) {
  digitalWrite(LCD_EN, LOW);
  digitalWrite(LCD_EN, HIGH);
  digitalWrite(LCD_EN, LOW);
}

/******************************************************************/

#define DEC_TO_STR( pt, n, dec )  *pt = '0' + n / dec; pt++; n = n % dec;
#define STATUS_SIZE 8

unsigned int greaterThanTen;
unsigned int fixedPoint;
char* charPointer;
char StatusString[STATUS_SIZE+1];

void FloatToStrPart1( char* pt, float pos )
{
  long l;
  if( pos < 0 ) { pt[0] = '-'; pos = -pos; } else pt[0] = ' ';
  l = round( pos );
  if( l > 9999 ) { fixedPoint = l / 10; greaterThanTen = true; } else { fixedPoint = l; greaterThanTen = false; }
  charPointer = pt+1;
}

void FloatToStrPart2( )
{
  DEC_TO_STR( charPointer, fixedPoint, 1000 )
  if( !greaterThanTen ) *charPointer++ = '.';
  DEC_TO_STR(charPointer, fixedPoint, 100)
}

void FloatToStrPart3( )
{
  if( greaterThanTen ) *charPointer++ = '.';
  DEC_TO_STR(charPointer, fixedPoint, 10)
  *charPointer = '0' + fixedPoint;
}

typedef enum {
  LCD_StateInit = 0,
  LCD_StatePrintX_Part1,
  LCD_StatePrintX_Part2,
  LCD_StatePrintX_Part3,
  LCD_StatePrintY_Part1,
  LCD_StatePrintY_Part2,
  LCD_StatePrintY_Part3,
  LCD_StatePrintZ_Part1,
  LCD_StatePrintZ_Part2,
  LCD_StatePrintZ_Part3,
  LCD_StatePrintStatus,
  LCD_StateUpdateSetCursorHigh,
  LCD_StateUpdateSetCursorLow,
  LCD_StateUpdateWriteHigh,
  LCD_StateUpdateWriteLow,
  LCD_StateMax
} LCD_State;

void LCD_UpdateTask( unsigned long timeWeHave )
{
  static LCD_State state = LCD_StateInit;
  static char line1[17];
  static char line2[17];
  static char next[17];
  static char* prev;
  static int col;
  static int row;
  static unsigned int maxTimeForState[LCD_StateMax];
  static int i;
  static long l;
  unsigned long timeItTook;

  if( timeWeHave > maxTimeForState[ state ] || state == 0 )
  {
    LCD_State curState = state;
    timeItTook = micros( );
    
    switch( state )
    {
    case LCD_StateInit :
      memset( maxTimeForState, 0, sizeof( maxTimeForState ));      
      memset( line1, 0, sizeof( line1 ));
      memset( line2, 0, sizeof( line2 ));
      memset( next, 0, sizeof( next ));
      state = LCD_StatePrintX_Part1;
      break;
    
    // ----------------------------- X --------------------------------
    case LCD_StatePrintX_Part1 :
      next[0] = 'X';
      next[7] = ' ';
      FloatToStrPart1( next+1, X.GetPos( ) * X_AXIS_RES * 1000 );
      state = LCD_StatePrintX_Part2;
      break;
    case LCD_StatePrintX_Part2 :
      FloatToStrPart2( );
      state = LCD_StatePrintX_Part3;
      break;
    case LCD_StatePrintX_Part3 :
      FloatToStrPart3( );    
      state = LCD_StatePrintY_Part1;
      break;
    // ----------------------------- Y --------------------------------      
    case LCD_StatePrintY_Part1 :
      next[8] = ' ';
      next[9] = 'Y';
      FloatToStrPart1( next+10, Y.GetPos( ) * Y_AXIS_RES * 1000 );
      state = LCD_StatePrintY_Part2;
      break;      
    case LCD_StatePrintY_Part2 :
      FloatToStrPart2( );
      state = LCD_StatePrintY_Part3;
      break;      
    case LCD_StatePrintY_Part3 :
      FloatToStrPart3( );
      next[16] = 0;
      col = 0;
      row = 0;
      prev = line1;
      state = LCD_StateUpdateSetCursorHigh;
      break;
    // ----------------------------- Z --------------------------------  
    case LCD_StatePrintZ_Part1 :
      next[0] = 'Z';
      next[7] = ' ';
      FloatToStrPart1( next+1, Z.GetPos( ) * Z_AXIS_RES * 1000 );
      state = LCD_StatePrintZ_Part2;
      break;
    case LCD_StatePrintZ_Part2 :
      FloatToStrPart2( );
      state = LCD_StatePrintZ_Part3;
      break;     
    case LCD_StatePrintZ_Part3 :
      FloatToStrPart3( );
      state = LCD_StatePrintStatus;
      break;
    // --------------------------- STATUS ----------------------------        
    case LCD_StatePrintStatus :
      memcpy( next+8, StatusString, 8 );
      next[16] = 0;  
      col = 0;
      row = 1;
      prev = line2;
      state = LCD_StateUpdateSetCursorHigh;
      break;
    
    case LCD_StateUpdateSetCursorHigh :
      while( prev[col] == next[col] )
      {
        col++;
        if( col >= 16 )
        {
          state = ( row == 0 ) ? LCD_StatePrintZ_Part1 : LCD_StatePrintX_Part1;
          break; 
        }
      }
      LCD_setCursor_high( col, row );
      state = LCD_StateUpdateSetCursorLow;
      break;
      
    case LCD_StateUpdateSetCursorLow :
      LCD_setCursor_low( col, row );
      state = LCD_StateUpdateWriteHigh;
      break;
    
    case LCD_StateUpdateWriteHigh :
      LCD_write_high( next[col] );
      state = LCD_StateUpdateWriteLow;
      break;
      
    case LCD_StateUpdateWriteLow :
      LCD_write_low( next[col] );
      prev[col] = next[col];
      col++;
      if( col < 16 ) 
        state = LCD_StateUpdateSetCursorHigh;
      else
        state = ( row == 0 ) ? LCD_StatePrintZ_Part1 : LCD_StatePrintX_Part1;
      break;
    }
        
    timeItTook = micros() - timeItTook;
    if( timeItTook > maxTimeForState[ curState ] ) maxTimeForState[ curState ] = timeItTook;
  }
}

void LCD_SetStatus( const char *str, int offset )
{
  int l = strlen( str );
  if( l > STATUS_SIZE-offset ) l = STATUS_SIZE-offset;
  memcpy( StatusString + offset, str, l );
  StatusString[STATUS_SIZE] = 0;  
}

// ADC readings expected for the 5 buttons on the ADC input
#define RIGHT_10BIT_ADC           0  // right
#define UP_10BIT_ADC            136  // up 
#define DOWN_10BIT_ADC          311  // down 
#define LEFT_10BIT_ADC          479  // left 
#define SELECT_10BIT_ADC        704  // right 
#define BUTTONHYSTERESIS         10  // hysteresis for valid button sensing window

LCD_Button LCD_ScanButtons( void )
{
  LCD_Button button = BUTTON_NONE;
  int buttonVoltage = analogRead( LCD_BTN );
  static LCD_Button debounce = BUTTON_NONE;
  static int count = 0;
  
  /*
  char tmp[6];
  sprintf( tmp, "%04d ", buttonVoltage );
  LCD_SetStatus( tmp );
  return BUTTON_NONE;
  */
  
   // Sense if the voltage falls within valid voltage windows
   if( buttonVoltage < ( RIGHT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_RIGHT;
   }
   else if(   buttonVoltage >= ( UP_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( UP_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_UP;
   }
   else if(   buttonVoltage >= ( DOWN_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( DOWN_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_DOWN;
   }
   else if(   buttonVoltage >= ( LEFT_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( LEFT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_LEFT;
   }
   else if(   buttonVoltage >= ( SELECT_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( SELECT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_SELECT;
   }

   if( debounce != button )
   {
     count++;
     if( count > 1 ) debounce = button;
   }
   else count = 0;

   return debounce;
}

#define REPEAT_DELAY  750
#define CONT_DELAY    100
#define IDLE_DELAY    30
#define MICRO_STEP_X  (long)(0.005 / X_AXIS_RES)
#define MICRO_STEP_Y  (long)(0.005 / Y_AXIS_RES)
#define MICRO_STEP_Z  (long)(0.005 / Z_AXIS_RES)
#define STEP_X        (long)(0.05 / X_AXIS_RES)
#define STEP_Y        (long)(0.05 / Y_AXIS_RES)
#define STEP_Z        (long)(0.05 / Z_AXIS_RES)

typedef enum {
 ButtonMode_XY = 0,
 ButtonMode_Z,
 ButtonMode_Reset,
 ButtonMode_Debug,
 ButtonMode_Max
} ButtonMode;

const char* ModeString[ButtonMode_Max] = { "XY", "Z ", "RS", "LK" };

void DoButtonAction( LCD_Button button, int longPress )
{
  static ButtonMode mode = ButtonMode_XY;
  long x = 0;
  long y = 0;
  long z = 0;
  unsigned long d = 0;
  
  if( button == BUTTON_SELECT )
  {
    mode = (ButtonMode)(mode + 1); 
    if( mode >= ButtonMode_Max ) mode = ButtonMode_XY;
  }
  else
  {
    switch( mode )
    {
    case ButtonMode_XY :
      switch(button)
      {
      case BUTTON_RIGHT: x = 1; break;
      case BUTTON_UP:    y = 1; break;
      case BUTTON_DOWN:  y = -1; break;
      case BUTTON_LEFT:  x = -1; break;
      }
      break;
    case ButtonMode_Z :
      switch(button)
      {
      case BUTTON_UP:    z = 1; break;
      case BUTTON_DOWN:  z = -1; break;
      }
    }
  }

  LCD_SetStatus( ModeString[ mode ], 5 );
 
  if( longPress )
  {
    d = (long)CONT_DELAY * 1100;
    x = x * STEP_X;
    y = y * STEP_Y;
    z = z * STEP_Z;
  }
  else
  {
    x = x * MICRO_STEP_X;
    y = y * MICRO_STEP_Y;
    z = z * MICRO_STEP_Z;
  }
  
  if( x || y || z ) Move( x, y, z, d );
}


void LCD_ButtonTask( )
{
  static unsigned long nextScanTime = 50;
  static LCD_Button prevButton = BUTTON_NONE;
  static int state = 0;  
  unsigned long curentTime = millis( );
  LCD_Button button = LCD_ScanButtons( );
  
  /*
  char str[6];
  sprintf( str, "% 4d", commandCount );
  LCD_SetStatus( str );
  
  char tmp[2];
  sprintf( tmp, "%d", state );
  LCD_SetStatus( tmp );
  */
  
  if( button != prevButton )
  {
    switch( button )
    {
    case BUTTON_RIGHT: LCD_SetStatus( "R", 7 );break;
    case BUTTON_LEFT: LCD_SetStatus( "L", 7 );break;
    case BUTTON_UP: LCD_SetStatus( "U", 7 );break;
    case BUTTON_DOWN: LCD_SetStatus( "D", 7 );break;
    case BUTTON_SELECT: LCD_SetStatus( "S", 7 );break;
    default : LCD_SetStatus( " ", 7 );break;
    }
  }
  
  switch( state )
  {
  case 0 :
    DoButtonAction( button, 0 );
    state = 1;
    break;
    
  case 1 :
    if( button != BUTTON_NONE )
    {
      nextScanTime = curentTime + REPEAT_DELAY;
      state = 2;
    }
    break;
    
  case 2 :
    if( button == BUTTON_NONE )
    {
      DoButtonAction( prevButton, 0 );
      DoButtonAction( button, 0 );
      state = 1;
    }
    else if( button != prevButton )
    {
      nextScanTime = curentTime + REPEAT_DELAY;
    }
    else if( curentTime > nextScanTime )
    {
      nextScanTime = curentTime + CONT_DELAY;
      DoButtonAction( prevButton, 1 );
      state = 3;
    }
    break;
    
  case 3 :
    if( button == BUTTON_NONE )
    {
      DoButtonAction( button, 0 );
      state = 1;
    }
    else if( button != prevButton )
    {
      nextScanTime = curentTime + REPEAT_DELAY;
      state = 3; 
    }
    else if( curentTime > nextScanTime )
    {
      nextScanTime = curentTime + CONT_DELAY;
      DoButtonAction( prevButton, 1 );
    }
  }
  prevButton = button;
}

