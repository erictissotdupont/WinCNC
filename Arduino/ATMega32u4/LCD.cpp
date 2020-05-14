
#define I2C_LCD

#include "CNC.h"
#include "LCD.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#ifdef I2C_LCD
  #include <Wire.h>
  #define LCD_I2C_ADDRESS 0x27
  #define LCD_LINE_LENGTH 20
#endif

#include "motor.h"

#define DEC_TO_STR( pt, n, dec ) { *pt = '0' + (n / dec); pt++; n = n % dec; }
#define STATUS_SIZE 10

void LCD_Send4bit( uint8_t value );
void LCD_Send( uint8_t value, int8_t mode );
void LCD_Command( uint8_t cmd );
void LCD_Write( uint8_t value );
void LCD_SetCursor( uint8_t col, uint8_t row);

// To get the current position 
extern Motor X;
extern Motor Y;
extern DualMotor Z;

extern unsigned long g_timeSleepingUs;
extern unsigned long g_maxUARTtaskTime;
extern unsigned long g_missedStepCount;

void LCD_Init( )
{
#ifdef I2C_LCD
  uint8_t cmd;
  uint8_t _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_2LINE | LCD_5x8DOTS;
  uint8_t _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  uint8_t _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;

  Wire.begin();

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way before 4.5V so we'll wait 50
  delay(50);

  // Now we pull both RS and R/W low to begin commands
  cmd = 0;
  Wire.transmitNoWait( LCD_I2C_ADDRESS, &cmd, 1 );
  while( Wire.transmitNoWait( LCD_I2C_ADDRESS, NULL, 0 ));
  //expanderWrite(_backlightval); // reset expanderand turn backlight off (Bit 8 =1)
  
  delay(100);

  //put the LCD into 4 bit mode
  // this is according to the hitachi HD44780 datasheet
  // figure 24, pg 46

  // we start in 8bit mode, try to set 4 bit mode
  //write4bits( 0x03 << 4 );
  LCD_Send4bit( 0x03 << 4 );
  while( Wire.transmitNoWait( LCD_I2C_ADDRESS, NULL, 0 ));
  delayMicroseconds(4100); // wait min 4.1ms
  
  // second try
  //write4bits( 0x03 << 4 );
  LCD_Send4bit( 0x03 << 4 );
  while( Wire.transmitNoWait( LCD_I2C_ADDRESS, NULL, 0 ));
  delayMicroseconds(4100); // wait min 4.1ms

  // third go!
  //write4bits( 0x03 << 4 );
  LCD_Send4bit( 0x03 << 4 );
  while( Wire.transmitNoWait( LCD_I2C_ADDRESS, NULL, 0 ));
  delayMicroseconds(150);

  // finally, set to 4-bit interface
  // write4bits(0x02 << 4);
  LCD_Send4bit( 0x02 << 4 );
  while( Wire.transmitNoWait( LCD_I2C_ADDRESS, NULL, 0 ));

  // set # lines, font size, etc.
  //command(LCD_FUNCTIONSET | _displayfunction);
  LCD_Command( LCD_FUNCTIONSET | _displayfunction );
  while( Wire.transmitNoWait( LCD_I2C_ADDRESS, NULL, 0 ));

  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  //display();
  LCD_Command( LCD_DISPLAYCONTROL | _displaycontrol );
  while( Wire.transmitNoWait( LCD_I2C_ADDRESS, NULL, 0 ));

  // clear it off
  //clear();
  LCD_Command( LCD_CLEARDISPLAY ); // clear display, set cursor position to zero
  delayMicroseconds(2000);  // this command takes a long time!

  // Initialize to default text direction (for roman languages)
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  // command(LCD_ENTRYMODESET | _displaymode);
  LCD_Command( LCD_ENTRYMODESET | _displaymode );
  while( Wire.transmitNoWait( LCD_I2C_ADDRESS, NULL, 0 ));

  // At 250kbit/s there is 40uS between bytes 10x(1/250000)
  // The EN pulse has to be longer than 37uS
  // Disabled because somehoe the LCD only works after a warm boot
  // Wire.setClock(250000);

#if 0
  int n = 842;
  unsigned long t = micros( );
  static char str[10];
  char tmp[10];
  volatile char *pt;
  sprintf( str, "%d", n );
  t = micros( ) - t;

  sprintf( str, "%d", t );
  LCD_SetStatus( str, 0 );

  t = micros( );
  for( int i=0; i<20; i++ )
  {
    n = 842;
    pt = str;
    DEC_TO_STR( pt, n, 100 );
    DEC_TO_STR( pt, n, 10 );
    *pt++ = '0' + n;
    *pt = 0;
  }
  t = micros( ) - t;
  
  sprintf( tmp, "%d %s", t, str );
  LCD_SetStatus( tmp, 3 );

  /*
  LCD_SetCursor( 1,1 );
  LCD_Write( '1' );
  LCD_Write( '2' );
  LCD_Write( '3' );
  LCD_Write( '4' );
  */
#endif

#else
  delay( 250 );
  
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
#endif
}

/******************************************************************/

void LCD_Send4bit( uint8_t value )
{
  uint8_t tmp[3];
  tmp[0] = value;
  tmp[1] = value | En;
  tmp[2] = value;
  Wire.transmitNoWait( LCD_I2C_ADDRESS, tmp, 3 );
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

/******************************************************************/

char sign;
unsigned int fixedPoint;
char* charPointer;

char StatusString[STATUS_SIZE];

void FloatToStrPart1( char* pt, float pos )
{
  if( pos < 0 )
  {
    sign = '-';
    pos = -pos;
  } 
  else
  {
    sign = ' ';
  }
  fixedPoint = round( pos );  
  charPointer = pt;
}

void FloatToStrPart2( )
{
  if( fixedPoint > 9999 )
  { 
    *charPointer++ = sign;
    DEC_TO_STR( charPointer, fixedPoint, 10000 );
  } 
  else
  {
    *charPointer++ = ' ';
    *charPointer++ = sign;
  }
  DEC_TO_STR( charPointer, fixedPoint, 1000 )
  *charPointer++ = '.';
}

void FloatToStrPart3( )
{
  DEC_TO_STR(charPointer, fixedPoint, 100)
  DEC_TO_STR(charPointer, fixedPoint, 10)
  *charPointer = '0' + fixedPoint;
}

typedef enum {
  LCD_StateInit = 0,            // 0
  LCD_StatePrintX_Part1,
  LCD_StatePrintX_Part2,
  LCD_StatePrintX_Part3,
  LCD_StatePrintX_Part4,
  LCD_StatePrintY_Part1,        // 5
  LCD_StatePrintY_Part2,
  LCD_StatePrintY_Part3,
  LCD_StatePrintY_Part4,
  LCD_StatePrintZ_Part1,
  LCD_StatePrintZ_Part2,        // 10
  LCD_StatePrintZ_Part3,
  LCD_StatePrintZ_Part4,
  LCD_StatePrintDebugStr,
  LCD_StatePrintStatusPart1,
  LCD_StatePrintStatusPart2,    // 15
  LCD_StatePrintStatusPart3,
  LCD_StatePrintStatusPart4,
  LCD_StateWriteTaskDuration,
  LCD_StateStartUpdate,
  LCD_StateSetCursor,           // 20
  LCD_StateWrite,
  LCD_StateTransmitI2Cdata,
  LCD_StateMax
} LCD_State;

void LCD_UpdateTask( )
{
  static LCD_State state = LCD_StateInit;
  static char line1[LCD_LINE_LENGTH+1];
  static char line2[LCD_LINE_LENGTH+1];
  static char line3[LCD_LINE_LENGTH+1];
  static char line4[LCD_LINE_LENGTH+1];  
  static char next[LCD_LINE_LENGTH+1];
  static float pos;
  static char* prev;
  static int col;
  static int row;
  static int i;
  static long l;
  static int CPUload;
  static unsigned long lastRefreshTime = 0;
  static unsigned int maxTimeForState[LCD_StateMax];
  unsigned long timeItTook = micros( );

  LCD_State curState = state;
  switch( state )
  {
  case LCD_StateInit :
    memset( maxTimeForState, 0, sizeof( maxTimeForState ));    
    memset( line1, 0, sizeof( line1 ));
    memset( line2, 0, sizeof( line2 ));
    memset( line3, 0, sizeof( line3 ));
    memset( line4, 0, sizeof( line4 )); 
    memset( next, 0, sizeof( next ));
    state = LCD_StatePrintX_Part1;
    break;
  
  // --------------------------- X / Y ------------------------------
  // X-99.999e Y-99.999e
  // 01234567890123456789
  //      
  case LCD_StatePrintX_Part1 :
    next[0] = 'X';
    next[9] = ' ';
    pos = X_AXIS_RES * 1000.0f * (float) X.GetPos( );
    state = LCD_StatePrintX_Part2;
    break;
  case LCD_StatePrintX_Part2 :
    FloatToStrPart1( next+1, pos );
    state = LCD_StatePrintX_Part3;
    break;
  case LCD_StatePrintX_Part3 :
    FloatToStrPart2( );    
    state = LCD_StatePrintX_Part4;
    break;
  case LCD_StatePrintX_Part4 :
    FloatToStrPart3( );
    next[8] = X.IsAtTheEnd( ) ? 'e' : ' ';
    state = LCD_StatePrintY_Part1;
    break;      
  case LCD_StatePrintY_Part1 :
    next[10] = 'Y';
    next[19] = ' ';
    pos = Y_AXIS_RES * 1000.0f * (float)Y.GetPos( );
    state = LCD_StatePrintY_Part2;
    break;      
  case LCD_StatePrintY_Part2 :
    FloatToStrPart1( next+11, pos );
    state = LCD_StatePrintY_Part3;
    break;
  case LCD_StatePrintY_Part3 :
    FloatToStrPart2( );
    state = LCD_StatePrintY_Part4;
    break;      
  case LCD_StatePrintY_Part4 :
    FloatToStrPart3( );
    next[18] = Y.IsAtTheEnd( ) ? 'e' : ' ';
    col = 0;
    row = 0;
    prev = line1;
    state = LCD_StateStartUpdate;
    break;
  // ----------------------------- Z --------------------------------  
  case LCD_StatePrintZ_Part1 :
    pos = Z_AXIS_RES * 1000.0f * (float)Z.GetPos( );
    next[0] = 'Z';
    next[9] = ' ';
    state = LCD_StatePrintZ_Part2;
    break;
  case LCD_StatePrintZ_Part2 :
    FloatToStrPart1( next+1, pos );
    state = LCD_StatePrintZ_Part3;
    break;     
  case LCD_StatePrintZ_Part3 :
    FloatToStrPart2( );
    state = LCD_StatePrintZ_Part4;
    break;
  case LCD_StatePrintZ_Part4 :
    FloatToStrPart3( );
    next[8] = Z.IsAtTheEnd( ) ? 'e' : ' ';
    state = LCD_StatePrintDebugStr;
    break;      
  case LCD_StatePrintDebugStr :
    memcpy( next+10, StatusString, sizeof(StatusString));
    col = 0;
    row = 1;
    prev = line2;
    state = LCD_StateStartUpdate;
    break;

  case LCD_StateWriteTaskDuration :
    {
      int slowState = 0;
      int slowTime = 0;
      int fastState = 0;
      int fastTime = 1000;
      
      for( i=1; i<LCD_StateMax; i++ )
      {
        // Find the slowest state (ignore this state)
        if( maxTimeForState[ i ] > slowTime && i != LCD_StateWriteTaskDuration )
        {
          slowState = i;
          slowTime = maxTimeForState[ i ];
        }
        // Find the fastest
        if( maxTimeForState[ i ] < slowTime )
        {
          fastState = i;
          fastTime = maxTimeForState[ i ];
        }
      }
      memcpy( next, line3, LCD_LINE_LENGTH );
      i = sprintf( next, "%d:%d %d:%d U:%d", slowState, slowTime, fastState, fastTime, g_maxUARTtaskTime );
      // Fill the rest of the line with sp
      if( i>0 && i < LCD_LINE_LENGTH ) memset( &next[i], ' ', LCD_LINE_LENGTH - i );
      col = 0;
      row = 2;
      prev = line3;
      state = LCD_StateStartUpdate;
    }      
    break;
    
  // --------------------------- STATUS ----------------------------        
  case LCD_StatePrintStatusPart1 :
    if( timeItTook > lastRefreshTime + 1000000 )
    { 
      CPUload = 100 - ( 100 * g_timeSleepingUs ) / ( timeItTook - lastRefreshTime );
      lastRefreshTime = timeItTook;
      g_timeSleepingUs = 0;         
    }
    state = LCD_StatePrintStatusPart2;
    break;
  case LCD_StatePrintStatusPart2 :
    fixedPoint = g_missedStepCount;
    memset( next, 0, LCD_LINE_LENGTH - 8 );
    charPointer = next + (LCD_LINE_LENGTH - 8);
    DEC_TO_STR( charPointer, fixedPoint, 1000 );
    state = LCD_StatePrintStatusPart3;
    break;
  case LCD_StatePrintStatusPart3 :
    DEC_TO_STR( charPointer, fixedPoint, 100 );
    DEC_TO_STR( charPointer, fixedPoint, 10 );
    *charPointer++ = '0' + fixedPoint;
    state = LCD_StatePrintStatusPart4;
    break;
  case LCD_StatePrintStatusPart4:
    *charPointer++ = ' ';
    if( CPUload > 99 ) CPUload = 99;    
    if( CPUload > 9 )
    {
      DEC_TO_STR( charPointer, CPUload, 10 );
    }
    else *charPointer++ = ' ';          
    *charPointer++ = '0' + CPUload;
    *charPointer = '%';
    col = 0;
    row = 3;
    prev = line4;
    state = LCD_StateStartUpdate;
    break;

  // --------------------------- UPDATE ----------------------------  
  case LCD_StateStartUpdate :
    while( prev[col] == next[col] )
    {
      col++;
      if( col >= LCD_LINE_LENGTH ) 
      {
        state = LCD_StateTransmitI2Cdata;
        break;
      }
    }
    if( state == LCD_StateStartUpdate )
    {
      state = LCD_StateSetCursor;
    }
    break;

  case LCD_StateSetCursor :
    LCD_SetCursor( col, row );
    state = LCD_StateWrite;
    break;

  case LCD_StateWrite :
    if( prev[col] != next[col] )
    {
      LCD_Write( next[col] );
      prev[col] = next[col];
      col++;
      if( col >= LCD_LINE_LENGTH )
      {
        state = LCD_StateTransmitI2Cdata;
        break;
      }
    }
    else 
    {
      state = LCD_StateStartUpdate;
    }
    break;
    
  case LCD_StateTransmitI2Cdata :
    if( Wire.transmitNoWait( LCD_I2C_ADDRESS, NULL, 0 ) == 0 )
    {
      switch( row )
      {
      case 0 :
        state = LCD_StatePrintZ_Part1;
        break;
      case 1 :
#if defined(DISPLAY_TASK_TIME)
        state = LCD_StateWriteTaskDuration;
#else
        state = LCD_StatePrintStatusPart1;
#endif
        break;
      case 2 :
        state = LCD_StatePrintStatusPart1;
        break;
      default :
        state = LCD_StatePrintX_Part1;
        break;
      }
    }
    break;
  }
#ifdef DISPLAY_TASK_TIME      
  timeItTook = micros() - timeItTook;
  if( timeItTook > maxTimeForState[ curState ] ) 
  {
    maxTimeForState[ curState ] = timeItTook;
  }
#endif
}

void LCD_SetStatus( const char *str, int offset )
{
  int l = strlen( str );
  if( l > STATUS_SIZE-offset ) l = STATUS_SIZE-offset;
  memcpy( StatusString + offset, str, l );
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
  static unsigned char count = 0;
  
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
     if( count++ >= 100 || button == BUTTON_NONE ) debounce = button;
   }
   else count = 0;

   return debounce;
}

#define REPEAT_  750
#define CONT_    100
#define IDLE_    30
#define MICRO_STEP_X  (long)(0.005 / X_AXIS_RES)
#define MICRO_STEP_Y  (long)(0.005 / Y_AXIS_RES)
#define MICRO_STEP_Z  (long)(0.005 / Z_AXIS_RES)
#define STEP_X        (long)(0.05 / X_AXIS_RES)
#define STEP_Y        (long)(0.05 / Y_AXIS_RES)
#define STEP_Z        (long)(0.05 / Z_AXIS_RES)

typedef enum {
 ButtonMode_XY = 0,
 ButtonMode_Z,
 ButtonMode_Max
} ButtonMode;

const char* ModeString[ButtonMode_Max] = { "XY", "Z " };

void DoButtonAction( LCD_Button button, int longPress )
{
  static ButtonMode mode = ButtonMode_XY;
  long x = 0;
  long y = 0;
  long z = 0;
  static unsigned int acc = 0;
  
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
      case BUTTON_RIGHT: x = -1; break;
      case BUTTON_UP:    y = -1; break;
      case BUTTON_DOWN:  y = 1; break;
      case BUTTON_LEFT:  x = 1; break;
      }
      break;
    case ButtonMode_Z :
      switch(button)
      {
      case BUTTON_RIGHT: x = 1; mode = ButtonMode_XY; break;
      case BUTTON_UP:    z = 1; break;
      case BUTTON_DOWN:  z = -1; break;
      case BUTTON_LEFT:  x = -1; mode = ButtonMode_XY; break;
      }
    }
  }

  LCD_SetStatus( ModeString[ mode ], 5 );
 
  if( longPress )
  {
    if( acc < 0x300 ) acc = acc + 32;
    x = ( x * STEP_X * acc ) >> 8;
    y = ( y * STEP_Y * acc ) >> 8;
    z = ( z * STEP_Z * acc ) >> 8;
  }
  else
  {
    acc = 0x100;
    x = x * MICRO_STEP_X;
    y = y * MICRO_STEP_Y;
    z = z * MICRO_STEP_Z;
  }
  
  if( x || y || z )
  {
    Motor_Move( x, y, z, CONT_ * 1100L );
  }
}


void LCD_ButtonTask( )
{

  delayMicroseconds( 2000 );
  g_timeSleepingUs += 2000;
  
  /*
  static unsigned long nextScanTime = 50;
  static LCD_Button prevButton = BUTTON_NONE;
  static int state = 0;  
  unsigned long curentTime = millis( );
  LCD_Button button = LCD_ScanButtons( );
  
  
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
  */
}


#if 0
/********** high level commands, for the user! */
void LCD_Clear( void )
{
  LCD_command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  delay( 250 );
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

#endif
