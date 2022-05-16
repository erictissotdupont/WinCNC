
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

extern unsigned long g_timeIdleUs;
extern unsigned long g_maxUARTtaskTime;
extern unsigned long g_missedStepCount;

tManualMode ManualMode = Manual_Disabled;

void LCD_Init( )
{
  startAnalogRead( LCD_X_JOY );

#ifdef I2C_LCD
  uint8_t cmd;
  uint8_t _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_2LINE | LCD_5x8DOTS;
  uint8_t _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  //uint8_t _displaymode = LCD_ENTRYRIGHT | LCD_ENTRYSHIFTINCREMENT;
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

  delay( 50 );

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
      CPUload = 100 - ( 100 * g_timeIdleUs ) / ( timeItTook - lastRefreshTime );
      lastRefreshTime = timeItTook;
      g_timeIdleUs = 0;         
    }
    state = LCD_StatePrintStatusPart2;
    break;
  case LCD_StatePrintStatusPart2 :
    fixedPoint = g_missedStepCount;
    if( ManualMode == Manual_Disabled )
    {
      memcpy( next, "AUTO", 4 );
    }
    else if( ManualMode == Manual_XY )
    {
      memcpy( next, "M-XY", 4 );
    }
    else
    {
      memcpy( next, "M-Z ", 4 );
    }
    
    memset( next+4, 0, LCD_LINE_LENGTH - 4 );
    
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


typedef enum {
  ButtonState_WaitingForX,
  ButtonState_WaitingForY,
  ButtonState_WaitingForButtons,
} tButtonState;

#define JOY_AVG_SAMPLE           32
#define JOY_DEAD_ZONE            5
#define ADC_AVG_COUNT            4

class JoysticAxis
{
public:
  JoysticAxis( Motor* pM, int dir, int nA ) : 
    pMotor(pM),
    NextAxis(nA),
    dir(dir),
    adcAtRest(0), 
    restAvgCount(JOY_AVG_SAMPLE),
    adcReadIdx(0) { };
    
  bool ProcessAdcRead( );
  
protected :
  Motor* pMotor;
  int NextAxis;
  int adcAtRest;
  int restAvgCount;
  int adcRead[ADC_AVG_COUNT];
  int adcReadIdx;
  int dir;
};

class DualJoysticAxis : public JoysticAxis
{
public :
  DualJoysticAxis ( Motor* pM, int dirM, Motor* pB, int dirB, int nA ) : 
    pPrimary(pM), primDir(dirM),
    pSecondary(pB), secDir(dirB),
    JoysticAxis(pM,dirM,nA)
    { pMotor = pPrimary; dir = primDir; };

  void SecondaryAxis( bool enable )
  {
    if( enable )
    {
      pPrimary->Manual( 0 );
      JoysticAxis::pMotor = pSecondary;
      JoysticAxis::dir = secDir;
    }
    else
    {
      pSecondary->Manual( 0 );
      JoysticAxis::pMotor = pPrimary;
      JoysticAxis::dir = primDir;
    }
  };

private :
  Motor* pPrimary;
  Motor* pSecondary;
  int primDir;
  int secDir;
};

bool JoysticAxis::ProcessAdcRead(  )
{
  uint32_t adc;
  if( fastAnalogRead( NextAxis, &adc ))
  {
    adcRead[adcReadIdx++] = adc;
    if( adcReadIdx >= ADC_AVG_COUNT ) adcReadIdx = 0;
    
    if( restAvgCount > 0 )
    {
      adcAtRest += (int)adc;
      if( --restAvgCount == 0 ) adcAtRest = adcAtRest / JOY_AVG_SAMPLE;
    }
    else
    {
      int avg = 0;      
      if( ManualMode != Manual_Disabled )
      {  
        for( int i=0; i<ADC_AVG_COUNT; i++ ) avg += adcRead[i];
        avg = avg / ADC_AVG_COUNT;
  
        // Get position relative to resting position and apply direction
        avg = ( dir > 0 ) ? adcAtRest - avg : avg - adcAtRest;
  
        if( avg > JOY_DEAD_ZONE ) avg -= JOY_DEAD_ZONE;
        else if( avg < -JOY_DEAD_ZONE ) avg += JOY_DEAD_ZONE;
        else avg = 0;
      }
      pMotor->Manual( avg );
    }
    return true;
  }
  return false;
}

DualJoysticAxis JoyX( &X,  1, &Z, -1, LCD_Y_JOY );
    JoysticAxis JoyY( &Y, -1,         LCD_BUTTONS );

#define BUTTON_DEBOUNCE    5

bool LCD_ScanButtons( void )
{
  uint32_t adc;
  static int pressed[4] = {0,0,0,0};
  
  if( fastAnalogRead( LCD_X_JOY, &adc ))
  {
    if( adc < 10 )
    {
      pressed[0]++;
    }
    else
    {
      if( pressed[0] > BUTTON_DEBOUNCE )
      {        
        switch( ManualMode )
        {
        case Manual_Disabled : 
          ManualMode = Manual_XY; 
          JoyX.SecondaryAxis( false );
          break;
        case Manual_XY :
          ManualMode = Manual_Z;
          JoyX.SecondaryAxis( true );
          break;
        case Manual_Z : 
          ManualMode = Manual_Disabled;
          X.Manual( 0 );
          Y.Manual( 0 );
          Z.Manual( 0 );
          break;
        }
      }
      pressed[0] = 0;
    }
    
    return true;
  }
  return false;
}

void LCD_ButtonTask( )
{
  static tButtonState state = ButtonState_WaitingForX;

  /*
  switch( state )
  {
  case ButtonState_WaitingForX :
    if( JoyX.ProcessAdcRead( )) state = ButtonState_WaitingForY;
    break;
  case ButtonState_WaitingForY :
    if( JoyY.ProcessAdcRead( )) state = ButtonState_WaitingForButtons;
    break;
  case ButtonState_WaitingForButtons :
    if( LCD_ScanButtons( )) state = ButtonState_WaitingForX;
    break;
  } 
  */   
}
