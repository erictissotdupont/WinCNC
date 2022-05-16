
typedef enum {
  Manual_Disabled,
  Manual_XY,
  Manual_Z,
  Manual_MaxMode
} tManualMode;

//return values for ReadButtons()
typedef enum {
  BUTTON_NONE = 0, 
  BUTTON_RIGHT,
  BUTTON_UP,
  BUTTON_DOWN,
  BUTTON_LEFT,
  BUTTON_SELECT
} LCD_Button;

// LCD GPIOs
/*
#define LCD_D0   4
#define LCD_D1   5
#define LCD_D2   6
#define LCD_D3   7
#define LCD_RS   8
#define LCD_EN   9
*/

#define LCD_X_JOY          A0
#define LCD_Y_JOY          A1
#define LCD_BUTTONS        A2

// LCD commands
#define LCD_CLEARDISPLAY   0x01
#define LCD_RETURNHOME     0x02
#define LCD_ENTRYMODESET   0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT    0x10
#define LCD_FUNCTIONSET    0x20
#define LCD_SETCGRAMADDR   0x40
#define LCD_SETDDRAMADDR   0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON           0x04
#define LCD_DISPLAYOFF          0x00
#define LCD_CURSORON            0x02
#define LCD_CURSOROFF           0x00
#define LCD_BLINKON             0x01
#define LCD_BLINKOFF            0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE         0x08
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x04
#define LCD_MOVELEFT            0x00

// flags for function set
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x04
#define LCD_5x8DOTS             0x00

// flags for backlight control
#define LCD_BACKLIGHT           0x08
#define LCD_NOBACKLIGHT         0x00

#define En 0x04 // B00000100  // Enable bit
#define Rw 0x02 // B00000010  // Read/Write bit
#define Rs 0x01 // B00000001  // Register select bit

void LCD_setCursor_high(uint8_t, uint8_t); 
void LCD_setCursor_low(uint8_t, uint8_t); 
void LCD_setCursor(uint8_t, uint8_t); 

void LCD_write_high(uint8_t value);
void LCD_write_low(uint8_t value);
void LCD_write(uint8_t);

void LCD_command(uint8_t);

void LCD_send_high(uint8_t, uint8_t);
void LCD_send_low(uint8_t);
void LCD_send(uint8_t, uint8_t);

void LCD_write4bits(uint8_t);
void LCD_write8bits(uint8_t);
void LCD_pulseEnable();

void LCD_UpdateTask( );

void LCD_ButtonTask( );

void LCD_SetStatus( const char *str, int offset );

void LCD_Init( void );
void LCD_Clear( void );

void LCD_Refresh( void );
