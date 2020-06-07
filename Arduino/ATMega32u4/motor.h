
#define REG_TYPE volatile uint32_t

class Motor {
public : 
  Motor( int sp, int dp, int ep, int s, unsigned long flags, unsigned long sbi );
  void Reset( );
  
  // This is virtual to make sure the derived class
  // implementation gets called
  virtual void SetDirection( int d );
  
  unsigned long InitMove( long s, long t );
  void PrepareNextStep( );
  void Step( unsigned long& t );

  void Manual( int dir );
  
  long GetPos( );
  long FakeMove( long s );
  bool IsAtTheEnd( );  
  
protected :
  long curPos;                // Current axis position in steps

  REG_TYPE* stepSetReg;       // Register to SET step pin
  REG_TYPE* stepClrReg;       // Register to CLEAR the step pin
  REG_TYPE stepPinMask;       // Bit mask for the step pin

  int stepPin;                // GPIO for stepping
  int dirPin;                 // GPIO for direction
  int reverseDir;             // Reverse the motor direction
  int endPin;                 // Optional GPIO for limit detection
  unsigned int limitFlag;     // Flags to set when limit is reached
  
  long curDir;                // Current movement direction (+/- 1)
  unsigned long moveLength;   // Movement total length in steps
  unsigned long moveStep;     // Steps performed in movement
  unsigned long moveDuration; // Expected total duration of the movement
  long stepDuration;          // Duration of a half step
  long stepModulo;            // The remainder of the division
  unsigned long nextStepTime; // Time when the next half step should be made
  unsigned long currentStepTime; // Time when the current step is happening
  long stepAcc;           // The fractional error accumulator

  // Rapid positionning (G0)
  //
  long decelDist;             // Step in movement when deceleration starts
  unsigned long decelTime;    // Duration of the deceleration phase 
  unsigned long decelStart;   // Time when the deceleration has started
  long minSpeedStep;          // Duration of a step at G0 min speed 
  long maxSpeedStep;          // Same for max speed
  long stepByInch;            // Number of steps for one inch (approx)

  // Manual mode
  int manual;
  int accManual;
  unsigned long nextSlowStep;
};

class DualMotor : public Motor 
{
public:
  DualMotor( int sp, int dp, int ep, int sp2, int dp2, int ep2, int s, unsigned long flags, unsigned long sbi );
  
private:
  int stepPin2;
  int dirPin2;
  int endPin2;

  REG_TYPE* step2SetReg;      // Register to SET step pin
  REG_TYPE* step2ClrReg;      // Register to CLEAR the step pin
  REG_TYPE step2PinMask;      // Bit mask for the step pin

public:
  void PrepareNextStep( );
  void Step( unsigned long& t );
  void Reset( );
  void SetDirection( int d );
 
};

void Motor_Init( );
void Motor_Move( long x, long y, long z, long d );
