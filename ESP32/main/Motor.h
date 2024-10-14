
#define REG_TYPE volatile uint32_t



class Motor {
public : 
  Motor( int sp, int dp, uint32_t em, unsigned long flags, unsigned long sbi );
  void Reset( );
  
  // This is virtual to make sure the derived class
  // implementation gets called
  virtual void SetDirection( int d );
  
  unsigned long InitMove( long s, long t );
  void PrepareNextStep( );
  void Step( );
  void Step( unsigned long& t );

  void Manual( int dir );
  
  long GetPos( );
  long FakeMove( long s );
  int GetLimit( );
  void CalibrateStart( );
  bool CalibrateTask( );
  
protected :
  long curPos;                // Current axis position in steps

  REG_TYPE* stepSetReg;       // Register to SET step pin
  REG_TYPE* stepClrReg;       // Register to CLEAR the step pin
  REG_TYPE stepPinMask;       // Bit mask for the step pin

  int stepPin;                // GPIO for stepping
  int dirPin;                 // GPIO for direction
  int reverseDir;             // Reverse the motor direction
  uint32_t endMask;           // Bitmask for limit detection
  unsigned int limitFlag;     // Flags to set when limit is reached
  
  long curDir;                // Current movement direction (+/- 1)
  unsigned long moveLength;   // Movement total length in steps
  unsigned long moveStep;     // Steps performed in movement
  unsigned long moveDuration; // Expected total duration of the movement
  long stepDuration;          // Duration of a half step
  long stepModulo;            // The remainder of the division
  unsigned long nextStepTime; // Time when the next half step should be made
  unsigned long currentStepTime; // Time when the current step is happening
  long stepAcc;               // The fractional error accumulator
  int endDetectionState;      // The current state of the end of course pin
  unsigned long endPinDebounceTime; // To measure how long the end of course
                              // detection pin stays high (debouncing)

  // Rapid positionning (G0)
  //
  long decelDist;             // Step in movement when deceleration starts
  unsigned long decelTime;    // Duration of the deceleration phase 
  unsigned long decelStart;   // Time when the deceleration has started
  unsigned long minSpeedStep;          // Duration of a step at G0 min speed 
  unsigned long maxSpeedStep;          // Same for max speed
  unsigned long stepByInch;            // Number of steps for one inch (approx)

  // Calibration
  int cal_state;
  long cal_count;
  long cal_stall;
  uint32_t cal_time;
  int cal_toward;
  int cal_away;

  // Manual mode
  int manual;
  int accManual;
  unsigned long nextSlowStep;
};

class DualMotor : public Motor 
{
public:
  DualMotor( int sp, int dp, uint32_t em, int sp2, int dp2, uint32_t em2, unsigned long flags, unsigned long sbi, long cof, float R );
  
private:
  int stepPin2;               // GPIO for stepping 2nd motor
  int dirPin2;                // GPIO for direction of 2nd motor
  uint32_t endMask2;          // Bitmask for limit detection for 2nd motor

  // Calibration
  long cal_offset;            // Position difference between the L and R calibration positions (in steps)
  long cal_delta;             // Difference of steps required between the L and R motor to reach each sensor
  long cal_dL, cal_dR;        // Number of steps to correct the slanting of the axis prior to calibration for L and R motors 
  float cal_R;                // The ratio betweem the position of the sensors and the motors. Used to correct the slanting effect.
  int cal_cycle;              // Number of calibration cycles. Echh cycle slows down to increase precision

  REG_TYPE* step2SetReg;      // Register to SET step pin
  REG_TYPE* step2ClrReg;      // Register to CLEAR the step pin
  REG_TYPE step2PinMask;      // Bit mask for the step pin

public:
  void PrepareNextStep( );
  void StepL( );
  void StepR( );
  void Step( unsigned long& t );
  void Reset( );
  void SetDirection( int d );
  int GetLimit( );
  void CalibrateStart( );
  bool CalibrateTask( );
};

void Motor_Init( );
void Motor_Move( long x, long y, long z, long d );
