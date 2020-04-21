class Motor {
public : 
  Motor( int sp, int dp, int ep, int s, unsigned int lf, unsigned long sbi );
  void Reset( );
  void SetDirection( int d );
  long InitMove( long s, long t );
  long Move( );
  long GetPos( );
  long FakeMove( long s );
  bool IsAtTheEnd( );  
  
private :
  long curPos;                // Current axis position in steps

  int stepPin;                // GPIO for stepping
  int toggle;                 // Current step polarity
  int dirPin;                 // GPIO for direction
  int reverseDir;             // Reverse the motor direction
  int endPin;                 // Optional GPIO for limit detection
  unsigned int limitFlag;     // Flags to set when limit is reached
  
  long curDir;                // Current movement direction (+/- 1)
  unsigned long moveLength;   // Movement total length in steps
  unsigned long moveStep;     // Steps performed in movement
  unsigned long moveDuration; // Expected total duration of the movement
  long halfStepDuration;      // Duration of a half step

  // Rapid positionning (G0)
  //
  long decelDist;             // Step in movement when deceleration starts
  unsigned long decelTime;    // Duration of the deceleration phase 
  unsigned long decelStart;   // Time when the deceleration has started
  long minSpeedHalfStep;      // Duration of half step at G0 min speed 
  long maxSpeedHalfStep;      // Same for max speed
  unsigned long stepByInch;   // Number of steps for one inch (approx)
};

void Motor_Init( );
void Motor_Move( long x, long y, long z, long d );
