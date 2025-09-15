
#define STEP_PULSE_US     100 // Duration of the motor step pulse
#define RAMP_SHIFT        19 // 19=524ms
#define RAMP_TIME         (1L << RAMP_SHIFT)
#define MIN_SPEED         15L
#define MAX_SPEED         150L
#define NO_STEP_TIME      (-1ULL)

#define REDUCED_RAPID_POSITIONING_SPEED   0x80000000L
#define CALIBRATION_REVERSED              0x40000000L
#define DIRECTION_REVERSED                0x20000000L
#define CALIBRATION_OFFSET_INTERIOR       0x10000000L

// Calculate the speed ramp for G0 accel / decel phases 
#define STEP_FROM_RAMP( Min, Max, t ) ( Min - ((( Min - Max ) * t ) >> RAMP_SHIFT))
// Speed is in inch by minute, hence the 60M micro seconds
#define SPEED_TO_STEP( sbi, s ) ( 60000000L / ((sbi) * (s)))

class Motor {
public : 
  Motor( gpio_num_t sp, gpio_num_t dp, uint32_t em, unsigned long flags, unsigned long sbi );
  
  // Virtual is to make sure the derived class
  // implementation gets called
  virtual bool SetDirection( int d );
  virtual void Pulse( bool on );
  virtual int GetLimit( );
  virtual void CalibrateTask( uint64_t now );
  
  // Those functions are not to overloaded
  void Reset( );
  long GetPos( );
  uint64_t InitMove( long s, unsigned long t, uint64_t now );
  uint64_t GetNextStepTime( );
  void MovementTask( uint64_t now );
  void CalibrateStart( uint64_t now, long max_step, unsigned long state_flag );
  void CalibrationComplete( );
  
protected :
  long curPos;                // Current axis position in steps

  gpio_num_t stepPin;         // GPIO for stepping
  bool pulseLevel;            // Save the current state of the pulse
  gpio_num_t dirPin;          // GPIO for direction
  int reverseDir;             // Reverse the motor direction
  uint32_t endMask;           // Bitmask for limit detection
  unsigned long motorFlags;   // Configutation flags for this axis
    
  long curDir;                // Current movement direction (+/- 1)
  unsigned long moveLength;   // Movement total length in steps
  unsigned long moveStep;     // Steps performed in movement
  unsigned long moveDuration; // Expected total duration of the movement
  long stepDuration;          // Duration of a half step
  long stepModulo;            // The remainder of the division
  uint64_t nextStepTime;      // Time when the next half step should be made
  bool dirLevel;
//uint64_t currentStepTime;   // Time when the current step is happening
  long stepAcc;               // The fractional error accumulator

  // Rapid positionning (G0)
  long decelDist;             // Step in movement when deceleration starts
  unsigned long decelTime;    // Duration of the deceleration phase 
  unsigned long decelStart;   // Time when the deceleration has started
  unsigned long minSpeedStep; // Duration of a step at G0 min speed 
  unsigned long maxSpeedStep; // Same for max speed
  unsigned long stepByInch;   // Number of steps for one inch (approx)

  // Calibration state and configuration
  // -----------------------------------
  bool cal_checkCurPos;
  int cal_state;
  long cal_count;
  long cal_stall;
  int cal_toward;
  int cal_away;
  int cal_cycle;              // Number of calibration cycles. Echh cycle slows down to increase precision
  unsigned long cal_state_flag; // The state flag to be signaled when this axis is calbrated
  long cal_max_step;

};

class DualMotor : public Motor 
{
public:
  DualMotor( gpio_num_t sp, gpio_num_t dp, uint32_t em, gpio_num_t sp2, gpio_num_t dp2, uint32_t em2, unsigned long flags, unsigned long sbi, long cof, float R );
  
private:
  gpio_num_t stepPin2;        // GPIO for stepping 2nd motor
  gpio_num_t dirPin2;         // GPIO for direction of 2nd motor
  uint32_t endMask2;          // Bitmask for limit detection for 2nd motor

  // Calibration
  long cal_offset;            // Position difference between the L and R calibration positions (in steps)
  long cal_delta;             // Difference of steps required between the L and R motor to reach each sensor
  long cal_dL, cal_dR;        // Number of steps to correct the slanting of the axis prior to calibration for L and R motors 
  float cal_R;                // The ratio betweem the position of the sensors and the motors. Used to correct the slanting effect.


public:
  virtual void Pulse( bool on ) override;
  void PulseLeft( bool on );
  void PulseRight( bool on );
  virtual bool SetDirection( int d )override;
  virtual int GetLimit( ) override;
  virtual void CalibrateTask( uint64_t now ) override;

};