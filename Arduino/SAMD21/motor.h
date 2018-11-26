class Motor {
public : 
  Motor( uint8_t sp, uint8_t dp, uint8_t ep, int8_t sw, uint32_t lf );
  void Reset( );
  void SetDirection( int8_t d );
  int32_t InitMove( int32_t s, uint32_t t );
  int32_t Move( );
  int32_t GetPos( );
  int32_t FakeMove( int32_t s );
  bool IsAtTheEnd( );  
  
private :
  int32_t curPos;
  int8_t curDir;
  int8_t toggle;
  int8_t swap;
  uint32_t limitFlag;
  
  uint8_t stepPin;
  uint8_t dirPin;
  uint8_t endPin;
  
  uint32_t moveLength;
  uint32_t moveStep;
  uint32_t moveDuration;
  uint32_t stepDuration;
  
  uint32_t decelDist;
  uint32_t stepDecrement;
};

void Motor_Init( );
void Motor_Move( int32_t x, int32_t y, int32_t z, uint32_t d );

