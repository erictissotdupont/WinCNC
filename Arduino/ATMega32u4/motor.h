class Motor {
public : 
  Motor( int sp, int dp, int ep, int s );
  void Reset( );
  void SetDirection( int d );
  long InitMove( long s, long t );
//  long TimeToNextMove( long t );
  long Move( );
  long GetPos( );
  long FakeMove( long s );
  bool IsAtTheEnd( );  
  
private :
  long curPos;
  char curDir;
  char toggle;
  char swap;
  
  char stepPin;
  char dirPin;
  char endPin;
  
  unsigned int moveLength;
  unsigned int moveStep;
  unsigned long moveDuration;
  long stepDuration;
  
  long decelDist;
  unsigned int stepDecrement;
};

long NonObjectTest( long i );

