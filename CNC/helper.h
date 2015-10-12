
void DebugPulse( int c );

long minOf2( long a, long b );
long minOf3( long a, long b, long c );

class Move 
{
public :
  Move( );
  void Init( long x, long y, long z, long d );
private :
  long x,y,z,d;

};

extern char DebugStr[80];
