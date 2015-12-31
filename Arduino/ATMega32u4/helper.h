
void DebugPulse( int c );

inline long minOf2( long a, long b )
{
  if( a < 0 ) return b;
  if( b < 0 ) return a;
  if( a < b ) return a;
  return b;
}

inline long minOf3( long a, long b, long c )
{  
  if( a < 0 ) return minOf2( b,c );
  if( b < 0 ) return minOf2( a,c );
  if( c < 0 ) return minOf2( a,b );
  
  if(( a < b ) && ( a < c )) return a;
  if( b < c ) return b;
  return c;
}


class Move 
{
public :
  Move( );
  void Init( long x, long y, long z, long d );
private :
  long x,y,z,d;

};

