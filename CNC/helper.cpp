#include <Arduino.h>
#include "helper.h"

char DebugStr[80];

void DebugPulse( int c )
{
  int i;
  while( 1 )
  {
    for( i=0; i<c; i++ )
    {
      digitalWrite( 13, HIGH );
      delay( 150 );
      digitalWrite( 13, LOW ); 
      delay( 300 );
    }
    delay( 1000 );
  }
}  

// ------------------------------------------------------------------------
long minOf2( long a, long b )
{
  if( a < 0 ) return b;
  if( b < 0 ) return a;
  if( a < b ) return a;
  return b;
}

long minOf3( long a, long b, long c )
{
  if( a < 0 ) return minOf2( b,c );
  if( b < 0 ) return minOf2( a,c );
  if( c < 0 ) return minOf2( a,b );
  
  if(( a < b ) && ( a < c )) return a;
  if( b < c ) return b;
  return c;
}

