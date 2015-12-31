#include <Arduino.h>
#include "helper.h"

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

