#include <stdarg.h> 
#include "CNC.h"
#include "helper.h"

#define MAX_DBG_STR 80

void DebugPrint( const char* format, ... )
{
  char str[MAX_DBG_STR];
  va_list ap;
  va_start(ap, format);
  vsnprintf( str, MAX_DBG_STR, format, ap );
  va_end(ap);
  Serial.print( str );
}

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

