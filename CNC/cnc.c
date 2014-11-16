#include <unistd.h>
#include <sys/stat.h> 
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include <string.h>

#include "status.h"
#include "keyboard.h"
#include "geometry.h"
#include "gcode.h"
#include "motor.h"
#include "socket.h"
#include "unitTest.h"

#define MAX_LINE 5000

#define RELMOVE "G91 G40 G1 "

// 1/256 of an inch.
#define SMALL_MOVE ".00390625"

void OnKey( int key )
{
  // printf( "OnKey( 0x%08X )\n", key );
  switch( key )
  {
  case VK_UP :
    doGcode( RELMOVE"X"SMALL_MOVE );
    break;
  case VK_DOWN :
    doGcode( RELMOVE"X-"SMALL_MOVE );
    break;
  case VK_RIGHT :
    doGcode( RELMOVE"Y-"SMALL_MOVE );
    break;
  case VK_LEFT :
    doGcode( RELMOVE"Y"SMALL_MOVE );
    break;
  case VK_PAGEUP :
    doGcode( RELMOVE"Z"SMALL_MOVE );
    break;
  case VK_PAGEDN :
    doGcode( RELMOVE"Z-"SMALL_MOVE );
    break;
  }
}

tStatus parseLine( char* cmd )
{
  tStatus ret = retSuccess;
  char* pt;

  pt = strstr( cmd, "\n" );
  if( pt )
  {
    *pt = 0;
    pt++;
  }
  
  //printf( "Parsing '%s'\n", cmd );

  if( strstr( cmd, "RUN " ) == cmd )
  {
    FILE *fp;
    char *fileName = cmd+4;
    char* buffer = (char*)malloc( MAX_LINE );
    char line[80];
    struct stat st;
    float read = 0.0;
    int l = 0;

    stat(fileName, &st);

    if( st.st_size == 0 )
    {
      printf( "File '%s' is empty. Cannot execute.\n", fileName );
      return retFileNotFound;
    }

    fp = fopen( fileName, "r");
    if( fp == NULL ) 
    {
      printf( "Opening command file '%s' failed.\n", fileName );
      return retFileNotFound;
    }

    while( fgets( buffer, MAX_LINE, fp ))
    {
      l = strlen( buffer );
      if( l > 0 )
      {
        read += l;
        if( buffer[l-1] == '\n' ) buffer[l-1] = 0;
        sprintf( line, "%.1f%% - %s", ( read * 100 ) / st.st_size, buffer );      

	l = strlen(line);
	memset( line+l, ' ', sizeof(line) - 1 - l );
        line[sizeof(line) - 1] = 0;

        printf( "\r%s", line );

        if(( ret = parseLine( buffer )) != retSuccess ) break;
      }
    }

    printf( "Done.\n" );
    fclose( fp );
    free( buffer );
  }
  else if( strstr( cmd, "EXP" ) == cmd )
  {
    char *fileName = cmd+4;
    setExportFile( fopen( fileName, "w"));
  }
  else if( strstr( cmd, "POS" ) == cmd )
  {
    showDistanceInfo( );
  }
  else if( strstr( cmd, "TST " ) == cmd )
  {
    int i;
    char str[20];
    int step,axis,cycle;
    sscanf( cmd+4, "%d %d %d", &axis, &step, &cycle );
    printf( "Test motor %d\n", step );
    // testMotor( axis, step, cycle );

    sprintf( str, "G10 G90 G1 F%d", step );
    doGcode( str );

    for( i=0; i<cycle; i++ )
    {
       sprintf( str, "Z%.6f", (( rand( ) % 1000 ) / 100000.0 ));
       sprintf( str, "Z%.6f", (( rand( ) % 1000 ) / 100000.0 ));
       sprintf( str, "Z%.6f", (( rand( ) % 1000 ) / 100000.0 ));
       doGcode( str );
       doGcode( "Z0" );
    }
   
  }
  else if( strstr( cmd, "UNT" ) == cmd )
  {
    unitTest( );
  }
  else if( strstr( cmd, "MAN" ) == cmd )
  { 
    int k;
    do
    {
      k = 0;
      if( getkey( (char*)&k, sizeof(k)) > 0 )
      {
        OnKey( k );
      }
      else
      {
        usleep( 10000 ); // 10ms
      }
    } while( k!=27);
  }
  else if( strstr( cmd, "QUIT" ) == cmd )
  {
    ret = retQuit;
  }
  else
  {
    switch( ret = doGcode( cmd ))
    {
    case retSuccess :
      // Success.
      break;

    case retInvalidParam :
      printf("Invalid command.\n");
      break;

    case retSyntaxError :
      printf("GCode syntax error.\n");
      break;

    case retFileNotFound :
      printf("File not found.\n");
      break;

    case retUserAborted :
      printf("User interruption.\n");
      break;

    case retNoOutputFound :
      printf("No output.\n");
      break;

    case retCncNotConnected :
      printf("CNC not connected.\n");
      break;

    case retCncCommError :
      printf("CNC communication error.\n");
      break;

    case retCncError :
      printf("CNC in error state.\n");
      break;

    case retQuit :
      printf("Quit.\n");
      break;

    case retUnknownErr :
      printf("Unexpected error.\n");
      break;
    }
  }

  return ret;
}

int main()
{
  char cmd[100];
 
  pthread_t thId = pthread_self();
  pthread_attr_t thAttr;
  int policy = 0;

  pthread_attr_init(&thAttr);
  if( pthread_attr_setschedpolicy(&thAttr,SCHED_FIFO)) printf( "pthread_attr_setschedpolicy( FIFO ) failed.\n" );
  if( pthread_attr_getschedpolicy(&thAttr, &policy)) printf( "pthread_attr_getschedpolicy failed.\n" );
  if( pthread_setschedprio(thId, 0 )) printf( "pthread_setschedprio(%d) failed\n", 0 );
  pthread_attr_destroy(&thAttr);

  // 1/16 step - 400 steps - 2.8in per turn = 0.0004375 per step
  // Error of 0.32% (too far) = 0.0004393

  initAxis( 0, 0.0004389 ); // X
  initAxis( 1, 0.0004389 ); // Y
  initAxis( 2, 0.0003125 ); // Z - 1/4 step - 400 steps - 0.5in per turn

  initSpindle( );

  initSocketCom( );

  do
  {
    printf( "CNC>" );
  } while( parseLine( fgets(cmd, sizeof(cmd), stdin)) != retQuit );

  return 0;
}

