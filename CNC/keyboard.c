
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>


int bRun;

int getkey( char* pt, int size )
{
  char c;
  int n = 0;
  struct termios orig_term_attr;
  struct termios new_term_attr;

  /* set the terminal to raw mode */
  tcgetattr(fileno(stdin), &orig_term_attr);
  memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
  new_term_attr.c_lflag &= ~(ECHO|ICANON);
  new_term_attr.c_cc[VTIME] = 0;
  new_term_attr.c_cc[VMIN] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

  /* read a character from the stdin stream without blocking */
  /*   returns EOF (-1) if no character is available */
  while((( c = fgetc(stdin)) != 0xFF ) && n<size )
  {
    printf( "%d[%d] ", c, n );
    pt[n++] = c;
  }

  /* restore the original terminal attributes */
  tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

  return n;
}

void* keyboardListener(void* pt)
{
  int n;
  long k = 0;
  void(*callback)(int) = pt;
  while( bRun )
  {
    // 10ms
    usleep( 10000 );
    n = getkey( (char*)&k, sizeof(k));
    if( n>0 ) callback( k );
  }
  return NULL;
}

int startKeyInput( void(*callback)(int))
{
  pthread_t keyboardListenerThread;
  bRun = 1;
  if( pthread_create (&keyboardListenerThread, NULL, keyboardListener, callback) < 0 ) {
    //fprintf (stderr, "serialStartListener - pthread_create failed: %s\n", strerror (errno));
    return -1;
  }
  return 0;
}
