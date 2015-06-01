
#include "geometry.h"

double absOf( double n )
{
  if( n < 0.0 ) return -n;
  return n;
}

double maxOf3( double a, double b, double c )
{
  if( a > b && a > c ) return a;
  if( b > c ) return b;
  return c;
}

double absMaxOf3( double a, double b, double c )
{
  if( a < 0.0) a = -a;
  if( b < 0.0) b = -b;
  if( c < 0.0) c = -c;
  return maxOf3( a,b,c );
}

double minOf3( double a, double b, double c )
{
  if( a < b && a < c ) return a;
  if( b < c ) return b;
  return c;
}

double absMinOf3( double a, double b, double c )
{
  if( a < 0.0) a = -a;
  if( b < 0.0) b = -b;
  if( c < 0.0) c = -c;
  return minOf3( a,b,c );
}

double minOf2( double a, double b )
{
  if( a < b ) return a;
  return b;
}

double vector3DLength( t3DPoint V )
{
  return sqrt( V.x*V.x + V.y*V.y + V.z*V.z );
}

double vectorLength( double U1, double U2 )
{
  return sqrt( U1*U1 + U2*U2 );
}

double distance3D( t3DPoint A, t3DPoint B )
{
  t3DPoint V;
  V.x = A.x - B.x;
  V.y = A.y - B.y;
  V.z = A.z - B.z;
  return vector3DLength( V ); 
}

double angleVector( double U1, double U2, double V1, double V2 )
{
  double a,b,r;
  a = U1*V1 + U2*V2;
  b = vectorLength(U1,U2) * vectorLength(V1,V2);
  r = acos( a/b ); 
  //printf( "angleVector:(%.3f,%.3f)|(%.3f,%.3f)=%.3f (a:%.3f b:%.3f)\n", U1,U2,V1,V2,r*180.0/PI,a,b );
  return r;
}

double dotProduct( double U1, double U2, double V1, double V2 )
{
  return U1*V1+U2*V2;
}

void rotateInXYPlane( t3DPoint* P, t3DPoint C, double a )
{
  t3DPoint R;
  // Translate axis to orign at C
  R.x = P->x - C.x;
  R.y = P->y - C.y;
  // Rotate in XY plane
  P->x = R.x*cos(a) - R.y*sin(a);
  P->y = R.x*sin(a) + R.y*cos(a);
  // Move back the axis origin
  P->x = P->x + C.x;
  P->y = P->y + C.y;
}
