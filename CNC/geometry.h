#pragma once

#include <math.h>

#define PI           3.14159265359
#define IPM_TO_IPMS  60000
#define NEAR_ZERO    0.000000001

typedef struct _tPoint {
  double x;
  double y;
  double z;
} tPoint;

double absOf( double n );
double maxOf3( double a, double b, double c );
double absMaxOf3( double a, double b, double c );
double minOf3( double a, double b, double c );
double absMinOf3( double a, double b, double c );
double minOf2( double a, double b );
double vector3DLength( tPoint V );
double vectorLength( double U1, double U2 );
double distance3D( tPoint A, tPoint B );
double angleVector( double U1, double U2, double V1, double V2 );
double dotProduct( double U1, double U2, double V1, double V2 );
void rotateInXYPlane( tPoint* P, tPoint C, double a );
