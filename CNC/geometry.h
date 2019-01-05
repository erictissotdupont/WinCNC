#pragma once

#include <math.h>

#define PI           3.14159265359
#define IPM_TO_IPMS  60000
#define NEAR_ZERO    0.00000001

typedef struct _t3DPoint {
  double x;
  double y;
  double z;
} t3DPoint;

typedef struct _t2DPoint {
	double x;
	double y;
} t2DPoint;

typedef struct _t2DintPoint {
	int x;
	int y;
} t2DintPoint;

double absOf( double n );
double maxOf3( double a, double b, double c );
double absMaxOf3( double a, double b, double c );
double minOf3( double a, double b, double c );
double absMinOf3( double a, double b, double c );
double minOf2( double a, double b );
double vector3DLength( t3DPoint V );
double vectorLength( double U1, double U2 );
double distance3D( t3DPoint A, t3DPoint B );
double distance2D( t2DPoint A, t2DPoint B );
double angleVector( double U1, double U2, double V1, double V2 );
double dotProduct( double U1, double U2, double V1, double V2 );
void rotateInXYPlane( t3DPoint* P, t3DPoint C, double a );
void rotate2Dvector(t2DPoint* P, double a);