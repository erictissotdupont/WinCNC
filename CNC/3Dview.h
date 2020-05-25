
#include "geometry.h"

void OnPaint(HWND hWnd);

bool init3DView( float x, float y );
void initToolShape( float radius );
void update3DView( );
void resetBlockSurface();
bool start3DViewer();

tStatus buildPath(t3DPoint P, long x, long y, long z, long d, long s);
void saveAltitude(LPWSTR szFilePath);

void Start3DSimulator(HWND hWnd);