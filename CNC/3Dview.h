
void OnPaint(HWND hWnd);
// void init3DView( float oX, float oY, float dX, float dY, float dZ, float res, float toolRadius );
void init3DView( float res );
tStatus buildPath(t3DPoint P, long x, long y, long z, long d, long s);
void saveAltitude(LPWSTR szFilePath);