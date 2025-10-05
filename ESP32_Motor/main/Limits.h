
#define XL_LIM   0x0001
#define XR_LIM   0x0002
#define Y_LIM    0x0004
#define ZL_LIM   0x0008
#define ZR_LIM   0x0010
#define HARD_LIM 0x0020

void Limits_Init( );
uint32_t GetLimitState( );