#include "CNC.h"
#include "Resource.h"
#include "Windowsx.h"
#include "math.h"

#include "gcode.h"
#include "fileParser.h"
#include "Shapes.h"
#include "3Dview.h"

typedef enum {
	modeContourOnly,
	modeCenterOnly,
	modeContourAndCenter,
	modeMatrix,
	modeHoneycomb,
} tCarveMode;

typedef struct
{
	tGeneralToolInfo tool;

	WCHAR szFilePath[MAX_PATH];

	double width;
	double height;
	double depth;
	tCarveMode contourOrCarve;
	int bHorizontalCarveOnly;
	double matrixPitch;
	double matrixXoffset;
	double matrixYoffset;

} tBitmapShapeParam;

typedef struct
{
	BITMAP* bm;
	int x;
	int y;
} tCleanBmInfo;

tBitmapShapeParam g_BmParams;
double g_Xres = (1/64.0f);
double g_Yres = (1/64.0f);

void BitmapShapeInit(HWND hWnd)
{
	int i;
	HWND hItem;

	hItem = GetDlgItem(hWnd, IDC_TOOL_SIZE);
	for (i = 0; i < ITEM_CNT(TOOL_SIZES); i++) ComboBox_AddString(hItem, TOOL_SIZES[i].str);

	hItem = GetDlgItem(hWnd, IDC_CUT_SPEED);
	for (i = 0; i < ITEM_CNT(CUT_SPEED); i++) ComboBox_AddString(hItem, CUT_SPEED[i].str);

	ShapeInitToolInfo(&g_BmParams.tool);
	g_BmParams.width = 3.25;
	g_BmParams.height = 3.25;

	HKEY hKey;
	if (RegOpenKey(HKEY_CURRENT_USER, L"SOFTWARE", &hKey) == ERROR_SUCCESS)
	{
		DWORD cbData = sizeof(g_BmParams);
		RegGetValue(hKey, L"WinCNC", L"BitmapShape", RRF_RT_REG_BINARY, NULL, &g_BmParams, &cbData);
		RegCloseKey(hKey);
	}
}

void BitmapShapeSave()
{
	HKEY hKey;
	if (RegCreateKey(HKEY_CURRENT_USER, L"SOFTWARE\\WinCNC", &hKey) == ERROR_SUCCESS)
	{
		RegSetValueEx(hKey, L"BitmapShape", 0, REG_BINARY, (BYTE*)&g_BmParams, sizeof(g_BmParams));
	}
}

UINT BitmapShapeGetSet(BOOL get, HWND hWnd)
{
	ShapeGetSetTool(hWnd, get, &g_BmParams.tool);

	ShapeGetSetString(hWnd, IDC_BITMAP_PATH, get, g_BmParams.szFilePath, MAX_PATH);

	int tmp = g_BmParams.contourOrCarve;
	ShapeGetSetRadio(hWnd, IDC_BITMAP_CONTOUR_CARVE, 5, get, &tmp );
	g_BmParams.contourOrCarve = (tCarveMode)tmp;

	ShapeGetSetDouble(hWnd, IDC_BITMAP_WIDTH, get, &g_BmParams.width);
	ShapeGetSetDouble(hWnd, IDC_BITMAP_HEIGHT, get, &g_BmParams.height);
	ShapeGetSetDouble(hWnd, IDC_BITMAP_DEPTH, get, &g_BmParams.depth);
	ShapeGetSetDouble(hWnd, IDC_BITMAP_MATRIX_PITCH, get, &g_BmParams.matrixPitch);

	ShapeGetSetBool(hWnd, IDC_BITMAP_CARVE_HORIZONTAL_ONLY, get, &g_BmParams.bHorizontalCarveOnly);

	ShapeGetSetDouble(hWnd, IDC_MATRIX_X_OFFSET, get, &g_BmParams.matrixXoffset);
	ShapeGetSetDouble(hWnd, IDC_MATRIX_Y_OFFSET, get, &g_BmParams.matrixYoffset);

	return 0;
}

void BitmapShapeExecute(HWND hWnd)
{
	char* cmd;
	HWND hItem;
	hItem = GetDlgItem(hWnd, IDC_GCODE);

	int l = GetWindowTextLength(hItem) + 1;
	cmd = (char*)malloc(l);
	if (cmd)
	{
		GetWindowTextA(hItem, cmd, l);
		ParseBuffer(hWnd, cmd, l, doGcode, FALSE );
		free(cmd);
	}
}

typedef enum {
	resultNoOverlap = 0,
	resultEdgeContact = 1,
	resultToolPartialOverlap = 2,
	resultToolHalfOverlap = 3,
	resultToolFullOverlap = 4,
	
} toolPosResult_t;

#define SMALL_OVELAP			(1/64.0f)
#define SMALLEST_RADIUS			( g_BmParams.tool.radius + SMALL_OVELAP )
#define SMALL_ANGLE				(PI/180)		// One degree

#define GET_BYTE( pt, width, x, y ) ((unsigned char*)pt + (y * width) + (x / 8));
#define GET_MASK( x ) (0x80 >> (x % 8))

BOOL GetPixel(BITMAP* bm, int x, int y)
{
	unsigned char* pt;
	// Test if the point is within the bitmap. Return TRUE as to create a boundary around
	// the carving surface the tool will contour
	if (x < 0 || y < 0 || x >= bm->bmWidth || y >= bm->bmHeight) return TRUE;
	// Move pointer to byte that contains this pixel
	//pt = (unsigned char*)bm->bmBits + (y * bm->bmWidthBytes) + (x / 8);
	pt = GET_BYTE(bm->bmBits, bm->bmWidthBytes, x, y);
	// Mask pixel in the byte
	return (*pt & GET_MASK( x )) == 0;
}

unsigned long SetPixel(BITMAP* bm, int x, int y)
{
	long carved = 0;
	unsigned char* pt;
	// Test if the point is within the bitmap
	if (x < 0 || y < 0 || x >= bm->bmWidth || y >= bm->bmHeight) return 0;
	// Move pointer to byte that contains this pixel
	//pt = (unsigned char*)bm->bmBits + (y * bm->bmWidthBytes) + (x / 8);
	pt = GET_BYTE(bm->bmBits, bm->bmWidthBytes, x, y);
	if (*pt & GET_MASK(x)) carved = 1;
	// Mask pixel in the byte
	*pt &= ~GET_MASK(x);
	return carved;
}

BITMAP* g_bm;

void RemovePointRecursive( unsigned short x, unsigned short y )
{
	unsigned char* pt;
	// Out of boundary, ignore
	if (x < 0 || y < 0 || x >= g_bm->bmWidth || y >= g_bm->bmHeight) return;
	
	//pt = (unsigned char*)g_bm->bmBits + (y * g_bm->bmWidthBytes) + (x / 8);
	pt = GET_BYTE(g_bm->bmBits, g_bm->bmWidthBytes, x, y);
	
	// If pixel is white (bit is set), fill it with black
	if ((*pt & GET_MASK( x )) != 0 )
	{
		// Clear the bit to make the pixel back
		*pt &= ~GET_MASK( x );
		// Recursively remove all the points around this pixel
		RemovePointRecursive( x, y + 1);
		RemovePointRecursive( x + 1, y);
		RemovePointRecursive( x, y - 1);
		RemovePointRecursive( x - 1, y);
	}
}

DWORD CleanBitmapThread(PVOID pParam)
{
	tCleanBmInfo* pInfo = (tCleanBmInfo*)pParam;
	g_bm = pInfo->bm;
	RemovePointRecursive( pInfo->x, pInfo->y);
	return 0;
}

typedef struct {
	unsigned long x : 16;
	unsigned long y : 16;
} tSmall2DPoint;

#define PUSH(X,Y) \
	{ queue[Qin].x = X; queue[Qin].y = Y; Qin++; if (Qin >= Qsize) Qin = 0; }

#define POP(X,Y) \
	{ X = queue[Qout].x; Y = queue[Qout].y; Qout++; if (Qout >= Qsize) Qout = 0; }

void CleanBitmap( BITMAP* bm, int x, int y )
{
	unsigned char* pt;
	tSmall2DPoint* queue;
	int Qin, Qout;
	int Qsize;

	// Only store point coordinates in a 16bit int
	if ((bm->bmWidth > 65535) || bm->bmHeight > 65535) return;

	Qin = 0;
	Qout = 0;
	Qsize = bm->bmHeight * bm->bmWidth;
	// Allocate a queue to hold at least 3x the # of points in the bitmap
	// This algorithm is quite inefficent because it will test the same points
	// multiple times but at least it won't blow up the stack.
	// This works with 2x but 3x is safer.
	queue = (tSmall2DPoint*)malloc( Qsize * sizeof(tSmall2DPoint) * 3 );
	if (!queue) return;

	// Add node to the end of Q.
	PUSH(x, y);
	while (Qin != Qout)
	{
		// Get node at the start of the Q
		POP(x, y);
		// Test if it's within the bitmap
		if (x < 0 || y < 0 || x >= bm->bmWidth || y >= bm->bmHeight) continue;
		
		// Test if color
		//pt = (unsigned char*)bm->bmBits + (y * bm->bmWidthBytes) + (x / 8);
		pt = GET_BYTE(bm->bmBits, bm->bmWidthBytes, x, y);

		if ((*pt & GET_MASK( x )) != 0)
		{
			// If white, clear the bit to make the pixel back
			*pt &= ~GET_MASK( x );
			PUSH(x + 1, y);
			PUSH(x, y + 1);
			PUSH(x - 1, y);
			PUSH(x, y - 1);
		}
	}
	free(queue);
}


void AddPoint(int x, int y, t2DintPoint* list, unsigned long* count, unsigned long max)
{
	if (*count + 4 >= max)
	{
		return;
	}
	list[*count].x = x;
	list[*count].y = y;
	(*count)++;
	if (x > 0)
	{
		list[*count].x = -x;
		list[*count].y = y;
		(*count)++;
	}
	if (y > 0)
	{
		list[*count].x = x;
		list[*count].y = -y;
		(*count)++;
	}
	if (x > 0 && y > 0)
	{
		list[*count].x = -x;
		list[*count].y = -y;
		(*count)++;
	}
}

toolPosResult_t TestToolPosition(BITMAP* bm, int x, int y, t2DintPoint* pTool, int nTool, t2DintPoint* pEdge, int nEdge, double* tangeant)
{
	double sX = 0.0;
	double sY = 0.0;
	int tCount = 0;

	for (int i = 0; i < nTool; i++)
	{
		if (GetPixel(bm, x + pTool[i].x, y + pTool[i].y)) tCount++;
	}

	if (tCount >= nTool) return resultToolFullOverlap;
	if (tCount >= (nTool / 2)) return resultToolHalfOverlap;
	if (tCount > 0) return resultToolPartialOverlap;

	for (int i = 0; i < nEdge; i++)
	{
		if (GetPixel(bm, x + pEdge[i].x, y + pEdge[i].y))
		{
			sX += pEdge[i].x;
			sY += pEdge[i].y;
			tCount++;
		}
	}
	if (tCount)
	{
		double a;
		if (sX != 0.0)
		{
			a = atan(sY / sX);
			// Results of atan( ) are +/- PI/2. 
			// Adjust angle when X is negative
			if (sX < 0) a += PI;
		}
		else
		{
			// If sX is zero, the angle is either +90 or
			// -90 degree depending on the sign of y 
			if (sY > 0)
				a = PI / 2.0;
			else
				a = (3.0 * PI) / 2.0;
		}
		// Make sure all angle are positive so that the
		// average angle is correct
		if (a < 0.0)
		{
			a += 2.0 * PI;
		}
		*tangeant = a;
		return resultEdgeContact;
	}
	else
	{
		return resultNoOverlap;
	}
}

typedef enum {
	fillRows,
	topRow,
	fillColumns,
	rightColumn
} tFillState;

#define MAX_POINTS_IN_PATH	1024

typedef struct {
	BITMAP originalBM;
	BITMAP previousBM;
	BITMAP halfCarvedBM;
	BITMAP fullCarvedBM;

	BITMAP* pTestedBM;
	BITMAP* pFullCarved;

	long iX, iY; // Current active position
	long tX, tY; // Last tested position
	double Xres, Yres;

	unsigned long toolRadiusInPixels;

	t2DintPoint* tool;
	unsigned long toolPtCnt;
	t2DintPoint* edge;
	unsigned long edgePtCnt;
	t2DintPoint* halfTool;
	unsigned long halfToolPtCount;

	BOOL bCleanup;
	BOOL bCarving;
	double carvingDepth;
	double safeToolHeight;
	double currentHeight;

	unsigned long carvingCount;
	double totalTravelDistance;
	double totalCarvingDistance;

	unsigned long pathCount;
	t2DintPoint path[MAX_POINTS_IN_PATH];


} CarvingContext_t;

tStatus GCode(const char* szFormat, ...)
{
	va_list ptr;
	tStatus ret;
	char buffer[MAX_STR];
	va_start(ptr, szFormat);
	vsprintf_s(buffer, MAX_STR, szFormat, ptr);

	if (strlen(buffer) >= 2 && (buffer[strlen(buffer) - 2] != '\r' || buffer[strlen(buffer) - 1] != '\n'))
	{
		strcat_s(buffer, "\r\n");
	}

	ret = doGcode(buffer);
	va_end(ptr);
	return ret;
}

toolPosResult_t TestToolPosition(CarvingContext_t *pCtx, int x, int y, double* tangeant)
{
	double sX = 0.0;
	double sY = 0.0;
	unsigned long tCount = 0;

	for (unsigned long i = 0; i < pCtx->toolPtCnt; i++)
	{
		int dX = x + pCtx->tool[i].x;
		int dY = y + pCtx->tool[i].y;

		if (GetPixel(pCtx->pTestedBM, dX, dY ))
		{
			tCount++;
		}
	}

	if (tCount >= pCtx->toolPtCnt) return resultToolFullOverlap;
	if (tCount >= (pCtx->toolPtCnt / 2)) return resultToolHalfOverlap;
	if (tCount > 0) return resultToolPartialOverlap;

	for (unsigned long i = 0; i < pCtx->edgePtCnt; i++)
	{
		int dX = x + pCtx->edge[i].x;
		int dY = y + pCtx->edge[i].y;

		if ( GetPixel(pCtx->pTestedBM, dX, dY ))
		{
			sX += pCtx->edge[i].x;
			sY += pCtx->edge[i].y;
			tCount++;
		}
	}
	if (tCount)
	{
		double a;
		if (sX != 0.0)
		{
			a = atan(sY / sX);
			// Results of atan( ) are +/- PI/2. 
			// Adjust angle when X is negative
			if (sX < 0) a += PI;
		}
		else
		{
			// If sX is zero, the angle is either +90 or
			// -90 degree depending on the sign of y 
			if (sY > 0)
				a = PI / 2.0;
			else
				a = (3.0 * PI) / 2.0;
		}
		// Make sure all angle are positive so that the
		// average angle is correct
		if (a < 0.0)
		{
			a += 2.0 * PI;
		}
		*tangeant = a;
		return resultEdgeContact;
	}
	else
	{
		return resultNoOverlap;
	}
}

// Returns the direction of a vector in the -PI / +PI range
// Note that a 0,0 vector will return pointing "up" (PI/2)
double vectorDirection(int x, int y)
{
	// Vertical
	if (x == 0)
	{
		// Pointing up
		if (y >= 0) return (PI / 2.0);
		// Pointing down
		else return -(PI / 2.0);
	}
	// Horizontal
	else if (y == 0)
	{
		// Pointing right
		if (x >= 0) return 0.0;
		// Pointing left
		else return PI;
	}
	// No special direction...
	double ret = atan((double)y / (double)x);
	// Result only gives angle +/- PI/2
	// If the vector is pointing backward
	if (x < 0)
	{
		// Down
		if (y > 0) ret += PI;
		// Up
		else ret -= PI;
	}
	return ret;
}

unsigned long MarkToolLocationAsCarved(BITMAP* pBitmap, int x, int y, t2DintPoint* pTool, unsigned long count, bool bSimulate )
{
	unsigned long carvedCount = 0;
	if (!bSimulate)
		for (unsigned long i = 0; i < count; i++) carvedCount += SetPixel(pBitmap, x + pTool[i].x, y + pTool[i].y);
	else
		for (unsigned long i = 0; i < count; i++) carvedCount += GetPixel(pBitmap, x + pTool[i].x, y + pTool[i].y);

	return carvedCount;
}

unsigned long MarkToolLocationAsCarved(CarvingContext_t* pCtx, int x, int y, bool bSimulate)
{
	unsigned long carved = 0;
	MarkToolLocationAsCarved(&pCtx->halfCarvedBM, x, y, pCtx->halfTool, pCtx->halfToolPtCount, bSimulate);
	carved += MarkToolLocationAsCarved(pCtx->pFullCarved, x, y, pCtx->tool, pCtx->toolPtCnt, bSimulate);
	if (pCtx->bCleanup) carved += MarkToolLocationAsCarved(pCtx->pFullCarved, x, y, pCtx->edge, pCtx->edgePtCnt, bSimulate);
	return carved;
}

unsigned long MarkToolPathAsCarved(CarvingContext_t* pCtx, int iX, int iY, int dX, int dY, bool bSimulate )
{
	unsigned long carved = 0;

	pCtx->halfToolPtCount = 0;

	pCtx->pFullCarved = bSimulate ? &pCtx->originalBM : &pCtx->fullCarvedBM;

	if (dY == 0)
	{
		// Left (-1) or right (1) ?
		int d = dX > 0 ? 1 : -1;

		if (!pCtx->bCleanup)
		{
			for (unsigned long i = 0; i < pCtx->toolPtCnt; i++)
			{
				if (dX > 0 && pCtx->tool[i].y < 0) pCtx->halfTool[pCtx->halfToolPtCount++] = pCtx->tool[i];
				else if (dX < 0 && pCtx->tool[i].y > 0) pCtx->halfTool[pCtx->halfToolPtCount++] = pCtx->tool[i];
			}
		}
		for (int i = 0; i <= abs(dX); i++)
		{
			carved += MarkToolLocationAsCarved( pCtx, iX + (i * d), iY, bSimulate);
		}
	}
	else if (dX == 0)
	{
		// Up (1) or down (-1) ?
		int d = dY > 0 ? 1 : -1;

		if (!pCtx->bCleanup)
		{
			for (unsigned long i = 0; i < pCtx->toolPtCnt; i++)
			{
				if (dY > 0 && pCtx->tool[i].x > 0) pCtx->halfTool[pCtx->halfToolPtCount++] = pCtx->tool[i];
				else if (dY < 0 && pCtx->tool[i].x < 0) pCtx->halfTool[pCtx->halfToolPtCount++] = pCtx->tool[i];
			}
		}
		for (int i = 0; i <= abs(dY); i++)
		{
			carved += MarkToolLocationAsCarved( pCtx, iX, iY + (i * d), bSimulate);
		}
	}
	else
	{
		double slope = (double)dY / (double)dX;

		if (!pCtx->bCleanup)
		{
			double direction = vectorDirection(dX, dY);
			for (unsigned long i = 0; i < pCtx->toolPtCnt; i++)
			{
				double toolDirection = vectorDirection(pCtx->tool[i].x, pCtx->tool[i].y);

				if (direction > 0)
				{
					if (toolDirection > 0 && toolDirection < direction) pCtx->halfTool[pCtx->halfToolPtCount++] = pCtx->tool[i];
					else if (toolDirection < 0 && toolDirection >(direction - PI)) pCtx->halfTool[pCtx->halfToolPtCount++] = pCtx->tool[i];
				}
				else
				{
					if (toolDirection > 0 && toolDirection > (direction + PI)) pCtx->halfTool[pCtx->halfToolPtCount++] = pCtx->tool[i];
					else if (toolDirection < 0 && toolDirection < direction) pCtx->halfTool[pCtx->halfToolPtCount++] = pCtx->tool[i];
				}
			}
		}

		if (abs(dX) >= abs(dY))
		{
			if (dX > 0) for (int i = 0; i <= dX; i++)
			{
				carved += MarkToolLocationAsCarved( pCtx, iX + i, iY + (int)((double)i * slope), bSimulate );
			}
			else if (dX < 0) for (int i = 0; i >= dX; i--)
			{
				carved += MarkToolLocationAsCarved( pCtx, iX + i, iY + (int)((double)i * slope), bSimulate);
			}
		}
		else
		{
			if (dY > 0) for (int i = 0; i <= dY; i++)
			{
				carved += MarkToolLocationAsCarved( pCtx, iX + (int)((double)i / slope), iY + i, bSimulate);
			}
			else if (dY < 0) for (int i = 0; i >= dY; i--)
			{
				carved += MarkToolLocationAsCarved( pCtx, iX + (int)((double)i / slope), iY + i, bSimulate);
			}
		}
	}

	return carved;
}

unsigned long CarveThisMoveInPixels(CarvingContext_t* pCtx, long dX, long dY)
{
	unsigned long carvedPixelsCount = 0;
	double x = dX * pCtx->Xres;
	double y = dY * pCtx->Yres;
	double length = sqrt(x * x + y * y);

	// If we're not carving, then carve
	if (!pCtx->bCarving)
	{
		// If we're above the carving altitude
		if (pCtx->currentHeight > 0)
		{
			// Let's go down to zero rapidely
			GCode("G0 Z%f", -pCtx->currentHeight);
			update3DView();
			pCtx->currentHeight = 0.0;
		}

		if (pCtx->bCleanup)
		{
			GCode("G1 Z%f", -pCtx->carvingDepth);
			update3DView();
			pCtx->currentHeight = -pCtx->currentHeight;
			pCtx->bCarving = TRUE;

			GCode("G1 X%f Y%f ", x, y );
			update3DView();
			carvedPixelsCount = MarkToolPathAsCarved(pCtx, pCtx->iX, pCtx->iY, dX, dY, false);
		}
		else
		{
			double dive = pCtx->carvingDepth + pCtx->currentHeight;

			if (length < dive / 2.0)
			{
				dive = length / 2.0;
			}

			GCode("G1 X%f Y%f Z%f", x, y, -dive);
			update3DView();
			pCtx->currentHeight -= dive;

			// Calculate the new height
			dive = pCtx->carvingDepth + pCtx->currentHeight;

			if (dive > -0.0000001 && dive < 0.0000001)
			{
				pCtx->bCarving = TRUE;
			}
		}
	}
	else
	{
		GCode("G1 X%f Y%f", x, y);
		carvedPixelsCount = MarkToolPathAsCarved(pCtx, pCtx->iX, pCtx->iY, dX, dY, false);
		update3DView();
	}
	
	pCtx->iX += dX;
	pCtx->iY += dY;
	pCtx->totalCarvingDistance += length;

	return carvedPixelsCount;
}

BOOL CarveBitmapContour(CarvingContext_t *pCtx)
{
	bool bDone;
	double tangeant;
	long dX, dY;
	unsigned long totalCarvedPixels;
	toolPosResult_t res;
	long TR = pCtx->toolRadiusInPixels;
	double dive = g_BmParams.tool.safeTravel + g_BmParams.depth;

	bDone = false;

	// Size of a equare that fits inside the tool sqrt(1/2)
	int sqInT = (int)(0.7f * pCtx->toolRadiusInPixels);

	dX = pCtx->tX;
	dY = pCtx->tY;

	const int Vx[4] = { 1, 0, -1, 0 };
	const int Vy[4] = { 0, 1, 0, -1 };
	int dir = 0;
	int V = 1;
	int v = 1;

	if (pCtx->bCleanup)
	{
		bool bGotSpot = false;

		// Check the too position against the original shape
		pCtx->pTestedBM = &pCtx->originalBM;

		do
		{
			// Look for a white (not carved) location
			if( GetPixel( &pCtx->fullCarvedBM, dX, dY ) == FALSE )
			{
				int entrapped = 0;
				if (GetPixel(&pCtx->fullCarvedBM, dX-1, dY) == TRUE) entrapped++;
				if (GetPixel(&pCtx->fullCarvedBM, dX+1, dY) == TRUE) entrapped++;
				if (GetPixel(&pCtx->fullCarvedBM, dX, dY+1) == TRUE) entrapped++;
				if (GetPixel(&pCtx->fullCarvedBM, dX, dY-1) == TRUE) entrapped++;

				// This pixel not trapped in between non carvable pixels.
				if (entrapped < 4)
				{
					// Save the pixel we just tested
					pCtx->tX = dX;
					pCtx->tY = dY;

					for (int i = -TR + 1; i < TR && !bGotSpot; i++)
						for (int n = -TR + 1; n < TR && !bGotSpot; n++)
						{
							if (sqrt(i * i + n * n) < (TR - 1))
							{
								res = TestToolPosition(pCtx, dX + i, dY + n, &tangeant);
								if (res == resultEdgeContact)
								{
									dX += i;
									dY += n;
									bGotSpot = true;
								}
							}
						}
					if (!bGotSpot)
					{
						res = TestToolPosition(pCtx, dX, dY, &tangeant);
						if (res == resultNoOverlap)
						{
							bGotSpot = true;
						}
						else
						{
							// Can't carve this spot. Mark as carved so we don't
							// come back here again
							SetPixel(&pCtx->fullCarvedBM, dX, dY);
						}
					}
				}
			}

			if (!bGotSpot)
			{
				dX += 1;
				if (dX >= (pCtx->originalBM.bmWidth - TR))
				{
					dX = TR;
					dY += 1;
					if (dY >= (pCtx->originalBM.bmHeight - TR))
					{
						bDone = true;
					}
				}
			}
		} while (!bDone && !bGotSpot);

		if (bDone)
		{
			if (pCtx->bCarving)
			{
				// Go back to safe travel height
				GCode("G0 Z%f", dive);
				pCtx->currentHeight = pCtx->safeToolHeight;
				pCtx->bCarving = FALSE;
			}

			// Return to origin
			GCode( "G0 X%f Y%f", -pCtx->iX * pCtx->Xres, -pCtx->iY * pCtx->Yres);		
			return FALSE;
		}
	}
	else
	{
		pCtx->pTestedBM = &pCtx->previousBM;

		do
		{
			// Check the tool status in the "carved" bitmap
			res = TestToolPosition(pCtx, dX, dY, &tangeant);
			if (res == resultEdgeContact)
			{
				pCtx->tX = dX;
				pCtx->tY = dY;
				break;
			}

			/*
			// If there is no overlap with any previous carving, go
			if (res == resultNoOverlap || res == resultToolFullOverlap)
			{
				// Check location with a coarse resolution
				dX += Vx[dir] * TR * V;
				dY += Vy[dir] * TR * V;
			}
			else
			*/
			{
				dX += Vx[dir];
				dY += Vy[dir];
			}

			if (dX >= (pCtx->originalBM.bmWidth - TR))
			{
				dX = pCtx->originalBM.bmWidth - TR;
			}
			else if (dX <= TR)
			{
				dX = TR;
			}
			if (dY >= (pCtx->originalBM.bmHeight - TR))
			{
				dY = pCtx->originalBM.bmHeight - TR;
			} 
			else if (dY <= TR)
			{
				dY = TR;
			}

			v--;
			if (v == 0)
			{
				dir++;
				if (dir > 3) dir = 0;
				if (dir == 0) V++;
				if (dir == 2) V++;
				v = V;

				if (V > pCtx->originalBM.bmWidth && V > pCtx->originalBM.bmHeight)
				{
					bDone = true;
				}
			}

		} while (!bDone);

		if (bDone)
		{
			pCtx->tX = TR;
			pCtx->tY = TR;
			pCtx->bCleanup = TRUE;
			return TRUE;
		}
	}

	// Get the movement needed to go to the next carving location
	dX = dX - pCtx->iX;
	dY = dY - pCtx->iY;

	// If we're not already there
	if (dX || dY)
	{
		if (pCtx->bCarving)
		{
			unsigned long test;
			test = MarkToolPathAsCarved(pCtx, pCtx->iX, pCtx->iY, dX, dY, true);

			if ( test == 0)
			{
				GCode("G1 X%f Y%f", dX* pCtx->Xres, dY* pCtx->Yres);
				MarkToolPathAsCarved(pCtx, pCtx->iX, pCtx->iY, dX, dY, false);
				update3DView();
			}
			else
			{
				GCode("G0 Z%f", dive);
				pCtx->currentHeight = pCtx->safeToolHeight;
				pCtx->bCarving = FALSE;
				pCtx->carvingCount++;
				GCode("G0 X%f Y%f", dX* pCtx->Xres, dY* pCtx->Yres);
			}
		}
		else
		{
			GCode("G0 X%f Y%f", dX * pCtx->Xres, dY * pCtx->Yres);
		}

		pCtx->iX += dX;
		pCtx->iY += dY;
		pCtx->totalTravelDistance += sqrt(dX * dX + dY * dY);
	}

	pCtx->pathCount = 0;
	dX = dY = 0;
	totalCarvedPixels = 0;

	do
	{
		int step;

		// The direction to follow the contour is 90 degrees
		// from the direction of the average contact points
		// Tool rotates clockwise.
		double a = tangeant + (PI / 2.0);
		double oldTangeant = tangeant;

		dX = dY = 0;

		do
		{
			step = 0;
			do
			{
				step++;
				dX = (long)(cos(a) * step);
				dY = (long)(sin(a) * step);

				oldTangeant = tangeant;
				res = TestToolPosition( pCtx, pCtx->iX + dX, pCtx->iY + dY, &tangeant);

			} while (res == resultEdgeContact && !bDone);

			// Go back to the previous step which was still having contact
			step--;
			dX = (long)(cos(a) * step);
			dY = (long)(sin(a) * step);
			tangeant = oldTangeant;

			if (res == resultNoOverlap)
				a -= SMALL_ANGLE;
			else if (res == resultToolPartialOverlap)
				a += SMALL_ANGLE;

			// We've tried in all directions and couln't find a way out
			if ((a > (PI * 4)) || (a < (-PI * 4.0)))
			{
				// If we're not carving, carve
				if (!pCtx->bCarving)
				{
					pCtx->bCarving = TRUE;
					pCtx->currentHeight = -pCtx->carvingDepth;
					GCode("G1 Z%f", -dive);
				}

				// Carve this spot so we won't come back again
				MarkToolLocationAsCarved(&pCtx->halfCarvedBM, pCtx->iX, pCtx->iY, pCtx->tool, pCtx->toolPtCnt, false );
				MarkToolLocationAsCarved(&pCtx->fullCarvedBM, pCtx->iX, pCtx->iY, pCtx->tool, pCtx->toolPtCnt, false );
				update3DView();
				bDone = true;
			}

		} while (dX == 0 && dY == 0 && !bDone);

		if (dX != 0 || dY != 0)
		{
			// Make the move that follows the tangeant direction
			// to the average direction (angle 'a')
			unsigned long carvedPixels = CarveThisMoveInPixels(pCtx, dX, dY);

			if (carvedPixels == 0)
			{
				for (unsigned long i = 0; i < pCtx->pathCount && !bDone; i++)
				{
					// Been here before!
					if (pCtx->path[i].x == pCtx->iX && pCtx->path[i].y == pCtx->iY)
					{
						bDone = true;
					}
				}
			}

			if( !bDone )
			{
				if (pCtx->pathCount >= MAX_POINTS_IN_PATH)
				{
					// This is too long of a path. Just start a new one
					bDone = true;
				}
				else
				{
					pCtx->path[pCtx->pathCount].x = pCtx->iX;
					pCtx->path[pCtx->pathCount].y = pCtx->iY;
					pCtx->pathCount++;
				}
			}

			totalCarvedPixels += carvedPixels;
		}

	} while ( !bDone);

	if (!pCtx->bCleanup)
	{
		memcpy(pCtx->previousBM.bmBits, pCtx->halfCarvedBM.bmBits, pCtx->originalBM.bmHeight * pCtx->originalBM.bmWidthBytes);
	}
	else
	{
		if (totalCarvedPixels == 0)
		{
			// Mark this pixel which caused us to carve without any result so 
			// that we don't come back to it again.
			SetPixel( &pCtx->fullCarvedBM, pCtx->tX, pCtx->tY);
		}
	}

	return TRUE;
}

void CopyBitmap(BITMAP* pDst, BITMAP* pSrc)
{
	size_t bitsSize = pSrc->bmWidthBytes * pSrc->bmHeight;
	*pDst = *pSrc;
	pDst->bmBits = malloc(bitsSize);
	if (pDst->bmBits != NULL)
	{
		memcpy(pDst->bmBits, pSrc->bmBits, bitsSize);
	}
}

void CarveBitmapContour( )
{
	CarvingContext_t ctx = { 0 };

	HANDLE hOriginalBM = LoadImage( NULL, g_BmParams.szFilePath, IMAGE_BITMAP, 0, 0,
		LR_CREATEDIBSECTION | LR_LOADFROMFILE | LR_DEFAULTSIZE | LR_MONOCHROME);

	GetObject(hOriginalBM, sizeof(BITMAP), &ctx.originalBM);

	CopyBitmap(&ctx.previousBM, &ctx.originalBM);
	CopyBitmap(&ctx.halfCarvedBM, &ctx.originalBM);
	CopyBitmap(&ctx.fullCarvedBM, &ctx.originalBM);

	// Calculate the size of one pixel
	ctx.Xres = g_BmParams.width / (double)ctx.originalBM.bmWidth;
	ctx.Yres = g_BmParams.height / (double)ctx.originalBM.bmHeight;

	// Build the array of points that compose the tool and the edge of the tool.
	ctx.toolRadiusInPixels = (int)(g_BmParams.tool.radius / ((ctx.Xres + ctx.Yres) / 2));

	// Estimate the # of points in the tool is the surface of the circle
	unsigned long maxToolCnt = (int)(PI * (ctx.toolRadiusInPixels + 4) * (ctx.toolRadiusInPixels + 4));
	ctx.tool = (t2DintPoint*)malloc(maxToolCnt * sizeof(t2DintPoint));

	// Estimate that the # of points in the edge twice the length of the circle
	unsigned long maxEdgeCnt = (int)(4 * PI * (ctx.toolRadiusInPixels + 2));
	ctx.edge = (t2DintPoint*)malloc(maxEdgeCnt * sizeof(t2DintPoint));

	ctx.toolPtCnt = 0;
	ctx.edgePtCnt = 0;
	for (unsigned long iX = 0; iX <= ctx.toolRadiusInPixels + 1; iX++) 
	for (unsigned long iY = 0; iY <= ctx.toolRadiusInPixels + 1; iY++)
	{
		double d = sqrt((iX * iX) + (iY * iY));
		// Points which are within the tool
		if (d <= ctx.toolRadiusInPixels)
		{
			AddPoint(iX, iY, ctx.tool, &ctx.toolPtCnt, maxToolCnt);
		}
		// Points which are within a ring around the tool. Make
		// this 2 pixels thick so that shapes must cross this area
		// before touching the tool itself
		else if (d < (ctx.toolRadiusInPixels + 2))
		{
			AddPoint(iX, iY, ctx.edge, &ctx.edgePtCnt, maxEdgeCnt);
		}
	}

	ctx.halfTool = (t2DintPoint*)malloc(ctx.toolPtCnt * sizeof(t2DintPoint));
	ctx.halfToolPtCount = 0;

	ctx.iX = 0;
	ctx.iY = 0;

	ctx.totalCarvingDistance = 0.0;
	ctx.totalTravelDistance = 0.0;
	ctx.carvingCount = 0;

	ctx.carvingDepth = g_BmParams.depth;
	ctx.safeToolHeight = g_BmParams.tool.safeTravel;
	
	// Where we start testing first
	ctx.tX = ctx.toolRadiusInPixels;
	ctx.tY = ctx.toolRadiusInPixels;

	ctx.bCarving = FALSE;
	ctx.bCleanup = FALSE;

	GCode( "G91 F%d %s",
		g_BmParams.tool.cutSpeed,
		g_BmParams.tool.motorControl ? "M3 G4 P1" : "");

	// Travel at safe altitude (starts from zero)
	GCode( "G0 Z%f", ctx.safeToolHeight);
	ctx.currentHeight = ctx.safeToolHeight;

	while (CarveBitmapContour(&ctx));

	// Return to zero, turn OFF the motor
	GCode("G0 Z%f M0", -g_BmParams.tool.safeTravel);

}


BOOL BitmapProcess(HWND hWnd)
{
	BITMAP bm = { 0 };
	HANDLE hBitmap;
	int iX, iY;
	int toolRadiusInPixels;
	unsigned long maxToolCnt, toolPtCnt;
	t2DintPoint *tool;
	unsigned long maxEdgeCnt, edgePtCnt;
	t2DintPoint *edge;
	t2DPoint curPos;
	t2DPoint contactPos;
	t2DPoint V;
	t2DPoint C;
	double Xres, Yres, a, step, dive, temp;
	int iContact;
	char cmd[MAX_STR];
	BOOL bCarving, bDone;
	tFillState fillState;

	//CarveBitmapContour();
	//return TRUE;

	hBitmap = LoadImage(
		NULL,
		g_BmParams.szFilePath,
		IMAGE_BITMAP,
		0, 0,
		LR_CREATEDIBSECTION | LR_LOADFROMFILE | LR_DEFAULTSIZE | LR_MONOCHROME);

	if (hBitmap != NULL)
	{
		// Get the color depth of the DIBSection
		GetObject(hBitmap, sizeof(BITMAP), &bm);

		// Calculate the size of one pixel
		Xres = g_BmParams.width / bm.bmWidth;
		Yres = g_BmParams.height / bm.bmHeight;
	}
	else
	{
		Xres = 1.0;
		Yres = 1.0;
	}

	// Build the array of points that compose the tool and the edge of the tool.
	toolRadiusInPixels = (int)(g_BmParams.tool.radius / ((Xres + Yres) / 2));

	// Estimate the # of points in the tool is the surface of the circle
	maxToolCnt = (int)(PI * (toolRadiusInPixels + 4) * (toolRadiusInPixels + 4));
	tool = (t2DintPoint*)malloc(maxToolCnt * sizeof(t2DintPoint));

	// Estimate that the # of points in the edge twice the length of the circle
	maxEdgeCnt = (int)(4 * PI * (toolRadiusInPixels + 2));
	edge = (t2DintPoint*)malloc(maxEdgeCnt * sizeof(t2DintPoint));

	dive = g_BmParams.tool.safeTravel + g_BmParams.depth;

	toolPtCnt = 0;
	edgePtCnt = 0;
	for ( iX = 0; iX <= toolRadiusInPixels+2; iX++)
	for ( iY = 0; iY <= toolRadiusInPixels+2; iY++)
	{
		double d = sqrt((iX*iX) + (iY*iY));
		// Points which are within the tool
		if ( d <= toolRadiusInPixels)
		{
			AddPoint(iX, iY, tool, &toolPtCnt, maxToolCnt);
		}
		// Points which are within a ring around the tool. Make
		// this 2 pixels thick so that shapes must cross this area
		// before touching the tool itself
		else if( d < (toolRadiusInPixels + 2))
		{
			AddPoint(iX, iY, edge, &edgePtCnt, maxEdgeCnt);
		}
	}

	// -----------------------------
	// START OF BITMAP SHAPE CARVING
	// -----------------------------

	sprintf_s(cmd, MAX_STR, "G91 F%d %s\r\n", 
		g_BmParams.tool.cutSpeed, 
		g_BmParams.tool.motorControl ? "M3 G4 P1" : "");
	doGcode(cmd);

	// Travel at safe altitude (starts from zero)
	sprintf_s(cmd, MAX_STR, "G0 Z%f\r\n", g_BmParams.tool.safeTravel);
	doGcode(cmd);
	bCarving = FALSE;

	if (g_BmParams.contourOrCarve == modeMatrix ||
		g_BmParams.contourOrCarve == modeHoneycomb )
	{
		t2DPoint realPos;
		ULONG savedCount = -1;
		ULONG count = 0;			
		realPos = { 0.0, 0.0 };
		int rowOffset = 0;

		// Move to first location to be tested
		curPos.x = g_BmParams.matrixXoffset;
		curPos.y = g_BmParams.matrixYoffset;
		C.x = g_BmParams.matrixPitch;
		if (g_BmParams.contourOrCarve == modeHoneycomb)
		{
			// Height of the equilateral triangle of side 'matrixPitch'
			C.y = (sqrt(3.0) * g_BmParams.matrixPitch ) / 2.0;
		}
		else
		{
			C.y = g_BmParams.matrixPitch;
		}

		bDone = FALSE;
		while (!bDone)
		{
			// Have we reached an edge ?
			if (curPos.x + C.x > g_BmParams.width)
			{
				if (curPos.y + C.y > g_BmParams.height)
				{
					// sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", -curPos.x, -curPos.y);
					// doGcode(cmd);
					bDone = TRUE;
					break;
				}
				// sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", -curPos.x, C.y);
				curPos.y += C.y;
				curPos.x = g_BmParams.matrixXoffset;

				// In honeycomb mode, offset every other row by half the pitch
				rowOffset++;
				if (g_BmParams.contourOrCarve == modeHoneycomb && (rowOffset & 1))
				{
					curPos.x += (g_BmParams.matrixPitch / 2.0);
				}
				// doGcode(cmd);
			}
			else
			{
				// sprintf_s(cmd, sizeof(cmd), "G0 X%f\r\n", C.x);
				curPos.x += C.x;
				// doGcode(cmd);
			}

			// Convert the current position in pixels
			iX = (int)((curPos.x ) / Xres);
			iY = (int)((curPos.y ) / Yres);

			if (TestToolPosition(&bm, iX, iY, tool, toolPtCnt, edge, edgePtCnt, &a) >= resultToolHalfOverlap )
			{
				sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", curPos.x - realPos.x, curPos.y - realPos.y);
				doGcode(cmd);
				realPos = curPos;

				sprintf_s(cmd, sizeof(cmd), "G0 Z%f\r\n", -g_BmParams.tool.safeTravel);
				doGcode(cmd);

				double z = 0.0f;
				double dz = g_BmParams.tool.radius * 2;
				while (1)
				{
					if ((z + dz) > g_BmParams.depth) dz = g_BmParams.depth - z;
					z += dz;
					sprintf_s(cmd, sizeof(cmd), "G1 Z%f\r\n", -dz);
					doGcode(cmd);

					if (z < g_BmParams.depth)
					{
						sprintf_s(cmd, sizeof(cmd), "G0 Z%f\r\n", z);
						doGcode(cmd);
						sprintf_s(cmd, sizeof(cmd), "G0 Z%f\r\n", -z);
						doGcode(cmd);
					}
					else
					{
						break;
					}
				}
				sprintf_s(cmd, sizeof(cmd), "G0 Z%f\r\n", z + g_BmParams.tool.safeTravel);
				doGcode(cmd);
				update3DView();
			}
		}

		// Return to origin
		sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", -realPos.x, -realPos.y);
		doGcode(cmd);
	}

#if 1
	/*   OLD WAY    */

	if (g_BmParams.contourOrCarve == modeCenterOnly ||
		g_BmParams.contourOrCarve == modeContourAndCenter )
	{
		//--------------------------------------------------------
		// This first carves the inside of the shape in a series
		// of horizontal and vertical passses.

		// Move to first location to be tested
		curPos.x = g_BmParams.tool.radius;
		curPos.y = g_BmParams.tool.radius;
		sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", curPos.x, curPos.y);
		doGcode(cmd);
		update3DView();

		bDone = FALSE;
		C = { 0.0 , 0.0 };

		// Start horizontally
		V = { Xres, 0.0 };
		fillState = fillRows;

		// Carve in 1/2 tool radius slices minus a small bit to
		// avoid hitting the material dead on (not sure if it does
		// anything but it won't hurt).
		step = g_BmParams.tool.radius - SMALL_OVELAP;

		while (!bDone)
		{
			// Have we reached an edge ?
			if ((curPos.x > (g_BmParams.width - g_BmParams.tool.radius)) ||
				(curPos.y > (g_BmParams.height - g_BmParams.tool.radius)))
			{
				// Back off
				C.x -= V.x;
				C.y -= V.y;
				curPos.x -= V.x;
				curPos.y -= V.y;

				if (bCarving)
				{
					if (C.x != 0.0 || C.y != 0.0)
					{
						sprintf_s(cmd, sizeof(cmd), "G1 X%f Y%f\r\n", C.x, C.y);
						doGcode(cmd);
						C = { 0.0,0.0 };
					}
					sprintf_s(cmd, MAX_STR, "G0 Z%f\r\n", dive);
					doGcode(cmd);
					bCarving = FALSE;
				}
				else
				{
					if (C.x != 0.0 || C.y != 0.0)
					{
						sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", C.x, C.y);
						doGcode(cmd);
						C = { 0.0,0.0 };
					}
				}

				switch (fillState)
				{
				case fillRows:
					if (curPos.y + step > g_BmParams.height - g_BmParams.tool.radius)
					{
						temp = g_BmParams.height - g_BmParams.tool.radius - curPos.y;
						fillState = topRow;
					}
					else temp = step;

					sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n",
						g_BmParams.tool.radius - curPos.x,
						temp);
					doGcode(cmd);

					curPos.x = g_BmParams.tool.radius;
					curPos.y += temp;

					update3DView();
					break;

				case fillColumns:
					// Either we've reached the right side column or we don't want
					// to carve vertically. Going straigh here will take the tool to
					// the right most column to clean the edge.
					if ((curPos.x + step >= g_BmParams.width - g_BmParams.tool.radius) ||
						(g_BmParams.bHorizontalCarveOnly))
					{
						temp = g_BmParams.width - g_BmParams.tool.radius - curPos.x;
						fillState = rightColumn;
					}
					else temp = step;

					sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n",
						temp,
						g_BmParams.tool.radius - curPos.y);
					doGcode(cmd);

					curPos.x += temp;
					curPos.y = g_BmParams.tool.radius;

					update3DView();
					break;

				case topRow:
				case rightColumn:
					// Bring the tool back to the origin...
					if (bCarving)
					{
						// ... at a safe altitude if needed
						sprintf_s(cmd, MAX_STR, "G0 Z%f\r\n", dive);
						doGcode(cmd);
						bCarving = FALSE;
					}
					sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", -curPos.x, -curPos.y);
					doGcode(cmd);

					if (fillState == topRow)
					{
						fillState = fillColumns;
						curPos.x = g_BmParams.tool.radius;
						curPos.y = g_BmParams.tool.radius;
						V = { 0.0 , Yres };
						sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", curPos.x, curPos.y);
						doGcode(cmd);
					}
					else
					{
						bDone = TRUE;
						continue;
					}

					update3DView();
					break;
				}
			}

			// Convert the current position in pixels
			iX = (int)(curPos.x / Xres);
			iY = (int)(curPos.y / Yres);

			switch (TestToolPosition(&bm, iX, iY, tool, toolPtCnt, edge, edgePtCnt, &a))
			{
			case resultNoOverlap:
				if (!bCarving)
				{
					if (C.x != 0.0 || C.y != 0.0)
					{
						sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", C.x, C.y);
						doGcode(cmd);
						C = { 0.0, 0.0 };
					}
					sprintf_s(cmd, MAX_STR, "G1 Z%f\r\n", -dive);
					doGcode(cmd);
					bCarving = TRUE;

					update3DView();
				}
				C.x += V.x;
				C.y += V.y;
				curPos.x += V.x;
				curPos.y += V.y;
				break;

			case resultEdgeContact:
			case resultToolPartialOverlap:
			case resultToolHalfOverlap:
			case resultToolFullOverlap:
				if (bCarving)
				{
					// Back off to avoid denting the edges of the shape
					C.x -= V.x;
					C.y -= V.y;
					curPos.x -= V.x;
					curPos.y -= V.y;
					// Execute the commited move
					if (C.x != 0.0 || C.y != 0.0)
					{
						sprintf_s(cmd, sizeof(cmd), "G1 X%f Y%f\r\n", C.x, C.y);
						doGcode(cmd);
						C = { 0.0, 0.0 };
					}
					// Rise the tool to stop carving
					sprintf_s(cmd, MAX_STR, "G0 Z%f\r\n", dive);
					doGcode(cmd);
					bCarving = FALSE;

					update3DView();
				}
				C.x += V.x;
				C.y += V.y;
				curPos.x += V.x;
				curPos.y += V.y;
				break;
			}
		}
	}

#endif

	//--------------------------------------------------------
	// Next follow contour of the surface to smoothen the
	// edges. Surface can be either convexe of concave.
	// Do this multiple times for each contiguous shape.
	if (g_BmParams.contourOrCarve == modeContourOnly ||
		g_BmParams.contourOrCarve == modeContourAndCenter )
	{
		double outerMostMove = g_BmParams.tool.radius;
		contactPos.x = 0;
		contactPos.y = 0;

		do
		{

			// First, look for a contact point with the edge of the
			// surface by scanning the surface in horizontal passes.
			// It doesn't really matter where this starts.
			iContact = 0;
			curPos.x = outerMostMove;
			curPos.y = g_BmParams.tool.radius;
			bDone = FALSE;

			// Set the distance for each movement. Do no go less than res * 2 or the contour
			// algorithm may get stuck in an infinite loop. Res is the size of one pixel.
			step = Xres * 5;
			// Start by scanning horizontally
			V = { step, 0.0 };

			while (!bDone)
			{
				// Test the boundaries
				if (curPos.x > (g_BmParams.width - g_BmParams.tool.radius))
				{
					curPos.x = outerMostMove;
					curPos.y += g_BmParams.tool.radius * 2 - SMALL_OVELAP;
					if (curPos.y > g_BmParams.height - g_BmParams.tool.radius)
					{
						// Bring the tool back to the origin
						sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", -contactPos.x, -contactPos.y);
						doGcode(cmd);
						break;
					}
				}

				// Convert the current position in pixels
				iX = (int)(curPos.x / Xres);
				iY = (int)(curPos.y / Yres);

				switch (TestToolPosition(&bm, iX, iY, tool, toolPtCnt, edge, edgePtCnt, &a))
				{
				case resultEdgeContact:
					// Is it the first edge contact of the tool with the surface?
					if (iContact++ == 0)
					{
						// Bring the tool where the first contact point is from it's previous
						// location (saved in contactPos)
						sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", curPos.x - contactPos.x, curPos.y - contactPos.y );
						doGcode(cmd);
						// Lower the tool to start carving
						sprintf_s(cmd, "G1 Z%f\r\n", -dive);
						doGcode(cmd);

						// Save the position of the contact point so that we can
						// determine when the tool as returned.
						contactPos = curPos;
					}
					else
					{
						// Make the move that follows the tangeant direction
						// to the average direction (angle 'a')
						sprintf_s(cmd, sizeof(cmd), "G1 X%f Y%f\r\n", V.x, V.y);
						doGcode(cmd);
					}

					// Don't check immediately if we're back at the first contact
					// point because if we backoff, the test will succeed immediately.
					if (iContact > 2)
					{
						if (distance2D(curPos, contactPos) < step)
						{
							// We're done. First, go back to safe altitude
							sprintf_s(cmd, "G0 Z%f\r\n", dive);
							doGcode(cmd);

							contactPos = curPos;
							bDone = TRUE;
							continue;
						}
					}

					// The direction to follow the contour is 90 degrees
					// from the direction of the average contact points
					a += (PI / 2.0);
					V.x = cos(a) * step;
					V.y = sin(a) * step;
					curPos.x += V.x;
					curPos.y += V.y;

					update3DView();
					break;

				case resultToolPartialOverlap:
				case resultToolHalfOverlap:
				case resultToolFullOverlap:
					if (iContact)
					{
						// Back off
						curPos.x -= V.x;
						curPos.y -= V.y;
						// Try with a different angle
						a += SMALL_ANGLE;
						V.x = cos(a) * step;
						V.y = sin(a) * step;
						curPos.x += V.x;
						curPos.y += V.y;
					}
					else
					{
						// Still looking for the first contact
						curPos.x += V.x;
						curPos.y += V.y;
					}
					break;

				case resultNoOverlap:
					if (iContact == 0)
					{
						// Still looking for the first contact
						curPos.x += V.x;
						curPos.y += V.y;
					}
					else
					{
						// Back off
						curPos.x -= V.x;
						curPos.y -= V.y;
						// Try with a different angle
						a -= SMALL_ANGLE;
						V.x = cos(a) * step;
						V.y = sin(a) * step;
						curPos.x += V.x;
						curPos.y += V.y;
					}
					break;
				}

				if (iContact)
				{
					// Update the furtest X position following the contour took us
					if (curPos.x > outerMostMove) outerMostMove = curPos.x;
				}
			}

			// We found one surface in this pass. Remove it from the
			// bitmap and go through again to find in the contour 
			// of the next surface in the image.
			if (iContact)
			{
				/*
				DWORD dwThreadId;
				tCleanBmInfo cleanBitmapInfo;
				HANDLE hThread;

				cleanBitmapInfo.bm = &bm;
				cleanBitmapInfo.x = (int)(contactPos.x / res);
				cleanBitmapInfo.y = (int)(contactPos.y / res);

				hThread = CreateThread(
					NULL, 
					// 100MB thread stack size to handle the recusive flood
					// fill algorithm for a 600 x 600 pixels bitmap. Gulp.
					// Looks like this could use an optimization.
					// https://en.wikipedia.org/wiki/Flood_fill
					//
					1024 * 1024 * 60,
					(LPTHREAD_START_ROUTINE)CleanBitmapThread, 
					(PVOID)&cleanBitmapInfo, 
					0, 
					&dwThreadId);

				WaitForSingleObject(hThread, INFINITE);
				CloseHandle(hThread);
				*/
				//iX = (int)(contactPos.x / Xres);
				//iY = (int)(contactPos.y / Yres);
				//CleanBitmap(&bm, iX, iY);
			}

		} while (iContact);
	}

#if 0

	if (g_BmParams.contourOrCarve == modeCenterOnly ||
		g_BmParams.contourOrCarve == modeContourAndCenter)
	{
		// Move to first location to be tested
		curPos.x = g_BmParams.tool.radius;
		curPos.y = g_BmParams.tool.radius;
		sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", curPos.x, curPos.y);
		doGcode(cmd);
		update3DView();

		bDone = FALSE;
		bCarving = FALSE;
		C = { 0.0 , 0.0 };

		// Start horizontally
		V = { Xres, 0.0 };
		fillState = fillRows;

		// Carve in 1/2 tool radius slices minus a small bit to
		// avoid hitting the material dead on (not sure if it does
		// anything but it won't hurt).
		step = g_BmParams.tool.radius - SMALL_OVELAP;

		while (!bDone)
		{
			// Have we reached the right side edge ?
			if ( curPos.x > (g_BmParams.width - g_BmParams.tool.radius))
			{
				// Back off to stay within the boundary
				C.x -= V.x;
				C.y -= V.y;
				curPos.x -= V.x;
				curPos.y -= V.y;

				if (bCarving)
				{
					if (C.x != 0.0 || C.y != 0.0)
					{
						sprintf_s(cmd, sizeof(cmd), "G1 X%f Y%f\r\n", C.x, C.y);
						doGcode(cmd);
						C = { 0.0,0.0 };
					}

					// Move up to the next row
					V = { 0.0, step };
					curPos.x += V.x;
					curPos.y += V.y;

					// Have we reached the top edge ?
					if (curPos.y > (g_BmParams.height - g_BmParams.tool.radius))
					{
						// Back off to stay within the boundary
						C.x -= V.x;
						C.y -= V.y;
						curPos.x -= V.x;
						curPos.y -= V.y;
					}




					sprintf_s(cmd, MAX_STR, "G0 Z%f\r\n", dive);
					doGcode(cmd);
					bCarving = FALSE;
				}
				else
				{
					if (C.x != 0.0 || C.y != 0.0)
					{
						sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", C.x, C.y);
						doGcode(cmd);
						C = { 0.0,0.0 };
					}
				}

				switch (fillState)
				{
				case fillRows:
					if (curPos.y + step > g_BmParams.height - g_BmParams.tool.radius)
					{
						temp = g_BmParams.height - g_BmParams.tool.radius - curPos.y;
						fillState = topRow;
					}
					else temp = step;

					sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n",
						g_BmParams.tool.radius - curPos.x,
						temp);
					doGcode(cmd);

					curPos.x = g_BmParams.tool.radius;
					curPos.y += temp;

					update3DView();
					break;

				case fillColumns:
					// Either we've reached the right side column or we don't want
					// to carve vertically. Going straigh here will take the tool to
					// the right most column to clean the edge.
					if ((curPos.x + step >= g_BmParams.width - g_BmParams.tool.radius) ||
						(g_BmParams.bHorizontalCarveOnly))
					{
						temp = g_BmParams.width - g_BmParams.tool.radius - curPos.x;
						fillState = rightColumn;
					}
					else temp = step;

					sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n",
						temp,
						g_BmParams.tool.radius - curPos.y);
					doGcode(cmd);

					curPos.x += temp;
					curPos.y = g_BmParams.tool.radius;

					update3DView();
					break;

				case topRow:
				case rightColumn:
					// Bring the tool back to the origin...
					if (bCarving)
					{
						// ... at a safe altitude if needed
						sprintf_s(cmd, MAX_STR, "G0 Z%f\r\n", dive);
						doGcode(cmd);
						bCarving = FALSE;
					}
					sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", -curPos.x, -curPos.y);
					doGcode(cmd);

					if (fillState == topRow)
					{
						fillState = fillColumns;
						curPos.x = g_BmParams.tool.radius;
						curPos.y = g_BmParams.tool.radius;
						V = { 0.0 , Yres };
						sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", curPos.x, curPos.y);
						doGcode(cmd);
					}
					else
					{
						bDone = TRUE;
						continue;
					}

					update3DView();
					break;
				}
			}

			// Convert the current position in pixels
			iX = (int)(curPos.x / Xres);
			iY = (int)(curPos.y / Yres);

			switch (TestToolPosition(&bm, iX, iY, tool, toolPtCnt, edge, edgePtCnt, &a))
			{
			case resultNoOverlap:
				if (!bCarving)
				{
					if (C.x != 0.0 || C.y != 0.0)
					{
						sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", C.x, C.y);
						doGcode(cmd);
						C = { 0.0, 0.0 };
					}
					sprintf_s(cmd, MAX_STR, "G1 Z%f\r\n", -dive);
					doGcode(cmd);
					bCarving = TRUE;

					update3DView();
				}
				C.x += V.x;
				C.y += V.y;
				curPos.x += V.x;
				curPos.y += V.y;
				break;

			case resultEdgeContact:
			case resultToolPartialOverlap:
			case resultToolHalfOverlap:
			case resultToolFullOverlap:
				if (bCarving)
				{
					// Back off to avoid denting the edges of the shape
					C.x -= V.x;
					C.y -= V.y;
					curPos.x -= V.x;
					curPos.y -= V.y;
					// Execute the commited move
					if (C.x != 0.0 || C.y != 0.0)
					{
						sprintf_s(cmd, sizeof(cmd), "G1 X%f Y%f\r\n", C.x, C.y);
						doGcode(cmd);
						C = { 0.0, 0.0 };
					}
					// Rise the tool to stop carving
					sprintf_s(cmd, MAX_STR, "G0 Z%f\r\n", dive);
					doGcode(cmd);
					bCarving = FALSE;

					update3DView();
				}
				C.x += V.x;
				C.y += V.y;
				curPos.x += V.x;
				curPos.y += V.y;
				break;
			}
		}
	}

#endif



	// Return to zero altitude, turn OFF spindle (if needed)
	sprintf_s(cmd, "G1 Z%f %s\r\n", 
		-g_BmParams.tool.safeTravel, 
		g_BmParams.tool.motorControl ? "M0" : "" );
	doGcode(cmd);

	update3DView();
	
	if( hBitmap != NULL ) DeleteObject(hBitmap);
	free(tool);
	free(edge);

	return TRUE;
}


WNDPROC g_oldBitmapDlgdProc = NULL;
BOOL CALLBACK BitmapInterceptWndProc(HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam)
{
	if (message == WM_KEYDOWN)
	{
		if (wParam == 'a' || wParam == 'A' && GetKeyState(VK_CONTROL))
		{
			PostMessage(hWnd, EM_SETSEL, 0, -1);
			return MNC_CLOSE << 16;
		}
	}
	return g_oldBitmapDlgdProc(hWnd, message, wParam, lParam);
}

void Carve(HWND hWnd)
{
	BitmapShapeGetSet(TRUE, hWnd);
	BitmapProcess(hWnd);
}

void Reset3dView(HWND hWnd)
{
	BitmapShapeGetSet(TRUE, hWnd);
	init3DView(g_BmParams.width, g_BmParams.height);
	resetBlockSurface();
}

void ResetAndRefeshIfAutoIsChecked(HWND hWnd)
{
	HWND hItem;
	BitmapShapeGetSet(FALSE, hWnd);
	hItem = GetDlgItem(hWnd, IDC_AUTO_REFRESH);
	if (Button_GetCheck(hItem))
	{
		Reset3dView(hWnd);
		Carve(hWnd);
	}
}

BOOL CALLBACK BitmapShapesProc(HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam)
{
	HWND hDlg;

	switch (message)
	{
	case WM_INITDIALOG:
		BitmapShapeInit(hWnd);
		BitmapShapeGetSet(FALSE, hWnd);

		// This is to capture the CTRL+A on the GCode edit box
		hDlg = GetDlgItem(hWnd, IDC_GCODE);
		g_oldBitmapDlgdProc = (WNDPROC)GetWindowLong(hDlg, GWL_WNDPROC);
		SetWindowLong(hDlg, GWL_WNDPROC, (LONG)BitmapInterceptWndProc);
		return TRUE;
		break;

	case WM_CLOSE:
		BitmapShapeGetSet(TRUE, hWnd);
		BitmapShapeSave();
		break;

	case WM_COMMAND:
		if (HIWORD(wParam) == BN_CLICKED)
		{
			switch (LOWORD(wParam))
			{
			case IDC_CARVE:
				Carve(hWnd);
				break;
/*
			case IDC_EXECUTE:
				BitmapShapeExecute(hWnd);
				break;
*/
			case IDC_RESET_SIM2:
				Reset3dView(hWnd);
				break;

			case IDC_OFFSET_UP:
				g_BmParams.matrixYoffset += g_Yres;
				ResetAndRefeshIfAutoIsChecked(hWnd);
				break;

			case IDC_OFFSET_DOWN:
				g_BmParams.matrixYoffset -= g_Yres;
				ResetAndRefeshIfAutoIsChecked(hWnd);
				break;

			case IDC_OFFSET_LEFT:
				g_BmParams.matrixXoffset -= g_Xres;
				ResetAndRefeshIfAutoIsChecked(hWnd);
				break;

			case IDC_OFFSET_RIGHT:
				g_BmParams.matrixXoffset += g_Xres;
				ResetAndRefeshIfAutoIsChecked(hWnd);
				break;

			case IDOK:
			case IDCANCEL:
				EndDialog(hWnd, wParam);
				return TRUE;
			}
		}
	}
	return FALSE;
}

void BitmapShapes(HWND hWnd)
{
	DialogBox(NULL,
		MAKEINTRESOURCE(IDD_BITMAP_SHAPES),
		hWnd,
		(DLGPROC)BitmapShapesProc);
}