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
	modeMatrix
} tCarveMode;

typedef struct
{
	tGeneralToolInfo tool;

	WCHAR szFilePath[MAX_PATH];

	float width;
	float height;
	float depth;
	tCarveMode contourOrCarve;
	int bHorizontalCarveOnly;
	float matrixPitch;

} tBitmapShapeParam;

typedef struct
{
	BITMAP* bm;
	int x;
	int y;
} tCleanBmInfo;

tBitmapShapeParam g_BmParams;

void BitmapShapeInit(HWND hWnd)
{
	int i;
	HWND hItem;

	hItem = GetDlgItem(hWnd, IDC_TOOL_SIZE);
	for (i = 0; i < ITEM_CNT(TOOL_SIZES); i++) ComboBox_AddString(hItem, TOOL_SIZES[i].str);

	hItem = GetDlgItem(hWnd, IDC_CUT_SPEED);
	for (i = 0; i < ITEM_CNT(CUT_SPEED); i++) ComboBox_AddString(hItem, CUT_SPEED[i].str);

	HKEY hKey;
	if (RegOpenKey(HKEY_CURRENT_USER, L"SOFTWARE", &hKey) == ERROR_SUCCESS)
	{
		DWORD cbData = sizeof(g_BmParams);
		if (RegGetValue(hKey, L"WinCNC", L"BitmapShape", RRF_RT_REG_BINARY, NULL, &g_BmParams, &cbData) != ERROR_SUCCESS)
		{
			g_BmParams.width = 3.25;
			g_BmParams.height = 3.25;
			//g_BmParams.X = 2;
			//g_BmParams.Y = 2;
		}
		RegCloseKey(hKey);
	}

	ShapeInitToolInfo(&g_BmParams.tool);
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
	ShapeGetSetRadio(hWnd, IDC_BITMAP_CONTOUR_CARVE, 4, get, &tmp );
	g_BmParams.contourOrCarve = (tCarveMode)tmp;

	ShapeGetSetFloat(hWnd, IDC_BITMAP_WIDTH, get, &g_BmParams.width);
	ShapeGetSetFloat(hWnd, IDC_BITMAP_HEIGHT, get, &g_BmParams.height);
	ShapeGetSetFloat(hWnd, IDC_BITMAP_DEPTH, get, &g_BmParams.depth);
	ShapeGetSetFloat(hWnd, IDC_BITMAP_MATRIX_PITCH, get, &g_BmParams.matrixPitch);

	ShapeGetSetBool(hWnd, IDC_BITMAP_CARVE_HORIZONTAL_ONLY, get, &g_BmParams.bHorizontalCarveOnly);

	return 0;
}

#if 0

#define RAW_SAMPLES_COUNT	256

// This array stores the collected samples from the ADC
uint16_t raw_samples[RAW_SAMPLES_COUNT]; // The values are from 0 to 4096
						   /*
						   * This function processes the raw samples stored in raw_samples,
						   * and recovers the phase delay (theta).
						   *
						   * When the function is called, we assume that raw_samples
						   * already has the most up-to-date samples from the ADC.
						   */
float getPhaseDelayFromRawSamples() {
	uin16_t i;
	uint32_t sum;
	float theta;

	// Calculate the sum of all the raw samples
	sum = 0;
	for (i = 0; i < RAW_SAMPLES_COUNT; i++) {
		sum = sum + raw_samples[i];
	}
	// Convert the sum to average and phase angle
	theta = (float)sum * (360.0f / (RAW_SAMPLES_COUNT * 4096.0f));
	
	return theta; // theta is between 0 and 359.9
}



#define USING_TMP36

// Assume Arduino AVR APIs
#define HEATER_GPIO          10
#define HEATER_ON			 HIGH
#define HEATER_OFF			 LOW
#define COOLING_FAN_GPIO     11
#define COOLING_FAN_ON		 HIGH
#define COOLING_FAN_OFF		 LOW

#define COOLING_FAN_ON_TEMP	 50		// Turns fan ON above this
#define HEATER_ON_TEMP		 5		// Turns heater ON below this
#define TEMP_HYSTEREIS		 1		// Can be zero. Can't exceed FAN_TEMP-HEATER_TEMP.

// This variable stores latest value from temperature sensor sampled by ADC
volatile uint16_t raw_adc_sample;
/*
* This function processes the raw ADC data returns the current
* temperature.
* When the function is called, we assume that raw_adc_sample
* already has the latest sample from the ADC.
*/
uint16_t getTemperatureFromRawSample()
{
	uint16_t degC;
	uint32_t voltage; // in mV

	// Converts ADC reading into voltage in mV
	// Assumes that filtering is not needed.
	voltage = raw_adc_sample * 625 / 128;  // (5000 / 1024)

#ifdef USING_TMP36
    // Using TMP36 which has range of -40 / +125
	// 10mV/C and 750mV at 25C.
	if (voltage >= 500) {
		degC = (voltage - 500 ) / 10;
	}
	else {
		// Below freezing always return zero
		degC = 0;
	}
#endif

	return degC; // degC is between 0 to 100
}
/*
* This function turns ON/OFF devices based on temperature.
* as returned by getTemperatureFrom RawSample(). When calling
* this function, we assume that that GPIO for controlling
* the devices have been initialized as outpout.
* This implementation also assumes that digitalWrite( ) is
* efficient. If not, should be changed to only write the
* GPIO when the temperature thresholds are crossed.
*/
void controlDevices()
{
	uint16_t curTemp = getTemperatureFromRawSample( );

	// Temperature is too cold, turn on heater
	if (curTemp < HEATER_ON_TEMP )
	{
		digitalWrite(HEATER_GPIO, HEATER_ON);
	}
	// Temperature is warm enough, turn off heater
	if (curTemp > (HEATER_ON_TEMP + TEMP_HYSTEREIS))
	{
		digitalWrite(HEATER_GPIO, HEATER_OFF);
	}

	// Temperature is too hot, turn on the fan
	if (curTemp > COOLING_FAN_ON_TEMP)
	{
		digitalWrite(COOLING_FAN_GPIO, COOLING_FAN_ON );
	}
	// Temperature is cool enough, turn off the fan
	if (curTemp < (COOLING_FAN_ON_TEMP - TEMP_HYSTEREIS))
	{
		digitalWrite(COOLING_FAN_GPIO, COOLING_FAN_OFF);
	}
}

#endif


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
	resultToolOverlap = 1,
	resultEdgeContact = 2,
} toolPosResult_t;

#define SMALL_OVELAP			(1/64.0f)
#define SMALLEST_RADIUS			( g_BmParams.tool.radius + SMALL_OVELAP )
#define SMALL_ANGLE				(PI/180)		// One degree

#define GET_BYTE( pt, width, x, y ) ((unsigned char*)pt + (y * width) + (x / 8));
#define GET_MASK( x ) (0x80 >> (x % 8))

BOOL GetPixel(BITMAP* bm, int x, int y)
{
	unsigned char* pt;
	// Test if the point is within the bitmap
	if (x < 0 || y < 0 || x >= bm->bmWidth || y >= bm->bmHeight) return FALSE;
	// Move pointer to byte that contains this pixel
	//pt = (unsigned char*)bm->bmBits + (y * bm->bmWidthBytes) + (x / 8);
	pt = GET_BYTE(bm->bmBits, bm->bmWidthBytes, x, y);
	// Mask pixel in the byte
	return (*pt & GET_MASK( x )) == 0;
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


void AddPoint(int x, int y, t2DintPoint* list, int* count, int max)
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
	float sX = 0.0;
	float sY = 0.0;
	int tCount = 0;

	for (int i = 0; i < nTool; i++)
	{
		if (GetPixel(bm, x + pTool[i].x, y + pTool[i].y)) return resultToolOverlap;
	}
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

BOOL BitmapProcess(HWND hWnd)
{
	BITMAP  bm;
	HANDLE hBitmap;
	int iX, iY;
	int toolRadiusInPixels;
	int maxToolCnt, toolPtCnt;
	t2DintPoint *tool;
	int maxEdgeCnt, edgePtCnt;
	t2DintPoint *edge;
	t2DPoint curPos;
	t2DPoint contactPos;
	t2DPoint V;
	t2DPoint C;
	double res, a, step, dive;
	int iContact;
	char cmd[MAX_STR];
	BOOL bCarving, bDone;
	tFillState fillState;

	hBitmap = LoadImage(
		NULL,
		g_BmParams.szFilePath,
		IMAGE_BITMAP,
		0, 0,
		LR_CREATEDIBSECTION | LR_LOADFROMFILE | LR_DEFAULTSIZE | LR_MONOCHROME);

	// Get the color depth of the DIBSection
	GetObject(hBitmap, sizeof(BITMAP), &bm);

	// Calculate the size of one pixel
	res = g_BmParams.width / bm.bmWidth;

	// Build the array of points that compose the tool and the edge of the tool.
	toolRadiusInPixels = (int)(g_BmParams.tool.radius / res);

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

	if (g_BmParams.contourOrCarve == modeMatrix )
	{
		t2DPoint realPos = { 0.0, 0.0 };
		// Move to first location to be tested
		curPos.x = 0;
		curPos.y = 0;
		C.x = g_BmParams.matrixPitch;
		C.y = g_BmParams.matrixPitch;
		
		bDone = FALSE;
		while (!bDone)
		{
			// Have we reached an edge ?
			if (curPos.x + C.x > g_BmParams.width)
			{
				if (curPos.y + C.y > g_BmParams.height)
				{
//					sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", -curPos.x, -curPos.y);
//					doGcode(cmd);
					bDone = TRUE;
					break;
				}
//				sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", -curPos.x, C.y);
				curPos.y += C.y;
				curPos.x = 0.0f;
//				doGcode(cmd);
			}
			else
			{
//				sprintf_s(cmd, sizeof(cmd), "G0 X%f\r\n", C.x);
				curPos.x += C.x;
//				doGcode(cmd);
			}

			// Convert the current position in pixels
			iX = (int)(curPos.x / res);
			iY = (int)(curPos.y / res);

			if (TestToolPosition(&bm, iX, iY, tool, toolPtCnt, edge, edgePtCnt, &a) == resultToolOverlap)
			{
				sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", curPos.x - realPos.x, curPos.y - realPos.y );
				doGcode(cmd);
				realPos = curPos;

				sprintf_s(cmd, sizeof(cmd), "G0 Z%f\r\n", -g_BmParams.tool.safeTravel );
				doGcode(cmd);

				float z = 0.0f;
				float dz = 1.0; // 0.18; // g_BmParams.tool.radius * 4;
				while( 1 )
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
			}

			update3DView();
		}

		// Return to origin
		sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", -realPos.x, -realPos.y);
		doGcode(cmd);
	}

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
		V = { res, 0.0 };
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
						step = g_BmParams.height - g_BmParams.tool.radius - curPos.y;
						fillState = topRow;
					}

					sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n",
						g_BmParams.tool.radius - curPos.x,
						step);
					doGcode(cmd);

					curPos.x = g_BmParams.tool.radius;
					curPos.y += step;

					update3DView();
					break;

				case fillColumns:
					// Either we've reached the right side column or we don't want
					// to carve vertically. Going straigh here will take the tool to
					// the right most column to clean the edge.
					if ((curPos.x + step >= g_BmParams.width - g_BmParams.tool.radius) ||
						(g_BmParams.bHorizontalCarveOnly))
					{
						step = g_BmParams.width - g_BmParams.tool.radius - curPos.x;
						fillState = rightColumn;
					}

					sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n",
						step,
						g_BmParams.tool.radius - curPos.y);
					doGcode(cmd);

					curPos.x += step;
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
						V = { 0.0 , res };
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
			iX = (int)(curPos.x / res);
			iY = (int)(curPos.y / res);

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
			case resultToolOverlap:
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

	if (g_BmParams.contourOrCarve == modeContourOnly ||
		g_BmParams.contourOrCarve == modeContourAndCenter )
	{
		//--------------------------------------------------------
		// Next follow contour of the surface to smoothen the
		// edges. Surface can be either convexe of concave.
		// Do this multiple times for each contiguous shape.
		do
		{

			// First, look for a contact point with the edge of the
			// surface by scanning the surface in horizontal passes.
			// It doesn't really matter where this starts.
			iContact = 0;
			curPos.x = g_BmParams.tool.radius;
			curPos.y = g_BmParams.tool.radius;
			bDone = FALSE;

			// Set the distance for each movement. Do no go less than res * 2 or the contour
			// algorithm may get stuck in an infinite loop. Res is the size of one pixel.
			step = res * 5;
			// Start by scanning horizontally
			V = { step, 0.0 };

			while (!bDone)
			{
				// Test the boundaries
				if (curPos.x > (g_BmParams.width - g_BmParams.tool.radius))
				{
					curPos.x = g_BmParams.tool.radius;
					curPos.y += g_BmParams.tool.radius * 2 - SMALL_OVELAP;
					if (curPos.y > g_BmParams.height - g_BmParams.tool.radius)
					{
						// Done. Found no contact... no shape. Weird
						break;
					}
				}

				// Convert the current position in pixels
				iX = (int)(curPos.x / res);
				iY = (int)(curPos.y / res);

				switch (TestToolPosition(&bm, iX, iY, tool, toolPtCnt, edge, edgePtCnt, &a))
				{
				case resultEdgeContact:
					// Is it the first edge contact of the tool with the surface?
					if (iContact++ == 0)
					{
						// Save the position of the contact point so that we can
						// determine when the tool as returned.
						contactPos = curPos;
						// Bring the tool where the first contact point is. Right
						// now it's at 0,0.
						sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", curPos.x, curPos.y);
						doGcode(cmd);
						// Lower the tool to start carving
						sprintf_s(cmd, "G1 Z%f\r\n", -dive);
						doGcode(cmd);
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
							// Bring the tool back to the origin
							sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", -curPos.x, -curPos.y);
							doGcode(cmd);

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

				case resultToolOverlap:
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
				iX = (int)(contactPos.x / res);
				iY = (int)(contactPos.y / res);
				CleanBitmap(&bm, iX, iY);
			}

		} while (iContact);
	}

	// Return to zero altitude, turn OFF spindle (if needed)
	sprintf_s(cmd, "G1 Z%f %s\r\n", 
		-g_BmParams.tool.safeTravel, 
		g_BmParams.tool.motorControl ? "M0" : "" );
	doGcode(cmd);

	update3DView();
	
	DeleteObject(hBitmap);
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
				BitmapShapeGetSet(TRUE, hWnd);
				BitmapProcess(hWnd);
				break;
			case IDC_EXECUTE:
				BitmapShapeExecute(hWnd);
				break;
			case IDC_RESET_SIM2:
				BitmapShapeGetSet(TRUE, hWnd);
				init3DView(g_BmParams.width, g_BmParams.height);
				resetBlockSurface();
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