#include "CNC.h"
#include "Resource.h"
#include "Windowsx.h"
#include "math.h"

#include "gcode.h"
#include "fileParser.h"
#include "Shapes.h"
#include "3Dview.h"

typedef struct
{
	tGeneralToolInfo tool;

	WCHAR szFilePath[MAX_PATH];

	float X;
	float Y;

} tBitmapShapeParam;

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
		if (RegGetValue(hKey, L"WinCNC", L"BitmapShape", RRF_RT_REG_BINARY, NULL, &g_BmParams, &cbData) == ERROR_SUCCESS)
		{
			g_BmParams.X = 3.25;
			g_BmParams.Y = 3.25;
			//g_BmParams.X = 2;
			//g_BmParams.Y = 2;
			return;
		}
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
		ParseBuffer(hWnd, cmd, l, doGcode);
		free(cmd);
	}
}

#define SMALL_OVELAP		(1/64.0f)
#define SMALLEST_RADIUS		( g_BmParams.tool.radius + SMALL_OVELAP )
#define SAFE_TRAVEL_HEIGHT	0.25f


BOOL GetPixel(BITMAP* bm, int x, int y)
{
	unsigned char* pt;
	// Test if the point is within the bitmap
	if (x < 0 || y < 0 || x >= bm->bmWidth || y >= bm->bmHeight) return FALSE;
	// Vertical axis is reversed
	// y = bm->bmHeight - y - 1;
	// Move pointer to byte that contains this pixel
	pt = (unsigned char*)bm->bmBits + (y * bm->bmWidthBytes) + (x / 8);
	// Mask pixel in the byte
	return (*pt & (0x80 >> (x % 8))) == 0;
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

typedef enum {
	resultNoOverlap = 0,
	resultToolOverlap = 1,
	resultEdgeContact = 2,
} toolPosResult_t;

toolPosResult_t TestToolPosition(BITMAP* bm, int x, int y, t2DintPoint* pTool, int nTool, t2DintPoint* pEdge, int nEdge, float* tangeant)
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
		float a;
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

// 1/2 degree angle
#define SMALL_CLOCKWISE				((2.0*PI)/360)
#define SMALL_COUNTER_CLOCKWISE		(-SMALL_CLOCKWISE)

BOOL BitmapProcess(HWND hWnd)
{
	BITMAP  bm;
	HANDLE hBitmap;
	float res;

	hBitmap = LoadImage( 
		NULL, 
		g_BmParams.szFilePath, 
		IMAGE_BITMAP, 
		0, 0, 
		LR_CREATEDIBSECTION | LR_LOADFROMFILE | LR_DEFAULTSIZE | LR_MONOCHROME);

	// Get the color depth of the DIBSection
	GetObject(hBitmap, sizeof(BITMAP), &bm);

	res = g_BmParams.X / bm.bmWidth;

	int iX, iY;
	int toolRadiusInPixels = (int)(g_BmParams.tool.radius / res);

	int toolPtCnt = 0;
	int maxToolCnt = PI * (toolRadiusInPixels + 1) * (toolRadiusInPixels + 1);
	t2DintPoint *tool = (t2DintPoint*)malloc(maxToolCnt * sizeof(t2DintPoint));

	int edgePtCnt = 0;
	int maxEdgeCnt = 4 * PI * (toolRadiusInPixels + 2);
	t2DintPoint *edge = (t2DintPoint*)malloc(maxEdgeCnt * sizeof(t2DintPoint));


	for ( iX = 0; iX <= toolRadiusInPixels+1; iX++)
	for ( iY = 0; iY <= toolRadiusInPixels+1; iY++)
	{
		float d = sqrt((iX*iX) + (iY*iY));
		if ( d <= toolRadiusInPixels)
		{
			AddPoint(iX, iY, tool, &toolPtCnt, maxToolCnt);
		}
		else if( d < (toolRadiusInPixels + 2))
		{
			AddPoint(iX, iY, edge, &edgePtCnt, maxEdgeCnt);
		}
	}

#define SMALL_ANGLE	(PI/180)

	float X, Y;
	float Xc, Yc;
	t2DPoint V;
	float a;
	// Set the distance for each movement. Do no go less than res * 2 or the contour
	// algorithm may get stuck in an infinite loop. Res is the size of one pixel.
	float step = res * 5;
	int iContact;
	char cmd[MAX_STR];
	BOOL bCarving = FALSE;
	BOOL bDone = FALSE;

	X = g_BmParams.tool.radius;
	Y = g_BmParams.tool.radius;
	
	V.x = step;
	V.y = 0.0;

	sprintf_s(cmd, MAX_STR, "G91 F%d %s\r\n", 
		g_BmParams.tool.cutSpeed, 
		g_BmParams.tool.motorControl ? "M3 G4 P1" : "");

	doGcode(cmd);
	doGcode("G0 Z0.4\r\n");
	update3DView();

	//--------------------------------------------------------

	bDone = FALSE;
	while (!bDone)
	{
		if (X > (g_BmParams.X - g_BmParams.tool.radius))
		{
			doGcode("G0 Z0.5\r\n");
			sprintf_s(cmd, sizeof(cmd), "G1 X%f Y%f\r\n", 
				g_BmParams.tool.radius - X, 
				g_BmParams.tool.radius * 2 - SMALL_OVELAP);

			doGcode(cmd);
			bCarving = FALSE;

			X = g_BmParams.tool.radius;
			Y += g_BmParams.tool.radius * 2 - SMALL_OVELAP;

			if (Y > g_BmParams.Y - g_BmParams.tool.radius)
			{
				bDone = TRUE;
				continue;
			}
		}

		iX = X / res;
		iY = Y / res;

		switch (TestToolPosition(&bm, iX, iY, tool, toolPtCnt, edge, edgePtCnt, &a))
		{
		case resultToolOverlap:
			if (bCarving)
			{
				// We ran straight into the shape. Back off amd reduce step by half
				X -= V.x;
				Y -= V.y;
				V.x = V.x / 2;
				V.y = V.y / 2;
			}
			else
			{
				sprintf_s(cmd, sizeof(cmd), "G1 X%f Y%f\r\n", V.x, V.y);
				doGcode(cmd);
				update3DView();
				Sleep(1);

				X += V.x;
				Y += V.y;
			}
			break;

		case resultEdgeContact:
	
			sprintf_s(cmd, sizeof(cmd), "G1 X%f Y%f\r\n", V.x, V.y);
			doGcode(cmd);

			if (bCarving)
			{
				doGcode("G0 Z0.5\r\n");
				bCarving = FALSE;
			}
			update3DView();
			Sleep(1);

			X += V.x;
			Y += V.y;
			break;

		case resultNoOverlap:
			if (!bCarving)
			{
				doGcode("G1 Z-0.5\r\n");
				bCarving = TRUE;
			}
			V.x = res;
			V.y = 0;
			X += V.x;
			Y += V.y;
			sprintf_s(cmd, sizeof(cmd), "G1 X%f Y%f\r\n", V.x, V.y);
			doGcode(cmd);
			update3DView();
			Sleep(1);
			break;
		}
	}

	// Bring the tool back to the origin
	if (bCarving)
	{
		// At a safe altitude if needed
		doGcode("G0 Z0.5\r\n");
		bCarving = FALSE;
	}
	sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", -X, -Y );
	doGcode(cmd);
	Sleep(1);

	//--------------------------------------------------------

	// Now look for the first contact with the edge of the
	// surface starting in the bottom left corner.
	iContact = 0;
	X = g_BmParams.tool.radius;
	Y = g_BmParams.tool.radius;
	bDone = FALSE;

	while( !bDone )
	{
		if (X > (g_BmParams.X - g_BmParams.tool.radius))
		{
			X = g_BmParams.tool.radius;
			Y += g_BmParams.tool.radius * 2 - SMALL_OVELAP;
			if (Y > g_BmParams.Y - g_BmParams.tool.radius)
			{
				// Done. Found no contact... no shape. Weird
				break;
			}
		}

		iX = X / res;
		iY = Y / res;

		switch (TestToolPosition(&bm, iX, iY, tool, toolPtCnt, edge, edgePtCnt, &a))
		{
		case resultEdgeContact:
			
			// Is it the first contact ?
			if (iContact++ == 0)
			{
				// Save the position of the contact point so that
				// we can determine when the tool as returned.
				Xc = X;
				Yc = Y;
				// Bring the tool where the first contact point
				// is. Right now it's at 0,0.
				sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", X, Y );
				doGcode(cmd);
				// Start carving
				doGcode("G1 Z-0.5\r\n");
				Sleep(1);
			}
			else
			{
				// Make the move that follows the tangeant direction
				// to the average direction (angle 'a')
				sprintf_s(cmd, sizeof(cmd), "G1 X%f Y%f\r\n", V.x, V.y);
				doGcode(cmd);
				update3DView();
				Sleep(1);
			}

			// Don't check immediately if we're back at the first contact
			// point because if we backoff, the test will succeed immediately.
			if (iContact > 2)
			{
				if (vectorLength(X - Xc, Y - Yc) < step)
				{
					// We're done. First, go back to safe altitude
					doGcode("G0 Z0.5\r\n");
					// Bring the tool back to the origin
					sprintf_s(cmd, sizeof(cmd), "G0 X%f Y%f\r\n", -X, -Y);
					doGcode(cmd);
					Sleep(1);

					bDone = TRUE;
					continue;
				}
			}

			// The direction to follow the contour is 90 degrees
			// from the direction of the average contact points
			a += (PI / 2.0);
			V.x = cos(a) * step;
			V.y = sin(a) * step;
			X += V.x;
			Y += V.y;			
			break;

		case resultToolOverlap:
			// Back off
			X -= V.x;
			Y -= V.y;
			if (iContact)
			{
				// Try with a different angle
				a += SMALL_ANGLE;
				V.x = cos(a) * step;
				V.y = sin(a) * step;
				X += V.x;
				Y += V.y;
			}
			else
			{
				// We ran into the shape. Reduce the step and try again
				V.x = V.x / 2;
				V.y = V.y / 2;
				X += V.x;
				Y += V.y;
			}
			break;

		case resultNoOverlap:
			if ( iContact == 0 )
			{
				// Still looking for the first contact
				X += V.x;
				Y += V.y;
			}
			else
			{
				// Back off
				X -= V.x;
				Y -= V.y;
				// Try with a different angle
				a -= SMALL_ANGLE;
				V.x = cos(a) * step;
				V.y = sin(a) * step;
				X += V.x;
				Y += V.y;
			}
			break;
		}
	}

	// Return to zero altitude, turn OFF spindle
	doGcode("G0 Z-0.4 M0\r\n");
	
/*
	int a, b, c, d, e, f;
	
	a = GetPixel(&bm, 312, 265);
	b = GetPixel(&bm, 98, 488);
	c = GetPixel(&bm, 353, 54);
	d = GetPixel(&bm, 423, 441);
	e = GetPixel(&bm, 429, 441);
	f = GetPixel(&bm, 355, 482);

	Sleep(1);
*/
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
				BitmapProcess(hWnd);
				break;
			case IDC_EXECUTE:
				BitmapShapeExecute(hWnd);
				break;
			case IDC_RESET_SIM:
				resetBlockSurface();
				break;
			case IDOK:
				/*
				if (!GetDlgItemText(hwndDlg, ID_ITEMNAME, szItemName, 80))
				*szItemName = 0;

				// Fall through.
				*/
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