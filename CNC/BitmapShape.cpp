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
			g_BmParams.X = 2.5;
			g_BmParams.Y = 2;
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


BOOL GetPixel(BITMAP* bm, unsigned int x, unsigned int y)
{
	unsigned char* pt;
	// Vertical axis is reversed
	y = bm->bmHeight - y - 1;
	// Move pointer to byte that contains this pixel
	pt = (unsigned char*)bm->bmBits + (y * bm->bmWidthBytes) + (x / 8);
	// Mask pixel in the byte
	return (*pt & (0x80 >> (x % 8))) == 0;
}


BOOL BitmapProcess(HWND hWnd)
{
	BITMAP  bm;
	HANDLE hBitmap;
	float xRes, yRes;

	hBitmap = LoadImage( 
		NULL, 
		g_BmParams.szFilePath, 
		IMAGE_BITMAP, 
		0, 0, 
		LR_CREATEDIBSECTION | LR_LOADFROMFILE | LR_DEFAULTSIZE | LR_MONOCHROME);

	// Get the color depth of the DIBSection
	GetObject(hBitmap, sizeof(BITMAP), &bm);

	xRes = g_BmParams.X / bm.bmWidth;
	yRes = g_BmParams.Y / bm.bmHeight;

	int toolWidth = (g_BmParams.tool.radius / xRes);
	int toolHeight = (g_BmParams.tool.radius / yRes);
	int toolPoints = toolWidth * toolWidth * 4;
	int *toolX = (int*)malloc(toolPoints*sizeof(int));
	int *toolY = (int*)malloc(toolPoints*sizeof(int));
	int toolPtCnt = 0;

	for (int x = 0; x <= toolWidth; x++ )
	for (int y = 0; y <= toolHeight; y++)
	{

	}

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