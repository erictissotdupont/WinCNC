#include "CNC.h"
#include "Resource.h"
#include "Windowsx.h"
#include "math.h"

#include "gcode.h"
#include "fileParser.h"
#include "Shapes.h"

typedef struct
{
	tGeneralToolInfo tool;

	// Box
	float boxA;
	float boxB;
	float boxC;
	float boxT;

} tComplexShapeParam;

tComplexShapeParam g_Params;

void ComplexShapeInit(HWND hWnd)
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
		DWORD cbData = sizeof(g_Params);
		if (RegGetValue(hKey, L"WinCNC", L"ComplexShape", RRF_RT_REG_BINARY, NULL, &g_Params, &cbData) == ERROR_SUCCESS)
		{
			return;
		}
	}
	ShapeInitToolInfo(&g_Params.tool);
}

void ComplexShapeSave()
{
	HKEY hKey;
	if (RegCreateKey(HKEY_CURRENT_USER, L"SOFTWARE\\WinCNC", &hKey) == ERROR_SUCCESS)
	{
		RegSetValueEx(hKey, L"ComplexShape", 0, REG_BINARY, (BYTE*)&g_Params, sizeof(g_Params));
	}
}


UINT ComplexShapeGetSet(BOOL get, HWND hWnd)
{
	ShapeGetSetTool(hWnd, get, &g_Params.tool);

	ShapeGetSetFloat(hWnd, IDC_BOX_A, get, &g_Params.boxA);
	ShapeGetSetFloat(hWnd, IDC_BOX_B, get, &g_Params.boxB);
	ShapeGetSetFloat(hWnd, IDC_BOX_C, get, &g_Params.boxC);
	ShapeGetSetFloat(hWnd, IDC_BOX_T, get, &g_Params.boxT);

	return 0;
}

#define MAX_BUF 100000

#define NEAR_ZERO		0.00001f
#define SMALL_OVELAP	(1/128.0f)
#define SMALLEST_RADIUS	( g_Params.toolRadius + SMALL_OVELAP )

#define TAB_WIDTH	0.25

void BoxMove(char* cmd, double X, double Y, double tab)
{
	char str[MAX_STR];
	double L = sqrt(X*X + Y*Y);
	double l = (L - TAB_WIDTH) / 2.0;

	if ((X == 0.0) && (Y == 0.0)) return;

	if (tab != 0.0 && l > 0.0)
	{
		double a = (-Y * tab) / L;
		double b = (X * tab) / L;
		double A = (X * TAB_WIDTH) / L;
		double B = (Y * TAB_WIDTH) / L;

		sprintf_s(str, MAX_STR, "G1 X%f Y%f\r\n", (X * l) / L, (Y * l) / L);
		strcat_s(cmd, MAX_BUF, str);

		sprintf_s(str, MAX_STR, "G1 Z%f\r\n", tab);
		strcat_s(cmd, MAX_BUF, str);
		sprintf_s(str, MAX_STR, "G1 X%f Y%f\r\n", a, b);
		strcat_s(cmd, MAX_BUF, str);
		sprintf_s(str, MAX_STR, "G1 X%f Y%f\r\n", A, B);
		strcat_s(cmd, MAX_BUF, str);
		sprintf_s(str, MAX_STR, "G1 X%f Y%f\r\n", -A, -B);
		strcat_s(cmd, MAX_BUF, str);
		sprintf_s(str, MAX_STR, "G1 X%f Y%f\r\n", -2.0 * a, -2.0 * b);
		strcat_s(cmd, MAX_BUF, str);
		sprintf_s(str, MAX_STR, "G1 X%f Y%f\r\n", A, B);
		strcat_s(cmd, MAX_BUF, str);
		sprintf_s(str, MAX_STR, "G1 X%f Y%f\r\n", -A, -B);
		strcat_s(cmd, MAX_BUF, str);
		sprintf_s(str, MAX_STR, "G1 X%f Y%f\r\n", a, b);
		strcat_s(cmd, MAX_BUF, str);
		sprintf_s(str, MAX_STR, "G1 X%f Y%f\r\n", A, B);
		strcat_s(cmd, MAX_BUF, str);

		sprintf_s(str, MAX_STR, "G1 Z%f\r\n", -tab);
		strcat_s(cmd, MAX_BUF, str);
		sprintf_s(str, MAX_STR, "G1 X%f Y%f\r\n", (X * l) / L, (Y * l) / L);
		strcat_s(cmd, MAX_BUF, str);
	}
	else
	{
		sprintf_s(str, MAX_STR, "G1 X%f Y%f\r\n", X, Y);
		strcat_s(cmd, MAX_BUF, str);
	}
}

void FastBoxMove(char* cmd, double X, double Y, double T)
{
	char str[MAX_STR];
	sprintf_s(str, MAX_STR, "G1 Z%f\r\n", 2 * T);
	strcat_s(cmd, MAX_BUF, str);
	sprintf_s(str, MAX_STR, "G0 X%f Y%f\r\n", X, Y);
	strcat_s(cmd, MAX_BUF, str);
	sprintf_s(str, MAX_STR, "G1 Z%f\r\n", -2 * T);
	strcat_s(cmd, MAX_BUF, str);
}

BOOL CarveBox(HWND hWnd)
{
	char str[MAX_STR];
	char* cmd;
	int cycles;
	float layer;
	float A, B, C, T;

	cmd = (char*)malloc(MAX_BUF);
	if (cmd == NULL) return FALSE;
	memset(cmd, 0x00, MAX_BUF);

	A = g_Params.boxA;
	B = g_Params.boxB;
	C = g_Params.boxC;
	T = g_Params.boxT;

	cycles = (int)(T / g_Params.tool.cutDepth) + 1;
	if (cycles < 2) cycles = 2;
	layer = T / cycles;

	// G91: Relative Motion
	// Fxx : Carve speed
	// M3 : Motor ON
	// G4 P1 : Wait 1sec for tool spindle to start
	sprintf_s(str, MAX_STR, "G91 F%d %s\r\n", g_Params.tool.cutSpeed, g_Params.tool.motorControl ? "M3 G4 P1" : "");
	strcat_s(cmd, MAX_BUF, str);

	// Do the center vertical cut first
	FastBoxMove(cmd, 4 * T + C + A, 0.0, T);

	for (int n = 0; n < cycles; n++)
	{
		double tab = (n == (cycles - 1)) ? layer : 0.0;

		// Dive in with a small zigzag move
		sprintf_s(str, MAX_STR, "G1 Y%f Z%f\r\n", T, -layer / 2);
		strcat_s(cmd, MAX_BUF, str);
		sprintf_s(str, MAX_STR, "G1 Y%f Z%f\r\n", -T, -layer / 2);
		strcat_s(cmd, MAX_BUF, str);

		BoxMove(cmd, 0.0, B + 2 * T, tab);
		BoxMove(cmd, 0.0, C + 2 * T, tab);
		FastBoxMove(cmd, 0.0, -(B + C + 4 * T), T);
	}

	// Return to starting height
	sprintf_s(str, MAX_STR, "G0 Z%f\r\n", T);
	strcat_s(cmd, MAX_BUF, str);

	// Do the U cut next
	FastBoxMove(cmd, -(2 * T + A), 0.0, T);

	for (int n = 0; n < cycles; n++)
	{
		double tab = (n == (cycles - 1)) ? layer : 0.0;

		// Dive in with a small zigzag move
		sprintf_s(str, MAX_STR, "G1 Y%f Z%f\r\n", T, -layer / 2);
		strcat_s(cmd, MAX_BUF, str);
		sprintf_s(str, MAX_STR, "G1 Y%f Z%f\r\n", -T, -layer / 2);
		strcat_s(cmd, MAX_BUF, str);

		BoxMove(cmd, 0.0, 2 * T + B, tab);
		BoxMove(cmd, 2 * T + A, 0.0, tab);
		BoxMove(cmd, 2 * T + A, 0.0, tab);
		BoxMove(cmd, 0.0, -(2 * T + B), tab);

		// return to start of loop
		FastBoxMove(cmd, -(4 * T + 2 * A), 0.0, T);
	}

	// Return to starting height and origin
	sprintf_s(str, MAX_STR, "G0 Z%f\r\n", T);
	strcat_s(cmd, MAX_BUF, str);
	FastBoxMove(cmd, -(2 * T + C), 0.0, T);

	// Finally do the outside cut
	for (int n = 0; n < cycles; n++)
	{
		double tab = (n == (cycles - 1)) ? layer : 0.0;

		// Dive in with a small zigzag move
		sprintf_s(str, MAX_STR, "G1 X%f Z%f\r\n", T, -layer / 2);
		strcat_s(cmd, MAX_BUF, str);
		sprintf_s(str, MAX_STR, "G1 X%f Z%f\r\n", -T, -layer / 2);
		strcat_s(cmd, MAX_BUF, str);

		BoxMove(cmd, C + 2 * T, 0.0, tab);
		BoxMove(cmd, A + 2 * T, 0.0, tab);
		BoxMove(cmd, A + 2 * T, 0.0, tab);
		BoxMove(cmd, C + 2 * T, 0.0, tab);
		BoxMove(cmd, 0.0, B + 2 * T, tab);

		BoxMove(cmd, -(C + 2 * T), 0.0, tab);
		BoxMove(cmd, 0.0, C + 2 * T, tab);

		BoxMove(cmd, -(A + 2 * T), 0.0, tab);
		BoxMove(cmd, -(A + 2 * T), 0.0, tab);

		BoxMove(cmd, 0.0, -(C + 2 * T), tab);
		BoxMove(cmd, -(C + 2 * T), 0.0, tab);
		BoxMove(cmd, 0.0, -(B + 2 * T), tab);
	}

	// Return to start height and turn off spindle
	sprintf_s(str, MAX_STR, "G0 Z%f %s\r\n", T, g_Params.tool.motorControl ? "M0" : "");
	strcat_s(cmd, MAX_BUF, str);

	HWND hItem = GetDlgItem(hWnd, IDC_GCODE);
	SetWindowTextA(hItem, cmd);
	free(cmd);

	return TRUE;
}


void ComplexShapeOneCommand(HWND hWnd)
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

WNDPROC g_oldComplexDlgdProc = NULL;
BOOL CALLBACK ComplexInterceptWndProc(HWND hWnd,
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
	return g_oldComplexDlgdProc(hWnd, message, wParam, lParam);
}

BOOL CALLBACK ComplexShapesProc(HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam)
{
	HWND hDlg;

	switch (message)
	{
	case WM_INITDIALOG:
		ComplexShapeInit(hWnd);
		ComplexShapeGetSet(FALSE, hWnd);

		// This is to capture the CTRL+A on the GCode edit box
		hDlg = GetDlgItem(hWnd, IDC_GCODE);
		g_oldComplexDlgdProc = (WNDPROC)GetWindowLong(hDlg, GWL_WNDPROC);
		SetWindowLong(hDlg, GWL_WNDPROC, (LONG)ComplexInterceptWndProc);
		return TRUE;
		break;

	case WM_CLOSE:
		ComplexShapeGetSet(TRUE, hWnd);
		ComplexShapeSave();
		break;

	case WM_COMMAND:
		switch (LOWORD(wParam))
		{
		case IDC_EXECUTE:
			ComplexShapeOneCommand(hWnd);
			break;
		case IDC_CARVE_BOX:
			ComplexShapeGetSet(TRUE, hWnd);
			CarveBox(hWnd);
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
	return FALSE;
}

void ComplexShapes(HWND hWnd)
{
	DialogBox(NULL,
		MAKEINTRESOURCE(IDD_COMPLEX_SHAPES),
		hWnd,
		(DLGPROC)ComplexShapesProc);
}
