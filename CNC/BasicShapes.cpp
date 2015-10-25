#include "CNC.h"
#include "Resource.h"
#include "Windowsx.h"

#include "gcode.h"
#include "fileParser.h"

#define MAX_STR	80

typedef struct
{
	// General parameters
	float toolRadius;
	float toolMaxDepth;
	float cutDepth;
	int cutSpeed;
	int motorControl;

	// Circle
	float circleRadius;
	float circleDepth;
	int circleFill;

} tSimpleShapeParam;

tSimpleShapeParam g_Params;

const struct { WCHAR* str; float val; } TOOL_SIZES[] = {
	{L"0.5       (1/2)",0.5f},
	{L"0.25     (1/4)",0.25f},
	{L"0.125   (1/8)",0.125f},
	{L"0.0625 (1/16)",0.0625}};

const struct { WCHAR* str; int val; } CUT_SPEED[] = {
	{L"F10",10},
	{L"F20",20},
	{L"F30",30},
	{L"F40",40},
	{L"F60",50}};

#define ITEM_CNT(x) (sizeof(x)/sizeof(x[0]))

void BasicShapeInit(HWND hWnd)
{
	int i;
	HWND hItem;

	hItem = GetDlgItem(hWnd, IDC_TOOL_SIZE);
	for (i = 0; i < ITEM_CNT(TOOL_SIZES);i++) ComboBox_AddString(hItem, TOOL_SIZES[i].str);

	hItem = GetDlgItem(hWnd, IDC_CUT_SPEED);
	for (i = 0; i < ITEM_CNT(CUT_SPEED); i++) ComboBox_AddString(hItem, CUT_SPEED[i].str);
	
	HKEY hKey;
	if (RegOpenKey(HKEY_CURRENT_USER, L"SOFTWARE", &hKey) == ERROR_SUCCESS)
	{
		DWORD cbData = sizeof(g_Params);
		if (RegGetValue(hKey, L"WinCNC", L"BasicShape", RRF_RT_REG_BINARY, NULL, &g_Params, &cbData) == ERROR_SUCCESS)
		{
			return;
		}
	}
	g_Params.toolRadius = 0.125f;
	g_Params.toolMaxDepth = 0.75f;
	g_Params.cutSpeed = 30;
	g_Params.circleDepth = 0.1f;
}

void BasicShapeSave()
{
	HKEY hKey;
	if (RegCreateKey(HKEY_CURRENT_USER, L"SOFTWARE\\WinCNC", &hKey) == ERROR_SUCCESS)
	{
		RegSetValueEx(hKey, L"BasicShape", 0, REG_BINARY, (BYTE*)&g_Params, sizeof(g_Params));
	}
}

UINT GetSetFloat(HWND hWnd, UINT id, BOOL get, float* val)
{
	WCHAR str[MAX_STR];
	HWND hItem;
	hItem = GetDlgItem(hWnd, id);
	if (get)
	{
		GetWindowText(hItem, str, MAX_STR);
		if (swscanf_s(str, L"%f", val) != 1) return id;
	}
	else
	{
		StringCbPrintf(str, sizeof(str), L"%.4f", *val );
		SetWindowText(hItem, str);
	}
	return 0;
}

UINT GetSetBool(HWND hWnd, UINT id, BOOL get, int* val)
{
	HWND hItem;
	hItem = GetDlgItem(hWnd, id);
	if (get)
	{
		*val = Button_GetCheck(hItem);
	}
	else
	{
		Button_SetCheck(hItem, *val);
	}
	return 0;
}



UINT BasicShapeGetSet(BOOL get, HWND hWnd )
{
	HWND hItem;
	WCHAR str[MAX_STR];
	float val;
	int i;

	hItem = GetDlgItem(hWnd, IDC_TOOL_SIZE);
	if (get)
	{
		ComboBox_GetText(hItem, str, MAX_STR);
		if (swscanf_s(str, L"%f", &val) != 1) return IDC_TOOL_SIZE;
		g_Params.toolRadius = val / 2.0f;
	}
	else
	{
		for (i = 0; i < ITEM_CNT(TOOL_SIZES); i++)
		{
			if (TOOL_SIZES[i].val == (g_Params.toolRadius * 2.0f))
			{
				ComboBox_SetCurSel(hItem, i);
				break;
			}
		}
		if( i== ITEM_CNT(TOOL_SIZES))
		{
			StringCbPrintf( str,sizeof(str),L"%f", g_Params.toolRadius * 2.0f);
			ComboBox_SetText(hItem, str);
		}
	}

	hItem = GetDlgItem(hWnd, IDC_CUT_SPEED);
	if (get)
	{
		i = ComboBox_GetCurSel(hItem);
		if (i != CB_ERR) 
			g_Params.cutSpeed = CUT_SPEED[i].val;
		else
		{
			ComboBox_GetText(hItem, str, MAX_STR);
			if (swscanf_s(str, L"%d", &i) != 1) return IDC_CUT_SPEED;
			g_Params.cutSpeed = i;
		}
	}
	else
	{
		for (i = 0; i < ITEM_CNT(CUT_SPEED); i++)
		{
			if (CUT_SPEED[i].val == g_Params.cutSpeed)
			{
				ComboBox_SetCurSel(hItem, i);
				break;
			}
		}
		if (i == ITEM_CNT(CUT_SPEED))
		{
			StringCbPrintf(str, sizeof(str), L"%d", g_Params.cutSpeed );
			ComboBox_SetText(hItem, str);
		}
	}

	GetSetFloat(hWnd, IDC_CUT_DEPTH, get, &g_Params.cutDepth);

	GetSetFloat(hWnd, IDC_MAX_TOOL_DEPTH, get, &g_Params.toolMaxDepth);

	GetSetBool(hWnd, IDC_MOTOR, get, &g_Params.motorControl);

	GetSetFloat(hWnd, IDC_CIRCLE_RADIUS, get, &g_Params.circleRadius);

	GetSetFloat(hWnd, IDC_CIRCLE_DEPTH, get, &g_Params.circleDepth);

	GetSetBool(hWnd, IDC_CIRCLE_FILL, get, &g_Params.circleFill);

	return 0;
}

#define MAX_BUF 100000

/*
G2 X2.3125 Y2.3125 I2.3125 Z - 0.3
G2 X2.3125 Y - 2.3125 J - 2.3125 P2
G0 Z0.3
G0 X - 4.625
*/

#define NEAR_ZERO 0.00001f

BOOL CarveCircle(HWND hWnd)
{
	char str[MAX_STR];
	char* cmd;
	float r,D,d;

	if (g_Params.circleRadius < g_Params.toolRadius * 2.0f) return FALSE;
	if (g_Params.circleDepth > g_Params.toolMaxDepth) return FALSE;
	if (g_Params.circleDepth < NEAR_ZERO) return FALSE;

	cmd = (char*)malloc(MAX_BUF);
	if (cmd == NULL) return FALSE;
	memset(cmd, 0x00, MAX_BUF);
	
	// G91: Relative Motion
	// Fxx : Carve speed
	// M3 : Motor ON
	// G4 P1 : Wait 1sec. for Spindle
	sprintf_s(str, MAX_STR, "G91 F%d %s\r\n", g_Params.cutSpeed, g_Params.motorControl ? "M3 G4 P1" : "");
	strcat_s(cmd, MAX_BUF, str);

	r = g_Params.circleRadius;
	do
	{
		// Move inward on X by tool radius
		if (r > g_Params.toolRadius)
		{
			r = r - g_Params.toolRadius;
			sprintf_s(str, MAX_STR, "G1 X%f\r\n", g_Params.toolRadius);
			strcat_s(cmd, MAX_BUF, str);
		}

		// Cut circle. Do multiple turns to ensure that we don't cut more
		// than g_Params.cutDepth at once.
		D = g_Params.circleDepth;
		while (D >= NEAR_ZERO )
		{
			d = (D > g_Params.cutDepth) ? g_Params.cutDepth : D;
			D = D - d;
			// First quarter (9 to 12) while diving by the cut depth
			sprintf_s(str, MAX_STR, "G2 X%f Y%f I%f Z%f\r\n", r, r, r, -d);
			strcat_s(cmd, MAX_BUF, str);
			// Second quarter (12 to 3)
			sprintf_s(str, MAX_STR, "G2 X%f Y%f J%f\r\n", r, -r, -r);
			strcat_s(cmd, MAX_BUF, str);
			// Third quarter (3 to 6)
			sprintf_s(str, MAX_STR, "G2 X%f Y%f I%f\r\n", -r, -r, -r);
			strcat_s(cmd, MAX_BUF, str);
			// Fourth quarter (6 to 9)
			sprintf_s(str, MAX_STR, "G2 X%f Y%f J%f\r\n", -r, r, r);
			strcat_s(cmd, MAX_BUF, str);
		}

		// To ensure flat botom, go one more quarter (9 to 12)
		sprintf_s(str, MAX_STR, "G2 X%f Y%f I%f\r\n", r, r, r);
		strcat_s(cmd, MAX_BUF, str);

		// Come back to starting height
		sprintf_s(str, MAX_STR, "G0 Z%f\r\n", g_Params.circleDepth - D);
		strcat_s(cmd, MAX_BUF, str);

		// Move back to start of circle.
		sprintf_s(str, MAX_STR, "G0 X%f Y%f\r\n", -r, -r);
		strcat_s(cmd, MAX_BUF, str);

		if (g_Params.circleFill)
		{
			// Move to next concentric circle
			// Overlap each concentic circle by 1/32th to ensure smooth bottom
			sprintf_s(str, MAX_STR, "G1 X%f\r\n", g_Params.toolRadius - 0.03125f);
			strcat_s(cmd, MAX_BUF, str);
			r = r - (g_Params.toolRadius - 0.03125f);
		}
	} while (( r > 0) && g_Params.circleFill);

	// Move back to starting point
	sprintf_s(str, MAX_STR, "G0 X%f M0\r\n", -(g_Params.circleRadius - r));
	strcat_s(cmd, MAX_BUF, str);

	HWND hItem = GetDlgItem(hWnd, IDC_GCODE);
	SetWindowTextA(hItem, cmd);
	free(cmd);

	return TRUE;
}


void BasicShapeOneCommand(HWND hWnd)
{
	char* cmd;
	HWND hItem;
	hItem = GetDlgItem(hWnd, IDC_GCODE );

	int l = GetWindowTextLength(hItem) + 1;
	cmd = (char*)malloc(l);
	if (cmd)
	{
		GetWindowTextA(hItem, cmd, l);
		ParseBuffer(hWnd, cmd, l, doGcode);
		free(cmd);
	}
}


BOOL CALLBACK BasicShapesProc(HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam)
{

	switch (message)
	{
	case WM_INITDIALOG:
		BasicShapeInit(hWnd);
		BasicShapeGetSet(FALSE, hWnd);
		return TRUE;
		break;

	case WM_CLOSE:
		BasicShapeGetSet(TRUE, hWnd);
		BasicShapeSave();
		break;

	case WM_COMMAND:
		switch (LOWORD(wParam))
		{
		case IDC_EXECUTE :
			BasicShapeOneCommand(hWnd);
			break;
		case IDC_CARVE_CIRCLE:
			BasicShapeGetSet(TRUE , hWnd );
			CarveCircle(hWnd);
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

void BasicShapes(HWND hWnd)
{
	DialogBox(NULL,
		MAKEINTRESOURCE(IDD_BASIC_SHAPES),
		hWnd,
		(DLGPROC)BasicShapesProc);
}