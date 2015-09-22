#include "CNC.h"
#include "Resource.h"
#include "Windowsx.h"


#define MAX_STR	80

typedef struct
{
	float toolRadius;
	float cutDepth;
	int cutSpeed;

	// Circle
	float circleRadius;
	float circleDepth;
	BOOL  circleFill;

} tSimpleShapeParam;

tSimpleShapeParam g_Params;

void BasicShapeInit(HWND hWnd)
{
	HWND hItem;

	hItem = GetDlgItem(hWnd, IDC_TOOL_SIZE);
	ComboBox_AddString(hItem, L"0.5       (1/2)");
	ComboBox_AddString(hItem, L"0.25     (1/4)");
	ComboBox_AddString(hItem, L"0.125   (1/8)");
	ComboBox_AddString(hItem, L"0.0625 (1/16)");

	hItem = GetDlgItem(hWnd, IDC_CUT_SPEED);
	ComboBox_AddString(hItem, L"F10");
	ComboBox_AddString(hItem, L"F20");
	ComboBox_AddString(hItem, L"F30");
	ComboBox_AddString(hItem, L"F40");
	ComboBox_AddString(hItem, L"F60");
	
	hItem = GetDlgItem(hWnd, IDC_CUT_DEPTH);
	ComboBox_AddString(hItem, L"0.05");
	ComboBox_AddString(hItem, L"0.1");
	ComboBox_AddString(hItem, L"0.2");
	ComboBox_AddString(hItem, L"0.3");
	ComboBox_AddString(hItem, L"0.4");

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
	g_Params.cutSpeed = 30;
	g_Params.circleDepth = 0.1f;
}

UINT BasicShapeGetSet(BOOL get, HWND hWnd )
{
	HWND hItem;
	WCHAR str[MAX_STR];
	float val;

	hItem = GetDlgItem(hWnd, IDC_TOOL_SIZE);
	if (get)
	{
		ComboBox_GetText(hItem, str, MAX_STR);
		if (swscanf_s(str, L"%f", &val) != 1) return IDC_TOOL_SIZE;
		g_Params.toolRadius = val / 2.0f;
	}
	else
	{
		if (g_Params.toolRadius == 0.25f) ComboBox_SetCurSel(hItem, 0);
		else if (g_Params.toolRadius == 0.125f) ComboBox_SetCurSel(hItem, 1);
		else if (g_Params.toolRadius == 0.0625f) ComboBox_SetCurSel(hItem, 2);
		else if (g_Params.toolRadius == 0.03125f) ComboBox_SetCurSel(hItem, 3);
		else
		{
			wsprintf(str, L"%f.3", g_Params.toolRadius * 2.0f);
			ComboBox_SetText(hItem, str);
		}
	}

	hItem = GetDlgItem(hWnd, IDC_CUT_SPEED);

	hItem = GetDlgItem(hWnd, IDC_CUT_DEPTH);

	return 0;
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
		break;

	case WM_COMMAND:
		switch (LOWORD(wParam))
		{
		case IDC_CARVE_CIRCLE:
			BasicShapeGetSet(TRUE , hWnd );
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