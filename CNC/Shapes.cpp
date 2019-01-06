#include "CNC.h"
#include "Windowsx.h"
#include "Resource.h"
#include "Shapes.h"

void ShapeInitToolInfo(tGeneralToolInfo *pToolInfo)
{
	pToolInfo->radius = 0.125f;
	pToolInfo->maxDepth = 0.75f;
	pToolInfo->cutSpeed = 20;
	pToolInfo->cutDepth = 0.125f;
	pToolInfo->motorControl = 1;
	pToolInfo->safeTravel = 0.5f;
}

UINT ShapeGetSetFloat(HWND hWnd, UINT id, BOOL get, float* val)
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
		StringCbPrintf(str, sizeof(str), L"%.4f", *val);
		SetWindowText(hItem, str);
	}
	return 0;
}

UINT ShapeGetSetBool(HWND hWnd, UINT id, BOOL get, int* val)
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

UINT ShapeGetSetString(HWND hWnd, UINT id, BOOL get, WCHAR* str, int cbStr )
{
	HWND hItem;
	hItem = GetDlgItem(hWnd, id);
	if (get)
	{
		GetWindowText(hItem, str, cbStr);
	}
	else
	{
		SetWindowText(hItem, str);
	}
	return 0;
}

UINT ShapeGetSetRadio(HWND hWnd, UINT id, int btnCnt, BOOL get, int* val )
{
	if (get)
	{
		for (int i = 0; i < btnCnt; i++)
		{
			if (IsDlgButtonChecked(hWnd, id + i) == BST_CHECKED)
			{
				*val = i;
				break;
			}
		}
	}
	else
	{
		CheckRadioButton(hWnd, id, id + btnCnt, id + *val);
	}
	return 0;
}

UINT ShapeGetSetTool(HWND hWnd, BOOL get, tGeneralToolInfo *pToolInfo )
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
		pToolInfo->radius = val / 2.0f;
	}
	else
	{
		for (i = 0; i < ITEM_CNT(TOOL_SIZES); i++)
		{
			if (TOOL_SIZES[i].val == (pToolInfo->radius * 2.0f))
			{
				ComboBox_SetCurSel(hItem, i);
				break;
			}
		}
		if (i == ITEM_CNT(TOOL_SIZES))
		{
			StringCbPrintf(str, sizeof(str), L"%f", pToolInfo->radius * 2.0f);
			ComboBox_SetText(hItem, str);
		}
	}

	hItem = GetDlgItem(hWnd, IDC_CUT_SPEED);
	if (get)
	{
		i = ComboBox_GetCurSel(hItem);
		if (i != CB_ERR)
			pToolInfo->cutSpeed = CUT_SPEED[i].val;
		else
		{
			ComboBox_GetText(hItem, str, MAX_STR);
			if (swscanf_s(str, L"%d", &i) != 1) return IDC_CUT_SPEED;
			pToolInfo->cutSpeed = i;
		}
	}
	else
	{
		for (i = 0; i < ITEM_CNT(CUT_SPEED); i++)
		{
			if (CUT_SPEED[i].val == pToolInfo->cutSpeed)
			{
				ComboBox_SetCurSel(hItem, i);
				break;
			}
		}
		if (i == ITEM_CNT(CUT_SPEED))
		{
			StringCbPrintf(str, sizeof(str), L"%d", pToolInfo->cutSpeed);
			ComboBox_SetText(hItem, str);
		}
	}

	ShapeGetSetFloat(hWnd, IDC_CUT_DEPTH, get, &pToolInfo->cutDepth);
	ShapeGetSetFloat(hWnd, IDC_MAX_TOOL_DEPTH, get, &pToolInfo->maxDepth);
	ShapeGetSetBool(hWnd, IDC_MOTOR, get, &pToolInfo->motorControl);
	ShapeGetSetFloat(hWnd, IDC_SAFE_TRAVEL, get, &pToolInfo->safeTravel);

	return 0;
}