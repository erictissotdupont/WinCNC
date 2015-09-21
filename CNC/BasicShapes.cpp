#include "CNC.h"
#include "Resource.h"
#include "Windowsx.h"

BOOL CALLBACK BasicShapesProc(HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam)
{
	HWND hItem;

	switch (message)
	{
	case WM_INITDIALOG:

		hItem = GetDlgItem(hWnd, IDC_TOOL_SIZE);

		ComboBox_AddString(hItem, L"1/2");
		ComboBox_AddString(hItem, L"1/4");
		ComboBox_AddString(hItem, L"1/8");
		ComboBox_AddString(hItem, L"1/16");
		ComboBox_SetCurSel(hItem, 1);

		return TRUE;
		break;

	case WM_COMMAND:
		switch (LOWORD(wParam))
		{
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