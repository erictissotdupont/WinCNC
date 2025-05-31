#include "Main.h"
#include <Commdlg.h>
#include "Shapes.h"

#include "motor.h"
#include "gcode.h"
#include "CNC.h"
#include "Simulator.h"
#include "Parser.h"
#include "ManualMode.h"
#include "..\CNC_Protocol.h"

#define MANUAL_MODE_RES   5
#define MANUAL_MAX_STEP   8

void OnManualModeTimer(HWND hWnd)
{
	RECT btn;
	RECT wnd;
	POINT curPos;
	int x = 0;
	int y = 0;
	int z = 0;
	static bool bZMode = false;
	static bool bCTRLdown = false;
	static int keyDownCount = 0;

	HWND hBtn = GetDlgItem(hWnd, IDOK);
	GetWindowRect(hBtn, &btn);
	GetWindowRect(hWnd, &wnd);
	GetCursorPos(&curPos);

	if (curPos.x > wnd.right || curPos.x < wnd.left || curPos.y > wnd.bottom || curPos.y < wnd.top)
	{
		PostMessage(hWnd, WM_CLOSE, 0, 0);
	}
	else
	{
		int Xres = ((wnd.right - wnd.left) - (btn.right - btn.left)) / ((MANUAL_MAX_STEP + 1) * 2);
		int Yres = ((wnd.bottom - wnd.top) - (btn.bottom - btn.top)) / ((MANUAL_MAX_STEP + 1) * 2);

		if (curPos.x < btn.left)
		{
			y = (curPos.x - btn.left) / Xres - 1;
		}
		else if (curPos.x > btn.right)
		{
			y = (curPos.x - btn.right) / Xres + 1;
		}

		if (curPos.y < btn.top)
		{
			x = (curPos.y - btn.top) / Yres - 1;
		}
		else if (curPos.y > btn.bottom)
		{
			x = (curPos.y - btn.bottom) / Yres + 1;
		}
		if (x > MANUAL_MAX_STEP) x = MANUAL_MAX_STEP;
		if (x < -MANUAL_MAX_STEP) x = -MANUAL_MAX_STEP;
		if (y > MANUAL_MAX_STEP) y = MANUAL_MAX_STEP;
		if (y < -MANUAL_MAX_STEP) y = -MANUAL_MAX_STEP;

		if (x == 0 && y == 0)
		{
			keyDownCount++;
			int step = 1 + keyDownCount / 10;
			if (GetKeyState(VK_UP) & 0x8000) { x = -step; }
			else if (GetKeyState(VK_DOWN) & 0x8000) { x = step; }
			else if (GetKeyState(VK_LEFT) & 0x8000) { y = -step; }
			else if (GetKeyState(VK_RIGHT) & 0x8000) { y = step; }
			else { keyDownCount = 0; }
		}
	}

	if (GetKeyState(VK_CONTROL) & 0x8000)
	{
		bCTRLdown = true;
	}
	else
	{
		if (bCTRLdown == true)
		{
			bCTRLdown = false;
			bZMode = !bZMode;
			SetCursorPos((btn.left + btn.right) / 2, (btn.top + btn.bottom) / 2);
			x = 0;
			y = 0;
			z = 0;
		}
	}

	if (bZMode)
	{
		SetWindowText(hBtn, L"Z");
		z = x;
		x = 0;
		y = 0;
	}
	else
	{
		SetWindowText(hBtn, L"X Y");
	}

	CNC_SendManualUpdate(x, y, z);
}

void OnManualModeInit(HWND hWnd)
{
	// Make the dialog appear in the lowe right corner of the main window (its parent)
	RECT dlgRect;
	RECT parentRect;
	HWND hParent = GetParent(hWnd);

	GetWindowRect(hParent, &parentRect);
	GetWindowRect(hWnd, &dlgRect);
	int width = dlgRect.right - dlgRect.left;
	int height = dlgRect.bottom - dlgRect.top;

	SetWindowPos(hWnd, hParent,
		parentRect.right - width,
		parentRect.bottom - height,
		width,
		height,
		0);

	// Put the cursor in the center of the X/Y button (idle)
	RECT btnRect;
	HWND hBtn = GetDlgItem(hWnd, IDOK);
	GetWindowRect(hBtn, &btnRect);
	SetCursorPos((btnRect.left + btnRect.right) / 2, (btnRect.top + btnRect.bottom) / 2);

	// Start the refresh timer
	SetTimer(hWnd, 1, 125, NULL);
}

BOOL CALLBACK ManualModeProc(HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam)
{
	switch (message)
	{
	case WM_INITDIALOG:
		OnManualModeInit(hWnd);
		return TRUE;
		break;

	case WM_TIMER:
		OnManualModeTimer(hWnd);
		break;

	case WM_CLOSE:
		CNC_SendManualUpdate(0, 0, 0);
		EndDialog(hWnd, wParam);
		return TRUE;
		break;

	case WM_COMMAND:
		switch (LOWORD(wParam))
		{
		case IDOK:
			CNC_SendManualUpdate(0, 0, 0);
			EndDialog(hWnd, wParam);
			return TRUE;
		}
	}
	return FALSE;
}

void ManualMode(HWND hWnd)
{
	DialogBox(NULL,
		MAKEINTRESOURCE(IDD_MANUAL_MODE),
		hWnd,
		(DLGPROC)ManualModeProc);
}
