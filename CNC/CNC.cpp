// CNC.cpp : Defines the entry point for the application.
//

#include "CNC.h"
#include <Commdlg.h>
#include "Shapes.h"

#include "motor.h"
#include "gcode.h"
#include "socket.h"
#include "3Dview.h"
#include "fileParser.h"
#include "..\CNC_Protocol.h"


// Global Variables:
HWND hMainWindow = NULL;						// main window handle
HINSTANCE hInst;								// current instance
TCHAR szTitle[MAX_PATH];					// The title bar text
TCHAR szWindowClass[MAX_PATH];			// the main window class name

tMetaData g_MetaData;

// Forward declarations of functions included in this code module:
ATOM				MyRegisterClass(HINSTANCE hInstance);
BOOL				InitInstance(HINSTANCE, int);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	About(HWND, UINT, WPARAM, LPARAM);

void OnMachineUpdate(PVOID param)
{
	PostMessage(hMainWindow, WM_UPDATE_POSITION, 0, 0);
}


int APIENTRY _tWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPTSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

 	// TODO: Place code here.
	MSG msg;
	HACCEL hAccelTable;

	// Initialize global strings
	LoadString(hInstance, IDS_APP_TITLE, szTitle, MAX_PATH);
	LoadString(hInstance, IDC_CNC, szWindowClass, MAX_PATH);
	MyRegisterClass(hInstance);

	// Perform application initialization:
	if (!InitInstance (hInstance, nCmdShow))
	{
		return FALSE;
	}

	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_CNC));

	initSocketCom();
	registerSocketCallback(CNC_MACHINE_UPDATE, OnMachineUpdate);

	MotorInit();
	
	// Main message loop:
	while (GetMessage(&msg, NULL, 0, 0))
	{
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	return (int) msg.wParam;
}

//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style			= CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc	= WndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 0;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= LoadIcon(hInstance, MAKEINTRESOURCE(IDI_CNC));
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
	wcex.lpszMenuName	= MAKEINTRESOURCE(IDC_CNC);
	wcex.lpszClassName	= szWindowClass;
	wcex.hIconSm		= LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

	return RegisterClassEx(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
	RECT rc;
	int x, y;

	hInst = hInstance; // Store instance handle in our global variable

	GetWindowRect(GetDesktopWindow(), &rc);
	x = (rc.right - rc.left) - (rc.bottom - rc.top);
	y = x * 2 / 3;
	AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW & ~WS_THICKFRAME, FALSE);

   hMainWindow = CreateWindow(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW & ~WS_THICKFRAME,
	   0, 0, x, y, NULL, NULL, hInstance, NULL);
      //CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, NULL, NULL, hInstance, NULL);
  
   if (!hMainWindow)
   {
      return FALSE;
   }

   ShowWindow(hMainWindow, nCmdShow);
   UpdateWindow(hMainWindow);
   //SetWindowPos(hMainWindow, NULL, 0, 0, 0, 0, SWP_SHOWWINDOW | SWP_NOSIZE | SWP_NOZORDER | SWP_DRAWFRAME);
   return TRUE;
}

void OnRunGCode(HWND hWnd,BOOL bDebug)
{
	WCHAR szFile[MAX_PATH];       // buffer for file name
	OPENFILENAME ofn;

	// Initialize OPENFILENAME
	memset(&ofn, 0x00, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = hWnd;
	ofn.lpstrFile = szFile;
	// Set lpstrFile[0] to '\0' so that GetOpenFileName does not 
	// use the contents of szFile to initialize itself.
	ofn.lpstrFile[0] = L'\0';
	ofn.nMaxFile = sizeof(szFile);
	ofn.lpstrFilter = L"GCode\0*.TXT\0";
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = NULL;
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = NULL;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	if (GetOpenFileName(&ofn))
	{
		ParseGCodeFile( hWnd, szFile, doGcode, bDebug );
	}
}

void MachineCalibrate(HWND hWnd)
{
	if ((getCNCState() & CNC_STATE_IDLE) == 0)
	{
		MessageBoxA(hWnd, "Calibration cannot be performed when the machine is not idle.", "CNC", MB_ICONERROR);
	}
	else
	{
		tStatus status = postCommand(CNC_CMD_CALIBRATE);
		if (status != retSuccess)
		{
			MessageBoxA(hWnd, "Calibration command failed.", "CNC", MB_ICONERROR);
		}
	}
}

void MachineReboot(HWND hWnd)
{
	if (MessageBoxA(hWnd, "The reboot will lose position and calibration. Are you sure?", "CNC", MB_YESNO | MB_ICONEXCLAMATION) == IDYES)
	{
		if ((getCNCState() & CNC_STATE_IDLE) != 0 ||
			MessageBoxA(hWnd, "THE MACHINE IS NOT IDLE. Reboot will abort in an unknown state. Are you really sure?", "CNC", MB_YESNOCANCEL | MB_ICONERROR) == IDYES)
		{
			CNC_Reboot();
		}
	}
}

void SenManualCommand( int x, int y, int z )
{
	char msg[100];
	int len = sprintf_s(msg, sizeof(msg), CNC_HEADER CNC_MANUAL_HEADER "|" CNC_MANUAL_PARAMS "|", x, y, z);
	sendToCNC(msg, len);
}

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

	if( curPos.x > wnd.right || curPos.x < wnd.left || curPos.y > wnd.bottom || curPos.y < wnd.top )
	{
		PostMessage(hWnd, WM_CLOSE, 0, 0);
	}
	else
	{
		int Xres = ((wnd.right - wnd.left) - (btn.right - btn.left)) / ((MANUAL_MAX_STEP+1)*2);
		int Yres = ((wnd.bottom - wnd.top) - (btn.bottom - btn.top)) / ((MANUAL_MAX_STEP+1)*2);

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

	SenManualCommand(x, y, z);
}

void OnManualModeInit(HWND hWnd)
{
	RECT btn;
	HWND hBtn = GetDlgItem(hWnd, IDOK);
	GetWindowRect(hBtn, &btn);
	
	SetCursorPos((btn.left + btn.right) / 2, (btn.top + btn.bottom) / 2);

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
		SenManualCommand(0,0,0);
		EndDialog(hWnd, wParam);
		return TRUE;
		break;

	case WM_COMMAND:
		switch (LOWORD(wParam))
		{
		case IDOK:
			SenManualCommand(0, 0, 0);
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

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE:  Processes messages for the main window.
//
//  WM_COMMAND	- process the application menu
//  WM_PAINT	- Paint the main window
//  WM_DESTROY	- post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent;

	switch (message)
	{
	case WM_COMMAND:
		wmId = LOWORD(wParam);
		wmEvent = HIWORD(wParam);
		// Parse the menu selections:
		switch (wmId)
		{
		case IDM_RUN_GCODE:
			OnRunGCode(hWnd,FALSE);
			break;
		case IDM_DEBUG_GCODE:
			OnRunGCode(hWnd, TRUE);
			break;
		case IDM_SIMULATE_GCODE:
			Start3DSimulator(hWnd);
			break;
		case IDM_MACHINE_CALIBRATE:
			MachineCalibrate(hWnd);
			break;
		case IDM_MACHINE_REBOOT:
			MachineReboot(hWnd);
			break;
		case IDM_MACHINE_MANUALMODE:
			ManualMode(hWnd);
			break;
		case IDM_BASIC_SHAPE:
			BasicShapes(hWnd);
			break;
		case IDM_BITMAP_SHAPE:
			BitmapShapes(hWnd);
			break;
		case IDM_COMPLEX_SHAPE:
			ComplexShapes(hWnd);
			break;
		case IDM_ABOUT:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
			break;
		case IDM_EXIT:
			DestroyWindow(hWnd);
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		break;

	case WM_PAINT:
		OnPaint(hWnd);
		break;

	case WM_UPDATE_POSITION:
		InvalidateRgn(hWnd, NULL, false);
		update3DView();
		break;

	case WM_REDRAW:
		InvalidateRgn(hWnd, NULL, false);
		break;

	case WM_DESTROY:
		PostQuitMessage(0);
		break;

	case WM_TIMER :
		break;

	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}
