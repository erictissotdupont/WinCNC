// CNC.cpp : Defines the entry point for the application.
//

#include "Main.h"
#include <Commdlg.h>
#include <Windowsx.h>
#include "Shapes.h"

#include "MainView.h"
#include "motor.h"
#include "gcode.h"
#include "CNC.h"
#include "Simulator.h"
#include "Parser.h"
#include "ManualMode.h"
#include "..\CNC_Protocol.h"


// Global Variables:
HWND g_hMainWindow = NULL;				// Main window handle
tMetaData g_MetaData;

TCHAR szTitle[MAX_PATH];				// The title bar text
TCHAR szWindowClass[MAX_PATH];			// the main window class name

// Forward declarations of functions included in this code module:
ATOM				MyRegisterClass(HINSTANCE hInstance);
BOOL				InitInstance(HINSTANCE, int);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	About(HWND, UINT, WPARAM, LPARAM);

HWND GetMainWindow()
{
	return g_hMainWindow;
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

	CNC_InitNetworkCom( );
	
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

	GetWindowRect(GetDesktopWindow(), &rc);
	x = (rc.right - rc.left) - (rc.bottom - rc.top);
	y = x * 2 / 3;
	AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW & ~WS_THICKFRAME, FALSE);

   g_hMainWindow = CreateWindow(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW & ~WS_THICKFRAME,
	   0, 0, x, y, NULL, NULL, hInstance, NULL);
      //CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, NULL, NULL, hInstance, NULL);
  
   if (!g_hMainWindow)
   {
      return FALSE;
   }

   ShowWindow(g_hMainWindow, nCmdShow);
   UpdateWindow(g_hMainWindow);
   //SetWindowPos(g_hMainWindow, NULL, 0, 0, 0, 0, SWP_SHOWWINDOW | SWP_NOSIZE | SWP_NOZORDER | SWP_DRAWFRAME);
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

BOOL CALLBACK CalibrationProc(HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam)
{
	tStatus status;
	static unsigned long flags = 0;

	switch (message)
	{
	case WM_INITDIALOG:
		EnableWindow(GetDlgItem(hWnd, IDOK), flags != 0);
		Button_SetCheck(GetDlgItem(hWnd, IDC_CALIBRATE_X), flags & CMD_FLAG_CALIBRATION_X );
		Button_SetCheck(GetDlgItem(hWnd, IDC_CALIBRATE_Y), flags & CMD_FLAG_CALIBRATION_Y );
		Button_SetCheck(GetDlgItem(hWnd, IDC_CALIBRATE_Z), flags & CMD_FLAG_CALIBRATION_Z );
		return TRUE;
		break;

	case WM_CLOSE:
		break;

	case WM_COMMAND:
		if (HIWORD(wParam) == BN_CLICKED)
		{
			switch (LOWORD(wParam))
			{
			// Update options based on new state of those items
			case IDC_CALIBRATE_X:
			case IDC_CALIBRATE_Y:
			case IDC_CALIBRATE_Z:
				flags = 0;
				flags |= Button_GetCheck(GetDlgItem(hWnd, IDC_CALIBRATE_X)) ? CMD_FLAG_CALIBRATION_X : 0;
				flags |= Button_GetCheck(GetDlgItem(hWnd, IDC_CALIBRATE_Y)) ? CMD_FLAG_CALIBRATION_Y : 0;
				flags |= Button_GetCheck(GetDlgItem(hWnd, IDC_CALIBRATE_Z)) ? CMD_FLAG_CALIBRATION_Z : 0;
				EnableWindow(GetDlgItem(hWnd, IDOK), flags != 0);
				break;

			case IDOK:
				status = CNC_Calibrate(flags);
				if (status != retSuccess)
				{
					MessageBoxA(hWnd, "Calibration command failed.", "CNC", MB_ICONERROR);
				}

				// Fallthough
			case IDCANCEL:
				EndDialog(hWnd, wParam);
				return TRUE;
			}
		}
	}
	return FALSE;
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
			if (((CNC_GetState() & CNC_STATE_IDLE) == 0 ) || (( CNC_GetState() & CNC_STATE_ERROR_MASK ) != 0 ))
			{
				MessageBoxA(hWnd, "Calibration cannot be performed when the machine is not idle or is in error state.", "CNC", MB_ICONERROR);
			}
			else
			{
				DialogBox(NULL,
					MAKEINTRESOURCE(IDD_CALIBRATION),
					hWnd,
					(DLGPROC)CalibrationProc);
			}
			break;

		case IDM_MACHINE_REBOOT:
			if (MessageBoxA(hWnd, "The reboot will reset the machine's position and the calibration state. Are you sure?", "CNC", MB_YESNO | MB_ICONEXCLAMATION) == IDYES)
			{
				if ((CNC_GetState() & CNC_STATE_IDLE) != 0 ||
					MessageBoxA(hWnd, "THE MACHINE IS NOT IDLE. Reboot will abort in an unknown state. Are you really sure?", "CNC", MB_YESNOCANCEL | MB_ICONERROR) == IDYES)
				{
					CNC_Reboot();
				}
			}
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
			DialogBox(NULL, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
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
