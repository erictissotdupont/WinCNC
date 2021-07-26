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

#define MAX_LOADSTRING 100

// Global Variables:
HWND hMainWindow = NULL;						// main window handle
HINSTANCE hInst;								// current instance
TCHAR szTitle[MAX_LOADSTRING];					// The title bar text
TCHAR szWindowClass[MAX_LOADSTRING];			// the main window class name

// Forward declarations of functions included in this code module:
ATOM				MyRegisterClass(HINSTANCE hInstance);
BOOL				InitInstance(HINSTANCE, int);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	About(HWND, UINT, WPARAM, LPARAM);

// Relative movement
#define RELMOVE "G91 G40 G1 "
// 1/256 of an inch.
#define SMALL_MOVE ".00390625"

// Information about the part being machined such as size, 
// tool position and tool size.
tMetaData g_MetaData;

// Current tool position to be displayed on the screen. This gets
// updated by the return of the "get status" commands
t3DPoint g_displayPos;
 
// The last motor step count received by the answer of the get status
// command ("S\n")
long g_actualX,g_actualY,g_actualZ;

// The value following the status information. This bitfield indicates
// various error condition of the CNC
unsigned short g_errorStatusFlags = 0;

// Definition of the bits in the value returned by CNC "Status" (S)
#define ERROR_LIMIT      0x0001
#define ERROR_NUMBER     0x0002
#define ERROR_SYNTAX     0x0004
#define ERROR_MATH       0x0008
#define ERROR_COMM       0x0010

// The values returned by the answer of the debug command
// Those are NOT fetched by the release of the code.
int g_debug[4];


tStatus GetCncStatus()
{
	if (!isCncConnected()) return retCncNotConnected;
	if (g_errorStatusFlags) return retCncError;
	if (getCncErrorCount() != 0 ) return retCncError;
	return retSuccess;
}

tStatus parseLine(char* cmd)
{
	tStatus ret = retSuccess;

	if (strstr(cmd, "RUN ") == cmd)
	{
		
	}
	else if (strstr(cmd, "EXP") == cmd)
	{
		char *fileName = cmd + 4;
		setExportFile(CreateFileA(fileName, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, 0, NULL));
	}
	else if (strstr(cmd, "POS") == cmd)
	{
		showDistanceInfo();
	}
	else if (strstr(cmd, "TST ") == cmd)
	{
		int i;
		char str[20];
		int step, axis, cycle;
		sscanf_s(cmd + 4, "%d %d %d", &axis, &step, &cycle);
		printf("Test motor %d\n", step);
		// testMotor( axis, step, cycle );

		sprintf_s(str, sizeof(str), "G10 G90 G1 F%d", step);
		doGcode(str);

		for (i = 0; i<cycle; i++)
		{
			sprintf_s(str, sizeof(str), "Z%.6f", ((rand() % 1000) / 100000.0));
			sprintf_s(str, sizeof(str), "Z%.6f", ((rand() % 1000) / 100000.0));
			sprintf_s(str, sizeof(str), "Z%.6f", ((rand() % 1000) / 100000.0));
			doGcode(str);
			doGcode("Z0");
		}

	}
	else if (strstr(cmd, "UNT") == cmd)
	{
		// unitTest();
	}
	else if (strstr(cmd, "MAN") == cmd)
	{
		/*
		int k;
		do
		{
			k = 0;
			if (getkey((char*)&k, sizeof(k)) > 0)
			{
				OnKey(k);
			}
			else
			{
				usleep(10000); // 10ms
			}
		} while (k != 27);
		*/
	}
	else if (strstr(cmd, "QUIT") == cmd)
	{
		ret = retQuit;
	}
	else
	{
		switch (ret = doGcode(cmd))
		{
		case retSuccess:
			// Success.
			break;
		default :
			printf("%S", GetCNCErrorString(ret));
			break;
		}
	}

	return ret;
}

void OnCncStatus( HWND hWnd, char* str )
{
	int x, y, z, s;	
	char* pt;
	unsigned long gotWhat = 0;

	x = y = z = s = 0;

	if (str)
	{
		pt = strchr(str, 'X');
		if (pt) if (sscanf_s(pt + 1, "%d", &x) == 1) gotWhat |= 0x01;
		pt = strchr(str, 'Y');
		if (pt) if (sscanf_s(pt + 1, "%d", &y) == 1) gotWhat |= 0x02;
		pt = strchr(str, 'Z');
		if (pt) if (sscanf_s(pt + 1, "%d", &z) == 1) gotWhat |= 0x04;
		pt = strchr(str, 'S');
		if (pt) if (sscanf_s(pt + 1, "%d", &s) == 1) gotWhat |= 0x08;

		pt = strchr(str, 'a');
		if (pt) sscanf_s(pt + 1, "%d", &g_debug[0]);
		pt = strchr(str, 'b');
		if (pt) sscanf_s(pt + 1, "%d", &g_debug[1]);
		pt = strchr(str, 'c');
		if (pt) sscanf_s(pt + 1, "%d", &g_debug[2]);
		pt = strchr(str, 'd');
		if (pt) sscanf_s(pt + 1, "%d", &g_debug[3]);
	}

	if (gotWhat & 0x08)
	{
		g_errorStatusFlags = s;
	}

	if (gotWhat & 0x01) g_actualX = x;
	if (gotWhat & 0x02) g_actualY = y;
	if (gotWhat & 0x04) g_actualZ = z;
	
	if (( gotWhat & 0x07 ) == 0x07 )
	{
		stepToPos(x, y, z, &g_displayPos);
	}

	// If we got some information, refresh the screen
	if( gotWhat ) PostMessage(hWnd, WM_REDRAW, 0, 0);
}

void OnConnected(PVOID param)
{
	if (param == NULL)
	{
		InvalidateRgn(hMainWindow, NULL, false);
	}
	else
	{
		InvalidateRgn(hMainWindow, NULL, false);
		SetTimer(hMainWindow, 1, 220, NULL);
		PostMessage(hMainWindow, WM_CHECK_INITIAL_STATUS, 0, 0);
	}
}

void OnResponse(PVOID param)
{
	OnCncStatus(hMainWindow, (char*)param);
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
	LoadString(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadString(hInstance, IDC_CNC, szWindowClass, MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	// Perform application initialization:
	if (!InitInstance (hInstance, nCmdShow))
	{
		return FALSE;
	}

	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_CNC));

	initSocketCom();
	registerSocketCallback(CNC_CONNECTED, OnConnected);
	registerSocketCallback(CNC_RESPONSE, OnResponse);

	motorInit();

	// 1/16 step - 400 steps - 2.8in per turn = 0.0004375 per step
	// Error of 0.32% (too far) = 0.0004393
	initAxis(0, 0.00043821); // X
	
	initAxis(1, 0.00049271); // Y

	initAxis(2, 0.0003925); // Z - 1/4 step - 400 steps - 0.5in per turn
	
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


void OnKey(int key)
{
	// printf( "OnKey( 0x%08X )\n", key );
	switch (key)
	{
	case VK_UP:
		doGcode(RELMOVE "X" SMALL_MOVE);
		break;
	case VK_DOWN:
		doGcode(RELMOVE "X-" SMALL_MOVE);
		break;
	case VK_RIGHT:
		doGcode(RELMOVE "Y-" SMALL_MOVE);
		break;
	case VK_LEFT:
		doGcode(RELMOVE "Y" SMALL_MOVE);
		break;

		/*
		case VK_PAGEUP:
		doGcode(RELMOVE"Z"SMALL_MOVE);
		break;
		case VK_PAGEDN:
		doGcode(RELMOVE"Z-"SMALL_MOVE);
		break;
		*/
	}
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
	tStatus status;
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

	case WM_CHECK_INITIAL_STATUS:

		status = CheckStatus(true);

		if( status == retCncError || g_errorStatusFlags )
		{
			WCHAR msg[MAX_PATH];
			wsprintf(msg, L"CNC in error state 0x%02X (", g_errorStatusFlags);
			if (g_errorStatusFlags & ERROR_LIMIT) wcscat_s(msg, MAX_PATH, L" Limit");
			if( g_errorStatusFlags & ERROR_NUMBER ) wcscat_s(msg, MAX_PATH, L" Number");
			if( g_errorStatusFlags & ERROR_SYNTAX ) wcscat_s(msg, MAX_PATH, L" Syntax");
			if( g_errorStatusFlags & ERROR_MATH   ) wcscat_s(msg, MAX_PATH, L" Math");
			if( g_errorStatusFlags & ERROR_COMM   ) wcscat_s(msg, MAX_PATH, L" Comm");
			wcscat_s(msg, MAX_PATH, L" ) \r\nClear error state?");

			if (MessageBox(hMainWindow,
				msg, L"Error", MB_YESNO | MB_ICONERROR) == IDYES)
			{
				ClearCNCError();
				ResetCNCPosition();
				CheckStatus(true);
				PostMessage(hWnd, WM_CHECK_INITIAL_STATUS, 0, 0);
			}
		}
		else if( status == retSuccess )
		{
			if (g_actualX != 0 || g_actualY != 0 || g_actualZ != 0)
			{
				if (MessageBox( hMainWindow, 
					            L"CNC is not at origin position.\r\n\r\nDo you want to reset the CNC? If you select 'No' the remote location will be used.", 
					            L"Warning", MB_YESNO | MB_ICONWARNING ) == IDYES)
				{
					ResetCNCPosition( );
				}
				else
				{
					resetMotorPosition(g_actualX, g_actualY, g_actualZ);
				}
				CheckStatus(true);
				PostMessage(hWnd, WM_CHECK_INITIAL_STATUS, 0, 0);
			}
		}
		else
		{
			WCHAR msg[MAX_PATH];
			wsprintf(msg, L"Unable to fetch CNC status. Reason: %s.", GetCNCErrorString(status));
			MessageBox(hMainWindow, msg, L"Error", MB_ICONERROR);
		}
		break;

	case WM_UPDATE_POSITION:
		getCurPos(&g_displayPos);
		InvalidateRgn(hWnd, NULL, false);
		update3DView();
		break;

	case WM_REDRAW:
		InvalidateRgn(hWnd, NULL, false);
		break;

	case WM_DESTROY:
		PostQuitMessage(0);
		break;

	case WM_KEYDOWN : 
		OnKey(wParam);
		break;

	case WM_TIMER :
		if( CheckStatus(false) != retSuccess )
		{
			InvalidateRgn(hWnd, NULL, false);
		}
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
