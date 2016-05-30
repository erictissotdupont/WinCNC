// CNC.cpp : Defines the entry point for the application.
//

#include "CNC.h"
#include <Commdlg.h>
#include "BasicShapes.h"

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
//
tMetaData g_MetaData;

t3DPoint g_displayPos;
long g_actualX;
long g_actualY;
long g_actualZ;
DWORD g_LastActualPos = 0;
unsigned short g_errorState = 0;

int g_debug[4];

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

void UpdatePosition( HWND hWnd, char* str )
{
	int x, y, z, s;	
	char* pt;
	unsigned long gotWhat = 0;

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

	if (gotWhat & 0x08)
	{
		g_errorState = s;
	}
	
	if (gotWhat & 0x07)
	{
		g_actualX = x;
		g_actualY = y;
		g_actualZ = z;
		g_LastActualPos = GetTickCount();

		stepToPos(x, y, z, &g_displayPos);
	}

	InvalidateRgn(hWnd, NULL, false);
}

void OnSocketEvent(CNC_SOCKET_EVENT event, PVOID param)
{
	switch (event)
	{
	case CNC_DISCONNECTED:
		InvalidateRgn(hMainWindow, NULL, false);
		break;

	case CNC_CONNECTED:
		InvalidateRgn(hMainWindow, NULL, false);
		SetTimer(hMainWindow, 1, 220, NULL);
		PostMessage(hMainWindow, WM_CHECK_INITIAL_STATUS, 0, 0);
		break;
	case CNC_RESPONSE:
		UpdatePosition(hMainWindow, (char*)param);
		break;
	}
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

	// 1/16 step - 400 steps - 2.8in per turn = 0.0004375 per step
	// Error of 0.32% (too far) = 0.0004393

	initAxis(0, 0.0004389); // X
	initAxis(1, 0.0004389); // Y
	initAxis(2, 0.0003125); // Z - 1/4 step - 400 steps - 0.5in per turn

	initSpindle();

	initSocketCom(OnSocketEvent);

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
   hInst = hInstance; // Store instance handle in our global variable

   hMainWindow = CreateWindow(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
      CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, NULL, NULL, hInstance, NULL);

   if (!hMainWindow)
   {
      return FALSE;
   }

   ShowWindow(hMainWindow, nCmdShow);
   UpdateWindow(hMainWindow);

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


void OnRunGCode(HWND hWnd,BOOL bSimulate)
{
	WCHAR szFile[260];       // buffer for file name
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
		if (bSimulate)
		{
			g_MetaData.blockX = 0;
			g_MetaData.blockY = 0;
			g_MetaData.blockZ = 0;
			g_MetaData.offsetX = 0;
			g_MetaData.offsetY = 0;
			g_MetaData.offsetZ = 0;
			g_MetaData.toolRadius = 0;
			g_MetaData.toolHeight = 0;
			g_MetaData.gotWhatTool = 0;
			g_MetaData.gotWhatStart = 0;
			g_MetaData.gotWhatBlock = 0;

			ParseGCodeFile(hWnd, szFile, preParse);

			init3DView(0.005f);

			setSimulationMode(buildPath);
		}
		
		ParseGCodeFile( hWnd, szFile, doGcode );
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
			OnRunGCode(hWnd, false);
			break;
		case IDM_SIMULATE_GCODE:
			OnRunGCode(hWnd, true);
			break;

		case IDM_BASIC_SHAPE:
			BasicShapes(hWnd);
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
		if (CheckStatus(true) == retSuccess)
		{
			if (g_errorState)
			{
				WCHAR msg[MAX_PATH];
				wsprintf(msg, L"CNC in error state 0x%02X (", g_errorState);
				if (g_errorState & ERROR_LIMIT) wcscat_s(msg, MAX_PATH, L" Limit");
				if( g_errorState & ERROR_NUMBER ) wcscat_s(msg, MAX_PATH, L" Number");
				if( g_errorState & ERROR_SYNTAX ) wcscat_s(msg, MAX_PATH, L" Syntax");
				if( g_errorState & ERROR_MATH   ) wcscat_s(msg, MAX_PATH, L" Math");
				if( g_errorState & ERROR_COMM   ) wcscat_s(msg, MAX_PATH, L" Comm");
				wcscat_s(msg, MAX_PATH, L" ) \r\nClear error state?");

				if (MessageBox(hMainWindow,
					msg, L"Error", MB_YESNO | MB_ICONERROR) == IDYES)
				{
					ClearCNCError();
					ResetCNCPosition();
				}
			}
			else if (g_actualX != 0 || g_actualY != 0 || g_actualZ != 0)
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
			}
		}
		else
		{
			MessageBox(hMainWindow, L"Unable to fetch CNC status.", L"Error", MB_ICONERROR);
		}
		break;

	case WM_UPDATE_POSITION:
		getCurPos(&g_displayPos);
		InvalidateRgn(hWnd, NULL, false);
		break;

	case WM_DESTROY:
		PostQuitMessage(0);
		break;

	case WM_KEYDOWN : 
		OnKey(wParam);
		break;

	case WM_TIMER :
		if (CheckStatus(false) != retSuccess)
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
