#include "CNC.h"
#include "resource.h"
#include "geometry.h"
#include "gcode.h"
#include "socket.h"
#include "motor.h"
#include "Shapes.h"

#include <wchar.h>
#include <mmsystem.h>
#include <psapi.h>
#include <windowsx.h>

DWORD pathSteps;
DWORD maxPathSteps;
t3DPoint *path;

extern HWND hMainWindow;

POINT O = { 50, 500 };

t2DPoint vuX = { 100, 0 };
t2DPoint vuY = { 60, -40 };
t2DPoint vuZ = { 0, -100 };

extern t3DPoint g_displayPos;
extern int g_debug[4];

#define VIEW_MARGIN			10
#define VIEW_STATUS_FONT	L"Arial"
#define VIEW_POSITION_FONT	L"Courier New"

void OnPaint(HWND hWnd)
{
	RECT view;
	RECT rect;
	HFONT font;
	CHAR str[100];
	HDC hdcMem;
	HBITMAP hbmMem;
	HANDLE hOld;
	PAINTSTRUCT ps;
	HDC hdc;
	int height,width;

	hdc = BeginPaint(hWnd, &ps);

	// Get the size of our client view for later
	GetClientRect(hWnd, &view);
	width = view.right - view.left;
	height = view.bottom - view.top;

	// Create an off-screen DC for double-buffering
	hdcMem = CreateCompatibleDC(hdc);
	hbmMem = CreateCompatibleBitmap(hdc, width, height);
	hOld = SelectObject(hdcMem, hbmMem);
	
	// This comes black by default. Clear the memory DC with a white brush.
	HBRUSH hBrush = CreateSolidBrush(GetBkColor(hdcMem));
	FillRect(hdcMem, &view, hBrush);

	// TOP banner : Show status of the CNC communication pipe
	int statusHeight = (height - (VIEW_MARGIN * 2)) / 16;
	rect.left = view.left + VIEW_MARGIN;
	rect.right = rect.left + width - VIEW_MARGIN * 2;
	rect.top = view.top + VIEW_MARGIN;
	rect.bottom = rect.top + statusHeight;
	font = CreateFont(
		statusHeight, 0, 0, 0, 
		FW_REGULAR, false, false, false, 
		DEFAULT_CHARSET, 
		OUT_DEFAULT_PRECIS, 
		CLIP_DEFAULT_PRECIS, 
		DEFAULT_QUALITY, 
		DEFAULT_PITCH, 
		VIEW_STATUS_FONT);
	SelectObject(hdcMem, font);
	getSocketStatusString(str, sizeof(str));
	DrawTextA(hdcMem, str, -1, &rect, 0);
	DeleteObject(font);

	// Upper Right corner : Real position of CNC 
	int positionHeight = (height - (VIEW_MARGIN * 3) - statusHeight ) / 2;
	rect.left = view.left + VIEW_MARGIN;
	rect.right = rect.left + ( width - ( VIEW_MARGIN * 3 )) / 2 ;
	rect.top = view.top + VIEW_MARGIN * 2 + statusHeight;
	rect.bottom = rect.top + positionHeight;
	font = CreateFont(
		positionHeight/3, 0, 0, 0, 
		FW_REGULAR, false, false, false, 
		DEFAULT_CHARSET, 
		OUT_DEFAULT_PRECIS, 
		CLIP_DEFAULT_PRECIS, 
		DEFAULT_QUALITY, 
		DEFAULT_PITCH,
		VIEW_POSITION_FONT );
	SelectObject(hdcMem, font);
	//swprintf(str, 100, L"X:%.4f\r\nY:%.4f\r\nZ:%.4f",
	sprintf_s(str, sizeof(str), "X:%.3f\r\nY:%.3f\r\nZ:%.3f",
		g_displayPos.x,
		g_displayPos.y,
		g_displayPos.z);
	DrawTextA(hdcMem, str, -1, &rect, 0);
	DeleteObject(font);

	// Lower right corner : Debug information
	int debugHeight = (height - (VIEW_MARGIN * 3) - statusHeight) / 2;
	rect.left = view.left + VIEW_MARGIN;
	rect.right = rect.left + (width - (VIEW_MARGIN * 3)) / 2;
	rect.top = view.top + VIEW_MARGIN * 2 + statusHeight + positionHeight;
	rect.bottom = rect.top + debugHeight;
	font = CreateFont(
		positionHeight / 10, 0, 0, 0,
		FW_REGULAR, false, false, false,
		DEFAULT_CHARSET,
		OUT_DEFAULT_PRECIS,
		CLIP_DEFAULT_PRECIS,
		DEFAULT_QUALITY,
		DEFAULT_PITCH,
		VIEW_POSITION_FONT);
	SelectObject(hdcMem, font);

	long xInPipe = 0, yInPipe = 0, zInPipe = 0;
	t3DPoint current, inPipe;
	getCurPos(&current);
	stepToPos(xInPipe, yInPipe, zInPipe, &inPipe);
	current.x -= inPipe.x;
	current.y -= inPipe.y;
	current.z -= inPipe.z;

	sprintf_s(str, sizeof(str), "X:%.3f Y:%.3f Z:%.3f\r\nA:%d\r\nB:%d\r\nC:%d\r\nD:%d",
		current.x,
		current.y,
		current.z,
		g_debug[0],
		g_debug[1],
		g_debug[2],
		g_debug[3]);
	DrawTextA(hdcMem, str, -1, &rect, 0);
	DeleteObject(font);

	// Transfer the off-screen DC to the screen
	BitBlt(hdc, 0, 0, width, height, hdcMem, 0, 0, SRCCOPY);

	// Free-up the off-screen DC
	SelectObject(hdcMem, hOld);
	DeleteObject(hbmMem);
	DeleteObject(hBrush);
	DeleteDC(hdcMem);
	EndPaint(hWnd, &ps);
}

typedef struct
{
	DWORD id;
	DWORD dx;
	DWORD dy;
	float res;
	DWORD cbAlt;
	float originX;
	float originY;
} header_t;

double g_res;
float *g_alt = NULL;
header_t *g_header = NULL;

WCHAR g_szAltFileName[MAX_PATH];
HANDLE g_hFileChangeEvent = INVALID_HANDLE_VALUE;

typedef struct {
	int dx;
	int dy;
} g_toolShape_t;
g_toolShape_t* g_toolShape = NULL;
DWORD iToolPoints = 0;

#define VIEWER_EXE_NAME	L"Viewer.exe"

bool start3DViewer( )
{
	STARTUPINFO si;
	PROCESS_INFORMATION pi;
	WCHAR szFileName[MAX_PATH];
	WCHAR *pt;
	DWORD pathLen;

	// GetProcessImageFileNameW returns the device form of the path \\device\\volumexxx
	// which isn't supported by CreateProcess( ) (it wants the drive letter)
	//pathLen = GetProcessImageFileNameW(GetCurrentProcess(), szFileName, MAX_PATH);

	pathLen = GetModuleFileNameW(NULL, szFileName, MAX_PATH);
	if(pathLen == 0)
	{
		return false;
	}
	pt = szFileName + pathLen;
	while (*pt != L'\\' && pt >= szFileName) pt--;
	pt++;
	*pt = 0;

	// This to make sure Viewer.exe finds the file Viewer.fx
	SetCurrentDirectoryW(szFileName);

	wcscpy_s(pt, MAX_PATH - (pt - szFileName), VIEWER_EXE_NAME );

	memset(&pi, 0x00, sizeof(pi));
	memset(&si, 0x00, sizeof(si));
	si.cb = sizeof(STARTUPINFO);
	GetStartupInfo(&si);

	g_hFileChangeEvent = CreateEvent(NULL, FALSE, FALSE, L"Local\\AltFileChangeEvent");

	WCHAR cmdLine[MAX_PATH];
	wcscpy_s(cmdLine, MAX_PATH, VIEWER_EXE_NAME);

	if (!CreateProcess(szFileName, cmdLine, NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi))
	{
		WCHAR msg[MAX_PATH + 80];
		wsprintf(msg, L"Unable to open 3D viewer.\r\n%s\r\nReason %d.\r\n", szFileName, GetLastError());
		MessageBox(NULL, msg, L"CNC", MB_OK);
		return false;
	}
	CloseHandle(pi.hProcess);
	CloseHandle(pi.hThread);
	return true;
}


extern tMetaData g_MetaData;

void resetBlockSurface()
{
	if (g_alt && g_header)
	{
		DWORD countX = g_header->dx;
		DWORD countY = g_header->dy;

		// Set the height of the block for all the vertexes on the surface
		for (DWORD x = 0; x < countX; x++) for (DWORD y = 0; y < countY; y++)
		{
			g_alt[x*countY + y] = (float)(g_MetaData.blockZ - g_MetaData.offsetZ);
		}

		// Set the 4 edges of the array to the zero offset so that the edges of
		// the block will be rendered. There is a small vertical line artifact 
		// when the tool crosses the edge of the block but it's acceptable.
		for (DWORD x = 0; x < countX; x++)
		{
			g_alt[x*countY + countY - 1] = (float)-g_MetaData.offsetZ;
			g_alt[x*countY] = (float)-g_MetaData.offsetZ;
		}
		for (DWORD y = 0; y < countY; y++)
		{
			g_alt[y] = (float)-g_MetaData.offsetZ;
			g_alt[(countX - 1)*countY + y] = (float)-g_MetaData.offsetZ;
		}

		SetEvent(g_hFileChangeEvent);
	}
}

// 96MB is enough for 24x24in at 0.005 resolution (float)
#define SIM_MAX_SIZE_BYTES		(96*1024*1024)
#define SIM_RESOLUTION_MIN		0.0001f	// 0.0254mm
#define SIM_RESOLUTION_MAX		0.01f	// 0.254mm

bool init3DView(float x, float y)
{
	HANDLE hMapFile;
	DWORD countX, countY;
	bool bAlreadyExists = false;

	g_res = ( x + y ) / 4000.0f;

	if(g_res < SIM_RESOLUTION_MIN) g_res = SIM_RESOLUTION_MIN;
	if(g_res > SIM_RESOLUTION_MAX) g_res = SIM_RESOLUTION_MAX;
	
	// Calculate the size of the altitude matrix
	countX = 1 + (DWORD)(x / g_res);
	countY = 1 + (DWORD)(y / g_res);

	// Check that it's not too big (max size is abitrary)
	if((countX * countY * sizeof(float)) > SIM_MAX_SIZE_BYTES )
	{
		return false;
	}

	// First time, create the mapped memory
	if (g_header == NULL)
	{
		DWORD cbFileSize = SIM_MAX_SIZE_BYTES + sizeof(header_t);

		hMapFile = CreateFileMapping(
			INVALID_HANDLE_VALUE,       // use paging file
			NULL,                       // default security
			PAGE_READWRITE,             // read/write access
			0,                          // maximum object size (high-order DWORD)
			cbFileSize,					// maximum object size (low-order DWORD)
			L"CncAltSimulationData");   // name of mapping object

		if (hMapFile == NULL)
		{
			return false;
		}
		else
		{
			bAlreadyExists = GetLastError() == ERROR_ALREADY_EXISTS;

			g_header = (header_t*)MapViewOfFile(hMapFile,   // handle to map object
				FILE_MAP_ALL_ACCESS, // read/write permission
				0,
				0,
				cbFileSize);

			if (g_header == NULL)
			{
				return false;
			}
			else
			{
				g_alt = (float*)(g_header + 1);
			}
		}
	}

	if (bAlreadyExists)
	{
		if ((g_header->id != 0x00010002) ||
			(g_header->dx != countX) ||
			(g_header->dy != countY) ||
			(g_header->res = (float)g_res))
		{
			MessageBoxA(NULL, "Simulation block header mismatch error", "3D simulator", MB_ICONERROR);
		}
		//g_header->cbAlt = SIM_MAX_SIZE_BYTES;
		//g_header->originX = -g_MetaData.offsetX;
		//g_header->originY = -g_MetaData.offsetY;
	}
	else
	{
		g_header->id = 0x00010002;
		g_header->dx = countX;
		g_header->dy = countY;
		g_header->res = (float)g_res;
		g_header->cbAlt = SIM_MAX_SIZE_BYTES;
		g_header->originX = -g_MetaData.offsetX;
		g_header->originY = -g_MetaData.offsetY;

		resetBlockSurface();
	}
	return true;
}

void initToolShape(double radius)
{
	// Fill a matrix that approximates the points the tool will remove over
	// the surface. This is an optimization to avoid calculating all the points
	// contained within the tool for every position of the tool motion.
	//
	int iRtool = (int)(radius / g_res) + 1;

	iToolPoints = 0;
	if (g_toolShape) free(g_toolShape);
	g_toolShape = (g_toolShape_t*)malloc(iRtool*iRtool*sizeof(g_toolShape_t));

	for (int i = 0; i < iRtool; i++) for (int j = 0; j < iRtool; j++)
	{
		double x = i*g_res;
		double y = j*g_res;
		if (sqrt(x*x + y*y) <= g_MetaData.toolRadius)
		{
			g_toolShape[iToolPoints].dx = i;
			g_toolShape[iToolPoints].dy = j;
			iToolPoints++;
		}
	}
}

void update3DView()
{
	if (g_alt)
	{
		SetEvent(g_hFileChangeEvent);
	}
}

inline void toolAt(long x, long y, float z)
{
	if (x >= 0 && y >= 0 && x<(long)g_header->dx && y<(long)g_header->dy)
	{
		float* point = &g_alt[x * g_header->dy + y];
		if (*point > z)
		{
			*point = z;
		}
	}
}

tStatus buildPath(t3DPoint Start, t3DPoint End, long d )
{
	static DWORD prevTime = 0;
	t3DPoint p;
	t3DPoint v = { End.x - Start.x, End.y - Start.y, End.z - Start.z };

	double l = vector3DLength(v);
	unsigned long step = (unsigned long)(l / g_res);

	for (unsigned long n = 0; n <= step; n++)
	{
		p.x = Start.x + (v.x * n) / step;
		p.y = Start.y + (v.y * n) / step;
		p.z = Start.z + (v.z * n) / step;

		DWORD iX = (DWORD)((p.x - g_MetaData.offsetX) / g_res);
		DWORD iY = (DWORD)((p.y - g_MetaData.offsetY) / g_res);

		for (DWORD i = 0; i < iToolPoints; i++)
		{
			toolAt(iX + g_toolShape[i].dx, iY + g_toolShape[i].dy, (float)p.z);
			toolAt(iX - g_toolShape[i].dx, iY + g_toolShape[i].dy, (float)p.z);
			toolAt(iX - g_toolShape[i].dx, iY - g_toolShape[i].dy, (float)p.z);
			toolAt(iX + g_toolShape[i].dx, iY - g_toolShape[i].dy, (float)p.z);
		}
		DWORD current = timeGetTime();
		if (current > prevTime + 50 || step == n )
		{
			PostMessage(hMainWindow, WM_UPDATE_POSITION, 0, 0);
			prevTime = current;
		}
		// Yield
		Sleep(0);
		//if (d && ((n % 5) == 1)) Sleep(1);
	}

	/*
	if (pathSteps >= maxPathSteps)
	{
	// Increase the size of our buffer
	maxPathSteps = (maxPathSteps + 2) * 2;
	t3DPoint* newPath = (t3DPoint*)malloc(sizeof(t3DPoint)*maxPathSteps);
	if (path && pathSteps )
	{
	memcpy(newPath, path, pathSteps * sizeof(t3DPoint));
	free(path);
	}
	path = newPath;
	}

	path[pathSteps++] = P;
	*/

	// Normal speed
	//Sleep( d / 1000 );

	// 3x faster
	//Sleep(d / 3000);

	return retSuccess;
}


void _3DSettingsInit(HWND hWnd)
{
	int i;
	HWND hItem;

	hItem = GetDlgItem(hWnd, IDC_3D_TOOL_RADIUS);
	for (i = 0; i < ITEM_CNT(TOOL_SIZES); i++) ComboBox_AddString(hItem, TOOL_SIZES[i].str);

	HKEY hKey;
	if (RegOpenKey(HKEY_CURRENT_USER, L"SOFTWARE", &hKey) == ERROR_SUCCESS)
	{
		DWORD cbData = sizeof(g_MetaData);
		if (RegGetValue(hKey, L"WinCNC", L"3Dsettings", RRF_RT_REG_BINARY, NULL, &g_MetaData, &cbData) != ERROR_SUCCESS)
		{
			// Pre-populate with some defaults
			g_MetaData.blockX = 5;
			g_MetaData.blockY = 5;
			g_MetaData.blockZ = 0.5;
			g_MetaData.offsetX = -1;
			g_MetaData.offsetY = -1;
			g_MetaData.offsetZ = 0.5;	// Make it same as blockZ to when tool starts flush with top of block
			g_MetaData.toolRadius = 0.125;	// 1/4
			g_MetaData.toolHeight = 0.5;
			g_MetaData.gotWhatTool = 1;
			g_MetaData.gotWhatStart = 1;
			g_MetaData.gotWhatBlock = 1;
		}
		RegCloseKey(hKey);
	}
}


void _3DSettingsSave()
{
	HKEY hKey;
	if (RegCreateKey(HKEY_CURRENT_USER, L"SOFTWARE\\WinCNC", &hKey) == ERROR_SUCCESS)
	{
		RegSetValueEx(hKey, L"3Dsettings", 0, REG_BINARY, (BYTE*)&g_MetaData, sizeof(g_MetaData));
		RegCloseKey(hKey);
	}
}

UINT _3DSettingsGetSet(BOOL get, HWND hWnd)
{
	ShapeGetSetFloat(hWnd, IDC_3D_X, get, &g_MetaData.blockX);
	ShapeGetSetFloat(hWnd, IDC_3D_Y, get, &g_MetaData.blockY);
	ShapeGetSetFloat(hWnd, IDC_3D_Z, get, &g_MetaData.blockZ);

	ShapeGetSetFloat(hWnd, IDC_3D_OFFSET_X, get, &g_MetaData.offsetX);
	ShapeGetSetFloat(hWnd, IDC_3D_OFFSET_Y, get, &g_MetaData.offsetY);
	ShapeGetSetFloat(hWnd, IDC_3D_OFFSET_Z, get, &g_MetaData.offsetZ);

	ShapeGetSetFloat(hWnd, IDC_3D_TOOL_HEIGHT, get, &g_MetaData.toolHeight);

	ShapeGetSetToolSize(hWnd, IDC_3D_TOOL_RADIUS, get, &g_MetaData.toolRadius);

	return 0;
}

void _3DLaunchSimulatorApp()
{
	// This is to get the metadata from the G-Code file
	//ParseGCodeFile(hWnd, szFile, preParse);

	init3DView(g_MetaData.blockX, g_MetaData.blockY);
	initToolShape(g_MetaData.toolRadius);

	setSimulationMode(buildPath);

	// This launches the 3D viewer window
	start3DViewer();
}


BOOL CALLBACK _3DSettingsProc(HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam)
{
	switch (message)
	{
	case WM_INITDIALOG:
		_3DSettingsInit(hWnd);
		_3DSettingsGetSet(FALSE, hWnd);
		return TRUE;
		break;

	case WM_CLOSE:
		_3DSettingsGetSet(TRUE, hWnd);
		_3DSettingsSave( );
		break;

	case WM_COMMAND:
		switch( HIWORD(wParam))
		{
		case BN_CLICKED :
			switch (LOWORD(wParam))
			{
			case IDC_3D_SIMULATOR :
				_3DSettingsGetSet(TRUE, hWnd);
				_3DSettingsSave();
				_3DLaunchSimulatorApp();
				break;

			case IDOK:
			case IDCANCEL:
				EndDialog(hWnd, wParam);
				return TRUE;
			}
			break;
		case CBN_SELCHANGE :
			switch (LOWORD(wParam))
			{
			case IDC_3D_TOOL_RADIUS :
				_3DSettingsGetSet(TRUE, hWnd);
				initToolShape(g_MetaData.toolRadius);
				break;
			}
		}
	}
	return FALSE;
}


void Start3DSimulator(HWND hWnd)
{
	DialogBox(NULL,
		MAKEINTRESOURCE(IDD_3D_SETTINGS),
		hWnd,
		(DLGPROC)_3DSettingsProc);
}
