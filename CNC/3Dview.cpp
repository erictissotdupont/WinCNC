#include "CNC.h"
#include "geometry.h"
#include "gcode.h"
#include <wchar.h>
#include <mmsystem.h>

DWORD pathSteps;
DWORD maxPathSteps;
t3DPoint *path;

extern HWND hMainWindow;

POINT O = { 50, 500 };

t2DPoint vuX = { 100, 0 };
t2DPoint vuY = { 60, -40 };
t2DPoint vuZ = { 0, -100 };

extern t3DPoint g_displayPos;

void MoveTo3D(HDC hdc, t3DPoint p)
{
	int x,y;
	x = O.x + (LONG)(p.x * vuX.x + p.y * vuY.x + p.z * vuZ.x);
	y = O.y + (LONG)(p.x * vuX.y + p.y * vuY.y + p.z * vuZ.y);
	MoveToEx(hdc, x, y, NULL);
}

void LineTo3D(HDC hdc, t3DPoint p)
{
	int x, y;
	x = O.x + (LONG)(p.x * vuX.x + p.y * vuY.x + p.z * vuZ.x);
	y = O.y + (LONG)(p.x * vuX.y + p.y * vuY.y + p.z * vuZ.y);
	LineTo(hdc, x, y);
}

const t3DPoint   O3D = { 0, 0, 0 };
const t3DPoint vu3DX = { 1, 0, 0 };
const t3DPoint vu3DY = { 0, 1, 0 };
const t3DPoint vu3DZ = { 0, 0, 1 };

#define MARGIN 10
#define STATUS_WIDTH	800
#define STATUS_HEIGHT	200

void OnPaint(HWND hWnd)
{
	RECT view;
	RECT rect;
	HFONT font;
	WCHAR str[100];
	HDC hdcMem;
	HBITMAP hbmMem;
	HANDLE hOld;
	PAINTSTRUCT ps;
	HDC hdc;
	int height,width;

	hdc = BeginPaint(hWnd, &ps);

	GetClientRect(hWnd, &view);
	width = view.right - view.left;
	height = view.bottom - view.top;

	// Create an off-screen DC for double-buffering
	hdcMem = CreateCompatibleDC(hdc);
	hbmMem = CreateCompatibleBitmap(hdc, width, height);
	hOld = SelectObject(hdcMem, hbmMem);
	// Comes black by default. Clear the memory DC with a white brush.
	HBRUSH hBrush = CreateSolidBrush(GetBkColor(hdcMem));
	FillRect(hdcMem, &view, hBrush);

	int statusHeight = (height - (MARGIN * 2)) / 2;

	rect.left = view.left + MARGIN;
	rect.right = rect.left + STATUS_WIDTH;
	rect.top = view.top + MARGIN;
	rect.bottom = rect.top + statusHeight;

	font = CreateFont(statusHeight/3, 0, 0, 0, FW_REGULAR, false, false, false, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY, DEFAULT_PITCH, L"Arial");
	SelectObject(hdcMem, font);

	swprintf(str, 100, L"X:%.4f\r\nY:%.4f\r\nZ:%.4f",
		g_displayPos.x,
		g_displayPos.y,
		g_displayPos.z);

	DrawText(hdcMem, str, -1, &rect, 0);

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
} header_t;

float g_res;
float *g_alt = NULL;
header_t *g_header = NULL;
DWORD g_cbAlt = 0;
HANDLE g_hAltFile = NULL;
HANDLE g_hMapFile = NULL;

DWORD countX,countY;

struct {
	int dx;
	int dy;
} toolShape[1000];

DWORD iToolPoints = 0;

void startViewer(const WCHAR* szFilePath)
{
	STARTUPINFO si;
	PROCESS_INFORMATION pi;

	GetStartupInfo(&si);

	SetCurrentDirectory(L"C:\\Users\\Eric\\Documents\\GitHub\\WinCnc\\Debug");

	WCHAR curDir[MAX_PATH];
	GetCurrentDirectory(MAX_PATH, curDir);

	WCHAR cmdLine[MAX_PATH];
	wcscpy_s(cmdLine, MAX_PATH, L"Viewer.exe ");
	wcscat_s(cmdLine, MAX_PATH, szFilePath);

	if (!CreateProcess(L"Viewer.exe", cmdLine, NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi))
	{
		WCHAR msg[MAX_PATH + 80];
		wsprintf(msg, L"Unable to open viewer. Reason %d.\r\n%s", GetLastError(), curDir);
		MessageBox(NULL, msg, L"CNC", MB_OK);
	}
}

void saveAltitude(WCHAR* szFilePath)
{
	HANDLE hFile;

	hFile = CreateFile(szFilePath, GENERIC_WRITE, FILE_SHARE_READ, NULL, CREATE_ALWAYS, 0, NULL);

	if (hFile != INVALID_HANDLE_VALUE)
	{
		DWORD dwWritten;
		WriteFile(hFile, g_header, sizeof(g_header), &dwWritten, NULL);
		WriteFile(hFile, g_alt, g_cbAlt, &dwWritten, NULL);
		CloseHandle(hFile);
	}
}

extern tMetaData g_MetaData;
const WCHAR szShareMemName[] = L"CNCFileMappingObject";

void init3DView( float res )
{
	volatile int err;

	g_res = res;

	countX = 1 + (DWORD)(g_MetaData.blockX / res);
	countY = 1 + (DWORD)(g_MetaData.blockY / res);

	g_cbAlt = sizeof(float) * countX * countY;
	DWORD cbMap = g_cbAlt + sizeof(header_t);

	g_hMapFile = CreateFileMapping(
		INVALID_HANDLE_VALUE,    // use paging file
		NULL,                    // default security
		PAGE_READWRITE,          // read/write access
		0,                       // maximum object size (high-order DWORD)
		cbMap,					 // maximum object size (low-order DWORD)
		szShareMemName);         // name of mapping object

	if (g_hMapFile == NULL)
	{
		err = GetLastError();
		return;
	}

	g_header = (header_t*)MapViewOfFile(g_hMapFile,   // handle to map object
		FILE_MAP_ALL_ACCESS, // read/write permission
		0,
		0,
		cbMap);

	if (g_header == NULL)
	{
		CloseHandle(g_hMapFile);
		return;
	}

	g_header->id = 0x00010002;
	g_header->dx = countX;
	g_header->dy = countY;
	g_header->res = g_res;
	g_header->cbAlt = g_cbAlt;

	g_alt = (float*)(g_header+1);

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

	// Fill a matrix that approximates the points the tool will remove over
	// the surface. This is an optimization to avoid calculating all the points
	// contained within the tool for every position of the tool motion.
	//
	int iRtool = (int)(g_MetaData.toolRadius / res);

	for (int i = 0; i <= iRtool; i++) for (int j = 0; j <= iRtool; j++)
	{
		float x = i*res;
		float y = j*res;
		if (sqrt(x*x + y*y) <= g_MetaData.toolRadius)
		{
			toolShape[iToolPoints].dx = i;
			toolShape[iToolPoints].dy = j;
			iToolPoints++;
		}
	}

	// This launches the 3D viewer window
	startViewer(szShareMemName);
}

void end3DView()
{
	WCHAR szFilePath[MAX_PATH];
	GetCurrentDirectory(MAX_PATH, szFilePath);
	wcscat_s(szFilePath, L"\\CNC_Result.dat");

	saveAltitude( szFilePath );

	UnmapViewOfFile(g_header);
	CloseHandle(g_hMapFile);

	return;
}

inline void toolAt(long x, long y, float z)
{
	if (x >= 0 && y >= 0 && x<(long)countX && y<(long)countY)
	{
		float* point = &g_alt[x * countY + y];
		if (*point > z)
		{
			*point = z;
		}
	}
}

tStatus buildPath(t3DPoint P, long x, long y, long z, long d, long s)
{
	static DWORD prevTime = 0;
	static t3DPoint p = { 0.0, 0.0, 0.0 };
	t3DPoint v = { P.x - p.x, P.y - p.y, P.z - p.z };

	double l = vector3DLength(v);
	unsigned long step = (unsigned long)(l / g_res);
	if (step == 0)
	{
		step = 1;
	}
	else
	{
		v.x = v.x / step;
		v.y = v.y / step;
		v.z = v.z / step;
	}

	for (unsigned long n = 0; n < step; n++)
	{
		p.x += v.x;
		p.y += v.y;
		p.z += v.z;

		DWORD iX = (DWORD)((p.x - g_MetaData.offsetX) / g_res);
		DWORD iY = (DWORD)((p.y - g_MetaData.offsetY) / g_res);

		for (DWORD i = 0; i < iToolPoints; i++)
		{
			toolAt(iX + toolShape[i].dx, iY + toolShape[i].dy, (float)p.z);
			if (i != 0)
			{
				toolAt(iX - toolShape[i].dx, iY + toolShape[i].dy, (float)p.z);
				toolAt(iX - toolShape[i].dx, iY - toolShape[i].dy, (float)p.z);
				toolAt(iX + toolShape[i].dx, iY - toolShape[i].dy, (float)p.z);
			}
		}
		DWORD current = timeGetTime();
		if (current > prevTime + 40)
		{
			PostMessage(hMainWindow, WM_UPDATE_POSITION, 0, 0);
			prevTime = current;
		}
	}
	p = P;

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
	// Sleep(d / 3000);

	return retSuccess;
}
