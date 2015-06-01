
#include "CNC.h"
#include "geometry.h"

DWORD pathSteps;
DWORD maxPathSteps;
t3DPoint *path;

POINT O = { 50, 500 };

t2DPoint vuX = { 100, 0 };
t2DPoint vuY = { 60, -40 };
t2DPoint vuZ = { 0, -100 };

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

void OnPaint(HWND hWnd, HDC hdc)
{
	MoveTo3D(hdc, O3D);
	LineTo3D(hdc, vu3DX);
	MoveTo3D(hdc, O3D);
	LineTo3D(hdc, vu3DY);
	MoveTo3D(hdc, O3D);
	LineTo3D(hdc, vu3DZ);
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
float g_szX,g_szY;
float g_oX, g_oY;
HANDLE g_hAltFile = NULL;
HANDLE g_hMapFile = NULL;

DWORD countX,countY;

struct {
	int dx;
	int dy;
} toolShape[10000];

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
	header_t h;
	HANDLE hFile;

	hFile = CreateFile(szFilePath, GENERIC_WRITE, FILE_SHARE_READ, NULL, CREATE_ALWAYS, 0, NULL);

	if (hFile != INVALID_HANDLE_VALUE)
	{
		DWORD dwWritten;
		WriteFile(hFile, g_header, sizeof(h), &dwWritten, NULL);
		WriteFile(hFile, g_alt, g_cbAlt, &dwWritten, NULL);
		CloseHandle(hFile);
	}
}

const WCHAR szShareMemName[] = L"CNCFileMappingObject";

void init3DView(float oX, float oY, float dX, float dY, float dZ, float res, float toolRadius)
{
	volatile int err;

	g_szX = dX;
	g_szY = dY;

	g_oX = oX;
	g_oY = oY;

	g_res = res;

	countX = 1 + (DWORD)(dX / res);
	countY = 1 + (DWORD)(dY / res);

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

	for (DWORD x = 0; x < countX; x++) for (DWORD y = 0; y < countY; y++)
	{
		g_alt[x*countY + y] = dZ;
	}

	int iRtool = (int)(toolRadius / res);

	for (int i = 0; i <= iRtool; i++) for (int j = 0; j <= iRtool; j++)
	{
		float x = i*res;
		float y = j*res;
		if (sqrt(x*x + y*y) <= toolRadius)
		{
			toolShape[iToolPoints].dx = i;
			toolShape[iToolPoints].dy = j;
			iToolPoints++;
		}
	}

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

void toolAt(int x, int y, float z)
{
	if (x >= 0 && y >= 0 && x<countX && y<countY)
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
	static t3DPoint p = { 0.0, 0.0, 0.0 };
	t3DPoint v = { P.x - p.x, P.y - p.y, P.z - p.z };

	float l = vector3DLength(v);
	int step = l / g_res;
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

	for (int n = 0; n < step; n++)
	{
		p.x += v.x;
		p.y += v.y;
		p.z += v.z;

		DWORD iX = (p.x - g_oX) / g_res;
		DWORD iY = (p.y - g_oY) / g_res;

		for (int i = 0; i < iToolPoints; i++)
		{
			toolAt(iX + toolShape[i].dx, iY + toolShape[i].dy, p.z);
			if (i != 0)
			{
				toolAt(iX - toolShape[i].dx, iY + toolShape[i].dy, p.z);
				toolAt(iX - toolShape[i].dx, iY - toolShape[i].dy, p.z);
				toolAt(iX + toolShape[i].dx, iY - toolShape[i].dy, p.z);
			}
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

	Sleep(200);

	return retSuccess;
}
