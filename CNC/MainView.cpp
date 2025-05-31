#include "Main.h"
#include "resource.h"
#include "geometry.h"
#include "gcode.h"
#include "CNC.h"
#include "motor.h"
#include "Shapes.h"

#include <wchar.h>
#include <mmsystem.h>
#include <psapi.h>
#include <windowsx.h>

#define VIEW_MARGIN			10
#define VIEW_STATUS_FONT	L"Arial"
#define VIEW_POSITION_FONT	L"Courier New"

void OnPaint(HWND hWnd)
{
	RECT view;
	RECT rect;
	HFONT font;
	CHAR str[1000];
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
	CNC_GetNetworkStatusString(str, sizeof(str));
	DrawTextA(hdcMem, str, -1, &rect, 0);
	DeleteObject(font);

	// Upper Right corner : Real position of CNC 
	int positionHeight = (height - (VIEW_MARGIN * 3) - statusHeight ) / 2;
	rect.left = view.left + VIEW_MARGIN;
	rect.right = rect.left + ( width - ( VIEW_MARGIN * 3 )) / 2 ;
	rect.top = view.top + VIEW_MARGIN * 2 + statusHeight;
	rect.bottom = rect.top + positionHeight;
	font = CreateFont(
		positionHeight / 3, 0, 0, 0, 
		FW_REGULAR, false, false, false, 
		DEFAULT_CHARSET, 
		OUT_DEFAULT_PRECIS, 
		CLIP_DEFAULT_PRECIS, 
		DEFAULT_QUALITY, 
		DEFAULT_PITCH,
		VIEW_POSITION_FONT );
	SelectObject(hdcMem, font);
	//swprintf(str, 100, L"X:%.4f\r\nY:%.4f\r\nZ:%.4f",

	t3DPoint pos;
	CNC_GetDisplayPosition(&pos);

	sprintf_s(str, sizeof(str), "X:%.4f\r\nY:%.4f\r\nZ:%.4f", 
		pos.x,
		pos.y, 
		pos.z );

	DrawTextA(hdcMem, str, -1, &rect, 0);
	DeleteObject(font);

	// Lower left corner : Debug information
	int debugHeight = (height - (VIEW_MARGIN * 3) - statusHeight) / 2;
	
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

	// Bottom Left : INFORMATION
	rect.left = view.left + VIEW_MARGIN;
	rect.right = rect.left + (width - (VIEW_MARGIN * 3)) / 2;
	rect.top = view.top + VIEW_MARGIN * 2 + statusHeight + positionHeight;
	rect.bottom = rect.top + debugHeight;
	CNC_GetStateString(str, sizeof(str), CNC_STATE_INFORMATION_MASK);
	DrawTextA(hdcMem, str, -1, &rect, 0);

	// Top right : WARNINGS
	rect.right = view.right - VIEW_MARGIN;
	rect.left = rect.right - (width - (VIEW_MARGIN * 3)) / 2;
	rect.top = view.top + VIEW_MARGIN * 2 + statusHeight;
	rect.bottom = rect.top + positionHeight;

	t3DPoint theoriCalPos;
	GetTheoricalPosition(&theoriCalPos);

#if 0 // Show the delta between the physical position and the theorical position x1000
	theoriCalPos.x -= pos.x;
	theoriCalPos.y -= pos.y;
	theoriCalPos.z -= pos.z;
	theoriCalPos.x *= 1000.0f;
	theoriCalPos.y *= 1000.0f;
	theoriCalPos.z *= 1000.0f;
#endif

	int n = sprintf_s(str, sizeof(str), "\r\nX:%.4f Y:%.4f Z:%.4f\r\n\r\n",
		theoriCalPos.x,
		theoriCalPos.y,
		theoriCalPos.z);

	CNC_GetStateString(str+n, sizeof(str)-n, CNC_STATE_WARNING_MASK);

	DrawTextA(hdcMem, str, -1, &rect, 0);

	// Bottom Right : ERRORS

	DeleteObject(font);
	font = CreateFont(
		positionHeight / 8, 0, 0, 0,
		FW_DEMIBOLD, false, false, false,
		DEFAULT_CHARSET,
		OUT_DEFAULT_PRECIS,
		CLIP_DEFAULT_PRECIS,
		DEFAULT_QUALITY,
		DEFAULT_PITCH,
		VIEW_POSITION_FONT);
	SelectObject(hdcMem, font);

	rect.right = view.right - VIEW_MARGIN;
	rect.left = rect.right - (width - (VIEW_MARGIN * 3)) / 2;
	rect.top = view.top + VIEW_MARGIN * 2 + statusHeight + positionHeight;
	rect.bottom = rect.top + debugHeight;
	CNC_GetStateString(str, sizeof(str), CNC_STATE_ERROR_MASK );
	SetTextColor(hdcMem, RGB(255, 0, 0));
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

