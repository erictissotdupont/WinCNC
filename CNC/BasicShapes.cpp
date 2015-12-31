#include "CNC.h"
#include "Resource.h"
#include "Windowsx.h"

#include "gcode.h"
#include "fileParser.h"

#define MAX_STR	80

typedef struct
{
	// General parameters
	float toolRadius;
	float toolMaxDepth;
	float cutDepth;
	int cutSpeed;
	int motorControl;

	// Circle
	float circleRadius;
	float circleDepth;
	int circleFill;

	// Rectangle
	float rectDepth;
	float rectX;
	float rectY;
	int rectFill;

} tSimpleShapeParam;

tSimpleShapeParam g_Params;

const struct { WCHAR* str; float val; } TOOL_SIZES[] = {
	{L"0.5       (1/2)",0.5f},
	{L"0.25     (1/4)",0.25f},
	{L"0.125   (1/8)",0.125f},
	{L"0.0625 (1/16)",0.0625}};

const struct { WCHAR* str; int val; } CUT_SPEED[] = {
	{L"F10",10},
	{L"F20",20},
	{L"F30",30},
	{L"F40",40},
	{L"F60",50}};

#define ITEM_CNT(x) (sizeof(x)/sizeof(x[0]))

void BasicShapeInit(HWND hWnd)
{
	int i;
	HWND hItem;

	hItem = GetDlgItem(hWnd, IDC_TOOL_SIZE);
	for (i = 0; i < ITEM_CNT(TOOL_SIZES);i++) ComboBox_AddString(hItem, TOOL_SIZES[i].str);

	hItem = GetDlgItem(hWnd, IDC_CUT_SPEED);
	for (i = 0; i < ITEM_CNT(CUT_SPEED); i++) ComboBox_AddString(hItem, CUT_SPEED[i].str);
	
	HKEY hKey;
	if (RegOpenKey(HKEY_CURRENT_USER, L"SOFTWARE", &hKey) == ERROR_SUCCESS)
	{
		DWORD cbData = sizeof(g_Params);
		if (RegGetValue(hKey, L"WinCNC", L"BasicShape", RRF_RT_REG_BINARY, NULL, &g_Params, &cbData) == ERROR_SUCCESS)
		{
			return;
		}
	}
	g_Params.toolRadius = 0.125f;
	g_Params.toolMaxDepth = 0.75f;
	g_Params.cutSpeed = 30;
	g_Params.circleDepth = 0.1f;
}

void BasicShapeSave()
{
	HKEY hKey;
	if (RegCreateKey(HKEY_CURRENT_USER, L"SOFTWARE\\WinCNC", &hKey) == ERROR_SUCCESS)
	{
		RegSetValueEx(hKey, L"BasicShape", 0, REG_BINARY, (BYTE*)&g_Params, sizeof(g_Params));
	}
}

UINT GetSetFloat(HWND hWnd, UINT id, BOOL get, float* val)
{
	WCHAR str[MAX_STR];
	HWND hItem;
	hItem = GetDlgItem(hWnd, id);
	if (get)
	{
		GetWindowText(hItem, str, MAX_STR);
		if (swscanf_s(str, L"%f", val) != 1) return id;
	}
	else
	{
		StringCbPrintf(str, sizeof(str), L"%.4f", *val );
		SetWindowText(hItem, str);
	}
	return 0;
}

UINT GetSetBool(HWND hWnd, UINT id, BOOL get, int* val)
{
	HWND hItem;
	hItem = GetDlgItem(hWnd, id);
	if (get)
	{
		*val = Button_GetCheck(hItem);
	}
	else
	{
		Button_SetCheck(hItem, *val);
	}
	return 0;
}



UINT BasicShapeGetSet(BOOL get, HWND hWnd )
{
	HWND hItem;
	WCHAR str[MAX_STR];
	float val;
	int i;

	hItem = GetDlgItem(hWnd, IDC_TOOL_SIZE);
	if (get)
	{
		ComboBox_GetText(hItem, str, MAX_STR);
		if (swscanf_s(str, L"%f", &val) != 1) return IDC_TOOL_SIZE;
		g_Params.toolRadius = val / 2.0f;
	}
	else
	{
		for (i = 0; i < ITEM_CNT(TOOL_SIZES); i++)
		{
			if (TOOL_SIZES[i].val == (g_Params.toolRadius * 2.0f))
			{
				ComboBox_SetCurSel(hItem, i);
				break;
			}
		}
		if( i== ITEM_CNT(TOOL_SIZES))
		{
			StringCbPrintf( str,sizeof(str),L"%f", g_Params.toolRadius * 2.0f);
			ComboBox_SetText(hItem, str);
		}
	}

	hItem = GetDlgItem(hWnd, IDC_CUT_SPEED);
	if (get)
	{
		i = ComboBox_GetCurSel(hItem);
		if (i != CB_ERR) 
			g_Params.cutSpeed = CUT_SPEED[i].val;
		else
		{
			ComboBox_GetText(hItem, str, MAX_STR);
			if (swscanf_s(str, L"%d", &i) != 1) return IDC_CUT_SPEED;
			g_Params.cutSpeed = i;
		}
	}
	else
	{
		for (i = 0; i < ITEM_CNT(CUT_SPEED); i++)
		{
			if (CUT_SPEED[i].val == g_Params.cutSpeed)
			{
				ComboBox_SetCurSel(hItem, i);
				break;
			}
		}
		if (i == ITEM_CNT(CUT_SPEED))
		{
			StringCbPrintf(str, sizeof(str), L"%d", g_Params.cutSpeed );
			ComboBox_SetText(hItem, str);
		}
	}

	GetSetFloat(hWnd, IDC_CUT_DEPTH, get, &g_Params.cutDepth);
	GetSetFloat(hWnd, IDC_MAX_TOOL_DEPTH, get, &g_Params.toolMaxDepth);
	GetSetBool(hWnd, IDC_MOTOR, get, &g_Params.motorControl);

	GetSetFloat(hWnd, IDC_CIRCLE_RADIUS, get, &g_Params.circleRadius);
	GetSetFloat(hWnd, IDC_CIRCLE_DEPTH, get, &g_Params.circleDepth);
	GetSetBool(hWnd, IDC_CIRCLE_FILL, get, &g_Params.circleFill);

	GetSetFloat(hWnd, IDC_RECT_DEPTH, get, &g_Params.rectDepth);
	GetSetFloat(hWnd, IDC_RECT_X, get, &g_Params.rectX);
	GetSetFloat(hWnd, IDC_RECT_Y, get, &g_Params.rectY);
	GetSetBool(hWnd, IDC_RECT_FILL, get, &g_Params.rectFill);

	return 0;
}

#define MAX_BUF 100000

/*
G2 X2.3125 Y2.3125 I2.3125 Z - 0.3
G2 X2.3125 Y - 2.3125 J - 2.3125 P2
G0 Z0.3
G0 X - 4.625
*/

#define NEAR_ZERO		0.00001f
#define SMALL_OVELAP	0.015625f // 1/64
#define SMALLEST_RADIUS	( g_Params.toolRadius + SMALL_OVELAP )

BOOL CarveCircle(HWND hWnd)
{
	char str[MAX_STR];
	char* cmd;
	float r;

	// Can't use a tool so small
	if (g_Params.toolRadius < SMALL_OVELAP * 2) return FALSE;

	// Can't carve a hole smaller than the smallest radius we can 
	// circle down (aribitrary to limit cutting down without advancing.
	if (g_Params.circleRadius < SMALLEST_RADIUS ) return FALSE;

	// Can't carve deeper than the tool max depth
	if (g_Params.circleDepth > g_Params.toolMaxDepth) return FALSE;

	// Can't carve 
	if (g_Params.circleDepth < NEAR_ZERO) return FALSE;

	cmd = (char*)malloc(MAX_BUF);
	if (cmd == NULL) return FALSE;
	memset(cmd, 0x00, MAX_BUF);
	
	// G91: Relative Motion
	// Fxx : Carve speed
	// M3 : Motor ON
	// G4 P1 : Wait 1sec. for tool spindle to start
	sprintf_s(str, MAX_STR, "G91 F%d %s\r\n", g_Params.cutSpeed, g_Params.motorControl ? "M3 G4 P1" : "");
	strcat_s(cmd, MAX_BUF, str);

	// Calculate the distance between concentric circles needed to fill
	// the circle with a small overlap between each of them.
	float R = g_Params.circleRadius + SMALL_OVELAP;
	int t =  1 + (int)(R / (( g_Params.toolRadius * 2 ) - SMALL_OVELAP ));
	float d = R / t;

	bool bDone = false;
	r = g_Params.circleRadius - g_Params.toolRadius;
	do
	{
		// Move from the center to the radius of the circle (9 o'clock)
		sprintf_s(str, MAX_STR, "G1 X%f\r\n", -r );
		strcat_s(cmd, MAX_BUF, str);

		// Spiral down, making as many turns as needed to cut less than 'cutDepth' 
		// for each pass. When P is two G2 makes a 450 degree turn. Starts at 9, end
		// at 12 o'clock.
		int P = 2 + (int)(g_Params.circleDepth / g_Params.cutDepth);
		sprintf_s(str, MAX_STR, "G2 X%f Y%f I%f Z%f P%d\r\n", r, r, r, -g_Params.circleDepth, P);
		strcat_s(cmd, MAX_BUF, str);
		
		// Make another full circle to finish the bottom flat. To avoid making
		// an additional 1/2 turn using the P parameter, do it in 4 G2 commands.
		// First quarter (12 to 3)
		sprintf_s(str, MAX_STR, "G2 X%f Y%f J%f\r\n", r, -r, -r);
		strcat_s(cmd, MAX_BUF, str);
		// Second quarter (3 to 6)
		sprintf_s(str, MAX_STR, "G2 X%f Y%f I%f\r\n", -r, -r, -r);
		strcat_s(cmd, MAX_BUF, str);
		// Third quarter (6 to 9)
		sprintf_s(str, MAX_STR, "G2 X%f Y%f J%f\r\n", -r, r, r);
		strcat_s(cmd, MAX_BUF, str);
		// Fourth quarter (9 to 12)
		sprintf_s(str, MAX_STR, "G2 X%f Y%f I%f\r\n", r, r, r);
		strcat_s(cmd, MAX_BUF, str);

		// Come back to starting height
		sprintf_s(str, MAX_STR, "G0 Z%f\r\n", g_Params.circleDepth);
		strcat_s(cmd, MAX_BUF, str);

		// Move back to center of circle.
		sprintf_s(str, MAX_STR, "G0 Y%f", -r);
		strcat_s(cmd, MAX_BUF, str);

		if( g_Params.circleFill &&
		  ( r > (g_Params.toolRadius - SMALL_OVELAP)) && 
		  (r > SMALLEST_RADIUS ))
		{
			// Move to next concentric circle
			// Overlap each concentic circle to ensure smooth bottom
			//r = r - ((g_Params.toolRadius * 2) - SMALL_OVELAP);
			r = r - d;
			if (r < SMALLEST_RADIUS)
			{
				r = SMALLEST_RADIUS;
			}
		}
		else
		{
			bDone = true;
		}
		// End the line and stop the motor if needed
		strcat_s(cmd, MAX_BUF, bDone && g_Params.motorControl ? " M0\r\n" : "\r\n");
	}
	while (!bDone);

	HWND hItem = GetDlgItem(hWnd, IDC_GCODE);
	SetWindowTextA(hItem, cmd);
	free(cmd);

	return TRUE;
}

void MakeRectMove(bool bXisLong, char* cmd, float L, float S, float z)
{
	char str[MAX_STR];
	float x, y;
	if (bXisLong)
	{
		x = L;
		y = S;
	}
	else
	{
		x = S;
		y = L;
	}
	strcat_s(cmd, MAX_BUF, "G1");
	if (x != 0.0f)
	{
		sprintf_s(str, MAX_STR, " X%f", x);
		strcat_s(cmd, MAX_BUF, str);
	}
	if (y != 0.0f)
	{
		sprintf_s(str, MAX_STR, " Y%f", y);
		strcat_s(cmd, MAX_BUF, str);
	}
	if (z != 0.0f)
	{
		sprintf_s(str, MAX_STR, " Z%f", z);
		strcat_s(cmd, MAX_BUF, str);
	}
	strcat_s(cmd, MAX_BUF, "\r\n");
}

BOOL CarveRect(HWND hWnd)
{
	char str[MAX_STR];
	char* cmd;

	// Can't use a tool so small
	if (g_Params.toolRadius < SMALL_OVELAP * 2) return FALSE;

	// Can't carve a rectangle so small the tool has no space to dive
	if (g_Params.rectX < SMALLEST_RADIUS * 2 ) return FALSE;
	if (g_Params.rectY < SMALLEST_RADIUS * 2) return FALSE;

	// Can't carve deeper than the tool max depth
	if (g_Params.rectDepth > g_Params.toolMaxDepth) return FALSE;

	// Nothing to carve if there is no depth
	if (g_Params.rectDepth < NEAR_ZERO) return FALSE;

	cmd = (char*)malloc(MAX_BUF);
	if (cmd == NULL) return FALSE;
	memset(cmd, 0x00, MAX_BUF);

	// G91: Relative Motion
	// Fxx : Carve speed
	// M3 : Motor ON
	// G4 P1 : Wait 1sec for tool spindle to start
	sprintf_s(str, MAX_STR, "G91 F%d %s\r\n", g_Params.cutSpeed, g_Params.motorControl ? "M3 G4 P1" : "");
	strcat_s(cmd, MAX_BUF, str);

	// True if X is longest side of the rect
	bool bXisLong = (g_Params.rectX > g_Params.rectY);

	// Current long and short side of the rect remaining to carve
	float L = bXisLong ? g_Params.rectX : g_Params.rectY;
	float S = bXisLong ? g_Params.rectY : g_Params.rectX;
	L = L - (g_Params.toolRadius * 2);
	S = S - (g_Params.toolRadius * 2);

	int t = 1 + (int)(S / ((g_Params.toolRadius * 2 ) - SMALL_OVELAP));
	float d = S / t;
	float Offset = 0.0f;
	bool bDone = false;
	bool bInside = false;

	do
	{
		// Start at zero depth.
		float Z = 0.0f;
		float w = d - SMALL_OVELAP;

		// Dive while depth has not reached target rect depth
		while (Z < g_Params.rectDepth - NEAR_ZERO )
		{
			float D;
			// If what remains to be carve is larger than the max cut depth 
			if (g_Params.rectDepth - Z > g_Params.cutDepth)
				// Dive by the max cut depth
				D = g_Params.cutDepth;
			else
				// Dive by what's left
				D = g_Params.rectDepth - Z;

			// update the current depth
			Z += D;

			// Dive by 'D' on the long side of the rect
			MakeRectMove(bXisLong, cmd, L, 0.0f, -D);
			if (bInside)
			{
				MakeRectMove(bXisLong, cmd, w, 0.0f, 0.0f );
				MakeRectMove(bXisLong, cmd, -w, 0.0f, 0.0f);
			}

			// Go along the short side
			MakeRectMove(bXisLong, cmd, 0.0f, S, 0.0f);
			if (bInside)
			{
				MakeRectMove(bXisLong, cmd, 0.0f, w, 0.0f);
				MakeRectMove(bXisLong, cmd, 0.0f, -w, 0.0f);
			}

			// Come back the long side
			MakeRectMove(bXisLong, cmd, -L, 0.0f, 0.0f);
			if (bInside)
			{
				MakeRectMove(bXisLong, cmd, -w, 0.0f, 0.0f);
				MakeRectMove(bXisLong, cmd, w, 0.0f, 0.0f);
			}

			// Come back the short side
			MakeRectMove(bXisLong, cmd, 0.0f, -S, 0.0f);
			if (bInside)
			{
				MakeRectMove(bXisLong, cmd, 0.0f, -w, 0.0f);
				MakeRectMove(bXisLong, cmd, 0.0f, w, 0.0f);
			}
		}

		// Flatten the bottom along the long side
		MakeRectMove(bXisLong, cmd, L, 0.0f, 0.0f);

		// Get back out to starting height
		sprintf_s(str, MAX_STR, "G0 Z%f\r\n", Z);
		strcat_s(cmd, MAX_BUF, str);

		// Come back to orgin
		if( bXisLong )
			sprintf_s(str, MAX_STR, "G0 X%f\r\n", -L );
		else
			sprintf_s(str, MAX_STR, "G0 Y%f\r\n", -L);	
		strcat_s(cmd, MAX_BUF, str);
		
		// If we need to carve the inside and if the current rectangle still
		// has material in its center.
		if (g_Params.rectFill && S > ((g_Params.toolRadius * 2) - SMALL_OVELAP ))
		{
			// Move along the short side by an increment before going through
			// another smaller square. 
			S = S - (d * 2);
			L = L - (d * 2);
			sprintf_s(str, MAX_STR, "G1 X%f Y%f\r\n", d, d);
			strcat_s(cmd, MAX_BUF, str);

			// Accumulate the deltas so that we can return back to origin
			// at the end.
			Offset += d;
			// Set to enable the removal of corner triangles
			bInside = true;
		}
		else
		{
			if (Offset != 0.0f)
			{
				// Come back up to origin
				sprintf_s(str, MAX_STR, "G0 X%f Y%f\r\n", -Offset, -Offset);
				strcat_s(cmd, MAX_BUF, str);
			}
			bDone = true;
		}

	} while (!bDone);

	// End the line and stop the motor if needed
	if (g_Params.motorControl)
	{
		strcat_s(cmd, MAX_BUF, "M0\r\n");
	}

	HWND hItem = GetDlgItem(hWnd, IDC_GCODE);
	SetWindowTextA(hItem, cmd);
	free(cmd);

	return TRUE;
}


void BasicShapeOneCommand(HWND hWnd)
{
	char* cmd;
	HWND hItem;
	hItem = GetDlgItem(hWnd, IDC_GCODE );

	int l = GetWindowTextLength(hItem) + 1;
	cmd = (char*)malloc(l);
	if (cmd)
	{
		GetWindowTextA(hItem, cmd, l);
		ParseBuffer(hWnd, cmd, l, doGcode);
		free(cmd);
	}
}

WNDPROC g_oldDlgdProc = NULL;
BOOL CALLBACK InterceptWndProc(HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam)
{
	if (message == WM_KEYDOWN )
	{
		if (wParam == 'a' || wParam == 'A' && GetKeyState(VK_CONTROL))
		{
			PostMessage(hWnd, EM_SETSEL, 0, -1);
			return MNC_CLOSE << 16;
		}	
	}
	return g_oldDlgdProc(hWnd, message, wParam, lParam);
}

BOOL CALLBACK BasicShapesProc(HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam)
{
	HWND hDlg;

	switch (message)
	{
	case WM_INITDIALOG:
		BasicShapeInit(hWnd);
		BasicShapeGetSet(FALSE, hWnd);

		// This is to capture the CTRL+A on the GCode edit box
		hDlg = GetDlgItem(hWnd, IDC_GCODE);
		g_oldDlgdProc = (WNDPROC)GetWindowLong(hDlg, GWL_WNDPROC);
		SetWindowLong(hDlg, GWL_WNDPROC, (LONG)InterceptWndProc);
		return TRUE;
		break;

	case WM_CLOSE:
		BasicShapeGetSet(TRUE, hWnd);
		BasicShapeSave();
		break;

	case WM_COMMAND:
		switch (LOWORD(wParam))
		{
		case IDC_EXECUTE :
			BasicShapeOneCommand(hWnd);
			break;
		case IDC_CARVE_CIRCLE:
			BasicShapeGetSet(TRUE , hWnd );
			CarveCircle(hWnd);
			break;
		case IDC_CARVE_RECT:
			BasicShapeGetSet(TRUE, hWnd);
			CarveRect(hWnd);
			break;

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