#pragma once

void BasicShapes(HWND hWnd);
void ComplexShapes(HWND hWnd);
void BitmapShapes(HWND hWnd);

typedef struct {
	float radius;
	float maxDepth;
	float cutDepth;
	int cutSpeed;
	int motorControl;
	float safeTravel;
} tGeneralToolInfo;

const struct { WCHAR* str; float val; } TOOL_SIZES[] = {
	{ L"0.5       (1/2)",0.5f },
	{ L"0.25     (1/4)",0.25f },
	{ L"0.125   (1/8)",0.125f },
	{ L"0.0625 (1/16)",0.0625 } };

const struct { WCHAR* str; int val; } CUT_SPEED[] = {
	{ L"F10",10 },
	{ L"F20",20 },
	{ L"F30",30 },
	{ L"F40",40 },
	{ L"F60",50 } };

#define ITEM_CNT(x)		(sizeof(x)/sizeof(x[0]))
#define MAX_STR			255

void ShapeInitToolInfo(tGeneralToolInfo *pToolInfo);

UINT ShapeGetSetFloat(HWND hWnd, UINT id, BOOL get, float* val);
UINT ShapeGetSetBool(HWND hWnd, UINT id, BOOL get, int* val);
UINT ShapeGetSetString(HWND hWnd, UINT id, BOOL get, WCHAR* str, int cbStr);
UINT ShapeGetSetRadio(HWND hWnd, UINT id, int btnCnt, BOOL get, int* val);
UINT ShapeGetSetToolSize(HWND hWnd, UINT id, BOOL get, float* pRadius);
UINT ShapeGetSetTool(HWND hWnd, BOOL get, tGeneralToolInfo *pToolInfo);
