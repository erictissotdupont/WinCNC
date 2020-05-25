#pragma once

#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files:
#include <windows.h>

// C RunTime Header Files
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>
#include <stdio.h>	// for printf
#include <Strsafe.h>

#include "resource.h"
#include "status.h"


typedef struct
{
	float blockX;       // Dimension of the block being machined
	float blockY;
	float blockZ;       // Z position has no offset 
	float offsetX;      // Position block relative to initial tool position
	float offsetY;
	float offsetZ;
	float toolRadius;
	float toolHeight;   // Max cutting height of the tool
	ULONG gotWhatBlock;
	ULONG gotWhatTool;
	ULONG gotWhatStart;
} tMetaData;

#define WM_UPDATE_POSITION	     WM_USER
#define WM_UPDATE_PROGRESS	     (WM_USER + 1)
#define WM_CHECK_INITIAL_STATUS  (WM_USER + 2)
#define WM_REDRAW				 (WM_USER + 3)
