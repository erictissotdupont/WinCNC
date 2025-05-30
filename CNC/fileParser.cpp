
#include "CNC.h"
#include "status.h"
#include "Windowsx.h"
#include "Commctrl.h"
#include "resource.h"
#include "3Dview.h"
#include "socket.h"

typedef struct {
	char* buffer;
	ULONG cbBuffer;
	tStatus(*cmd)(char*);
	tStatus status;
	HWND hDialog;
	HANDLE hThread;
	BOOL bStop;
	BOOL bPause;
	HANDLE hDebugStepEvent;
} tParserJob;

#define PROGRESS_RES 512
#define HISTORY_DEPTH 3

DWORD ParserThread(PVOID pParam)
{
	tParserJob *pJob = (tParserJob*)pParam;
	char* pt = pJob->buffer;
	char* eol;
	int l = 0;
	int cmdHistIdx = 0;
	int cmdCount = 0;
	char* cmdHistory[HISTORY_DEPTH];
	char strStatus[1024];

	for (int i = 0; i < HISTORY_DEPTH; i++)
	{
		cmdHistory[i] = (char*)malloc(MAX_PATH);
		*cmdHistory[i] = 0;
	}
	pJob->status = retSuccess;

	int timeout = 100;
	while (pJob->hDialog == NULL && --timeout >= 0) Sleep(100);

	while (pt && !pJob->bStop)
	{
		int progress = (PROGRESS_RES * (pt - pJob->buffer)) / pJob->cbBuffer;

		eol = strchr(pt, '\r');
		if (!eol) eol = strchr(pt, '\n');
		if (eol)
		{
			*eol = 0;
			eol++;
			while (*eol == '\n' || *eol == '\r') eol++;
		}
		l = strlen(pt);
		if (l > 0)
		{
			cmdCount++;
			cmdHistIdx++;
			if (cmdHistIdx >= HISTORY_DEPTH)
			{
				cmdHistIdx = 0;
			}
			sprintf_s(cmdHistory[cmdHistIdx], MAX_PATH, "%d %s\r\n", cmdCount, pt);
			//strcpy_s(cmdHistory[cmdHistIdx], MAX_PATH, pt);
			strStatus[0] = 0;
			for (int i = 0; i < HISTORY_DEPTH; i++)
			{
				int n = cmdHistIdx - i;
				if (n < 0) n += HISTORY_DEPTH;
				if (n >= HISTORY_DEPTH) n -= HISTORY_DEPTH;
				strcat_s(strStatus, sizeof(strStatus), cmdHistory[n]);
			}

			if (pJob->hDialog)
			{
				PostMessage(pJob->hDialog, WM_UPDATE_PROGRESS, progress, (LPARAM)strStatus);
			}

			if (pJob->hDebugStepEvent)
			{
				WaitForSingleObject(pJob->hDebugStepEvent, INFINITE);
			}

			if ((pJob->status = pJob->cmd(pt)) != retSuccess) break;
		}
		pt = eol;

		while (pJob->bPause && !pJob->bStop)
		{
			Sleep(500);
		}
	}

	if (pJob->status == retSuccess )
	{
		// Not implemented
		// pJob->status = pJob->cmd("M114\n");
	}

	if (pJob->hDialog) PostMessage(pJob->hDialog, WM_UPDATE_PROGRESS, PROGRESS_RES, 0);
	Sleep(250);
	pJob->bStop = true;
	if (pJob->hDialog) PostMessage(pJob->hDialog, WM_CLOSE, 0, 0);

	return 0;
}

BOOL CALLBACK FileParserProc(HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam)
{
	HWND hItem;
	static tParserJob* pJob = NULL;
	
	switch (message)
	{
	case WM_UPDATE_PROGRESS:
		hItem = GetDlgItem(hWnd, IDC_PROGRESS);
		SendMessage(hItem, PBM_SETPOS, wParam, 0);
		hItem = GetDlgItem(hWnd, IDC_PARSER_STATE);
		SetWindowTextA(hItem, (char*)lParam);
		break;

	case WM_INITDIALOG:
		pJob = (tParserJob*)lParam;
		hItem = GetDlgItem(hWnd, IDC_PROGRESS);
		SendMessage(hItem, PBM_SETRANGE, 0, MAKELONG(0, PROGRESS_RES));
		pJob->hDialog = hWnd;
		return TRUE;
		break;

	case WM_CLOSE:
		break;

	case WM_COMMAND:
		switch (LOWORD(wParam))
		{
		case IDD_PAUSE:
			hItem = GetDlgItem(hWnd, IDD_PAUSE);
			if (pJob->hDebugStepEvent == NULL)
			{
				pJob->hDebugStepEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
				SetWindowText(hItem, L"RESUME");
			}
			else
			{
				HANDLE hEvent = pJob->hDebugStepEvent;
				pJob->hDebugStepEvent = NULL;
				SetEvent(hEvent);
				CloseHandle(hEvent);
				SetWindowText(hItem, L"PAUSE");
			}
			break;

		case IDD_STEP:
			SetEvent(pJob->hDebugStepEvent);
			break;

		case IDOK:
			// Fall through.
		case IDCANCEL:
			if (pJob->bStop == false)
			{
				if (MessageBox(hWnd,
					L"This will stop the current program.\r\nAre you sure?",
					L"GCode",
					MB_YESNO | MB_ICONEXCLAMATION) == IDYES)
				{
					ForceStop( );
					pJob->bStop = true;
					WaitForSingleObject(pJob->hThread, 5000);
					EndDialog(hWnd, wParam);
				}
			} 
			else
			{
				EndDialog(hWnd, wParam);
			}

			pJob->hDialog = NULL;
			pJob = NULL;

			return TRUE;
		}
	}
	return FALSE;
}

WCHAR* GetCNCErrorString(tStatus status)
{
	switch (status)
	{
	case retSuccess: return L"No error";
	case retInvalidParam: return L"Invalid parameter";
	case retSyntaxError: return L"Syntax error";
	case retFileNotFound: return L"File not found";
	case retUserAborted: return L"User aborted";
	case retNoOutputFound: return L"No output found";
	case retCncNotConnected: return L"Cnc not connected";
	case retCncStatusTimeout: return L"Cnc status timeout";
	case retCncError: return L"Cnc error";
	case retOutOfMemory: return L"Out of memory";
	case retCncCBusy: return L"Busy";
	case retBufferBusyTimeout: return L"Buffer busy timeout";
	case retCncCommunicationError: return L"Cnc communication error";
	case retBufferMutexTimeout: return L"Buffer mutex timeout";
	case retStopRequested: return L"Stop requested";
	case retInternalError: return L"Internal error";
	case retNotImplemented: return L"Not implemented";
	case retPreParseComplete: return L"Pre-parse complete";
	case retQuit: return L"Quit";
	case retUnknownErr: return L"Unknown error";
	default: return(L"Unknown error (Case).");
	}
}

tStatus ParseBuffer( HWND hParent, char* pt, ULONG cbBuffer, tStatus(*cmd)(char*), BOOL bDebug )
{
	DWORD dwThread;
	tParserJob job = { 0 };

	job.buffer = pt;
	job.cbBuffer = cbBuffer;
	job.cmd = cmd;
	job.hDialog = NULL;
	job.bStop = false;
	job.hDebugStepEvent = bDebug ? CreateEvent(NULL, FALSE, FALSE, NULL) : NULL;
	job.status = retInternalError;
	memcpy_s(job.buffer, cbBuffer, pt, cbBuffer);

	job.hThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ParserThread, &job, 0, &dwThread);
	if (job.hThread != NULL)
	{
		DialogBoxParam(NULL,
			MAKEINTRESOURCE(IDD_GCODE),
			hParent,
			(DLGPROC)FileParserProc,
			(LPARAM)&job);

		if (job.status != retSuccess &&
			job.status != retPreParseComplete &&
			job.status != retStopRequested)
		{
			MessageBox(hParent, GetCNCErrorString(job.status), L"GCode", MB_OK | MB_ICONERROR);
		}
		CloseHandle(job.hThread);
	}

	while (job.hDialog) Sleep(100);
	return job.status;
}

tStatus ParseGCodeFile( HWND hParent, LPWSTR szFileName, tStatus(*cmd)(char*), BOOL bDebug )
{
	HANDLE hFile;
	char* buffer;
	DWORD fileSize;
	tStatus ret;

	hFile = CreateFile(szFileName, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, 0, NULL);
	if (hFile == INVALID_HANDLE_VALUE)
	{
		printf("Opening command file '%S' failed.\n", szFileName);
		return retFileNotFound;
	}

	fileSize = GetFileSize(hFile, NULL);

	buffer = (char*)malloc(fileSize + 1);
	if (buffer == NULL)
	{
		return retOutOfMemory;
	}

	ReadFile(hFile, buffer, fileSize, NULL, NULL);
	buffer[fileSize] = 0;

	ret = ParseBuffer(hParent, buffer, fileSize, cmd, bDebug );

	printf("Done.\n");
	CloseHandle(hFile);
	free(buffer);

	return ret;
}


