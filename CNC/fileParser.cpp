
#include "CNC.h"
#include "status.h"
#include "Windowsx.h"
#include "Commctrl.h"
#include "resource.h"
#include "3Dview.h"

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
		pJob->status = pJob->cmd("M114\n");
	}

	if (pJob->hDialog) PostMessage(pJob->hDialog, WM_UPDATE_PROGRESS, PROGRESS_RES, 0);
	Sleep(250);
	pJob->bStop = true;
	if (pJob->hDialog) PostMessage(pJob->hDialog, WM_CLOSE, 0, 0);

	return 0;
}

void ParserUpdate(HWND hWnd, WPARAM progress,LPARAM state)
{
	HWND hItem;
	hItem = GetDlgItem(hWnd, IDC_PROGRESS);
	SendMessage(hItem, PBM_SETPOS, progress, 0);

	hItem = GetDlgItem(hWnd, IDC_PARSER_STATE);
	SetWindowTextA(hItem, (char*)state);
}

tParserJob job;

void ParserInit(HWND hWnd)
{
	HWND hItem = GetDlgItem(hWnd, IDC_PROGRESS);
	SendMessage(hItem, PBM_SETRANGE, 0, MAKELONG(0, PROGRESS_RES));
	job.hDialog = hWnd;
}

void ParserOnPause(HWND hWnd)
{
	HWND hItem = GetDlgItem(hWnd, IDD_PAUSE);
	if (job.hDebugStepEvent == NULL)
	{
		job.hDebugStepEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
		SetWindowText(hItem, L"RESUME");
	}
	else
	{
		HANDLE hEvent = job.hDebugStepEvent;
		job.hDebugStepEvent = NULL;
		SetEvent(hEvent);
		CloseHandle(hEvent);
		SetWindowText(hItem, L"PAUSE");
	}
	
}

void ParserOnStep(HWND hWnd)
{
	SetEvent(job.hDebugStepEvent);
}

BOOL CALLBACK FileParserProc(HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam)
{
	switch (message)
	{
	case WM_UPDATE_PROGRESS:
		ParserUpdate(hWnd,wParam,lParam);
		break;
	case WM_INITDIALOG:
		ParserInit(hWnd);
		return TRUE;
		break;

	case WM_CLOSE:
		break;

	case WM_COMMAND:
		switch (LOWORD(wParam))
		{
		case IDD_PAUSE:
			ParserOnPause(hWnd);
			break;
		case IDD_STEP:
			ParserOnStep(hWnd);
			break;
		case IDOK:
			// Fall through.
		case IDCANCEL:
			if (job.bStop == false)
			{
				if (MessageBox(hWnd,
					L"This will stop the current program.\r\nAre you sure?",
					L"GCode",
					MB_YESNO | MB_ICONEXCLAMATION) == IDYES)
				{
					job.bStop = true;
					WaitForSingleObject(job.hThread, 5000);
					EndDialog(hWnd, wParam);
				}
			} else EndDialog(hWnd, wParam);
			return TRUE;
		}
	}
	return FALSE;
}

WCHAR* GetCNCErrorString(tStatus status)
{
	switch (status)
	{
	case retInvalidParam:
		return(L"Invalid parameter.");
	case retSyntaxError:
		return(L"GCode syntax error.");
	case retFileNotFound:
		return(L"File not found.");
	case retUserAborted:
		return(L"User interruption.");
	case retNoOutputFound:
		return(L"No output.");
	case retCncNotConnected:
		return(L"CNC not connected.");
	case retBufferBusyTimeout:
		return(L"CNC Output buffer overflow (timeout).");
	case retCncStatusTimeout:
		return(L"CNC Status request timeout.");
	case retCncError:
		return(L"CNC in error state.");
	case retCncCommunicationError:
		return(L"CNC communication error.");
	case retBufferMutexTimeout:
		return(L"Buffer mutex timeout.");
	case retQuit:
		return(L"Quit.");
	case retUnknownErr:
		return(L"Unexpected error.");
	default:
		return(L"Unknown error.");
	}
}

tStatus ParseBuffer( HWND hParent, char* pt, ULONG cbBuffer, tStatus(*cmd)(char*), BOOL bDebug )
{
	DWORD dwThread;
	job.buffer = pt;
	job.cbBuffer = cbBuffer;
	job.cmd = cmd;
	job.hDialog = NULL;
	job.bStop = false;
	job.hDebugStepEvent = bDebug ? CreateEvent(NULL, FALSE, FALSE, NULL) : NULL;
	memcpy_s(job.buffer, cbBuffer, pt, cbBuffer);

	job.hThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ParserThread, &job, 0, &dwThread);

	DialogBox(NULL,
		MAKEINTRESOURCE(IDD_GCODE),
		hParent,
		(DLGPROC)FileParserProc);

	if (job.status != retSuccess && job.status != retPreParseComplete )
	{
		MessageBox(hParent, GetCNCErrorString(job.status), L"GCode", MB_OK | MB_ICONERROR);
	}

	CloseHandle(job.hThread);
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


