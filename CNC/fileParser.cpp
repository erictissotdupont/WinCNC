
#include "CNC.h"
#include "status.h"
#include "Windowsx.h"
#include "Commctrl.h"
#include "resource.h"

typedef struct {
	char* buffer;
	ULONG cbBuffer;
	tStatus(*cmd)(char*);
	tStatus status;
	HWND hDialog;
	HANDLE hThread;
	BOOL bStop;
} tParserJob;

#define PROGRESS_RES 512

DWORD ParserThread(PVOID pParam)
{
	tParserJob *pJob = (tParserJob*)pParam;
	char* pt = pJob->buffer;
	char* eol;
	int l = 0;

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
			if ((pJob->status = pJob->cmd(pt)) != retSuccess) break;

			if (pJob->hDialog) PostMessage(pJob->hDialog, WM_USER, progress, (LPARAM)pt);
		}
		pt = eol;
	}
	if (pJob->status == retSuccess)
	{
		pJob->status = pJob->cmd("M114\n");
	}

	if (pJob->hDialog) PostMessage(pJob->hDialog, WM_USER, PROGRESS_RES, 0);
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

BOOL CALLBACK FileParserProc(HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam)
{
	switch (message)
	{
	case WM_USER :
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
		return(L"Invalid command.");
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
	case retCncCommError:
		return(L"CNC communication error.");
	case retCncError:
		return(L"CNC in error state.");
	case retQuit:
		return(L"Quit.");
	case retUnknownErr:
		return(L"Unexpected error.");
	default:
		return(L"Unknown error.");
	}
}

tStatus ParseBuffer( HWND hParent, char* pt, ULONG cbBuffer, tStatus(*cmd)(char*))
{
	DWORD dwThread;
	job.buffer = pt;
	job.cbBuffer = cbBuffer;
	job.cmd = cmd;
	job.hDialog = NULL;
	job.bStop = false;

	memcpy_s(job.buffer, cbBuffer, pt, cbBuffer);

	job.hThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ParserThread, &job, 0, &dwThread);

	DialogBox(NULL,
		MAKEINTRESOURCE(IDD_GCODE),
		hParent,
		(DLGPROC)FileParserProc);

	if (job.status != retSuccess)
	{
		MessageBox(hParent, GetCNCErrorString(job.status), L"GCode", MB_OK | MB_ICONERROR);
	}

	CloseHandle(job.hThread);
	return job.status;
}

tStatus ParseGCodeFile( HWND hParent, LPWSTR szFileName, tStatus(*cmd)(char*))
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

	ret = ParseBuffer(hParent, buffer, fileSize, cmd);

	printf("Done.\n");
	CloseHandle(hFile);
	free(buffer);

	return ret;
}


