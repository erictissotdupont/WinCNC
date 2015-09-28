
#include "CNC.h"
#include "status.h"

tStatus ParseBuffer(char* pt, tStatus(*cmd)(char*))
{
	tStatus ret;
	char* eol;
	int l = 0;

	while( pt )
	{
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
			if ((ret = cmd(pt)) != retSuccess) break;
		}
		pt = eol;
	}
	return ret;
}

tStatus ParseGCodeFile(LPWSTR szFileName, tStatus(*cmd)(char*))
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

	ret = ParseBuffer(buffer, cmd);

	printf("Done.\n");
	CloseHandle(hFile);
	free(buffer);

	return ret;
}
