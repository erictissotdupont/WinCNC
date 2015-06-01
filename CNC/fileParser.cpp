
#include "CNC.h"
#include "status.h"

tStatus ParseGCodeFile(LPWSTR szFileName, tStatus(*cmd)(char*))
{
	HANDLE hFile;
	char* pt;
	char* buffer;
	char* eol;
	char line[256];
	float read = 0.0;
	int l = 0;
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
		printf("Memory allocation for %d bytes failed.\n", buffer);
		return retOutOfMemory;
	}

	ReadFile(hFile, buffer, fileSize, NULL, NULL);
	buffer[fileSize] = 0;

	pt = buffer;

	do {
		eol = strchr(pt, '\r');
		if (!eol) eol = strchr(pt, '\n');
		if (eol)
		{
			*eol = 0;
			eol++;
			while (*eol == '\n' || *eol == '\r') eol++;
		}
		l = strlen(pt);
		if (l >= sizeof(line))
		{
			printf("Line in file is too long(%d).\n", l);
			return retSyntaxError;
		}

		if (l > 0)
		{
			read += l;

			sprintf_s(line, sizeof(line), "%.1f%% - %s", (read * 100) / fileSize, pt);

			l = strlen(line);
			memset(line + l, ' ', sizeof(line) - 1 - l);
			line[sizeof(line) - 1] = 0;

			//printf("\r%s", line);

			if ((ret = cmd(pt)) != retSuccess) break;
		}
		pt = eol;
	} while (pt);

	printf("Done.\n");
	CloseHandle(hFile);
	free(buffer);

	return ret;
}
