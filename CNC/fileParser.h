
tStatus ParseBuffer(HWND hParent, char* pt, ULONG cbBuffer, tStatus(*cmd)(char*));
tStatus ParseGCodeFile(HWND hParent, LPWSTR szFileName, tStatus(*cmd)(char*));
WCHAR* GetCNCErrorString(tStatus status);