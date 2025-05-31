#pragma once

tStatus ParseBuffer(HWND hParent, char* pt, ULONG cbBuffer, tStatus(*cmd)(char*), BOOL bDebug);
tStatus ParseGCodeFile(HWND hParent, LPWSTR szFileName, tStatus(*cmd)(char*), BOOL bDebug);
