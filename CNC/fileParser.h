
tStatus ParseBuffer(char* pt, tStatus(*cmd)(char*));
tStatus ParseGCodeFile(LPWSTR szFileName, tStatus(*cmd)(char*));