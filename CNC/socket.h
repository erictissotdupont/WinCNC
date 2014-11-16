

typedef enum CNC_SOCKET_EVENT
{
	CNC_CONNECTED = 1,
	CNC_DISCONNECTED,
};


int initSocketCom(void(*callback)(CNC_SOCKET_EVENT, PVOID));
tStatus sendCommand( char* cmd, unsigned long d );

