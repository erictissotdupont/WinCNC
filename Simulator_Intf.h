
typedef struct
{
	DWORD id;
	DWORD dx;
	DWORD dy;
	float res;
	DWORD cbAlt;
	float originX;
	float originY;
} header_t;

#define SIMULATOR_DATA_CHANGED_EVENT_NAME         L"Local\\AltFileChangeEvent"
#define SIMULATOR_DATA_FILE_NAME                  L"CncAltSimulationData"