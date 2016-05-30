#pragma once

typedef enum {
	retSuccess = 0,
	retInvalidParam = -1,
	retSyntaxError = -2,
	retFileNotFound = -3,
	retUserAborted = -4,
	retNoOutputFound = -5,
	retCncNotConnected = -6,
	retCncStatusTimeout = -7,
	retCncError = -8,
	retOutOfMemory = -9,
	retBusy = -10,
	retBufferBusyTimeout = -11,

	retPreParseComplete = -98,
	retQuit = -99,
	retUnknownErr = -100,
} tStatus;
