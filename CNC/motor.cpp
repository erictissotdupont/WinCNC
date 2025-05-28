
#include "CNC.h"
#include "status.h"
#include "geometry.h"
#include "socket.h"
#include "gcode.h"
#include "motor.h"

tAxis XMotor,YMotor,ZMotor;
t3DPoint g_TheoricalPosition = { 0.0, 0.0, 0.0 };
tAxis* pMotor[] = {&XMotor,&YMotor,&ZMotor};
tSpindle Spindle;
tStatus(*g_pSimulation)(t3DPoint, t3DPoint, long) = NULL;

// Queue to track the movements sent to the machine but not executed yet
t3DPoint* g_CmdQueue;
unsigned int g_CmdInIndex;
unsigned int g_CmdQueueSize;

// ----------------------------------------------------------------------------

// Returns the position based on the motors current step. This is where the
// machine really is. Each new movement should be calculated using this
// position as the starting point. This should only be different from the 
// real position by less than a motor step.
//
void GetRealPosition( t3DPoint* P )
{
	P->x = XMotor.step * XMotor.scale;
	P->y = YMotor.step * YMotor.scale;
	P->z = ZMotor.step * ZMotor.scale;
}

// Return the position based on the previous movement accumulation. This is
// where the commands would like the machine to be. Each new movement should
// be using this position to calculate the ending point.
//
void GetTheoricalPosition( t3DPoint* R )
{
	*R = g_TheoricalPosition;
}

// Returns the estimated postion of the machine at a given time. This position
// should ONLY be used for display. It uses the history of commands sent
// to the machine and the reported number of commands currently in the queue
// to estimate the position of the actual machine.
//
void GetDisplayPosition(t3DPoint* pPos)
{
	int inQueue = GetInQueueCount();
	if (inQueue == 0 || g_CmdQueue == NULL )
	{
		GetRealPosition(pPos);
	}
	else
	{
		int oldestCmd = g_CmdInIndex - inQueue;
		if (oldestCmd < 0) oldestCmd += g_CmdQueueSize;
		*pPos = g_CmdQueue[oldestCmd];
	}
}

// Called when first connecting to the machine and receiving its current
// idle position. This resets both the REAL and THEORICAL positions.
// This also initialize the command queue used to track the actual
// position of the machine.
//
void ResetMachinePosition(long x, long y, long z, int cmdQueueSize)
{
	XMotor.step = x;
	YMotor.step = y;
	ZMotor.step = z;

	GetRealPosition(&g_TheoricalPosition);

	// Add one slot just in case
	cmdQueueSize++;
	if (cmdQueueSize != g_CmdQueueSize)
	{
		if (g_CmdQueue) free(g_CmdQueue);
		g_CmdQueue = (t3DPoint*)malloc(sizeof(t3DPoint) * cmdQueueSize);
		g_CmdQueueSize = cmdQueueSize;
	}
}

// Called by every movement command to update the new theorical position
//
void UpdateTheoricalPosition(double X, double Y, double Z)
{
	g_TheoricalPosition.x += X;
	g_TheoricalPosition.y += Y;
	g_TheoricalPosition.z += Z;
}

// Initialize the motor scale from the machine's provided information
//
void InitMotorAxis( int a, double scale )
{
  tAxis* pA = pMotor[a];
  pA->scale = scale;
}

double GetMotorSmalestStep( )
{
  return minOf3( XMotor.scale, YMotor.scale, ZMotor.scale );
}

double GetMaxMotorDistanceError()
{
	t3DPoint oneStep;
	oneStep.x = XMotor.scale;
	oneStep.y = YMotor.scale;
	oneStep.z = ZMotor.scale;
	return vector3DLength(oneStep);
}

void SetMotorSimulationMode(tStatus(*callback)(t3DPoint, t3DPoint, long))
{
	g_pSimulation = callback;
}

int SetMachineSpindleState(int state)
{
	if (Spindle.nextState != state)
	{
		Spindle.nextState = state;
		return 1;
	}
	return 0;
}

long MotorMakeTheMove(tAxis* A, double target)
{
	double delta = target - (A->step * A->scale);
	long step = (long)(delta / A->scale);
	A->step += step;
	return step;
}

long getSpindleState()
{
	Spindle.currentState = Spindle.nextState;
	return Spindle.currentState;
}

tStatus MotorDoTheMode(void(*posAtStep)(t3DPoint*, int, int, void*), int stepCount, double duration, void* pArg)
{
	int i;
	long x, y, z;
	unsigned long d, s;
	t3DPoint End;
	char str[100] = { 0 };
	tStatus status = retNoOutputFound;

	if (stepCount <= 0)
	{
		return retUnknownErr;
	}

	// Split the duration of the whole move for each step
	duration = duration / stepCount;
	// Convert that in uS for the CNC
	d = (unsigned long)(duration * 1000);

	LockMachinePosition(true);

	for (i = 1; i <= stepCount; i++)
	{
		t3DPoint Start;
		GetRealPosition(&Start);

		// Get the position we should be at for step i of stepCount
		posAtStep(&End, i, stepCount, pArg);

		x = MotorMakeTheMove(&XMotor, End.x);
		y = MotorMakeTheMove(&YMotor, End.y);
		z = MotorMakeTheMove(&ZMotor, End.z);
		s = (getSpindleState() == 3) ? CMD_FLAG_SPINDLE_ON : 0;

		if (g_pSimulation)
		{
			status = g_pSimulation(Start, End, d);
		}
		else
		{
			// Calculate the CRC of the position at the end of the movement
			// so that the machine can check if its position and distance 
			// corresponds to what the host wants. Includes duration and
			// flags
			s = s | GetPosCRC(XMotor.step, YMotor.step, ZMotor.step, d, s);

			sprintf_s(str, sizeof(str), "@" CNC_CMD_PARAMS, x, y, z, d, s);

			status = postCommand(str);

			if (status != retSuccess)
			{
				break;
			}
			else
			{
				GetRealPosition(&g_CmdQueue[g_CmdInIndex]);
				g_CmdInIndex++;
				if (g_CmdInIndex >= g_CmdQueueSize) g_CmdInIndex = 0;
			}
		}
	}

	LockMachinePosition(false);

	return status;
}

void MotorInit()
{
	Spindle.currentState = 0;
	Spindle.nextState = 0;
}
