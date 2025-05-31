
#include "Main.h"
#include "status.h"
#include "geometry.h"
#include "CNC.h"
#include "gcode.h"
#include "motor.h"

tAxis XMotor,YMotor,ZMotor;
t3DPoint g_TheoricalPosition = { 0.0, 0.0, 0.0 };
tAxis* pMotor[] = {&XMotor,&YMotor,&ZMotor};
tSpindle Spindle;
tStatus(*g_pSimulation)(t3DPoint, t3DPoint, long) = NULL;

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

// Called when first connecting to the machine and receiving its current
// idle position. This resets both the REAL and THEORICAL positions.
// This also initialize the command queue used to track the actual
// position of the machine.
//
void ResetMachinePosition(long x, long y, long z)
{
	XMotor.step = x;
	YMotor.step = y;
	ZMotor.step = z;

	GetRealPosition(&g_TheoricalPosition);
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

	CNC_LockMachinePosition(true);

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
			s = s | CNC_GetPositionCRC(XMotor.step, YMotor.step, ZMotor.step, d, s);

			status = CNC_PostMovementCommand(x, y, z, d, s);
		}
	}

	CNC_LockMachinePosition(false);

	return status;
}

void MotorInit()
{
	Spindle.currentState = 0;
	Spindle.nextState = 0;
}

