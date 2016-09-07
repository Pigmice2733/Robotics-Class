#include <printf.h>
#include <RoboClaw.h>

#define address 0x80

class PigmiceDriveTrain
{

public:	
	PigmiceDriveTrain(int serialPortNumber)
		: currentMotor1Speed(0), currentMotor2Speed(0), roboclaw(&Serial1, 10000)
	{
    serialPortNum = serialPortNumber;
	}

	void initializeDriveTrain()
	{
		roboclaw.begin(38400);
	}

	void setMotor1Speed(int speed)
	{
		roboclaw.ForwardBackwardM1(address, speed);  
		currentMotor1Speed = speed;
	}

	void setMotor2Speed(int speed)
	{
		roboclaw.ForwardBackwardM2(address, speed); 
		currentMotor2Speed = speed;

	}

	int getMotor1Speed() const
	{
		return currentMotor1Speed;
	}

	int getMotor2Speed() const
	{
		return currentMotor2Speed;
	}

private:
	RoboClaw roboclaw;
	int currentMotor1Speed;
	int currentMotor2Speed;
  int serialPortNum;
};
