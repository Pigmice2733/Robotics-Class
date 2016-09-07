#include <RF24.h>
#include <RF24_config.h>
const byte rxAddr[6] = "00001";

class PigmiceCommandReader
{
public:
	
	PigmiceCommandReader()
		: btnUp(0), btnDown(0), btnLeft(0), btnRight(0), btnStart(0), btnSelect(0), btnJoystick(0), btnJoystickX(512), btnJoystickY(512), radio(22, 23)
	{
	
	}

 void initializeCommandReader()
 {
    radio.begin();
    radio.openReadingPipe(0, rxAddr);
    radio.startListening();
 }

	bool readCommand()
	{
		if (radio.available())
		{
			radio.read(&pmPBotDrive, sizeof(pmPBotDrive));

			//set values
			btnUp = pmPBotDrive.btnUp;
			btnDown = pmPBotDrive.btnDown;
			btnLeft = pmPBotDrive.btnLeft;
			btnRight = pmPBotDrive.btnRight;
			btnStart = pmPBotDrive.btnStart;

			btnSelect = pmPBotDrive.btnSelect;
			btnJoystick = pmPBotDrive.btnJoystick;
			btnJoystickX = pmPBotDrive.btnJoystickX;
			btnJoystickY = pmPBotDrive.btnJoystickY;


			return true;
		}
		else
		{
			return false;
		}
	
	}

	int getBtnUp() const
	{
		return btnUp;
	}

	int getBtnDown() const
	{
		return btnDown;
	}

	int getBtnLeft() const
	{
		return btnLeft;
	}

	int getBtnRight() const
	{
		return btnRight;
	}

	int getBtnStart() const
	{
		return btnStart;
	}

	int getBtnSelect() const
	{
		return btnSelect;
	}

	int getBtnJoystick() const
	{
		return btnJoystick;
	}

	int getBtnJoystickX() const
	{
		return btnJoystickX;
	}

	int getBtnJoystickY() const
	{
		return btnJoystickY;
	}


private:

	int btnUp;
	int btnDown;
	int btnLeft;
	int btnRight;
	int btnStart;

	int btnSelect;
	int btnJoystick;
	int btnJoystickX;
	int btnJoystickY;
	
	RF24 radio;

	struct pigmicePBotDrive_t {
		int btnUp;
		int btnDown;
		int btnLeft;
		int btnRight;
		int btnStart;
		int btnSelect;
		int btnJoystick;
		int btnJoystickX;
		int btnJoystickY;
	}
	pmPBotDrive;
  
};
