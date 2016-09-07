#include <RF24.h>
#include <RF24_config.h>
const byte rxAddr[6] = "00001";

class PigmiceCommandWriter
{
public:
  
  PigmiceCommandWriter()
    : radio(9, 10)
  {
    pmPBotDrive.btnUp = 0;
    pmPBotDrive.btnDown = 0; 
    pmPBotDrive.btnLeft = 0;
    pmPBotDrive.btnRight = 0;
    pmPBotDrive.btnStart = 0;
  
    pmPBotDrive.btnSelect = 0;
    pmPBotDrive.btnJoystick = 0;
    pmPBotDrive.btnJoystickX = 0;
    pmPBotDrive.btnJoystickY = 0;
  }

 void initializeCommandWriter()
 {
    radio.begin();
    radio.setRetries(15, 15);
    radio.openWritingPipe(rxAddr);    
    radio.stopListening();
 }

  bool writeCommand(int upVal, int downVal, int leftVal, int rightVal, int startVal, int selectVal, int joystickVal, int joystickXVal, int joystickYVal)
  {

    pmPBotDrive.btnUp = upVal;
    pmPBotDrive.btnDown = downVal; 
    pmPBotDrive.btnLeft = leftVal;
    pmPBotDrive.btnRight = rightVal;
    pmPBotDrive.btnStart = startVal;
  
    pmPBotDrive.btnSelect = selectVal;
    pmPBotDrive.btnJoystick = joystickVal;
    pmPBotDrive.btnJoystickX = joystickXVal;
    pmPBotDrive.btnJoystickY = joystickYVal;
    
    if (!radio.write( &pmPBotDrive, sizeof(pmPBotDrive) ))
    {         
       return true;
    }
    else
    {
      return false;
    }

  }


private:
  
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
