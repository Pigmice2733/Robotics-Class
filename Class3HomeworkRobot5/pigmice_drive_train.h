#include <printf.h>



class PigmiceDriveTrain
{

public:	
	PigmiceDriveTrain()
		: currentMotor1Speed(0), currentMotor2Speed(0), enA(10), in1(8), in2(9), enB(5), in3(7), in4(6)
	{
   
	}

 void setSerialPortNumber(int serialPortNumber)
 {
   serialPortNum = serialPortNumber;
 }

	void initializeDriveTrain()
	{
	  pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
	}

	void setMotor1Speed(int speed)
	{
    if (speed > 64)
    {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      int actualForwardSpeed = map(speed, 64, 127, 0, 255);
      analogWrite(enA, actualForwardSpeed);
      
    }else if (speed < 64)
    {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);  
      int actualBackwardSpeed = map(speed, 64, 0, 0, 255);
      analogWrite(enA, actualBackwardSpeed);
      
    }else
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);  
       
    }
		
		currentMotor1Speed = speed;
	}

	void setMotor2Speed(int speed)
	{

        if (speed > 64)
        {
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
          int actualForwardSpeed = map(speed, 64, 127, 0, 255);
          analogWrite(enB, actualForwardSpeed);
          
        }else if (speed < 64)
        {
          digitalWrite(in3, LOW);
          digitalWrite(in4, HIGH);  
          int actualBackwardSpeed = map(speed, 64, 0, 0, 255);
          analogWrite(enB, actualBackwardSpeed);
          
        }else
        {
            digitalWrite(in3, LOW);
            digitalWrite(in4, LOW);  
           
        }
  
		
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
	int currentMotor1Speed;
	int currentMotor2Speed;
  int serialPortNum;
  // connect motor controller pins to Arduino digital pins
  int enA;
  int in1;
  int in2;
  // motor two
  int enB;
  int in3;
  int in4;
};
