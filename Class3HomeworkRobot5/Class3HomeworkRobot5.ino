#include "pigmice_command_reader.h"
#include "pigmice_drive_train.h"
//#include "Timer.h"
#include <NewPing.h>
//#include "newping_distance_sensor.h"
//#include "moving_average.h"
#include "pigmice_autonomous.h"

#define MAX_DISTANCE 350 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 43 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define STOP_MOTOR_VAL 64 // Value to set the motor to stop
#define FORWARD_AUTO_MOTOR_VAL 84 // Value to set the motor for full autonomous forward speed
#define BACKWARD_AUTO_MOTOR_VAL 46 // Value to set the motor for full autonomous backward speed
#define PM_ROBOT_OD_THRESHOLD_CM 40 //Robot stop threshold





PigmiceCommandReader pmCR;
PigmiceDriveTrain pmDT;
PigmiceAutonomousRobot pmAuto;

int speedPrevL = 64;
int speedPrevR = 64;
int curSpeedL = 64; 
int curSpeedR = 64;
int throttleCurveThreshold = 5;

int avoidObstaclesEvent;
bool isAvoidingObstacles = false;

void ComputeMotorSpeeds(int joystickX, int joystickY, int &speedMotor1, int &speedMotor2)
{
      // We use these values for computational ease, these hold the values of the joystick for our computation
    int     nJoyX;              // Joystick X input                     (-128..+127)
    int     nJoyY;              // Joystick Y input                     (-128..+127)

    
    // The mixed motor output that is on the compute scale/range, before being mapped back
    int     nMotMixL;           // Motor (left)  mixed output           (-128..+127)
    int     nMotMixR;           // Motor (right) mixed output           (-128..+127)

    // these will hold the final values to map it back to what our pigmice robot will need
    int     nMotFinalL; 
    int     nMotFinalR; 
    
    // Core configuration setting
    // - fPivYLimit  : The threshold at which the pivot action starts as we approach X=0
    //                This threshold is measured in units on the Y-axis
    //                away from the X-axis (Y=0). A greater value will assign
    //                more of the joystick's range to pivot actions.
    //                Allowable range: (0..+127)  
    float fPivYLimit = 32.0;
          
    // These are temporary values are going to use during our computation
    float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
    float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
    int     nPivSpeed;      // Pivot Speed                          (-128..+127)
    float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )


    nJoyX = map(joystickX, 0, 1024, -128, 127);
    nJoyY = map(joystickY, 0, 1024, -128, 127);


    //First we can calculate the wheel speeds needed for moving the robot forward and backward
    if (nJoyY >= 0) {
      // Forward movement of the robot
      nMotPremixL = (nJoyX>=0)? 127.0 : (127.0 + nJoyX);
      nMotPremixR = (nJoyX>=0)? (127.0 - nJoyX) : 127.0;
    } else {
      // Reverse movement of the robot: 
      nMotPremixL = (nJoyX>=0)? (127.0 - nJoyX) : 127.0;
      nMotPremixR = (nJoyX>=0)? 127.0 : (127.0 + nJoyX);
    }
    
    // We have to scale it before we compute the final mix.  So We scale the wheel speeds based on Y input (throttle)
      nMotPremixL = nMotPremixL * nJoyY/128.0;
      nMotPremixR = nMotPremixR * nJoyY/128.0;
      
      // This is where we compute the "peeks" or the pivot amount, so to speak. 
      // 1: How strong we want the pivot to be (nPivSpeed) based on Joystick X input
      // 2: Blending of pivot vs drive (fPivScale) based on Joystick Y input
      nPivSpeed = nJoyX;

      //The scale is 0 if it is above the limit, if not we factor in the limit to compute the scale
      fPivScale = (abs(nJoyY)>fPivYLimit)? 0.0 : (1.0 - abs(nJoyY)/fPivYLimit);
      
      // Calculate final mix of Drive and Pivot
      nMotMixL = (1.0-fPivScale)*nMotPremixL + fPivScale*( nPivSpeed);
      nMotMixR = (1.0-fPivScale)*nMotPremixR + fPivScale*(-nPivSpeed);

      //map it back to final values for our robot motor
      nMotFinalL = map(nMotMixL, -128, 127, 0, 127);
      nMotFinalR = map(nMotMixR, -128, 127, 0, 127);
    
      speedMotor1 = nMotFinalR;
      speedMotor2 = nMotFinalL;
    
}


void SafetyCheckAndDrive() { 
  
   if (pmCR.getBtnUp() == 0)
    {
      isAvoidingObstacles = false;
      
    }else  if (pmCR.getBtnDown() == 0)
    {

      if (!isAvoidingObstacles)
      {
        isAvoidingObstacles = true;
        pmAuto.initialize();
      }

      pmAuto.run();
   
    }else
    {
      isAvoidingObstacles = false;
       DrivePigmiceRobot(curSpeedL, curSpeedR);    
     
    }

}





void DrivePigmiceRobot(int reqSpeedL, int reqSpeedR) 
{

  if ((reqSpeedL >= 60) && (reqSpeedL <= 68))
    reqSpeedL = 64;

  if ((reqSpeedR >= 60) && (reqSpeedR <= 68))
    reqSpeedR = 64;
    
      pmDT.setMotor1Speed(reqSpeedL);
      pmDT.setMotor2Speed(reqSpeedR);
      speedPrevL = reqSpeedL;
      speedPrevR = reqSpeedR;
}

void setup() {

  //we initialize the command reader and drive train objects
    pmCR.initializeCommandReader(); //we initialize the pigmice command reader
    pmDT.initializeDriveTrain(); //we initialize the pigmice drive train

    Serial.begin(115200);

    pmAuto.SetDriveTrain(pmDT);
    
}

void loop() {


  int speedL = 64;
  int speedR = 64;

 // Check for commands from the teleoperator via RF
  if (pmCR.readCommand()) //if we are successful in reading 
  {
     ComputeMotorSpeeds(pmCR.getBtnJoystickX(), pmCR.getBtnJoystickY(), speedR, speedL); //computes wheel speeds from joystick values
     curSpeedL = speedL; 
     curSpeedR = speedR;    
  }

  //Decide what action to perform: teleoperate or autonomous
  SafetyCheckAndDrive();

}
