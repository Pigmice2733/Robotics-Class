#include "pigmice_command_reader.h"
#include "pigmice_drive_train.h"
#include <NewPing.h>

#define SONAR_NUM     1 // Number of sensors.
#define MAX_DISTANCE 350 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 110 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define STOP_MOTOR_VAL 64 // Value to set the motor to stop
#define PM_ROBOT_OD_THRESHOLD_CM 40 //Robot stop threshold

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(9, 9, MAX_DISTANCE)
};



PigmiceCommandReader pmCR;
PigmiceDriveTrain pmDT(1);

int speedPrevL = 0;
int speedPrevR = 0;
int throttleCurveThreshold = 5;

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

void CheckUltrasonicSensors(int reqSpeedL, int reqSpeedR)
{
    for (uint8_t i = 0; i < SONAR_NUM; i++) 
    { // Loop through all the sensors.
      if (millis() >= pingTimer[i]) 
      {        
        // Is it this sensor's time to ping?
        pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
        if (i == 0 && currentSensor == SONAR_NUM - 1) 
          SafetyCheck(reqSpeedL, reqSpeedR); // Sensor ping cycle complete, do something with the results.
        sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
        currentSensor = i;                          // Sensor being accessed.
        cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
        sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
      }
  }
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void SafetyCheck(int reqSpeedL, int reqSpeedR) { // Check safety

 //check first ultrasonic sensor
  if (cm[0] == 0) //no obstacles
  {
    DrivePigmiceRobot(reqSpeedL, reqSpeedR);        
  }else
  {
    if ((reqSpeedL > STOP_MOTOR_VAL) || (reqSpeedR > STOP_MOTOR_VAL)) //if robot is moving forward
    {      
        if (cm[0] < PM_ROBOT_OD_THRESHOLD_CM)// too close to obstacles
        {      
              DrivePigmiceRobot(STOP_MOTOR_VAL, STOP_MOTOR_VAL); //stop pigmice robot
        }else
        {
              DrivePigmiceRobot(reqSpeedL, reqSpeedR);
        }
    }else
    {
             DrivePigmiceRobot(reqSpeedL, reqSpeedR);
    }
  }

}


void DrivePigmiceRobot(int reqSpeedL, int reqSpeedR) 
{
      pmDT.setMotor1Speed(reqSpeedL);
      pmDT.setMotor2Speed(reqSpeedR);
      speedPrevL = reqSpeedL;
      speedPrevR = reqSpeedR;
}

void setup() {

  //we initialize the command reader and drive train objects
  pmCR.initializeCommandReader(); //we initialize the pigmice command reader
  pmDT.initializeDriveTrain(); //we initialize the pigmice drive train

  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  
    
}

void loop() {
  
  int speedL = 0;
  int speedR = 0;

  if (pmCR.readCommand()) //if we are successful in reading 
  {
    ComputeMotorSpeeds(pmCR.getBtnJoystickX(), pmCR.getBtnJoystickY(), speedR, speedL); //computes wheel speeds from joystick values
    if (pmCR.getBtnUp() == 0)  //we only want the safety mode to be engaged when the driver requests it by pressing AND holding down the top button
    {
      //process safe mode
      CheckUltrasonicSensors(speedL, speedR);    
    }
    else 
    {
      //simply drive if top button is NOT pressed
      DrivePigmiceRobot(speedL, speedR);     
    }
    
  }

}
