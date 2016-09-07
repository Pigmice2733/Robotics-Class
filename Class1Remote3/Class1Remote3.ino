#include "pigmice_command_writer.h"



int trim_upper = 522;
int trim_lower = 492;

int joystick_mid = 512;

int up_button = 2;
int down_button = 4;
int left_button = 5;
int right_button = 3;
int start_button = 6;
int select_button = 7;
int joystick_button = 8;
int joystick_axis_x = A0;
int joystick_axis_y = A1;
int buttonCount = 7;
int buttons[] = {up_button, down_button, left_button, right_button, start_button, select_button, joystick_button};
int joyXVal; //holds final values after trimming
int joyYVal; //holds final values after trimming

PigmiceCommandWriter pmWR;

void setup() {

  
  for (int i; i < buttonCount; i++)
  {
    pinMode(buttons[i], INPUT);
    digitalWrite(buttons[i], HIGH);
  }

   pmWR.initializeCommandWriter(); //we initialize the pigmice command writer

}

void loop() {

  joyXVal = analogRead(joystick_axis_x);
  joyYVal = analogRead(joystick_axis_y);

  if ((joyXVal >= trim_lower) && (joyXVal <= trim_upper))
  {
    joyXVal = joystick_mid;
  }
  
  if ((joyYVal >= trim_lower) && (joyYVal <= trim_upper))
  {
    joyYVal = joystick_mid;
  }

  if (pmWR.writeCommand(digitalRead(up_button), digitalRead(down_button), digitalRead(left_button), 
          digitalRead(right_button), digitalRead(start_button), digitalRead(select_button), 
          digitalRead(joystick_button),
          joyXVal, joyYVal))
  {
    //sending commands succeeded
    
  }else
  {
    //write failed for some reason
    
  }

   delay(100);
}
