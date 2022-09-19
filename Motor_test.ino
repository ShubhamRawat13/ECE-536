#include "SimpleRSLK.h"

#define driveSpeed 20      // This was a good motor speed to complete this lab

int WheelSpeed = driveSpeed  ;

void setup()
{
  setupRSLK();
  /* Left button on Launchpad */
  setupWaitBtn(LP_LEFT_BTN);
  /* Red led in rgb led */
  setupLed(RED_LED);
  clearMinMax(sensorMinVal, sensorMaxVal);
  delay(1000);
}


void loop() {
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD); // Cause the robot to drive forward
  
  enableMotor(BOTH_MOTORS);                         // "Turn on" the motor
  
  setMotorSpeed(RIGHT_MOTOR, WheelSpeed);   // Set motor speed
  setMotorSpeed(LEFT_MOTOR, WheelSpeed);    // Set motor speed

  delay(1000);

  //Turn off the motors
  disableMotor(BOTH_MOTORS);
 
  delay(1000);
  }
