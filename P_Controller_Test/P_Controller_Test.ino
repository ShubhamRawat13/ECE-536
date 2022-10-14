#include "SimpleRSLK.h"
#include "Encoder.h"

// Debug Control
#define DEBUG 0
#define PLX 1

#if DEBUG == 1
#define debugln(x) Serial.println(x)
#define output(x) Serial.println(x) // Print newline char when debugging for clarity
#define debugDelay(x) delay(x) // For Debug Serial Monitor Delay
#else
#define debugln(x)
#define output(x) Serial.print(x) // Don't print newline char when not debugging
#define debugDelay(x)
#endif

#if PLX == 1
#define PLXout(x) Serial.print(x)
#define PLXoutln(x) Serial.println(x)
#else
#define PLXout(x)
#define PLXoutln(x)
#endif

void setupEncoder  ( uint8_t   ela_pin, uint8_t   elb_pin, uint8_t   era_pin, uint8_t   erb_pin );  // initalize the encoders

#define driveSpeed 20      // This was a good motor speed to complete this lab
#define TurnSpd 10

const int trigPin = 32;  //This is Port Pin 3.5 on the MSP432 Launchpad
const int echoPin = 33; //This is Port Pin 5.1 on the MSP432 Launchpad

int lWheelSpeed = driveSpeed  ;
int rWheelSpeed = driveSpeed+1 ;

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];

uint16_t LSpeed = 20;
uint16_t RSpeed = 20;
uint16_t Speed = 30;
//uint16_t fastSpeed = 100;
int center = 3500;
int error;
double adjustment;

// Range of output/ max error
double Ke = 0.5/3500; 
double DR = 0.5;
/* Valid values are either:
    DARK_LINE  if your floor is lighter than your line
    LIGHT_LINE if your floor is darker than your line
*/
uint8_t lineColor = DARK_LINE;

void setup()
{
  Serial.begin(9600);

  PLXoutln("CLEARDATA");
  PLXoutln("LABEL, Time, Time From Start, Sensor Output, Drive Ratio, Error");
  PLXoutln("RESETTIMER");

  setupRSLK();
  /* Left button on Launchpad */
  setupWaitBtn(LP_LEFT_BTN);
  /* Red led in rgb led */
  setupLed(RED_LED);
  clearMinMax(sensorMinVal, sensorMaxVal);
  
  delay(1000);
}

void floorCalibration() {
  /* Place Robot On Floor (no line) */
  delay(2000);
  String btnMsg = "Push left button on Launchpad to begin calibration.\n";
  btnMsg += "Make sure the robot is on the floor away from the line.\n";
  /* Wait until button is pressed to start robot */
  //waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED);

  delay(1000);

  debugln("Running calibration on floor");
  simpleCalibrate();
  debugln("Reading floor values complete");

  btnMsg = "Push left button on Launchpad to begin line following.\n";
  btnMsg += "Make sure the robot is on the line.\n";
  /* Wait until button is pressed to start robot */
  //waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED);
  delay(1000);

  enableMotor(BOTH_MOTORS);
}

void simpleCalibrate() {
  /* Set both motors direction forward */
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  /* Enable both motors */
  enableMotor(BOTH_MOTORS);
  /* Set both motors speed 20 */
  setMotorSpeed(BOTH_MOTORS, 20);

  for (int x = 0; x < 100; x++) {
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal, sensorMinVal, sensorMaxVal);
      /* Disable both motors */
  disableMotor(BOTH_MOTORS);
}
  }

  bool isCalibrationComplete = false;

void loop() {

  /* Run this setup only once */
  if (isCalibrationComplete == false) {
    floorCalibration();
    isCalibrationComplete = true;
  }

  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal, sensorCalVal, sensorMinVal, sensorMaxVal, lineColor);

  //could use average
  uint32_t linePos = getLinePosition(sensorCalVal, lineColor);
  //start P logic
  
  error =   linePos-center;

  adjustment = error*Ke;
  
//  RSpeed = constrain(RSpeed - Ke * error, 0, 20);
//  LSpeed = constrain(LSpeed + Ke * error, 0, 20);

// RSpeed = constrain(RSpeed - Ke * error, 0, 20);
// LSpeed = constrain(LSpeed + Ke * error, 0, 20);

  double DRnow = DR + adjustment;
  DRnow = constrain(DRnow,0 ,1);
  
  debugln("linePos = " + String(linePos) + " DR = " + String(DRnow) + " Error = " + String(error));
  debugln(" LSpeed = " + String(DRnow*Speed) + " RSpeed = " + String((1-DRnow)*Speed));

//    setMotorSpeed(LEFT_MOTOR, LSpeed);
//    setMotorSpeed(RIGHT_MOTOR, RSpeed );
//  setMotorSpeed(LEFT_MOTOR, constrain((DR) * Speed, 10, Speed));
//  setMotorSpeed(RIGHT_MOTOR, constrain((1-DR) * Speed, 15, Speed));
  setMotorSpeed(LEFT_MOTOR, ((DRnow) * Speed));
  setMotorSpeed(RIGHT_MOTOR, ((1-DRnow) * Speed));

  

  
//  if (linePos > 0 && linePos < 3000) { 
//    setMotorSpeed(LEFT_MOTOR, normalSpeed);
//    setMotorSpeed(RIGHT_MOTOR, fastSpeed);
//  } else if (linePos > 3500) {
//    setMotorSpeed(LEFT_MOTOR, fastSpeed);
//    setMotorSpeed(RIGHT_MOTOR, normalSpeed);
//  } else {
//    setMotorSpeed(LEFT_MOTOR, normalSpeed);
//    setMotorSpeed(RIGHT_MOTOR, normalSpeed);
//  }

  // PLX Data Out
  volatile unsigned long timeNow = millis();
  PLXout("DATA, TIME,");
  PLXout(timeNow);
  PLXout(" ,");
  PLXout(linePos);
  PLXout(" ,");
  PLXout(DRnow);
  PLXout(" ,");
  PLXout(error);
  PLXoutln(" ,");
  delay(500);
}




  
