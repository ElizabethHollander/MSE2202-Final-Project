// MSE2202 Design Project Group 15
// Master code for arduino 1 (arduino 2 is running on slave code)
// Master version, has main program functionality, charlieplex, button inputs, arms and pyramid-related stuff


//GUYS FOLLOW THIS NAMING CONVENTION
// <letter for type of variable>_<function variable is for>_<Custom name of choice>
//Example: int i_main_courseIndex

#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <SoftwareSerial.h>


//DEBUGGING
// uncomment lines based on what needs debugging, will print values to serial every loop
//#define debug_servos
//#define debug_limitSwitches
#define debug_communications
const unsigned int cui_debug_displayInterval = 1000; //Time between outputs displayed when debugging is enabled, in ms


//Pin mapping
SoftwareSerial tellSlave(5, 6); //comm ports with arduino 2 for communication
const int ci_pin_startButton = 7;
const int ci_pin_charlieplex1 = 10;
const int ci_pin_charlieplex2 = 9;
const int ci_pin_charlieplex3 = 8;
const int ci_pin_charlieplex4 = 7;
const int ci_pin_rearArm = 4;
const int ci_pin_rearHand = 3;
const int ci_pin_frontArm = 13;
const int ci_pin_frontHand = 12;
const int ci_pin_tipArm = 11;
const int ci_pin_tipPlate = 2;
const int ci_pin_rearSwitch = A5; //we don't have enough digital pins for everything, so analog is being used as digital
const int ci_pin_frontSwitch = A3;
const int ci_pin_tipSwitch = A4;

//charlieplex LED uses, if somebody disagrees with me on assignments, we can change it
const int ci_charlieplex_followWallParallel = 12; //turned on when tracking parallel to wall
const int ci_charlieplex_IRsearch = 9; //turned on when scanning/analysing IR and planning path
const int ci_charlieplex_blindPyramidSearch = 6; //turned on when lost IR, but thinks it knows what to do to find pyramid
const int ci_charlieplex_hasIR = 3; //turns on when correct IR is picked up, off when lost
const int ci_charlieplex_foundCube = 11; //on when either arm limit switch is triggered
const int ci_charlieplex_foundPyramid = 8; //on when pyramid limit switch is triggered
const int ci_charlieplex_successLight = 2; //turns on when finished course
const int ci_charlieplex_errorLight = 5; //turns on when some kind of error in code logic happens
const int ci_charlieplex_calibration = 7; //turns on when in middle of a calibration
const int ci_charlieplex_setupLight = 1; //turns on when set up is initiating
//LED 4, 10 still free for stuff, 1 can also blink or something for re-use

//Data from sensors
Servo servo_rearArm;
Servo servo_rearHand;
Servo servo_frontArm;
Servo servo_frontHand;
Servo servo_tipArm;
Servo servo_tipPlate;
bool b_sensor_rearSwitch;  //Note: sensor means its a value we can use in rest of code, it has already gone through debouncer
bool b_sensor_frontSwitch;
bool b_sensor_tipSwitch;
byte bt_slave_message; //stores message recieved from the slave arduino 2
byte bt_slave_lastMessage;
bool b_slave_newCommand;

//Main loop variables
int i_main_modeIndex; //switch statement to tell what is going on will use this variable (sit idle, course navigation, calibration, etc)
int i_main_courseIndex; // switch statement embedded in course navigation will read this, used to determine exactly what's going on inside course
unsigned long ul_startButton_3secTime; //startButton stuff is used to count button presses, don't touch these variables
bool b_startButton_3secTimeUp;
bool b_startButton_doOnce;
bool b_main_calibrationStart; //used as a one off variable to start calibration functions, resets to false when idle
unsigned long ul_debug_secTimer; //used to control debugging outputs to be every 2 seconds, and not constantly
bool b_cube_rearPossession;

//communications with slave variables
bool b_slave_isFinished; //toggles to true when slave sends 1, false after something has been done about that
int i_slave_modeIndex; //keep track of index number slave has responded with, saying that its on
int i_slave_courseIndex;
int i_slave_commsindex;
//limit switch reading variables
bool b_switch_rearCurrent;
bool b_switch_frontCurrent;
bool b_switch_tipCurrent;
bool b_switch_rearPrev;
bool b_switch_frontPrev;
bool b_switch_tipPrev;
const int ci_switch_debounceTime = 20;
unsigned long ul_switch_debounceRear;
unsigned long ul_switch_debounceFront;
unsigned long ul_switch_debounceTip;

//Servo control variables
bool b_servo_rearArmOn; //bool variables we use to turn on and off servos, useful for debugging/clean code?
bool b_servo_rearHandOn;
bool b_servo_frontArmOn;
bool b_servo_frontHandOn;
bool b_servo_tipArmOn;
bool b_servo_tipPlateOn;
const int ci_servo_rearArmUp = 90; //constant values used to control servo positions, may need to use different for rear and front
const int ci_servo_frontArmUp = 100;
const int ci_servo_rearArmDown = 8;
const int ci_servo_frontArmDown = 180;
const int ci_servo_rearArmDrop = 105;
const int ci_servo_frontArmDrop = 85;
const int ci_servo_rearHandOpen = 45;
const int ci_servo_rearHandClose = 140;
const int ci_servo_frontHandOpen = 140;
const int ci_servo_frontHandClose = 52;
const int ci_servo_tipPlateUp = 20;
const int ci_servo_tipPlateDown = 105;
const int ci_servo_tipArmDown = 90;
const int ci_servo_tipArmUp = 140;
int i_servo_rearArmPos;
int i_servo_rearHandPos;
int i_servo_frontArmPos;
int i_servo_frontHandPos;
int i_servo_tipArmPos;
int i_servo_tipPlatePos;
unsigned int ui_servo_waitTime; //time in ms to wait after telling a servo to move
unsigned long ul_servo_timer;

//placing cube under pyramid variables
int i_pyramid_index;
unsigned long ul_pyramid_timer; //used to wait for servos after each command before detaching and moving them on
unsigned int ui_pyramid_cubeDropTime; //time to wait after realeasing cube to dropping pyramid

//keep adding varibles you need in cadegories labeled like this


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600); //for debugging
  tellSlave.begin(9600);

  //set pinmodes (note servos will be switching throughout code)
  //pinMode(ci_pin_startButton, INPUT_PULLUP);
  pinMode(ci_pin_rearArm, OUTPUT);
  pinMode(ci_pin_rearHand, OUTPUT);
  pinMode(ci_pin_frontArm, OUTPUT);
  pinMode(ci_pin_frontHand, OUTPUT);
  pinMode(ci_pin_tipArm, OUTPUT);
  pinMode(ci_pin_tipPlate, OUTPUT);
  pinMode(ci_pin_rearSwitch, INPUT_PULLUP);
  pinMode(ci_pin_frontSwitch, INPUT_PULLUP);
  pinMode(ci_pin_tipSwitch, INPUT_PULLUP);

  //more initialization
  CharliePlexM::setBtn(ci_pin_charlieplex1, ci_pin_charlieplex2, ci_pin_charlieplex3, ci_pin_charlieplex4, ci_pin_startButton);
  CharliePlexM::Write(1, 1); //turn on LED to tell when booting up, may take awhile due to delay times

  //initialize variables
  i_main_modeIndex = 0;
  ul_startButton_3secTime = 0;
  b_startButton_doOnce = false;
  b_startButton_3secTimeUp = false;
  b_servo_rearArmOn = false;
  b_servo_rearHandOn = false;
  b_servo_frontArmOn = false;
  b_servo_frontHandOn = false;
  b_servo_tipArmOn = false;
  b_servo_tipPlateOn = false;
  ul_debug_secTimer = 0;
  ui_servo_waitTime = 2500;
  i_pyramid_index = 0;
  ui_pyramid_cubeDropTime = 5000;
  b_slave_isFinished = false;
  bt_slave_message = 0; //just to stop early errors
  i_slave_modeIndex = -1; //giving impossible value to prevent false assumption errors
  i_slave_courseIndex = -1;


  //sets servos to desired position
  //delay is used because this is 1 time set up, only cycles once, but we have to let minimal servos on at one time
  servo_rearArm.attach(ci_pin_rearArm);
  servo_rearHand.attach(ci_pin_rearHand);
  servo_rearArm.write(ci_servo_rearArmUp);
  servo_rearHand.write(ci_servo_rearHandOpen);
  delay(ui_servo_waitTime);
  servo_rearArm.detach();
  servo_rearHand.detach();

  servo_frontArm.attach(ci_pin_frontArm);
  servo_frontHand.attach(ci_pin_frontHand);
  servo_frontArm.write(ci_servo_frontArmUp);
  servo_frontHand.write(ci_servo_frontHandOpen);
  delay(ui_servo_waitTime);
  servo_frontArm.detach();
  servo_frontHand.detach();

  servo_tipArm.attach(ci_pin_tipArm);
  servo_tipPlate.attach(ci_pin_tipPlate);
  servo_tipArm.write(ci_servo_tipArmDown);
  servo_tipPlate.write(ci_servo_tipPlateUp);
  delay(ui_servo_waitTime);
  servo_tipArm.detach();
  servo_tipPlate.detach();

  //end of set up
  CharliePlexM::Write(1, 0);
}

void loop() {
  Serial.println("ping1");
  // Button presses (affects i_main_modeIndex)
  // 0 = idle
  // 1 = run main program
  // 2 = calibrate motors
  // 3 = testing functions, edit in whatever you want to test here, we'll see how this goes

  //button press counter
  if ((millis() - ul_startButton_3secTime) > 3000)
  {
    Serial.println("ping2");
    b_startButton_3secTimeUp = true;
  }
  Serial.println("ping3");
  // button-based mode selection
  if (CharliePlexM::ui_Btn)
  {
    Serial.println("ping4");
    if (b_startButton_doOnce == false)
    {
      Serial.println("ping5");
      b_startButton_doOnce = true;
      i_main_modeIndex++;
      i_main_modeIndex = i_main_modeIndex & 7;
      ul_startButton_3secTime = millis();
      b_startButton_3secTimeUp = false;
      b_main_calibrationStart = false;
    }
    Serial.println("ping6");
  }
  else
  {
    Serial.println("ping7");
    b_startButton_doOnce = LOW;
  }

  Serial.println("ping8");
  //communication with other board all the time, and read all sensors
  readSlave();
  Serial.println("ping9");
  readLimitSwitches();
  Serial.println("ping10");


  //main switch statement to drive mode that is operating in
  //will not be active until after 3 second time out from last button press (other than case 0)
  switch (i_main_modeIndex)
  {
      Serial.println("ping11");
    case 0:
      {
        Serial.println("ping12");
        //sits idle, default on start up
        if (i_slave_modeIndex != 0)
        {
          Serial.println("ping13");
          //untill acknowledgement, tell slave that it is in idle mode
          tellSlave.write(2);
          //i_slave_commsindex = 2;
          Serial.println("ping14");
        }
        Serial.println("ping15");
        turnOffAllServos();
        Serial.println("ping16");
        break;
      }
    case 1:
      {
        Serial.println("ping17");

        if (b_startButton_3secTimeUp)
        {
          if (i_slave_modeIndex != 1)
          {
            Serial.println("ping18");
            tellSlave.write(3);
            //i_slave_commsindex = 3; //tell slave its in main course

            Serial.println("ping19");
          }
          else
          {
            Serial.println("ping20");
            //don't do stuff if write is occuring (softwareserial library has issues)

            //course navigation
            switch (i_main_courseIndex)
            {
                Serial.println("ping21");
              case 0:
                {
                  Serial.println("ping22");
                  //lower arm and open rear hand on wall
                  b_servo_rearArmOn = true;
                  b_servo_rearHandOn = true;
                  b_servo_frontArmOn = true;
                  i_servo_rearArmPos = ci_servo_rearArmDown;
                  i_servo_rearHandPos = ci_servo_rearHandOpen;
                  i_servo_frontArmPos = ci_servo_frontArmUp;
                  ul_servo_timer = millis();
                  i_main_courseIndex++;
                  break;
                  Serial.println("ping23");
                }
              case 1:
                {
                  Serial.println("ping24");
                  //wait for servos to be in position, then disconnect
                  if (millis() - ul_servo_timer > ui_servo_waitTime)
                  {
                    b_servo_rearArmOn = false;
                    b_servo_rearHandOn = false;
                    b_servo_frontArmOn = false;
                    i_main_courseIndex++;
                    Serial.println("ping25");
                  }
                  break;
                }
              case 2:
                {
                  //communicate with slave, tell it to begin following wall
                  Serial.println("ping26");
                  if (i_slave_courseIndex != 1)
                  {
                    tellSlave.write(7);
                    //i_slave_commsindex = 7;
                    Serial.println("ping27");
                  }

                  else
                    i_main_courseIndex++;
                  Serial.println("ping28");
                  //no break intentional
                  if (b_sensor_rearSwitch)
                  {
                    //found cube
                    tellSlave.write(8);
                    i_main_courseIndex = 8;
                    b_cube_rearPossession = true;
                  }
                  else if (b_slave_isFinished)
                  {
                    //slave ran out of wall to follow, switch to reverse
                    i_main_courseIndex = 4;
                  }
                  Serial.println("ping30");
                  break;
                }
              case 3:
                {
                  Serial.println("ping29");
                  //checks for cube
                  if (b_sensor_rearSwitch)
                  {
                    //found cube
                    tellSlave.write(8);
                    i_main_courseIndex = 8;
                    b_cube_rearPossession = true;
                  }
                  else if (b_slave_isFinished)
                  {
                    //slave ran out of wall to follow, switch to reverse
                    i_main_courseIndex = 4;
                  }
                  Serial.println("ping30");
                  break;
                }
              case 4:
                {
                  Serial.println("ping31");
                  //case 4-8 is similar to 0-3, just in reverse direction
                  b_servo_frontArmOn = true;
                  b_servo_frontHandOn = true;
                  b_servo_rearArmOn = true;
                  i_servo_frontArmPos = ci_servo_frontArmDown;
                  i_servo_frontHandPos = ci_servo_frontHandOpen;
                  i_servo_rearArmPos = ci_servo_rearArmUp;
                  ul_servo_timer = millis();
                  i_main_courseIndex++;
                  Serial.println("ping32");
                  break;
                }
              case 5:
                {
                  Serial.println("ping33");
                  //wait for servos to be in position, then disconnect
                  if (millis() - ul_servo_timer > ui_servo_waitTime)
                  {
                    b_servo_frontArmOn = false;
                    b_servo_frontHandOn = false;
                    b_servo_rearArmOn = false;
                    i_main_courseIndex++;
                  }
                  Serial.println("ping34");
                  break;
                }
              case 6:
                {
                  Serial.println("ping35");
                  //communicate with slave, tell it to begin following wall backwards
                  if (i_slave_courseIndex != 2)
                  {
                    tellSlave.write(8);
                    //i_slave_commsindex = 8;
                    Serial.println("ping36");
                  }

                  else
                    i_main_courseIndex++;
                  Serial.println("ping37");
                  //no break intentional
                }
              case 7:
                {
                  //checks for cube
                  Serial.println("ping38");
                  if (b_sensor_frontSwitch)
                  {
                    //found cube
                    tellSlave.write(8);
                    i_main_courseIndex = 8;
                    b_cube_rearPossession = false;
                    Serial.println("ping39");
                  }
                  else if (b_slave_isFinished)
                  {
                    Serial.println("ping40");
                    //slave ran out of wall to follow, switch back to forwards

                    i_main_courseIndex = 0;
                  }
                  break;
                }

              case 8:
                {
                  //grab cube
                  if (b_cube_rearPossession)
                  {
                    b_servo_rearHandOn = true;
                    b_servo_rearArmOn = true;
                  }
                  else
                  {
                    b_servo_frontHandOn = true;
                    b_servo_frontArmOn = true;
                  }
                  i_servo_rearHandPos = ci_servo_rearHandClose;
                  i_servo_rearArmPos = ci_servo_rearArmDown;
                  i_servo_frontHandPos = ci_servo_frontHandClose;
                  i_servo_frontArmPos = ci_servo_frontArmDown;
                  ul_servo_timer = millis();
                  i_main_courseIndex++;
                  break;
                }
              case 9:
                {
                  //hand motors close, begin arm motor
                  if (millis() - ul_servo_timer > ui_servo_waitTime)
                  {
                    i_servo_rearArmPos = ci_servo_rearArmDrop;
                    i_servo_frontArmPos = ci_servo_frontArmDrop;
                    ul_servo_timer = millis();
                    i_main_courseIndex++;
                  }
                  break;
                }
              case 10:
                {
                  //servos in position,
                  if (millis() - ul_servo_timer > ui_servo_waitTime)
                  {
                    b_servo_rearHandOn = false;
                    b_servo_rearArmOn = false;
                    b_servo_frontHandOn = false;
                    b_servo_frontArmOn = false;
                    i_main_courseIndex++;
                  }
                  break;
                }
              case 11:
                {
                  //found a cube, get slave to stop moving
                  if (i_slave_courseIndex != 0)
                  {
                    Serial.println("ping41");
                    tellSlave.write(6);
                    //i_slave_commsindex = 6;
                    Serial.println("ping42");
                  }
                  else
                  {
                    //slave acknowledges and is stopped
                    i_main_courseIndex++;
                    Serial.println("ping43");
                  }
                  break;
                }
              case 12:
                {
                  //check that we still have cube
                  if (b_cube_rearPossession && (!b_sensor_rearSwitch))
                  {
                    //there is a problem
                    Serial.print("ERROR: CUBE SHOULD BE IN SWITCH BUT IS GONE");
                    CharliePlexM::Write(ci_charlieplex_errorLight, 1);
                  }
                  if ((!b_cube_rearPossession) && (!b_sensor_frontSwitch))
                  {
                    //there is a problem
                    Serial.print("ERROR: CUBE SHOULD BE IN FRONT SWITCH BUT IS MISSING");
                    CharliePlexM::Write(ci_charlieplex_errorLight, 1);
                  }
                  //continue on anyway
                  i_main_courseIndex++;
                  break;
                }
              case 13:
                {
                  //tell slave to find pyramid
                  if (i_slave_courseIndex != 3)
                  {
                    tellSlave.write(9);
                    //i_slave_commsindex = 9;
                  }
                  else
                  {
                    //fix the tip plate, since it can drop down over time
                    b_servo_tipPlateOn = true;
                    i_servo_tipPlatePos = ci_servo_tipPlateUp;
                  }
                  if (b_sensor_tipSwitch)
                  {
                    //pyramid is found! stop!
                    b_servo_tipPlateOn = false;
                    tellSlave.write(6);
                    //i_slave_commsindex = 6;
                    i_main_courseIndex++;
                  }
                  break;
                }
              case 14:
                {
                  //wait for response from slave
                  if (i_slave_courseIndex != 0)
                  {
                    tellSlave.write(6);
                    //i_slave_commsindex = 6;
                  }
                  else
                    i_main_courseIndex++;
                  break;
                }
              case 15:
                {
                  //stopped, just place the pyramid
                  if (b_cube_rearPossession)
                  {
                    b_servo_rearHandOn = true;
                    b_servo_rearArmOn = true;
                  }
                  else
                  {
                    b_servo_frontHandOn = true;
                    b_servo_frontArmOn = true;
                  }
                  i_servo_rearArmPos = ci_servo_rearArmDrop;
                  i_servo_frontArmPos = ci_servo_frontArmDrop;
                  ul_servo_timer = millis();
                  i_main_courseIndex++;
                  break;
                }
              case 16:
                {
                  //done moving arms
                  if (millis() - ul_servo_timer > ui_servo_waitTime)
                  {
                    b_servo_rearArmOn = false;
                    b_servo_frontArmOn = false;
                    i_main_courseIndex++;
                  }
                  break;
                }
              case 17:
                {
                  //drop pyramid
                  placeCube();
                  break;
                }


            }

          }
        }
        break;
      }
    case 2:
      {
        Serial.println("ping44");
        if (b_startButton_3secTimeUp)
        {
          //calibrate motors
          Serial.println("ping45");
          CharliePlexM::Write(ci_charlieplex_calibration, 1); //turn on calibration indicator
          if (i_slave_modeIndex != 2)
          {
            Serial.println("ping46");
            //if slave didn't give confirmation, tell it to
            tellSlave.write(4);
            //i_slave_commsindex = 4; //tell slave to go to calibration
            Serial.println("ping47");
          }
          if (b_slave_isFinished)
          {
            Serial.println("ping48");
            //calibration complete, back to main
            CharliePlexM::Write(ci_charlieplex_calibration, 0);
            i_main_modeIndex = 0;
          }
        }
        break;
      }
    case 3:
      {
        Serial.println("ping49");
        if (b_startButton_3secTimeUp)
        {
          Serial.println("ping50");
          //throw in whatever code you want to test but not mess with case 1 here
          if (i_slave_modeIndex != 3)
          {
            Serial.println("ping51");
            //tell slave to go in testing mode until aknowledged
            tellSlave.write(5);
            //i_slave_commsindex = 5;
            Serial.println("ping52");
          }
          else
          {
            Serial.println("ping53");
            //current testing function
            placeCube();

            //currently just closes servo hands
            //b_servo_frontHandOn = true;
            //b_servo_rearHandOn = true;
            //i_servo_frontHandPos = ci_servo_frontHandClose;
            //i_servo_rearHandPos = ci_servo_rearHandOpen;

            //b_servo_tipArmOn = true;
            //i_servo_tipArmPos = ci_servo_tipArmUp;

            //b_servo_rearArmOn = true;
            //i_servo_rearArmPos = 100;

            //            b_servo_frontArmOn=true;
            //            i_servo_frontArmPos=180;
          }
          //i_main_modeIndex = 0;
        }
        break;
      }
    default:
      {
        //you pushed the button too many times
        Serial.println("ping54");
        Serial.print("too many button pushes");
        i_main_modeIndex = 0;
        break;
      }
  }

  Serial.println("ping55");
  moveServos(); //tells the servos that are turned on to move to desired position, also auto turns them off (according to a bool for on off and int for position)
  Serial.println("ping56");
  //charlieplexing status updates here
  CharliePlexM::Write(ci_charlieplex_foundCube, (b_sensor_rearSwitch || b_sensor_frontSwitch));
  CharliePlexM::Write(ci_charlieplex_foundPyramid, b_sensor_tipSwitch);
  if (i_main_modeIndex == 0)
  {
    CharliePlexM::Write(9, 1);
  }
  else
  {
    CharliePlexM::Write(9, 0);
  }
  if (i_main_modeIndex == 1)
  {
    CharliePlexM::Write(6, 1);
  }
  else
  {
    CharliePlexM::Write(6, 0);
  }
  if (i_main_modeIndex == 3)
  {
    CharliePlexM::Write(3, 1);
  }
  else
  {
    CharliePlexM::Write(3, 0);
  }

  if ((i_main_courseIndex == 2) || (i_main_courseIndex == 3) || (i_main_courseIndex == 6) || (i_main_courseIndex == 7))
  {
    CharliePlexM::Write(ci_charlieplex_followWallParallel, 1);
  }
  else
    CharliePlexM::Write(ci_charlieplex_followWallParallel, 0);
  Serial.println("ping57");


  //debug stuff here
  Serial.println("ping58");
  if (millis() - ul_debug_secTimer > cui_debug_displayInterval)
  {
    ul_debug_secTimer = millis();
    CharliePlexM::Write(ci_charlieplex_errorLight, 0); //turn off the error light after a while so we know if error is still happening
    //servo debugging
#ifdef debug_servos
    Serial.print("Rear arm is on: ");
    Serial.print(b_servo_rearArmOn);
    Serial.print("  At position: ");
    Serial.print(i_servo_rearArmPos);
    Serial.print("     Rear hand is on: ");
    Serial.print(b_servo_rearHandOn);
    Serial.print("  At position: ");
    Serial.println(i_servo_rearHandPos);
    Serial.print("Front arm is on: ");
    Serial.print(b_servo_frontArmOn);
    Serial.print("  At position: ");
    Serial.print(i_servo_frontArmPos);
    Serial.print("     Front hand is on: ");
    Serial.print(b_servo_frontHandOn);
    Serial.print("  At position: ");
    Serial.println(i_servo_frontHandPos);
    Serial.print("Tip arm is on: ");
    Serial.print(b_servo_tipArmOn);
    Serial.print("  At position: ");
    Serial.print(i_servo_tipArmPos);
    Serial.print("     Tip plate is on: ");
    Serial.print(b_servo_tipPlateOn);
    Serial.print("  At position: ");
    Serial.println(i_servo_tipPlatePos);
#endif

    //limit switch debugging
#ifdef debug_limitSwitches
    Serial.print("Rear arm switch: ");
    Serial.print(b_sensor_rearSwitch);
    Serial.print("  Front arm switch: ");
    Serial.print(b_sensor_frontSwitch);
    Serial.print("  Pyramid tipping switch: ");
    Serial.println(b_sensor_tipSwitch);
#endif

    //comms port debugging
#ifdef debug_communications
    Serial.print("Last message recieved from slave: ");
    Serial.println(bt_slave_message);
#endif
  }

}




// --------------------------------------------------------------------------------------------------
//
// Just putting a break on code, scrolling outside of void tends to happen a lot and having
// a break between void and our functions is helpful.
//
// Put all extra functions below this break
//
// ---------------------------------------------------------------------------------------------------


void readSlave()
{
  Serial.println("ping59");
  //check for any new messages from the slave comm port
  //also have an interpretation of that message run in here
  //we're getting 1 byte of data, so something to take the number and change whatever variables the meaning affects
  b_slave_newCommand = false;
  b_slave_isFinished = false; //default sets to false, if true, will change for one loop then go back to false
  Serial.println("ping60");

  if (tellSlave.available())
  {
    Serial.println("ping61-----------------------------------------------------------------------------");
    bt_slave_lastMessage = bt_slave_message;
    if (bt_slave_lastMessage != bt_slave_message)
    {
      b_slave_newCommand = true;
    }

    bt_slave_message = tellSlave.read();
    //code to interpret message here
    //255 is error message?
    Serial.println("ping62");
    switch (bt_slave_message)
    {
        Serial.println("ping63");
      case 0:
        //default empty
        break;
      case 1:
        {
          //slave tells that it has finished its task
          if (b_slave_newCommand)
          {
            b_slave_isFinished = true;
          }

          bt_slave_message = 0; //clears receiving the message
          break;
        }
      case 2:
      case 3:
      case 4:
      case 5:
        {
          //case 2-5 reserved for telling mode indicator index, map to 0-3
          //now using these to indicate slave has confirmation of command
          i_slave_modeIndex = bt_slave_message - 2;
          break;
        }
      case 6:
      case 7:
      case 8:
      case 9:
      case 10:
      case 11:
      case 12:
      case 13:
      case 14:
      case 15:
      case 16:
      case 17:
      case 18:
      case 19:
      case 20:
        {
          //case 6-20 is to indicate progress though course index
          //master recieves slave's confirmation signal
          i_slave_courseIndex = bt_slave_message - 6;
          break;
        }
      case 255: //error message
        {
          Serial.println("Error from slave!");
          CharliePlexM::Write(ci_charlieplex_errorLight, 1);
          break;
        }
      default:
        {
          Serial.println("Error: unknown slave message");
          CharliePlexM::Write(ci_charlieplex_errorLight, 1);
          break;
        }
    }
    Serial.println("ping65");
  }
  Serial.println("------------------------------------------------------------");
  Serial.println("------------------------------------------------------------");
  //  tellSlave.write(i_slave_commsindex);
}


void turnOffAllServos()
{
  Serial.println("ping66");
  //does as it says
  //note, due to power draw issues if all 6 (8 if including other arduino) are on at once, keep as many off at once as possible

  if (b_servo_rearArmOn)
  {
    servo_rearArm.detach();
    b_servo_rearArmOn = false;
  }
  if (b_servo_rearHandOn)
  {
    servo_rearHand.detach();
    b_servo_rearHandOn = false;
  }
  if (b_servo_frontArmOn)
  {
    servo_frontArm.detach();
    b_servo_frontArmOn = false;
  }
  if (b_servo_frontHandOn)
  {
    servo_frontHand.detach();
    b_servo_frontHandOn = false;
  }
  if (b_servo_tipArmOn)
  {
    servo_tipArm.detach();
    b_servo_tipArmOn = false;
  }
  if (b_servo_tipPlateOn)
  {
    servo_tipPlate.detach();
    b_servo_tipPlateOn = false;
  }
  Serial.println("ping67");
}





void readLimitSwitches()
{
  Serial.println("ping68");
  //take sensor readings for all limit switches, includes debouncing
  //update to previous states
  b_switch_rearPrev = b_switch_rearCurrent;
  b_switch_frontPrev = b_switch_frontCurrent;
  b_switch_tipPrev = b_switch_tipCurrent;

  //get current state, use ! because 1 is off, 0 is triggered, I want opposite notation
  b_switch_rearCurrent = !digitalRead(ci_pin_rearSwitch);
  b_switch_frontCurrent = !digitalRead(ci_pin_frontSwitch);
  b_switch_tipCurrent = !digitalRead(ci_pin_tipSwitch);

  //debouncing check
  //rear
  if (b_switch_rearCurrent != b_switch_rearPrev)
  {
    //reset debounce timer
    ul_switch_debounceRear = millis();
  }
  else if (millis() - ul_switch_debounceRear > ci_switch_debounceTime)
  {
    //has passed debouncing, only update if updating needed
    if (b_sensor_rearSwitch != b_switch_rearCurrent)
    {
      b_sensor_rearSwitch = b_switch_rearCurrent;
    }
  }

  //front
  if (b_switch_frontCurrent != b_switch_frontPrev)
  {
    //reset debounce timer
    ul_switch_debounceFront = millis();
  }
  else if (millis() - ul_switch_debounceFront > ci_switch_debounceTime)
  {
    //has passed debouncing, only update if updating needed
    if (b_sensor_frontSwitch != b_switch_frontCurrent)
    {
      b_sensor_frontSwitch = b_switch_frontCurrent;
    }
  }

  //tip
  if (b_switch_tipCurrent != b_switch_tipPrev)
  {
    //reset debounce timer
    ul_switch_debounceTip = millis();
  }
  else if (millis() - ul_switch_debounceTip > ci_switch_debounceTime)
  {
    //has passed debouncing, only update if updating needed
    if (b_sensor_tipSwitch != b_switch_tipCurrent)
    {
      b_sensor_tipSwitch = b_switch_tipCurrent;
    }
  }
  Serial.println("ping69");

}


void moveServos()
{
  Serial.println("ping70");
  //writes position to servos, when active
  if (b_servo_rearArmOn)
  {
    if (!servo_rearArm.attached())
    {
      //nothing currently attached, add servo
      servo_rearArm.attach(ci_pin_rearArm);
    }
    //write desired position to servo
    servo_rearArm.write(i_servo_rearArmPos);
  }
  else if (servo_rearArm.attached())
  {
    //also updates to detach servo when bool changed to false
    servo_rearArm.detach();
  }

  if (b_servo_rearHandOn)
  {
    if (!servo_rearHand.attached())
    {
      //nothing currently attached, add servo
      servo_rearHand.attach(ci_pin_rearHand);
    }
    //write desired position to servo
    servo_rearHand.write(i_servo_rearHandPos);
  }
  else if (servo_rearHand.attached())
  {
    servo_rearHand.detach();
  }

  if (b_servo_frontArmOn)
  {
    if (!servo_frontArm.attached())
    {
      //nothing currently attached, add servo
      servo_frontArm.attach(ci_pin_frontArm);
    }
    //write desired position to servo
    servo_frontArm.write(i_servo_frontArmPos);
  }
  else if (servo_frontArm.attached())
  {
    servo_frontArm.detach();
  }

  if (b_servo_frontHandOn)
  {
    if (!servo_frontHand.attached())
    {
      //nothing currently attached, add servo
      servo_frontHand.attach(ci_pin_frontHand);
    }
    //write desired position to servo
    servo_frontHand.write(i_servo_frontHandPos);
  }
  else if (servo_frontHand.attached())
  {
    servo_frontHand.detach();
  }

  if (b_servo_tipArmOn)
  {
    if (!servo_tipArm.attached())
    {
      //nothing currently attached, add servo
      servo_tipArm.attach(ci_pin_tipArm);
    }
    //write desired position to servo
    servo_tipArm.write(i_servo_tipArmPos);
  }
  else if (servo_tipArm.attached())
  {
    servo_tipArm.detach();
  }

  if (b_servo_tipPlateOn)
  {
    if (!servo_tipPlate.attached())
    {
      //nothing currently attached, add servo
      servo_tipPlate.attach(ci_pin_tipPlate);
    }
    //write desired position to servo
    servo_tipPlate.write(i_servo_tipPlatePos);
  }
  else if (servo_tipPlate.attached())
  {
    servo_tipPlate.detach();
  }
  Serial.println("ping71");
}

void readyPyramid()
{
  Serial.println("ping72");
  //turns on servo motor for pyramid tipper and holds it at vertical
  //do this before pyramid gets driven over
  //reason is to prevent driving too far, and have solid surface for limit switch to trigger (not certain if needed, but it shouldnt hurt)
  //also raises plate
  //I reccomend using dropPyramid and raisePlate
  Serial.println("ping73");

}

void placeCube()
{
  Serial.println("ping74");
  //final function call in course navigation
  //assuming pyramid in place, it drops plate, tips pyramid, releases cube and releases pyramid
  switch (i_pyramid_index)
  {
    case 0:
      {
        //drop plate
        b_servo_tipPlateOn = true;
        i_servo_tipPlatePos = ci_servo_tipPlateDown;
        ul_pyramid_timer = millis();
        i_pyramid_index++;
        break;
      }
    case 1:
      {
        //timer for tip plate to get in position
        if (millis() - ul_pyramid_timer > ui_servo_waitTime)
        {
          i_pyramid_index++;
        }
        break;
      }
    case 2:
      {
        //tip pyramid with arm
        b_servo_tipArmOn = true;
        i_servo_tipArmPos = ci_servo_tipArmUp;
        ul_pyramid_timer = millis();
        i_pyramid_index++;
        break;
      }
    case 3:
      {
        //timer that pyramid was tipped
        if (millis() - ul_pyramid_timer > ui_servo_waitTime)
        {
          i_pyramid_index++;
        }
        break;
      }
    case 4:
      {
        //drop cube
        if(b_cube_rearPossession)
        {
          b_servo_rearHandOn=true;
        }
        else
        {
          b_servo_frontHandOn=true;
        }
        i_servo_rearHandPos = ci_servo_rearHandOpen; //write both open, only one will be on from holding the cube
        i_servo_frontHandPos = ci_servo_frontHandOpen;
        ul_pyramid_timer = millis();
        i_pyramid_index++;
        break;
      }
    case 5:
      {
        //wait for cube to get under pyramid
        if (millis() - ul_pyramid_timer > ui_pyramid_cubeDropTime)
        {
          b_servo_rearHandOn = false; //turn off hand motors, no longer needed
          b_servo_frontHandOn = false;
          i_pyramid_index++;
        }
        break;
      }
    case 6:
      {
        //drop pyramid
        i_servo_tipArmPos = ci_servo_tipArmDown; //already true from earlier
        ul_pyramid_timer = millis();
        i_pyramid_index++;
        break;
      }
    case 7:
      {
        //wait for pyramid to drop
        if (millis() - ul_pyramid_timer > ui_servo_waitTime)
        {
          b_servo_tipArmOn = false; //done using tip arm, turn off
          i_pyramid_index++;
        }
        break;
      }
    case 8:
      {
        //raise plate up
        i_servo_tipPlatePos = ci_servo_tipPlateUp;
        ul_pyramid_timer = millis();
        i_pyramid_index++;
        break;
      }
    case 9:
      {
        //wait for plate to raise
        if (millis() - ul_pyramid_timer > ui_servo_waitTime)
        {
          b_servo_tipPlateOn = false;
          i_pyramid_index++;
        }
        break;
      }
    case 10:
      {
        //completed course, awaiting reset
        CharliePlexM::Write(ci_charlieplex_successLight, 1);
        Serial.println("COURSE COMPLETE");
        break;
      }
    default:
      {
        //error
        CharliePlexM::Write(ci_charlieplex_errorLight, 1);
        Serial.println("Error: unknown pyramid drop index");
        break;
      }

  }
  Serial.println("ping75");
}



// --------------------------------------------------------------------------------------------------
//
// Code after this point is optional, we have completed requirements, but it would be fun if these were also included
// Any functions you have to make, please add them above this break (unless the function is also non-manditory)
//
// ---------------------------------------------------------------------------------------------------



void DANCE()
{
  //we're done, party! Swing arms around, flash charlieplex randomly, tell slave to drive in circles

}

