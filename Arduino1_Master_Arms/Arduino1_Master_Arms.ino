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
#define debug_servos
//#define debug_limitSwitches
//#define debug_communications
#define using_debug_value
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
const int ci_charlieplex_AEmode = 10; //on when AE detector, off when IO detector
const int ci_charlieplex_calibration = 7; //turns on when in middle of a calibration
const int ci_charlieplex_setupLight = 1; //turns on when set up is initiating
//LED 4 still free for stuff, 1 can also blink or something for re-use

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
bool b_sensor_rearSwtichPrev;
bool b_sensor_frontSwitchPrev;
bool b_sensor_tipSwitchPrev;
byte bt_slave_message; //stores message recieved from the slave arduino 2

//Main loop variables
int i_main_modeIndex; //switch statement to tell what is going on will use this variable (sit idle, course navigation, calibration, etc)
int i_main_courseIndex; // switch statement embedded in course navigation will read this, used to determine exactly what's going on inside course
unsigned long ul_startButton_3secTime; //startButton stuff is used to count button presses, don't touch these variables
bool b_startButton_3secTimeUp;
bool b_startButton_doOnce;
bool b_main_calibrationStart; //used as a one off variable to start calibration functions, resets to false when idle
bool b_main_tellSlaveIndexChange; //changes to true when we need to tell the slave a mode change (only tells once)
unsigned long ul_debug_secTimer; //used to control debugging outputs to be every 2 seconds, and not constantly

//debouncing variables
long l_debouceLimitSwitches_currentmillis;
const int ci_debounceLimitSwitches_delay = 50;
bool b_resetDelay;

//Servo control variables
bool b_servo_rearArmOn; //bool variables we use to turn on and off servos, useful for debugging/clean code?
bool b_servo_rearHandOn;
bool b_servo_frontArmOn;
bool b_servo_frontHandOn;
bool b_servo_tipArmOn;
bool b_servo_tipPlateOn;
const int ci_servo_rearArmUp = 90; //constant values used to control servo positions, may need to use different for rear and front
const int ci_servo_frontArmUp = 100;
const int ci_servo_rearArmDown;
const int ci_servo_frontArmDown;
const int ci_servo_rearArmDrop;
const int ci_servo_frontArmDrop;
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
int i_testing_intval;

//placing cube under pyramid variables
int i_pyramid_index;
unsigned long ul_pyramid_timer; //used to wait for servos after each command before detaching and moving them on
unsigned int ui_pyramid_cubeDropTime; //time to wait after realeasing cube to dropping pyramid

//Grabbing cube off wall variables
int i_cubegrab_index;
unsigned long ul_cubegrab_timer;

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
  b_main_tellSlaveIndexChange = true;
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
  i_cubegrab_index = 0;


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

  // Button presses (affects i_main_modeIndex)
  // 0 = idle
  // 1 = run main program
  // 2 = calibrate motors
  // 3 = testing functions, edit in whatever you want to test here, we'll see how this goes

  //button press counter
  if ((millis() - ul_startButton_3secTime) > 3000)
  {
    b_startButton_3secTimeUp = true;
    b_main_tellSlaveIndexChange = true;
  }

  // button-based mode selection
  if (CharliePlexM::ui_Btn)
  {
    if (b_startButton_doOnce == false)
    {
      b_startButton_doOnce = true;
      i_main_modeIndex++;
      i_main_modeIndex = i_main_modeIndex & 7;
      ul_startButton_3secTime = millis();
      b_startButton_3secTimeUp = false;
      b_main_calibrationStart = false;
    }
  }
  else
  {
    b_startButton_doOnce = LOW;
  }



  //communication with other board all the time, and read all sensors
  //readSlave();
  //readLimitSwitches();
  if (b_main_tellSlaveIndexChange)
  {
    //tell slave when mode changes
    b_main_tellSlaveIndexChange = false;
    //figure out comminication code/language and tell slave desired message
  }


  //main switch statement to drive mode that is operating in
  //will not be active until after 3 second time out from last button press (other than case 0)
  switch (i_main_modeIndex)
  {
    case 0:
      {
        //sits idle, default on start up
        turnOffAllServos();
        break;
      }
    case 1:
      {
        if (b_startButton_3secTimeUp)
        {
          //course navigation
          //currently just closes servo hands
          b_servo_frontHandOn = true;
          b_servo_rearHandOn = true;
          i_servo_frontHandPos = ci_servo_frontHandClose;
          i_servo_rearHandPos = ci_servo_rearHandClose;

          //b_servo_tipArmOn=true;
          //i_servo_tipArmPos=140;
        }
        break;
      }
    case 2:
      {
        if (b_startButton_3secTimeUp)
        {
          //calibrate motors
          CharliePlexM::Write(ci_charlieplex_calibration, 1); //turn on calibration indicator
          tellSlave.write(2); //tell slave to do that
        }
        break;
      }
    case 3:
      {
        if (b_startButton_3secTimeUp)
        {
          if (!b_servo_rearArmOn)
          {
            b_servo_rearArmOn = true;
          }
          if (!b_servo_rearHandOn)
          {
            b_servo_rearHandOn = true;
          }
          //throw in whatever code you want to test but not mess with case 1 here
          //we can probably keep going up in numbers
          placeCube(); //current testing
          //i_main_modeIndex = 0;
        }
        break;
      }
    default:
      {
        //you pushed the button too many times
        Serial.print("too many button pushes");
        i_main_modeIndex = 0;
        break;
      }
  }

  b_sensor_rearSwitch = digitalRead(ci_pin_rearSwitch);
  b_sensor_frontSwitch = digitalRead(ci_pin_frontSwitch);
  b_sensor_tipSwitch = digitalRead(ci_pin_tipSwitch);

  moveServos(); //tells the servos that are turned on to move to desired position, also auto turns them off (according to a bool for on off and int for position)


  //debug stuff here
#ifdef using_debug_value
  if (Serial.available())
  {
    i_testing_intval = Serial.read();
  }
#endif

  if (millis() - ul_debug_secTimer > cui_debug_displayInterval)
  {
    ul_debug_secTimer = millis();
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

/*making a bunch of functions that activate and deactivate the servos, and change their respective booleans at the same time to reduce human error
  //if compilation size becomes too big a problem they can be removed
  //took a closer look at home code was working, and these are not nessicary
  void attachFrontArm()
  {
	servo_frontArm.attach(ci_pin_frontArm);
	b_servo_frontArmOn = true;
  }

  void attachRearArm()
  {
	servo_rearArm.attach(ci_pin_rearArm);
	b_servo_frontArmOn = true;
  }

  void attachFrontHand() {
	servo_frontHand.attach(ci_pin_frontHand);
	b_servo_frontHandOn = true;
  }

  void attachRearHand() {
	servo_rearHand.attach(ci_pin_rearHand);
	b_servo_rearHandOn = true;
  }

  void attachTipPlate() {
	servo_tipPlate.attach(ci_pin_tipPlate);
	b_servo_tipPlateOn = true;
  }

  void attachTipArm() {
	servo_tipArm.attach(ci_pin_tipArm);
	b_servo_tipArmOn = true;
  }

  void detachFrontArm()
  {
	servo_frontArm.detach();
	b_servo_frontArmOn = false;
  }

  void detachRearArm()
  {
	servo_rearArm.detach();
	b_servo_frontArmOn = false;
  }

  void detachFrontHand() {
	servo_frontHand.detach();
	b_servo_frontHandOn = false;
  }

  void detachRearHand() {
	servo_rearHand.detach();
	b_servo_rearHandOn = false;
  }

  void detachTipPlate() {
	servo_tipPlate.detach();
	b_servo_tipPlateOn = false;
  }

  void detachTipArm() {
	servo_tipArm.detach();
	b_servo_tipArmOn = false;
  }
*/
void readSlave()
{
  //check for any new messages from the slave comm port
  //also have an interpretation of that message run in here
  //we're getting 1 byte of data, so something to take the number and change whatever variables the meaning affects
  if (tellSlave.available())
  {
    bt_slave_message = tellSlave.read();

    //code to interpret message here
    //255 is error message?
    switch (bt_slave_message)
    {
      case 0:
      case 1:
      case 2:
      case 3:
      case 4:
      case 5:
        {
          //case 0-5 reserved for telling mode indicator index
          //...wait, that only makes sense for the slave hearing from master
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
  }
}

void turnOffAllServos()
{
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
}

/*void limitSwitchDebounce()
  {
  //any readings we get from limit switches have to do through this before we use the
  if (b_resetDelay == LOW)
  { l_debouceLimitSwitches_currentmillis = millis();
    b_resetDelay = HIGH;
  }

  if ( millis() - l_debouceLimitSwitches_currentmillis > ci_debounceLimitSwitches_delay)
  {

    if (  b_sensor_rearSwitchPrev == digitalRead(ci_pin_rearSwitch))
    {
      b_sensor_rearSwitch = b_sensor_rearSwitchPrev;
    }
    else
    {
      b_sensor_rearSwitch = !b_sensor_rearSwitchPrev;
    }

    if (  b_sensor_frontSwitchPrev == digitalRead(ci_pin_frontSwitch))
    {
      b_sensor_frontSwitch = b_sensor_frontSwitchPrev;
    }
    else
    {
      b_sensor_frontSwitch = !b_sensor_frontSwitchPrev;
    }

    if (  b_sensor_tipSwitchPrev == digitalRead(ci_pin_tipSwitch))
    {
      b_sensor_tipSwitch = b_sensor_tipSwitchPrev;
    }
    else
    {
      b_sensor_tipSwitch = !b_sensor_tipSwitchPrev;
    }

    b_resetDelay = LOW;

  }


  }

  void readLimitSwitches()
  {
  //take sensor readings for all limit switches, should use debounce as part of this function
  b_sensor_rearSwitchPrev = digitalRead(ci_pin_rearSwitch);
  b_sensor_frontSwitchPrev = digitalRead(ci_pin_frontSwitch);

  b_sensor_tipSwitchPrev = digitalRead(ci_pin_tipSwitch);

  limitSwitchDebounce();


  }
*/

void moveServos()
{
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
}

void readyPyramid()
{
  //turns on servo motor for pyramid tipper and holds it at vertical
  //do this before pyramid gets driven over
  //reason is to prevent driving too far, and have solid surface for limit switch to trigger (not certain if needed, but it shouldnt hurt)
  //also raises plate
  //I reccomend using dropPyramid and raisePlate

}

void placeCube()
{
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
        //ensure arms are in correct position, only the arm that is active should move
        i_servo_rearArmPos = i_testing_intval;
        i_servo_frontArmPos = ci_servo_frontArmDrop;
        ul_pyramid_timer = millis();
        i_pyramid_index++;
        break;
      }
    case 5:
      {
        //timer that arms got there
        if (millis() - ul_pyramid_timer > ui_servo_waitTime)
        {
          i_pyramid_index++;
        }
        break;
      }
    case 6:
      {
        //drop cube
        i_servo_rearHandPos = ci_servo_rearHandOpen; //write both open, only one will be on from holding the cube
        i_servo_frontHandPos = ci_servo_frontHandOpen;
        ul_pyramid_timer = millis();
        i_pyramid_index++;
        break;
      }
    case 7:
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
    case 8:
      {
        //drop pyramid
        i_servo_tipArmPos = ci_servo_tipArmDown; //already true from earlier
        ul_pyramid_timer = millis();
        i_pyramid_index++;
        break;
      }
    case 9:
      {
        //wait for pyramid to drop
        if (millis() - ul_pyramid_timer > ui_servo_waitTime)
        {
          b_servo_tipArmOn = false; //done using tip arm, turn off
          i_pyramid_index++;
        }
        break;
      }
    case 10:
      {
        //raise plate up
        i_servo_tipPlatePos = ci_servo_tipPlateUp;
        ul_pyramid_timer = millis();
        i_pyramid_index++;
        break;
      }
    case 11:
      {
        //wait for plate to raise
        if (millis() - ul_pyramid_timer > ui_servo_waitTime)
        {
          b_servo_tipPlateOn = false;
          i_pyramid_index++;
        }
        break;
      }
    case 12:
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
}

bool Click_n_GrabCube() {
  switch (i_cubegrab_index)
  {
    case 0:
      {
        if (b_sensor_rearSwitch) {
          i_cubegrab_index++;
        }
        break;
      }

  }
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

