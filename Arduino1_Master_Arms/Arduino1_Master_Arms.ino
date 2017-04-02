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
#define debug_limitSwitches
//#define debug_communications
const unsigned int cui_debug_displayInterval = 1000; //Time between outputs displayed when debugging is enabled, in ms


//Pin mapping
SoftwareSerial tellSlave(5,6); //comm ports with arduino 2 for communication
const int ci_pin_startButton=A0;
const int ci_pin_charlieplex1 = A2; //we don't have enough digital pins for everything, so analog is being used as digital
const int ci_pin_charlieplex2 = A3;
const int ci_pin_charlieplex3 = A4;
const int ci_pin_charlieplex4 = A5;
const int ci_pin_rearArm = 4;
const int ci_pin_rearHand = 3;
const int ci_pin_frontArm = 13;
const int ci_pin_frontHand = 12;
const int ci_pin_tipArm = 8;
const int ci_pin_tipPlate = 2;
const int ci_pin_rearSwitch = 9;
const int ci_pin_frontSwitch = 10;
const int ci_pin_tipSwitch = 11;

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
//LEDs 1 and 4 still free for stuff

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
const int ci_servo_armsUp;  //constant values used to control servo positions, may need to use different for rear and front
const int ci_servo_armsDown;
const int ci_servo_rearArmDrop;
const int ci_servo_frontArmDrop;
const int ci_servo_openHand;
const int ci_servo_closeHand;
const int ci_servo_tipPlateUp;
const int ci_servo_tipPlateDown;
const int ci_servo_tipArmDown=90;
const int ci_servo_tipArmUp;
int i_servo_rearArmPos;
int i_servo_rearHandPos;
int i_servo_frontArmPos;
int i_servo_frontHandPos;
int i_servo_tipArmPos;
int i_servo_tipPlatePos;

//keep adding varibles you need in cadegories labeled like this


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600); //for debugging
  tellSlave.begin(9600);

  //set pinmodes (note servos will be switching throughout code)
  pinMode(ci_pin_startButton, INPUT_PULLUP);
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
  //servo_rearArm.attach(ci_pin_rearArm);
  //servo_rearHand.attach(ci_pin_rearHand);
  //servo_frontArm.attach(ci_pin_frontArm);
  //servo_frontHand.attach(ci_pin_frontHand);
  //servo_tipArm.attach(ci_pin_tipArm);
  servo_tipPlate.attach(ci_pin_tipPlate);

  //initialize variables
  i_main_modeIndex = 0;
  ul_startButton_3secTime = 0;
  b_startButton_doOnce = false;
  b_startButton_3secTimeUp = false;
  b_main_tellSlaveIndexChange = true;
  b_servo_rearArmOn = true;
  b_servo_rearHandOn = true;
  b_servo_frontArmOn = true;
  b_servo_frontHandOn = true;
  b_servo_tipArmOn = true;
  b_servo_tipPlateOn = true;
  ul_debug_secTimer = 0;

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
        //servo_tipArm.attach(ci_pin_tipArm);
        servo_tipPlate.write(18 0);
        break;
      } 
    case 1:
      {
        if (b_startButton_3secTimeUp)
        {
          //course navigation
          i_main_modeIndex = 0;
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
          //throw in whatever code you want to test but not mess with case 1 here
          //we can probably keep going up in numbers
          i_main_modeIndex = 0;
        }
        break;
      }
    default:
      {
        //you pushed the button too many times
        i_main_modeIndex = 0;
        break;
      }
  }

b_sensor_rearSwitch=digitalRead(ci_pin_rearSwitch);
b_sensor_frontSwitch=digitalRead(ci_pin_frontSwitch);
b_sensor_tipSwitch=digitalRead(ci_pin_tipSwitch);


  //debug stuff here

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
        CharliePlexM::Write(ci_charlieplex_errorLight,1);
        break;
      }
    }
  }
}


void turnOffAllServos()
{
  //does as it says
  //note, due to power draw issues if all 6 (8 if including other arduino) are on at once, keep as many off at once as possible
  servo_rearArm.detach();
  servo_rearHand.detach();
  servo_frontArm.detach();
  servo_frontHand.detach();
  servo_tipArm.detach();
  servo_tipPlate.detach();

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
void armsUp()
{
  //sets both arms up, in vertical position. Storage? freely roaming areana?

}

void rearArmDown()
{
  //sets the rear arm at wall height, and the front arm up

}

void frontArmDown()
{
  //sets the front arm on wall height and raises rear arm

}

void openHands()
{
  //opens hands on both arms,should be >180 deg, I'm thinking 225 deg?

}

void closeRearHand()
{
  //closes rear hand to grab cube

}

void closeFrontHand()
{
  //closes front hand to grab cube

}

void rearArmOverChute()
{
  //moves rear arm over chute, moves front to up to prevent collision

}

void frontArmOverChute()
{
  //moves front arm over chute position, and rear to up for no collision

}

void readyPyramid()
{
  //turns on servo motor for pyramid tipper and holds it at vertical
  //do this before pyramid gets driven over
  //reason is to prevent driving too far, and have solid surface for limit switch to trigger (not certain if needed, but it shouldnt hurt)
  //also raises plate
  //I reccomend using dropPyramid and raisePlate

}

void dropPlate()
{
  //drops pyramid plate to position behind pyramid needed to tip pyramid

}

void tipPyramid()
{
  //turns servo to tip pyramid over

}

void dropPyramid()
{
  //turns pyramid tipper back, returning pyramid to floor

}

void raisePlate()
{
  //raises the plate to make room for pyramid to drive in

}

bool wait(unsigned int ui_wait_millisTime)
{
  //returns true when time given after its first call has passed
  //need to set b_wait_start=true before calling... or something. Whoever works on this can figure it out (will probably be me anyway)
  //I think this will be useful to have a function that acts similar to delay, but we can still read sensors and stuff while running a timer

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

