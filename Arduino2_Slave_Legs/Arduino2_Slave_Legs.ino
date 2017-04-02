// MSE2202 Design Project Group 15
// Slave code for arduino 2 (arduino 1 is running on master code)
// Slave version, has driving motors, ultrasonic, IR, and IR switch


//GUYS FOLLOW THIS NAMING CONVENTION
// <letter for type of variable>_<function variable is for>_<Custom name of choice>
//Example: unsigned long ul_ultrasonic_distance

#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <SoftwareSerial.h>

//DEBUGGING
// uncomment lines based on what needs debugging, will print values to serial every loop
#define debug_ultrasonic
//#define debug_motors
//#define debug_encoders
//#define debug_IR
//#define debug_communciations
const unsigned int cui_debug_displayInterval = 1000; //time between display on debug output, in ms


//Pin mapping
SoftwareSerial tellMaster(A2, A3); //comm ports with arduino 2 for communication
const int ci_pin_usTrigLeftFront = 8; //might want to make this an array instead of 5 seperate variables
const int ci_pin_usEchoLeftFront = 9;
const int ci_pin_usTrigLeftRear = 2;
const int ci_pin_usEchoLeftRear = 3;
const int ci_pin_usTrigRight = 4;
const int ci_pin_usEchoRight = 5;
const int ci_pin_usTrigRear = 12;
const int ci_pin_usEchoRear = 13;
const int ci_pin_usTrigFront = 10;
const int ci_pin_usEchoFront = 11;
const int ci_pin_leftMotor = 6; //Left and right motors could be flipped
const int ci_pin_rightMotor = 7;
I2CEncoder encoder_leftMotor;
I2CEncoder encoder_rightMotor;
const int ci_I2C_SDA = A4;         // I2C data = white    //these lines may need to be included to make encoders work, idk...
const int ci_I2C_SCL = A5;         // I2C clock = yellow
SoftwareSerial pin_IR(14, 40); //A0 is D14, 40 is non-existant (we only need to read, not write)
const int ci_pin_IRswitch=A1;

//Data from sensors
Servo servo_leftMotor;
Servo servo_rightMotor;
byte bt_master_message;
byte bt_sensor_IR;
bool b_sensor_IRswitch;
long l_sensor_usFront;
long l_sensor_usRight;
long l_sensor_usRear;
long l_sensor_usLeftFront;
long l_sensor_usLeftRear;
char c_serial_input;

//Main loop variables
int i_main_modeIndex;
int i_main_courseIndex;
unsigned long ul_debug_secTimer;
bool b_main_modeIndexChange; //similar to calibration variable in master, make true whenever new modeIndex comes through com port
unsigned long ul_main_calibrationTime; //used for calibrating motors

//Motor speed variables
const unsigned int cui_motor_forwardSpeed = 2000;
const unsigned int cui_motor_reverseSpeed;
const unsigned int cui_motor_stop = 1500;
unsigned int ui_motor_leftOffset;
unsigned int ui_motor_rightOffset;
unsigned int ui_motor_rightSpeed; //set this to change how fast the motors are going
unsigned int ui_motor_leftSpeed;
bool b_motor_enabled; //will not alter motor speeds unless true
long l_motor_leftPosition;
long l_motor_rightPosition;

//EEPROM stuff for calibration of motors
const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;
byte b_LowByte;
byte b_HighByte;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600); //for debugging
  tellMaster.begin(9600);
  pin_IR.begin(2400);

  //set pinmodes
  pinMode(ci_pin_usTrigLeftFront, OUTPUT);
  pinMode(ci_pin_usEchoLeftFront, INPUT);
  pinMode(ci_pin_usTrigLeftRear, OUTPUT);
  pinMode(ci_pin_usEchoLeftRear, INPUT);
  pinMode(ci_pin_usTrigRight, OUTPUT);
  pinMode(ci_pin_usEchoRight, INPUT);
  pinMode(ci_pin_usTrigFront, OUTPUT);
  pinMode(ci_pin_usEchoFront, INPUT);
  pinMode(ci_pin_usTrigRear, OUTPUT);
  pinMode(ci_pin_usEchoRear, INPUT);
  pinMode(ci_pin_IRswitch, INPUT);
  pinMode(ci_pin_rightMotor, OUTPUT);
  pinMode(ci_pin_leftMotor, OUTPUT);


  //more initialization
  servo_leftMotor.attach(ci_pin_leftMotor);
  servo_rightMotor.attach(ci_pin_rightMotor);
  encoder_leftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_leftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_rightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_rightMotor.setReversed(true);  // adjust for positive count when moving forward


  //initialize varaibles
  i_main_modeIndex = 0;
  i_main_courseIndex = 0;
  ul_debug_secTimer = 0;
  b_motor_enabled=true;
  b_main_modeIndexChange = false;


  //eeprom set up for motors offset
  b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
  ui_motor_leftOffset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
  ui_motor_rightOffset = word(b_HighByte, b_LowByte);

}

void loop() {
  // code runs similar to master, instead of button input, gets instructions from serial port
  //reads data from all sensors
  //readMaster();
  pingAll();
  //readIR();
  readEncoders();
  if (Serial.available())
  {
    c_serial_input = Serial.read();
    if (c_serial_input == '1')
    {
      i_main_modeIndex = 1;
    }
    if (c_serial_input == '2')
    {
      i_main_modeIndex = 2;
    }
    if (c_serial_input == '3')
    {
      i_main_modeIndex = 3;
    }
    b_main_modeIndexChange = true;
  }
  switch (i_main_modeIndex)
  {
    case 0:
      {
        //sitting idle
        encoder_leftMotor.zero();
        encoder_rightMotor.zero();
        detachMotors();
        break;
      }
    case 1:
      {
        //main course navigation
        switch (i_main_courseIndex)
        {
          case 0:
            {
              //wait for instructions
              encoder_leftMotor.zero();
              encoder_rightMotor.zero();
              detachMotors();
              break;
            }
          case 1:
            {
              //begin following wall parallel, forwards
              break;
            }
          case 2:
            {
              //begin following wall, parrallel backwards
              break;
            }
          default:
            {
              //mis-communication or something to cause error
              //tellMaster.write(255);
              Serial.println("Error! unknown courseIndex");
              break;
            }
        }
        break;
      }
    case 2:
      {
        //calibrate motors
        if (b_main_modeIndexChange)
        {
          b_main_modeIndexChange = false;
          encoder_leftMotor.zero();
          encoder_rightMotor.zero();
          ul_main_calibrationTime = millis();
          attachMotors();
          servo_leftMotor.writeMicroseconds(cui_motor_forwardSpeed);
          servo_rightMotor.writeMicroseconds(cui_motor_forwardSpeed);
        }
        else if ((millis() - ul_main_calibrationTime) > 5000)
        {
          servo_leftMotor.writeMicroseconds(cui_motor_stop);
          servo_rightMotor.writeMicroseconds(cui_motor_stop);
          l_motor_leftPosition = encoder_leftMotor.getRawPosition();
          l_motor_rightPosition = encoder_rightMotor.getRawPosition();
          if (l_motor_leftPosition > l_motor_rightPosition)
          {
            // May have to update this if different calibration time is used
            //these may need updating since its a different bot at a different speed that's running this
            ui_motor_rightOffset = 0;
            ui_motor_leftOffset = (l_motor_leftPosition - l_motor_rightPosition) / 4;
          }
          else
          {
            // May have to update this if different calibration time is used
            ui_motor_rightOffset = (l_motor_rightPosition - l_motor_leftPosition) / 4;
            ui_motor_leftOffset = 0;
          }

          EEPROM.write(ci_Right_Motor_Offset_Address_L, lowByte(ui_motor_rightOffset));
          EEPROM.write(ci_Right_Motor_Offset_Address_H, highByte(ui_motor_rightOffset));
          EEPROM.write(ci_Left_Motor_Offset_Address_L, lowByte(ui_motor_leftOffset));
          EEPROM.write(ci_Left_Motor_Offset_Address_H, highByte(ui_motor_leftOffset));
        }
        break;
      }
    case 3:
      {
        //extra for testing or something
        attachMotors();
        servo_leftMotor.writeMicroseconds(cui_motor_forwardSpeed);
        break;
      }
    default:
      {
        //this really shouldn't be possible, something is broken
        tellMaster.write(255);
        Serial.println("ERROR unknown mode index");
        break;
      }
  }


  //debugging stuff
  if (millis() - ul_debug_secTimer > cui_debug_displayInterval)
  {
    ul_debug_secTimer = millis();
Serial.print(l_sensor_usRear);
    //ultrasonic debug
#ifdef debug_ultrasonic
    Serial.println("Debug ultrasonic values:");
    Serial.print("  Front: ");
    //Serial.print(l_sensor_usFront);
    Serial.print("   in cm: ");
    Serial.println(l_sensor_usFront / 58.2);
    Serial.print("  Rear: ");
    //Serial.print((double)l_sensor_usRear);
    Serial.print("   in cm: ");
    Serial.println(l_sensor_usRear / 58.2);
    Serial.print("  Left front: ");
    Serial.print(l_sensor_usLeftFront);
    Serial.print("   in cm: ");
    Serial.println(l_sensor_usLeftFront / 58.2);
    Serial.print("  Left rear: ");
    Serial.print(l_sensor_usLeftRear);
    Serial.print("   in cm: ");
    Serial.println(l_sensor_usLeftRear / 58.2);
    Serial.print("  Right: ");
    Serial.print(l_sensor_usRight);
    Serial.print("   in cm: ");
    Serial.println(l_sensor_usRight / 58.2);
#endif

    //motor debugging
#ifdef debug_motors
    Serial.print("Motors enabled: ");
    Serial.println(b_motor_enabled);
    Serial.print("Left motor speed: ");
    Serial.print(ui_motor_leftSpeed);
    Serial.print("   Offset: ");
    Serial.println(ui_motor_leftOffset);
    Serial.print("Right motor speed: ");
    Serial.print(ui_motor_rightSpeed);
    Serial.print("   Offset: ");
    Serial.println(ui_motor_rightOffset);
#endif

    //encoder debugging
#ifdef debug_encoders
    Serial.print("Left encoder position: ");
    Serial.print(l_motor_leftPosition);
    Serial.print("   Right encoder position: ");
    Serial.println(l_motor_rightPosition);
#endif

    //IR sensor debug
#ifdef debug_IR
    if (b_sensor_IRswitch)
    {
      Serial.print("IR mode: AE,  IR reading: ");
    }
    else
    {
      Serial.print("IR mode: OI,  IR reading: ");
    }
    Serial.println(bt_sensor_IR);
#endif

    //debugging communications with master
#ifdef debug_communciations
    Serial.print("Last message recieved from master: ");
    Serial.println(bt_master_message);
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


void readMaster()
{
  //check for any new messages from the master comm port
  //also have an interpretation of that message run in here
  //we're getting 1 byte of data, so something to take the number and change whatever variables the meaning affects
  if (tellMaster.available())
  {
    bt_master_message = tellMaster.read();

    //code to interpret message here
    //255 is error message?
    switch (bt_master_message)
    {
      case 0:
      case 1:
      case 2:
      case 3:
      case 4:
      case 5:
        {
          //case 0-5 reserved for telling mode indicator index
          i_main_modeIndex = bt_master_message;
          break;
        }
      case 6:
      case 7:
      case 8:
      case 9:
      case 10:
        {
          //cases 6-20 reserved for course index stuff
          //case 6 is course index 0, "wait"
          //case 7 is course index 1, "follow wall forwards"
          //case 8 is course index 2, "follow wall backwards"
          //case 9 is course index 3, "begin pyramid search" - initial IR scan
          i_main_courseIndex = bt_master_message - 6;
          break;
        }
      default:
        {
          Serial.println("Error: unknown master message");
          break;
        }
    }
  }
}

void pingAll()
{
  //reads all ultrasonic values
  digitalWrite(ci_pin_usTrigFront, HIGH);
  delayMicroseconds(10);
  digitalWrite(ci_pin_usTrigFront, LOW);
  l_sensor_usFront = pulseIn(ci_pin_usEchoFront, HIGH);

  digitalWrite(ci_pin_usTrigLeftFront , HIGH);
  delayMicroseconds(10);
  digitalWrite(ci_pin_usTrigLeftFront, LOW);
  l_sensor_usLeftFront = pulseIn(ci_pin_usEchoLeftFront, HIGH);

  digitalWrite(ci_pin_usTrigLeftRear, HIGH);
  delayMicroseconds(10);
  digitalWrite(ci_pin_usTrigLeftRear, LOW);
  l_sensor_usLeftRear = pulseIn(ci_pin_usEchoLeftRear, HIGH);

  digitalWrite(ci_pin_usTrigRight, HIGH);
  delayMicroseconds(10);
  digitalWrite(ci_pin_usTrigRight, LOW);
  l_sensor_usRight = pulseIn(ci_pin_usEchoRight, HIGH);

  digitalWrite(ci_pin_usTrigRear, HIGH);
  delayMicroseconds(10);
  digitalWrite(ci_pin_usTrigRear, LOW);
  l_sensor_usRear = pulseIn(ci_pin_usEchoRear, HIGH);
}

void readIR()
{
  //takes IR reading
  if (pin_IR.available())
  {
    bt_sensor_IR = pin_IR.read();
  }
}

void pingDebounce()
{
  //debounce on ping distance, throws out values if distance has random spike
  //note that ultrasonic can pick up bumps, and pyramid, may cause random change in distance read
  // do something about this change? trigger a variable, make a different function for it? idk...
  //especially for the pyramid, that one causes completely random pings, bumps are at least consistant

}

void readEncoders()
{
  //read both encoder values
  encoder_leftMotor.getRawPosition();
  encoder_rightMotor.getRawPosition();
}

void switchDebounce()
{
  //debounce for the IRswitch

}

void IRdebounce()
{
  //I don't think this is nessisary, due to interpreting IR in different functions, and accuracy discovered in testing

}

void readAllSensors()
{
  //read values for ultrasonic, IR, IR switch, encoders

}

void detachMotors()
{
  //detaches the two motors, use when idle for instructions
  //important to detach motors to minimize power use
  if (b_motor_enabled)
  {
    servo_rightMotor.detach();
    servo_leftMotor.detach();
    b_motor_enabled = false;
  }
}

void attachMotors()
{
  //reattaches motors, use this before motor use
  if (!b_motor_enabled)
  {
    servo_rightMotor.attach(ci_pin_rightMotor);
    servo_leftMotor.attach(ci_pin_leftMotor);
    b_motor_enabled = true;
  }
}

void driveForwards()
{
  //drive in a straight line forwards

}

void driveBackwards()
{
  //drive in a straight line backwards

}

void followWall()
{
  //uses 2 ultrasonics to drive parallel to wall, forwards

}

void followWallBackwards()
{
  //uses 2 ultrasonic to drive parallel to the wall, backwards

}

void rotateClockwise()
{
  //Rotates robot clockwise, attempt to turn on the spot

}

void rotateCounterclockwise()
{
  //rotates robot counterclockwise, practically on the spot

}

void stopMotors()
{
  //stop motor movement
  //may not need to be a function, can be a bool locking movement, similar to linefollower

}

void rotateOffWall()
{
  //start when 2 ultrasonic side parallel to wall, then rotate clockwise until other ultrasonic (on opposite side) is close to wall

}

void smallPyramidScan()
{
  //rotates a few degrees in each direction, trying to pick up an IR signal

}

void bigPyramidScan()
{
  //rotates 360 in attempt to find the pyramid, scans entire areana

}

void driveOverPyramid()
{
  //when tracking pyramid and lost IR signal, drive for set count until master says pyramid hit, or timeout and begin search again
  //alternative, we may just drive straight even if missed pyramid, until wall, then turn and calculate where next
}

void findPyramid()
{
  //shell function, called once trying to trackdown pyramid, possibly after doing initial wall swivel
  //or call once cube is found, but have a do once thing to include it

}

void interpretIR()
{
  //function used while reading IR in rotation scan
  //basically compares every time an IR signal changes, or is lost
  //try to map to angle bot is facing, either using ultrasonic or encoder counts
  //no clue how to handle the data, arrays, pointers, dynamic... idk this is getting complicated for arduino

}

void usTestForPyramid()
{
  //if not done in reading ping function, do something to test if the ultrasonic (front?) is pointed at pyramid

}

void analyseIR()
{
  //run after interpret IR, basically compares output and decides where pyramid could be
  //look for a spot where at minimum AEA, or EAE, or IOI, or OIO

}

void calculatePath()
{
  //takes data from analyseIR, and its previous outcomes (this is getting nuts)
  //if first analyseIR, and multiple possible positions, picks one (middle?) and changes some variables to tell it where to turn and drive to
  //if only one position, pick that one
  //if no positions, large rotate, or drive away from wall, etc?
  //if past first analyseIR, somehow triangulate position between current and previous analysis, and angles at which pyramid is, to find it
  //maybe only remember past 3 analysis? in case blip in samle and bad info goes through?
  //maybe have some default, where if pyramid against wall, this will catch it, begin driving around perimeter of areana, parrallel to all walls
}

void calculatePosition()
{
  //probably will be needed in calculate path, use ultrasonic, measeured course dimensions, and maybe encoder count
  //encoder requires careful watching of turns and straight stretches if we use it here

}

void updateAngle()
{
  //when rotating, call this to calculate angle of ratation, used for IR readings and analysis

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
