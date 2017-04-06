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
#define debug_communciations
const unsigned int cui_debug_displayInterval = 1000; //time between display on debug output, in ms


//Pin mapping
SoftwareSerial tellMaster(16, 17); //comm ports with arduino 2 for communication, A2 and A3
const int ci_pin_usTrigLeftFront = 9; //might want to make this an array instead of 5 seperate variables
const int ci_pin_usEchoLeftFront = 8;
const int ci_pin_usTrigLeftRear = 3;
const int ci_pin_usEchoLeftRear = 2;
const int ci_pin_usTrigRight = 5;
const int ci_pin_usEchoRight = 4;
const int ci_pin_usTrigRear = 13;
const int ci_pin_usEchoRear = 12;
const int ci_pin_usTrigFront = 11;
const int ci_pin_usEchoFront = 10;
const int ci_pin_leftMotor = 6; //Left and right motors could be flipped
const int ci_pin_rightMotor = 7;
I2CEncoder encoder_leftMotor;
I2CEncoder encoder_rightMotor;
const int ci_I2C_SDA = A4;         // I2C data = white    //these lines may need to be included to make encoders work, idk...
const int ci_I2C_SCL = A5;         // I2C clock = yellow
const int ci_pin_IRswitch = A1;
int trig_array[5] = {9, 3, 5, 13, 11};
int echo_array[5] = {8, 2, 4, 12, 10};


//Data from sensors
Servo servo_leftMotor;
Servo servo_rightMotor;
byte bt_master_message;
byte bt_sensor_IR;
bool b_sensor_IRswitch; //true for AE false of IO
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
bool b_main_motorCalForward; //determines direction of calibration, flips after each calibration

//Motor speed variables
const unsigned int cui_motor_forwardSpeed = 1900;
const unsigned int cui_motor_reverseSpeed = 1200;
const unsigned int cui_motor_stop = 1500;
unsigned int ui_motor_leftOffset;
unsigned int ui_motor_rightOffset;
unsigned int ui_motor_leftOffsetBack;
unsigned int ui_motor_rightOffsetBack;
unsigned int ui_motor_rightSpeed; //set this to change how fast the motors are going
unsigned int ui_motor_leftSpeed;
bool b_motor_attached; //will tell if motors are attached or detached
bool b_motor_changeEnabled; //true when adjusting motor speeds will do something
long l_motor_leftPosition;
long l_motor_rightPosition;

//EEPROM stuff for calibration of motors
const int ci_Left_Motor_Offset_Address_L_Back = 8;
const int ci_Left_Motor_Offset_Address_H_Back = 9;
const int ci_Right_Motor_Offset_Address_L_Back = 10;
const int ci_Right_Motor_Offset_Address_H_Back = 11;
const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;
byte b_LowByte;
byte b_HighByte;

//variables used in communicating with master
byte bt_master_lastMessage;
bool b_master_newCommand; //true when issuing command from master has changed, false if same as last cycle

//ultrasonic data processing variables
double d_us_tolerence; //used to accept when a fixed reading is reasonable, used as a "percentage" (is multiplied to get upper range, divided for lower)
long l_us_prev[5][3];//previous data
long l_us_raw[5][3];//raw data
bool b_us_frontIsTrue; //is true when current reading is within tolerance
bool b_us_rearIsTrue;
bool b_us_rightIsTrue;
bool b_us_leftFrontIsTrue;
bool b_us_leftRearIsTrue;
long l_us_pSensorFront; //keeps track of previous cycle data, for comparision
long l_us_pSensorRear;
long l_us_pSensorRight;
long l_us_pSensorLeftFront;
long l_us_pSensorLeftRear;
bool b_us_frontWasTrue; //previous cycle data
bool b_us_rearWasTrue;
bool b_us_rightWasTrue;
bool b_us_leftFrontWasTrue;
bool b_us_leftRearWasTrue;

//IR switch debouncing variables
bool b_switch_prevState;
bool b_switch_currentState;
unsigned long ul_switch_debounceTimer;
const int ci_switch_debounceTime = 20; //time to wait after state change to update

//pyramid locating variables, and IR interpreting stuff
const int ci_pyramid_minEncoderAngle; //minimum encoder angle between scanning for IR
bool b_ir_goodSign; //is the current IR reading what we are looking for?
int i_pyramid_locateIndex;
long l_pyramid_mirrorFront;
long l_pyramid_mirrorRear;


//wall following variables
const int ci_wall_rearTargetDis = 290; //use to keep wall following parallel
const int ci_wall_frontTargetDis = 360;
const int ci_wall_tolerenceDis = 40; //anything beyond this scope will

int i2;


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(2400); //for debugging and IR output, don't worry about shorting since IR is read only
  tellMaster.begin(9600);

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
  b_motor_attached = true;
  b_main_modeIndexChange = false;
  b_motor_attached = true;
  b_motor_changeEnabled = false;
  b_main_motorCalForward = true;
  d_us_tolerence = 1.5; //this must be greater than 1 for ultrasonic readings to make sense
  i_pyramid_locateIndex = 0;


  //eeprom set up for motors offset
  b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
  ui_motor_leftOffset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
  ui_motor_rightOffset = word(b_HighByte, b_LowByte);

  b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L_Back);
  b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H_Back);
  ui_motor_leftOffsetBack = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L_Back);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H_Back);
  ui_motor_rightOffsetBack = word(b_HighByte, b_LowByte);

}

void loop() {
  // code runs similar to master, instead of button input, gets instructions from serial port
  //reads data from all sensors
  readMaster();
  pingAll();
  readIR();
  readEncoders();
  Serial.println("ping1");


  switch (i_main_modeIndex)
  {
    case 0:
      {
        //sitting idle
        Serial.println("ping2");
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
              Serial.println("ping3");
              //wait for instructions
              encoder_leftMotor.zero();
              encoder_rightMotor.zero();
              detachMotors();
              break;
            }
          case 1:
            {
              //begin following wall parallel, forwards
              followWall();
              break;
            }
          case 2:
            {
              //begin following wall, parrallel backwards
              followWallBackwards();
              break;
            }
          case 3:
            {
              //search for pyramid
              findPyramid();
              break;
            }
          default:
            {
              //mis-communication or something to cause error
              Serial.println("Error! unknown courseIndex");
              break;
            }
        }
        break;
      }
    case 2:
      {
        //calibrate motors, direction determined
        if (b_main_motorCalForward)
        {
          if (b_master_newCommand)
          {
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

            b_main_motorCalForward = false;
            ul_main_calibrationTime = millis(); //this is to prevent immediate skipping through reverse mode
            detachMotors();
            tellMaster.write(1); //tell master that it is done calibrating
          }
        }
        else
        {
          if (b_master_newCommand)
          {
            encoder_leftMotor.zero();
            encoder_rightMotor.zero();
            ul_main_calibrationTime = millis();
            attachMotors();
            servo_leftMotor.writeMicroseconds(cui_motor_reverseSpeed);
            servo_rightMotor.writeMicroseconds(cui_motor_reverseSpeed);
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
              ui_motor_rightOffsetBack = 0;
              ui_motor_leftOffsetBack = (l_motor_leftPosition - l_motor_rightPosition) / 4;
            }
            else
            {
              // May have to update this if different calibration time is used
              ui_motor_rightOffsetBack = (l_motor_rightPosition - l_motor_leftPosition) / 4;
              ui_motor_leftOffsetBack = 0;
            }

            EEPROM.write(ci_Right_Motor_Offset_Address_L_Back, lowByte(ui_motor_rightOffsetBack));
            EEPROM.write(ci_Right_Motor_Offset_Address_H_Back, highByte(ui_motor_rightOffsetBack));
            EEPROM.write(ci_Left_Motor_Offset_Address_L_Back, lowByte(ui_motor_leftOffsetBack));
            EEPROM.write(ci_Left_Motor_Offset_Address_H_Back, highByte(ui_motor_leftOffsetBack));

            b_main_motorCalForward = true;
            ul_main_calibrationTime = millis(); //this is to prevent immediate skipping through forward mode
            detachMotors();
            tellMaster.write(1); //tell master that it is done calibrating
          }
        }
        break;
      }
    case 3:
      {
        //extra for testing or something
//        attachMotors();
//        b_motor_changeEnabled = true;
//        driveForwards();
//        delay(3000);
//        stopMotors();
//        delay(3000);
        //driveBackwards();
        //ui_motor_leftSpeed=1000;
        //ui_motor_rightSpeed=2000;
        //driveMotors();
        //followWall();
        //rotateClockwise();
        //findPyramid();

        
        break;
      }
    default:
      {
        //this really shouldn't be possible, something is broken
        Serial.println("ERROR unknown mode index");
        break;
      }
  }


  //debugging stuff
  if (millis() - ul_debug_secTimer > cui_debug_displayInterval)
  {
    ul_debug_secTimer = millis();
    //ultrasonic debug
#ifdef debug_ultrasonic
    Serial.println("Debug ultrasonic values (first number 1=reliable, 0=random):");
    Serial.print(b_us_frontIsTrue);
    Serial.print("  Front: ");
    Serial.print(l_sensor_usFront);
    Serial.print("   in cm: ");
    Serial.println(l_sensor_usFront / 58.2);
    Serial.print(b_us_rearIsTrue);
    Serial.print("  Rear: ");
    Serial.print(l_sensor_usRear);
    Serial.print("   in cm: ");
    Serial.println(l_sensor_usRear / 58.2);
    Serial.print(b_us_leftFrontIsTrue);
    Serial.print("  Left front: ");
    Serial.print(l_sensor_usLeftFront);
    Serial.print("   in cm: ");
    Serial.println(l_sensor_usLeftFront / 58.2);
    Serial.print(b_us_leftRearIsTrue);
    Serial.print("  Left rear: ");
    Serial.print(l_sensor_usLeftRear);
    Serial.print("   in cm: ");
    Serial.println(l_sensor_usLeftRear / 58.2);
    Serial.print(b_us_rightIsTrue);
    Serial.print("  Right: ");
    Serial.print(l_sensor_usRight);
    Serial.print("   in cm: ");
    Serial.println(l_sensor_usRight / 58.2);
#endif

    //motor debugging
#ifdef debug_motors
    Serial.print("Motors attached: ");
    Serial.println(b_motor_attached);
    Serial.print("Left motor speed: ");
    Serial.print(ui_motor_leftSpeed);
    Serial.print("   Offset: ");
    Serial.print(ui_motor_leftOffset);
    Serial.print("   Rev Offset: ");
    Serial.println(ui_motor_leftOffsetBack);
    Serial.print("Right motor speed: ");
    Serial.print(ui_motor_rightSpeed);
    Serial.print("   Offset: ");
    Serial.print(ui_motor_rightOffset);
    Serial.print("   Rev Offset: ");
    Serial.println(ui_motor_rightOffsetBack);
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
    Serial.print("Message recieved before that: ");
    Serial.println(bt_master_lastMessage);
    Serial.print("New message: ");
    Serial.println(b_master_newCommand);
#endif

  }
}



// ---------------------------------------------------------------------------------------------------
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

  b_master_newCommand = false; //set to false by default, will change to true if new command comes
  Serial.println("ping5");


  if (tellMaster.available())
  {
    Serial.println("ping5.1-------------------------------------------------------------------");
    bt_master_lastMessage = bt_master_message;
    bt_master_message = tellMaster.read();
    Serial.println("ping6");

    //check and toggle if message has changed
    if (bt_master_lastMessage != bt_master_message)
    {
      //got a new message
      //sends confirmation once
      b_master_newCommand = true;
      tellMaster.write(bt_master_message); //sends a confirmation to master that it got message
      Serial.println("ping7");
    }
    Serial.println("ping8");

    //code to interpret message here
    //255 is error message?
    if(b_master_newCommand)
    {
    switch (bt_master_message)
    {
      case 0:
        //default empty
        break;
      case 1:
        //reserved for slave to tell master when it has finished a task
        break;
      case 2:
      case 3:
      case 4:
      case 5:
        {
          //case 2-5 reserved for telling mode indicator index, map to 0-3
          i_main_modeIndex = bt_master_message - 2;
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
    Serial.println("ping9");
  }
  tellMaster.write(bt_master_message);
}

void pingAll()
{
  Serial.println("ping10");
  //reads all ultrasonic values
  pingAllSubFunction();

  //calculate average for each reading, zero values are kept, just because of less processing and code will get filterd later
  for (int i = 0; i < 3; i++)
  {
    if (i == 0)
    {
      //reset sensor readings to zero, before summing up average
      l_sensor_usFront = 0;
      l_sensor_usRear = 0;
      l_sensor_usLeftFront = 0;
      l_sensor_usLeftRear = 0;
      l_sensor_usRight = 0;
    }
    l_sensor_usFront += l_us_raw[4][i];
    l_sensor_usRear += l_us_raw[3][i];
    l_sensor_usLeftFront += l_us_raw[0][i];
    l_sensor_usLeftRear += l_us_raw[1][i];
    l_sensor_usRight += l_us_raw[2][i];
  }
  Serial.println("ping11");
  //make average
  l_sensor_usFront /= 3;
  l_sensor_usRear /= 3;
  l_sensor_usLeftFront /= 3;
  l_sensor_usLeftRear /= 3;
  l_sensor_usRight /= 3;

  //calculate if values of each reading lie within the tolerance of average
  //set to true by default, if false for loop will change that
  b_us_frontIsTrue = true;
  b_us_rearIsTrue = true;
  b_us_leftFrontIsTrue = true;
  b_us_leftRearIsTrue = true;
  b_us_rightIsTrue = true;
  Serial.println("ping12");
  for (int i = 0; i < 3; i++)
  {
    if ((l_sensor_usFront * d_us_tolerence < l_us_raw[4][i]) || (l_sensor_usFront / d_us_tolerence > l_us_raw[4][i]))
    {
      //a value in front ultrasonic reading is out of bounds, random readings may exist with skewed data
      b_us_frontIsTrue = false;
    }
    if ((l_sensor_usRear * d_us_tolerence < l_us_raw[3][i]) || (l_sensor_usRear / d_us_tolerence > l_us_raw[3][i]))
    {
      //a value in rear ultrasonic reading is out of bounds, random readings may exist with skewed data
      b_us_rearIsTrue = false;
    }
    if ((l_sensor_usLeftFront * d_us_tolerence < l_us_raw[0][i]) || (l_sensor_usLeftFront / d_us_tolerence > l_us_raw[0][i]))
    {
      //a value in left front ultrasonic reading is out of bounds, random readings may exist with skewed data
      b_us_leftFrontIsTrue = false;
    }
    if ((l_sensor_usLeftRear * d_us_tolerence < l_us_raw[1][i]) || (l_sensor_usLeftRear / d_us_tolerence > l_us_raw[1][i]))
    {
      //a value in left rear ultrasonic reading is out of bounds, random readings may exist with skewed data
      b_us_leftRearIsTrue = false;
    }
    if ((l_sensor_usRight * d_us_tolerence < l_us_raw[2][i]) || (l_sensor_usRight / d_us_tolerence > l_us_raw[2][i]))
    {
      //a value in right ultrasonic reading is out of bounds, random readings may exist with skewed data
      b_us_rightIsTrue = false;
    }
  }
  Serial.println("ping13");

}

void readIR()
{
  Serial.println("ping14");
  readIRSwitch();
  //takes IR reading

  if (Serial.available())
  {
    bt_sensor_IR = Serial.read();
  }
  Serial.println("ping15");

  //update if looking at good IR or not
  if (b_sensor_IRswitch)
  {
    if ((bt_sensor_IR == 65) || (bt_sensor_IR == 69))
    {
      b_ir_goodSign = true;
    }
    else
      b_ir_goodSign = false;
  }
  else
  {
    if ((bt_sensor_IR == 73) || (bt_sensor_IR == 79))
    {
      b_ir_goodSign = true;
    }
    else
      b_ir_goodSign = false;
  }


}

void readIRSwitch()
{
  Serial.println("ping16");
  //reads the value off IR switch, with debouncing
  //when the value changes, sends one message to master to indicate that
  //the single use of write shouldn't affect performance, as this switch is assumed to not be changed during course navigation anyway

  //update to previous states
  b_switch_prevState = b_switch_currentState;

  //get current state
  b_switch_currentState = digitalRead(ci_pin_IRswitch);

  //debouncing check
  if (b_switch_currentState != b_switch_prevState)
  {
    //reset debounce timer
    ul_switch_debounceTimer = millis();
  }
  else if (millis() - ul_switch_debounceTimer > ci_switch_debounceTime)
  {
    //has passed debouncing, only update if updating needed
    if (b_sensor_IRswitch != b_switch_currentState)
    {
      b_sensor_IRswitch = b_switch_currentState;

      //tell master about the update in state change...
      //actually, this is only to change a light in charliplex. not needed, scarp the communication on this value
    }
  }
  Serial.println("ping17");
}

void pingAllSubFunction()
{
  Serial.println("ping18");
  //used to actually ping and measure the ultrasonic readings, do not use on its own
  //for loop to measure 5 readings at a time, process average later
  //int trig_array[5] = {9, 3, 5, 13, 11};
  //int echo_array[5] = {8, 2, 4, 12, 10};
  //long l_us_prev[5][3];
  //long l_us_raw[5][3];
  if (i2 > 2) {
    i2 = 0;
  }
  for (int index = 0; index < 5; index++) {
    l_us_prev[index][i2] = l_us_raw[index][i2];
    digitalWrite(trig_array[index] , HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_array[index], LOW);
    l_us_raw[index][i2] = pulseIn(echo_array[index], HIGH);
  }
  i2++;
  Serial.println("ping19");

  //convert old data to previous data variables
  l_us_pSensorFront = l_sensor_usFront;
  l_us_pSensorRear = l_sensor_usRear;
  l_us_pSensorRight = l_sensor_usRight;
  l_us_pSensorLeftFront = l_sensor_usLeftFront;
  l_us_pSensorLeftRear = l_sensor_usLeftRear;

  b_us_frontWasTrue = b_us_frontIsTrue;
  b_us_rearWasTrue = b_us_rearIsTrue;
  b_us_rightWasTrue = b_us_rightIsTrue;
  b_us_leftFrontWasTrue = b_us_leftFrontIsTrue;
  b_us_leftRearWasTrue = b_us_leftRearIsTrue;
}

void readEncoders()
{
  Serial.println("ping20");
  //read both encoder values
  encoder_leftMotor.getRawPosition();
  encoder_rightMotor.getRawPosition();
  Serial.println("ping21");
}

void detachMotors()
{
  Serial.println("ping22");
  //detaches the two motors, use when idle for instructions
  //important to detach motors to minimize power use
  if (b_motor_attached)
  {
    servo_rightMotor.detach();
    servo_leftMotor.detach();
    b_motor_attached = false;
  }
  Serial.println("ping23");
}

void attachMotors()
{
  Serial.println("ping24");
  //reattaches motors, use this before motor use
  if (!b_motor_attached)
  {
    servo_rightMotor.attach(ci_pin_rightMotor);
    servo_leftMotor.attach(ci_pin_leftMotor);
    b_motor_attached = true;
  }
  Serial.println("ping25");
}

void driveMotors()
{
  Serial.println("ping26");
  //actually sets motor speeds to be custom
  if (b_motor_attached && b_motor_changeEnabled)
  {
    servo_leftMotor.writeMicroseconds(ui_motor_leftSpeed + ui_motor_leftOffset);
    servo_rightMotor.writeMicroseconds(ui_motor_rightSpeed + ui_motor_rightOffset);
  }
  Serial.println("ping27");
}

void driveForwards()
{
  Serial.println("ping28");
  //drive in a straight line forwards
  if (b_motor_attached && b_motor_changeEnabled)
  {
    //only drive if both true
    servo_leftMotor.writeMicroseconds(constrain(cui_motor_forwardSpeed + ui_motor_leftOffset, 1600, 2100));
    servo_rightMotor.writeMicroseconds(constrain(cui_motor_forwardSpeed + ui_motor_rightOffset, 1600, 2100));
  }
  Serial.println("ping29");
}

void driveBackwards()
{
  Serial.println("ping30");
  //drive in a straight line backwards
  if (b_motor_attached && b_motor_changeEnabled)
  {
    servo_leftMotor.writeMicroseconds(constrain(cui_motor_reverseSpeed + ui_motor_leftOffsetBack, 300, 1400));
    servo_rightMotor.writeMicroseconds(constrain(cui_motor_reverseSpeed + ui_motor_rightOffsetBack, 300, 1400));
  }
  Serial.println("ping31");
}

void followWall()
{
  Serial.println("ping32");
  //uses 2 ultrasonics to drive parallel to wall, forwards
  attachMotors();
  Serial.println("ping33");
  b_motor_changeEnabled = true;
  ui_motor_leftSpeed = cui_motor_forwardSpeed + ui_motor_leftOffset;
  ui_motor_rightSpeed = cui_motor_forwardSpeed + ui_motor_rightOffset;

  if (b_us_leftFrontIsTrue && ((l_sensor_usLeftFront > ci_wall_frontTargetDis + ci_wall_tolerenceDis) || (l_sensor_usLeftFront < ci_wall_frontTargetDis - ci_wall_tolerenceDis)))
  { //check value is not nonsense
    if (l_sensor_usLeftFront > ci_wall_frontTargetDis + ci_wall_tolerenceDis)
    {
      //front corner too far away, slow down right wheel
      ui_motor_leftSpeed -= 200;
    }
    if (l_sensor_usLeftFront < ci_wall_frontTargetDis - ci_wall_tolerenceDis)
    {
      //front is too close to wall, slow down left wheel
      ui_motor_rightSpeed -= 200;
    }
    Serial.println("ping34");
  }

  else if (b_us_leftRearIsTrue && ((l_sensor_usLeftRear > ci_wall_rearTargetDis + ci_wall_tolerenceDis) || (l_sensor_usLeftRear < ci_wall_rearTargetDis - ci_wall_tolerenceDis)))
  {
    if (l_sensor_usLeftRear > ci_wall_rearTargetDis + ci_wall_tolerenceDis)
    {
      //rear corner too far, slow down left
      ui_motor_rightSpeed -= 200;
    }
    if (l_sensor_usLeftRear < ci_wall_rearTargetDis - ci_wall_tolerenceDis)
    {
      //rear corner too close, slow down right
      ui_motor_leftSpeed -= 200;
    }
  }
  Serial.println("ping35");

  //ensure this makes them go forwards
  ui_motor_leftSpeed = constrain(ui_motor_leftSpeed, 1500, 2100);
  ui_motor_rightSpeed = constrain(ui_motor_rightSpeed, 1500, 2100);


  //write speeds to motors
  driveMotors();
  Serial.println("ping35");
  //safety check that end was not hit
  //if (b_us_frontIsTrue)
  {
    //its a reliable reading
    if (l_sensor_usFront < 300)
    {
      //right in front of wall, stop motors, return to index 0, tell master, wait for further instruction
      stopMotors();
      detachMotors();
      i_main_courseIndex = 0;
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      tellMaster.write(1);
      
    }
  }
  Serial.println("ping36");

}

void followWallBackwards()
{
  //uses 2 ultrasonic to drive parallel to the wall, backwards

}

void rotateClockwise()
{
  //Rotates robot clockwise, attempt to turn on the spot
  if (b_motor_attached && b_motor_changeEnabled)
  {
    servo_leftMotor.writeMicroseconds(constrain(cui_motor_forwardSpeed + ui_motor_leftOffset, 1600, 2100));
    servo_rightMotor.writeMicroseconds(constrain(cui_motor_reverseSpeed + ui_motor_rightOffsetBack, 300, 1400));
  }

}

void rotateCounterclockwise()
{
  //rotates robot counterclockwise, practically on the spot

}

void stopMotors()
{
  Serial.println("ping37");
  //stop motor movement
  //may not need to be a function, can be a bool locking movement, similar to linefollower
  if (b_motor_attached)
  {
    servo_leftMotor.writeMicroseconds(cui_motor_stop);
    servo_rightMotor.writeMicroseconds(cui_motor_stop);
  }
  Serial.println("ping38");

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
  attachMotors();
  switch (i_pyramid_locateIndex)
  {
    case 0:
      {
        //just starting to locate
        encoder_leftMotor.zero();
        encoder_rightMotor.zero();
        i_pyramid_locateIndex++;
      }
    case 1:
      {
        //starts facing along left wall rotate outwards
        b_motor_changeEnabled = true;
        ui_motor_leftSpeed = cui_motor_reverseSpeed;
        ui_motor_rightSpeed = cui_motor_forwardSpeed;
        driveMotors();
        i_pyramid_locateIndex++;
        break;
      }
    case 2:
      {
        //check if found good IR signal
        if (b_ir_goodSign)
        {
          stopMotors();
          i_pyramid_locateIndex++;
        }
        break;
      }
    case 3:
      {
        //drive until wall is hit
        driveForwards();
        if (b_us_frontIsTrue && (l_sensor_usFront < 300))
        {
          stopMotors();
          i_pyramid_locateIndex++;
        }
        break;
      }
    case 4:
      {
        //record ultrasonic positions to mirror
        if (b_us_leftFrontIsTrue && b_us_leftRearIsTrue)
        {
          //only get values and continue with reliable data
          l_pyramid_mirrorFront = l_sensor_usLeftFront;
          l_pyramid_mirrorRear = l_sensor_usLeftRear;
          i_pyramid_locateIndex++;
        }
        break;
      }
    case 5:
      {
        //rotate clockwise along wall
        b_motor_changeEnabled = true;
        ui_motor_leftSpeed = cui_motor_reverseSpeed;
        ui_motor_rightSpeed = cui_motor_forwardSpeed;
        driveMotors();
        i_pyramid_locateIndex++;
        break;
      }
    case 6:
      {
        //enable IR to stop rotation when mirror is close
        if ((l_pyramid_mirrorRear - 100 > l_sensor_usLeftFront) && (l_pyramid_mirrorFront + 100 < l_sensor_usLeftRear))
        {
          //mirroring is close, enable IR check again
          i_pyramid_locateIndex = 2;
          //entire code loops, by properties of reflections, we should find the pyramid eventually
        }
        break;
      }
  }

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
