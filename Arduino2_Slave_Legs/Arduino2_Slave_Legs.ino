// MSE2202 Design Project Group 15
// Slave code for arduino 2 (arduino 1 is running on master code)
// Slave version, has driving motors, ultrasonic, IR, and IR switch


//GUYS FOLLOW THIS NAMING CONVENTION
// <letter for type of variable>_<function variable is for>_<Custom name of choice>
//Example: unsigned long ul_ultrasonic_distance


#include <SoftwareSerial.h>
#include <SoftwareSerial.h>

//DEBUGGING
// uncomment lines based on what needs debugging, will print values to serial every loop
//#define debug_ultrasonic
//#define debug_motors
//#define debug_encoders
//#define debug_IR
//#define debug_communciations


//Pin mapping
SoftwareSerial tellMaster(,); //comm ports with arduino 2 for communication
const int ci_pin_usTrigLeftFront = 8; //might want to make this an array instead of 5 seperate variables
const int ci_pin_usEchoLeftFront = 9;
const int ci_pin_usTrigLeftBack = 2;
const int ci_pin_usEchoLeftBack = 3;
const int ci_pin_usTrigRight = 4;
const int ci_pin_usEchoRight = 5;
const int ci_pin_usTrigBack = 12;
const int ci_pin_usEchoBack = 13;
const int ci_pin_usTrigFront = 10;
const int ci_pin_usEchoFront = 11;
const int ci_pin_leftMotor;
const int ci_pin_rightMotor;
const int ci_pin_leftEncoder;
const int ci_pin_rightEncoder;
SoftwareSerial pin_IR(,);
const int ci_pin_IRswitch;

//Data from sensors
Servo servo_leftMotor;
Servo servo_rightMotor;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //for debugging
  tellMaster.begin(9600);
  pin_IR.begin(2400);

  //set pinmodes


  //more initialization
  servo_leftMotor.attach(ci_pin_leftMotor);
  servo_rightMotor.attach(ci_pin_rightMotor);

  //initialize varaibles

}

void loop() {
  // code runs similar to master, instead of button input, gets instructions from serial port
//reads data from all sensors
readMaster();
pingAll();
readIR();
readEncoders();
  
switch(i_main_modeIndex)
{
  case 0:
  {
    //sitting idle
    break;
  }
  case 1:
  {
    //main course navigation
    switch(i_main_courseIndex)
    {
      
    }
    break;
  }
  case 2;
  {
    //calibrate motors
    break;
  }
  case 3:
  {
    //extra for testing or something
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
//ultrasonic debug
#ifdef debug_ultrasonic

#endif

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
  
}

void pingAll()
{
  //reads all ultrasonic values
  
}

void readIR()
{
  //takes IR reading
  
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

void calibrate motors()
{
  //same as linefollower, drive straight and use encoders to measure motor difference, then calibrate to it
  
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
