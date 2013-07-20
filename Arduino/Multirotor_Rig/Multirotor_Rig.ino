/*
Licensed under the Academic Free License ("AFL") v. 3.0
Created: Tuesday, ‎25 ‎June ‎2013
Author: Rod Persky
Source Location: https://github.com/Rod-Persky/cruftRepo/tree/master/Arduino/Multirotor_Rig

This program is intended to connect between Arduino and MatLab for the development of a multirotor control loop.
Control is done though serial, using a set of single letter commands these are:

To start the program:
- s(tart): starts the program (or press a button attatched to the pin defined in the code)
- ! : stops the control loop, the menu is still accessible

For initially determining values for the motor controller:
- c(onfigure): performs an automatic routine where the system aids in setting the ESC parameters (if motors continue to beep)
- q(quit configuring): stops the configuration routine (if a wrong menu was selected)

Configuring the control loop requires a selection of axis and parameter:
Axis,
- y(aw): puts the system in a mode to configure yaw PID
- r(oll): puts the system in a mode to configure roll PID
- p(itch): puts the system in a mode to configure pitch PID

Parameter,
- p(roportional): Kp
- d(erivitive): Kd
- i(ntegral): Ki
- w(indup): set max value that Ki can be and then resets the integrator
- z: setpoint for channel

For saving the control loop parameters across restarts
- k(eep): store the configuration in flash
- f(actoy): restore the PID values to factory default, or what was defined in this code (warning this restarts the system)

HowTo:
1) Turn the motors and arduino on, when the motors start beeping
2) Send 's' over serial to start the program or use a toggle switch attatched to EnableSwitchPin, the
   control loop will run with values that have been saved to flash.

If the motors continue to beep, then
3) send '!c' over serial to start the setup function (this will reset customised values)
4) watch the power value on the serial output and see what value causes the motors to engage (stop beeping)
5) send '!' over serial to stop this process, otherwise it will stop when the pitch changes
6) update ESCuSOn to this value
7) send 'k!' over serial (this will save and reset the system)

If the motors did work, then
8) There is nothing to fix, tune the PID control

Variables:
  Constants:
    LeftMotorPin = The IO pin that the left motor control line is connected to
    RightMotorPin = ditto
    Pitch = The analog pin that the pitch potentiometer is connected to
    Yaw = ditto
    Roll = ditto
    
    ESCuSOn = The microseconds on period of the control cycle that causes the motor to engage
    ESCPropMin =                        ""                    that causes the propeller to start running
    ESCPropMax =                        ""                    at which the max speed of the propeller is reached (or wanted to reach)
    setpoint = The setpoint for pitch that is desired
    kp = Proportinal coefficient of the PID control
    kd = Differential coefficient of the PID control (note: this isn't a time derivitive, so will be different for different control update speeds)
    ki = Integral control, this will automatically reset if it gets to the value of +- ESCPropMax/2
  
  Standard Variables:
    pitch, yaw, roll = Physical system variables
    micro_on = microseconds that the on pulse continues for
    dYaw, dPitch = differential pitch and yaw
    currentvalue = a temporary storage variable

*/



#include <Servo.h>

#define EnableSwitchPin 8
#define LeftMotorPin 10
#define RightMotorPin 11
#define StatusLEDPin 13
#define PitchPot A0
#define YawPot A1
#define RollPot A2

#define ESCuSOn 680
#define ESCPropMin 860
#define ESCPropMax 2000

#define correctPitchMin 70

#define Yaw 0
#define Pitch 1
#define Roll 2

Servo LeftMotor;
Servo RightMotor;

int currentvalue;         // raw data from sensors
int pitch, yaw, roll;     // data from sensors (after any corrections)
int dYaw, dPitch, dRoll;  // differential term
int iYaw, iPitch, iRoll;  // integral term
int micro_on, error;      // speed setting for motor controller
boolean expectNumber, expectChannelMenu, systemEnabled = false;

//points to the parameter being modified
unsigned int* paramPointer; //parameter data (for Kx, Setpoint, Windup)
unsigned int channelPointer; //parameter channel (index for Kx, Setpoint, Windup)

//hardcoded default values, the setup code will then overwrite this from flash, all are positive
unsigned int Kp[3]       = {0,  7, 0};
unsigned int Ki[3]       = {0,  0, 0};
unsigned int Kd[3]       = {0, 50, 0};
unsigned int Setpoint[3] = {0, 20, 0};
unsigned int Windup[3]   = {2000, 2000, 2000};


void setup() {
  //Setup pins
  pinMode(Pitch, INPUT);  
  pinMode(Yaw, INPUT);  
  pinMode(Roll, INPUT);
  pinMode(StartButtonPin, INPUT);
  pinMode(LeftMotorPin, OUTPUT);
  pinMode(RightMotorPin, OUTPUT);
  pinMode(StatusLEDPin, OUTPUT);
  
  //The ESCs are servos with a range between ESCuSOn and ESCPropMax
  LeftMotor.attach(LeftMotorPin, ESCuSOn, ESCPropMax);
  RightMotor.attach(RightMotorPin, ESCuSOn, ESCPropMax);

  //Get inital values for yaw and pitch
  yaw = analogRead(Yaw);
  pitch = (analogRead(Pitch) - correctPitchMin);
  
  //Start the serial communication... NOTE: the 115200 bits per seconds speed
  Serial.begin(115200);
  
  while(!(systemEnabled || digitalRead(StartButtonPin))) //if systemenabled or butten then move on
  {
    readSerial();
  }
  
  
  //startESC(11);
  //startESC(12);
  
  /*
  //Engage the ESCs and delay for 3 seconds
  LeftMotor.writeMicroseconds(ESCuSOn);
  RightMotor.writeMicroseconds(ESCuSOn);
  delay(3000);*/
}


void sendPlotData(String series, int data) {
  //Send data to Meguinolink to monitor what's going on
  Serial.print("{");
  Serial.print(series);
  Serial.print(",T,");
  Serial.print(data);
  Serial.println("}");
}

void startESC(unsigned int channel){
  //This function finds the ESC start command,
  // once found, you can hard code it. It should take
  // 100 seconds to complete this and the propellor may turn on
  // when/if this happens, turn the arduino or unplug the motor power.
  for(unsigned int i=0; i<1000; i++){
    LeftMotor.writeMicroseconds(i);
    RightMotor.writeMicroseconds(i);
    sendPlotData("Power", micro_on);
    delay(100);
  }
  
  //Once completed stop the program
  while(true){
    //Stop the motors
    LeftMotor.writeMicroseconds(0);
    RightMotor.writeMicroseconds(0);
    delay(1000);
  }
}

//Define the control loop constants
void loop() {
  
  // Get data, beware Pitch is a #def and pitch is the variable
  currentvalue = analogRead(Pitch) - correctPitchMin;
  dPitch = currentvalue - pitch;
  pitch = currentvalue;

  currentvalue = analogRead(Yaw);
  dYaw = currentvalue - yaw;
  yaw = currentvalue;
  
  currentvalue = analogRead(Roll);
  dRoll = currentvalue - roll;
  roll = currentvalue;
  
  
  // Calculate new output
  //Note: rising physical pitch causes a drop in pitch value
  error = pitch - Setpoint[Pitch];
  iPitch = iPitch + Ki[Pitch] * error;
  micro_on = ESCPropMin + Kp[Pitch] * (pitch - Setpoint[Pitch]) + Kd[Pitch] * dPitch + iPitch; 
  sendPlotData("Power", micro_on);
  sendPlotData("kPitch", Kp[Pitch] * error);
  sendPlotData("dPitch", Kd[Pitch] * dPitch);
  sendPlotData("iPitch", iPitch);

 // Ensure micro_on is within range, otherwise clamp to max or min
 if (ESCPropMax < micro_on){micro_on = ESCPropMax;}
 if (micro_on < ESCPropMin) {micro_on = ESCPropMin;}

/* Re-enable when rig becomes available to test with
 // Operate the motors
 LeftMotor.writeMicroseconds(micro_on - 150);
 RightMotor.writeMicroseconds(micro_on);

 delay(50);
 */
  Serial.println("working");
  readSerial();
  delay(1000);
 
  while(!(systemEnabled || digitalRead(StartButtonPin))) // if systemenabled or butten then keep working
  {
    readSerial();
  }
}

// .. warning: not very pretty
boolean readSerial()
{
  //Serial values are assumed to be an 8 bit value
  //Read serial, determine if a state change is requred else default to set value
  byte inByte; // recieved command
 
  if(Serial.available() > 0)
  {
    inByte = Serial.peek(); //get byte
    
    if (expectNumber==true)
    {
      if (48 <= inByte && inByte <= 57)
        {
          *paramPointer = Serial.parseInt();
          expectNumber = false;
          Serial.println(*paramPointer);
          return true;
        }
      else
        {
          if (!inByte == 'q') {            
            Serial.println("Expected number or q");
            Serial.read(); // Drop command from serial que
            return false;  // Reject commands until a parameter is selected
          }
          else
            Serial.println("Quit number entry");
            expectNumber = false;
            Serial.read();
            return true;
        }
    }
    
    if (expectChannelMenu){     
      switch (inByte) {
        case 'p': //proportional
          Serial.print(F(" Kp = "));
          Serial.print(Kp[channelPointer]);
          paramPointer = &Kp[channelPointer];
          break;
          
        case 'd': //derivitive
          Serial.print(F(" Kd = "));
          Serial.print(Kd[channelPointer]);
          paramPointer = &Kd[channelPointer];
          break;
          
        case 'i': //integral
          Serial.print(F(" Ki = "));
          Serial.print(Ki[channelPointer]);
          paramPointer = &Ki[channelPointer];
          break;
          
        case 'w': //windup
          Serial.print(F(" Windup = "));
          Serial.print(Windup[channelPointer]);
          paramPointer = &Windup[channelPointer];
          break;
          
        case 'z': //setpoint
          Serial.print(F(" Setpoint = "));
          Serial.print(Setpoint[channelPointer]);
          paramPointer = &Setpoint[channelPointer];
          break;
          
        default:
          if (!inByte == 'q') {            
            Serial.println("Expected parameter selection or q");
            Serial.read(); // Drop command from serial que
            return false;  // Reject commands until a parameter is selected
          }
          else
            Serial.println("Exiting menu");
            expectChannelMenu = false;
            Serial.read();
            return true;
      } // finish switch successfully
      
      Serial.print(F(" , Enter new value: "));
      Serial.read();             // drop inByte
      expectNumber = true;       // go to number input
      expectChannelMenu = false; // menu finished
      return true;               // function successful
    }

    
    switch (inByte) {
      case 's': //start
        Serial.println(F("Start"));
        systemEnabled = true;
        break;
        
      case '!': //restart
        Serial.println(F("Restart"));
        systemEnabled = false;
        break;
        
      case 'c':  //configure
        Serial.println(F("Configure"));
        break;
        
      case 'q': //quit configure
        Serial.println(F("done."));
        break;
        
      case 'y': //yaw
        Serial.print(F("Yaw"));
        channelPointer = Yaw;
        expectChannelMenu = true;
        break;
        
      case 'r': //roll
        Serial.print(F("Roll"));
        channelPointer = Roll;
        expectChannelMenu = true;
        break;
        
      case 'p': //pitch
        Serial.print(F("Pitch"));
        channelPointer = Pitch;
        expectChannelMenu = true;
        break;       
       
      case 'k': //keep
        Serial.println(F("Keep"));
        break;
        
      case 'f': //factory
        Serial.println(F("Factory"));
        break;
        
      default: //if none of the prior cases are the inByte
          Serial.print(F("Check command syntax for "));
          Serial.write(inByte);
          Serial.println();
    } //end switch case
    
    Serial.read(); //consume character
  } //end if
  
  return true;
}
