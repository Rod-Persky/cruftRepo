/*
Licensed under the Academic Free License ("AFL") v. 3.0
Created: Tuesday, ‎25 ‎June ‎2013
Author: Rod Persky
Source Location: https://github.com/Rod-Persky/cruftRepo/tree/master/Arduino/Multirotor_Rig

This program is intended to connect between Arduino and MatLab for the development of a multirotor control loop.
Control is done though serial, using a set of single letter commands these are:

To start the program:
- s(tart): starts the program (or press a button attatched to the pin defined in the code)
- ! : stops the control loop, reloads the MRR settings

For initially determining values for the motor controller:
- c(onfigure): performs an automatic routine where the system aids in setting the ESC parameters (if motors continue to beep)
- q(quit configuring): stops the configuration routine (if a wrong menu was selected)
- g(raph): Export graph data in Meguinolink format

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


#include <avr/eeprom.h>
#include <Servo.h>

#define EnableSwitchPin 8
#define LeftMotorPin 10
#define RightMotorPin 11
#define StatusLEDPin 13
#define PitchPot A0
#define YawPot A1
#define RollPot A2

#define Yaw 0
#define Pitch 1
#define Roll 2

Servo LeftMotor;
Servo RightMotor;

boolean expectNumber, expectChannelMenu, systemEnabled, enablePlot = false;

//points to the parameter being modified
unsigned int* paramPointer; //parameter data (for Kx, Setpoint, Windup)
unsigned int channelPointer; //parameter channel (index for Kx, Setpoint, Windup)

struct settings_t {
  unsigned int Kp[3];
  unsigned int Ki[3];
  unsigned int KiDiv[3];
  unsigned int Kd[3];
  unsigned int Setpoint[3];
  unsigned int Windup[3];
  
  unsigned int ESCuSOn;
  unsigned int ESCPropMin;
  unsigned int ESCPropMax;
  
  unsigned int correctPitchMin;
  
  unsigned int CheckEEPROM;
} MRR; // MRR... Multirotor rig

//factoryDefaults, aka - what's been tested already
  unsigned int _Kp[3]       = {0,  7, 0};
  unsigned int _Ki[3]       = {0,  0, 0};
  unsigned int _KiDiv[3]    = {1, 100, 1}; //needs to be over 0
  unsigned int _Kd[3]       = {0, 50, 0};
  unsigned int _Setpoint[3] = {0, 20, 0};
  unsigned int _Windup[3]   = {0, 100, 0};
  
  unsigned int _ESCuSOn    = 680;
  unsigned int _ESCPropMin = 860;
  unsigned int _ESCPropMax = 2000;
  
  unsigned int _correctPitchMin = 70;
  
  unsigned int _CheckEEPROM = 12345;


void factoryDefaults() {
  for(byte i=0; i<3; i++) {
    MRR.Kp[i] = _Kp[i];
    MRR.Ki[i] = _Ki[i];
    MRR.KiDiv[i] = _KiDiv[i];
    MRR.Kd[i] = _Kd[i];
    MRR.Setpoint[i] = _Setpoint[i];
    MRR.Windup[i]   = _Windup[i];
  }
        
  MRR.ESCuSOn    = _ESCuSOn;
  MRR.ESCPropMin = _ESCPropMin;
  MRR.ESCPropMax = _ESCPropMax;
  
  MRR.correctPitchMin = _correctPitchMin;
  MRR.CheckEEPROM = _CheckEEPROM;
}


void setup() {
  pinMode(Pitch, INPUT);  
  pinMode(Yaw, INPUT);  
  pinMode(Roll, INPUT);
  pinMode(EnableSwitchPin, INPUT);
  pinMode(LeftMotorPin, OUTPUT);
  pinMode(RightMotorPin, OUTPUT);
  pinMode(StatusLEDPin, OUTPUT);
  
  //Load custom settings from EEPROM
  eeprom_read_block((void*)&MRR, (void*)0, sizeof(MRR));
  if (!MRR.CheckEEPROM == _CheckEEPROM) factoryDefaults(); //check eeprom is initialised
  
  //The ESCs are servos with a range between ESCuSOn and ESCPropMax
  LeftMotor.attach(LeftMotorPin, MRR.ESCuSOn, MRR.ESCPropMax);
  RightMotor.attach(RightMotorPin, MRR.ESCuSOn, MRR.ESCPropMax);

  //Start the serial communication... NOTE: the 115200 bits per seconds speed
  Serial.begin(115200);
}


void loop() {
  //This loop keeps the system in a safe state (motors 'off')
  LeftMotor.writeMicroseconds(0);
  RightMotor.writeMicroseconds(0);
  
  while(!(systemEnabled || digitalRead(EnableSwitchPin))) { //if systemenabled or butten then move on
    readSerial();
  }
  
  controlLoop();
    
}

//Define the control loop constants
void controlLoop() {
  int currentvalue[3];
  int error[3];
  int proportional[3];
  int integral[3] = {0, 0, 0};
  int lastvalue[3];
  int differential[3];
  int power[3];     // speed setting for motor controller
  
  //Get inital values for yaw and pitch
  lastvalue[Yaw] = analogRead(Yaw);
  lastvalue[Pitch] = (analogRead(Pitch) - MRR.correctPitchMin);
  lastvalue[Roll] = analogRead(Roll);
  
  while(systemEnabled){
    // Get data, beware Pitch is a #def and pitch is the variable
    currentvalue[Pitch] = analogRead(Pitch) - MRR.correctPitchMin;
    currentvalue[Yaw]   = analogRead(Yaw);
    currentvalue[Roll]  = analogRead(Roll);
    
    for(byte i = 0; i <3; i++) { // calculate PID terms for each axis
      error[i]        = currentvalue[i] - MRR.Setpoint[i];
      proportional[i] = error[i] * MRR.Kp[i];
      integral[i]     = integral[i] + (error[i]/int(MRR.KiDiv)) * MRR.Ki[i];
      differential[i] = (currentvalue[i] - lastvalue[i]) * MRR.Kd[i];
      lastvalue[i]    = currentvalue[i];
      
      if (MRR.Windup[i] <= integral[i])       integral[i] = MRR.Windup[i]; else
      if (integral[i] <= int(-MRR.Windup[i])) integral[i] = int(-MRR.Windup[i]);
      
      //power = proportional[i] + integral[i] + differential[i]
      
      // Ensure micro_on is within range, otherwise clamp to max or min
      //if (MRR.ESCPropMax < power[i]) {power = MRR.ESCPropMax;}
      //if (power[i] < MRR.ESCPropMin) {power = MRR.ESCPropMin;}
    }
    
    sendPlotData("proportional", &proportional[Pitch]);
    sendPlotData("integral",     &integral[Pitch]);
    sendPlotData("differential", &differential[Pitch]);
    sendPlotData("currentvalue", &currentvalue[Pitch]);
    
    /* Re-enable when rig becomes available to test with
    LeftMotor.writeMicroseconds(micro_on - 150);
    RightMotor.writeMicroseconds(micro_on);
    */
    
    delay(50);
    readSerial();
    }
}

void sendPlotData(String series, int* data) {
  //Send data to Meguinolink to monitor what's going on
  if (enablePlot) {
    Serial.print("{");
    Serial.print(series);
    Serial.print(",T,");
    Serial.print(*data);
    Serial.println("}");
  }
}


void startESC(unsigned int channel){
  //This function finds the ESC start command,
  // once found, you can hard code it. It should take
  // 100 seconds to complete this and the propellor may turn on
  // when/if this happens, turn the arduino or unplug the motor power.
  for(int i=0; i<1000; i++){
    LeftMotor.writeMicroseconds(i);
    RightMotor.writeMicroseconds(i);
    sendPlotData("Power", &i);
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
          Serial.print(MRR.Kp[channelPointer]);
          paramPointer = &MRR.Kp[channelPointer];
          break;
          
        case 'd': //derivitive
          Serial.print(F(" Kd = "));
          Serial.print(MRR.Kd[channelPointer]);
          paramPointer = &MRR.Kd[channelPointer];
          break;
          
        case 'i': //integral
          Serial.print(F(" Ki = "));
          Serial.print(MRR.Ki[channelPointer]);
          paramPointer = &MRR.Ki[channelPointer];
          break;
          
        case 'w': //windup
          Serial.print(F(" Windup = "));
          Serial.print(MRR.Windup[channelPointer]);
          paramPointer = &MRR.Windup[channelPointer];
          break;
          
        case 'z': //setpoint
          Serial.print(F(" Setpoint = "));
          Serial.print(MRR.Setpoint[channelPointer]);
          paramPointer = &MRR.Setpoint[channelPointer];
          break;
          
        default:
          if (!inByte == 'q') {            
            Serial.println("Expected parameter selection or q");
            Serial.read(); // Drop command from serial que
            return false;  // Reject commands until a parameter is selected
          }
          else
            Serial.println(" Exiting menu");
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
        systemEnabled = !systemEnabled;
        if (systemEnabled) Serial.println(F("Start"));
        else Serial.println(F("Stop"));
        break;
        
      case '!': //restart
        Serial.println(F("Restart"));
        systemEnabled = false; //stop control loop
        eeprom_read_block((void*)&MRR, (void*)0, sizeof(MRR)); // reload custom settings
        break;
        
      case 240: //Meguinolink connect
        Serial.println(F("Meguinolink connect"));
        systemEnabled = false; //stop control loop
        enablePlot = true;
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
        
     case 'g': //graph
        Serial.print(F("Graph"));
        enablePlot = !enablePlot;
        if (enablePlot) Serial.println(F(" Enabled"));
        else Serial.println(F(" Disabled"));
        break;  
       
      case 'k': //keep
        Serial.println(F("Keep"));
        eeprom_write_block((const void*)&MRR, (void*)0, sizeof(MRR));
        break;
        
      case 'f': //factory
        Serial.println(F("Factory"));
        factoryDefaults();
        break;
        
      default: //if none of the prior cases are the inByte
          Serial.print(F("Check command syntax for "));
          Serial.write(inByte);
          Serial.print(" (");
          Serial.print(inByte);
          Serial.println(")");
    } //end switch case
    
    Serial.read(); //consume character
  } //end if
  
  return true;
}
