/*
Licensed under the Academic Free License ("AFL") v. 3.0
Created: Tuesday, ‎25 ‎June ‎2013
Author: Rod Persky
Source Location: https://github.com/Rod-Persky/cruftRepo/tree/master/Arduino/Multirotor_Rig


HowTo:
1) Run this program and see if the ESC works (the motors need to be on and beeping)

If the motors continue to beep, then
2) uncomment the startESC routine in the setup function
3) watch the power value on the serial output and see what value causes the motors to engage (stop beeping)
4) update ESCuSOn to this value
5) comment the startESC routine out

If the motors did work, then
6) There is nothing to fix, tune the PD control or add I etc.

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
  
  Standard Variables:
    pitch, yaw, roll = Physical system variables
    micro_on = microseconds that the on pulse continues for
    dYaw, dPitch = differential pitch and yaw
    currentvalue = a temporary storage variable
    
    
*/



#include <Servo.h>
#define LeftMotorPin 10
#define RightMotorPin 11
#define Pitch A0
#define Yaw A1
#define Roll A2

#define ESCuSOn 680
#define ESCPropMin 860
#define ESCPropMax 2000


Servo LeftMotor;
Servo RightMotor;

unsigned int pitch, yaw, roll, micro_on;
int dYaw, dPitch;
int currentvalue;

void setup() {
  //Setup the system
  
  //Setup pins
  pinMode(Pitch, INPUT);  
  pinMode(Yaw, INPUT);  
  pinMode(Roll, INPUT);  
  pinMode(LeftMotorPin, OUTPUT);
  pinMode(RightMotorPin, OUTPUT);
  
  //The ESCs are essentially servos
  LeftMotor.attach(LeftMotorPin, ESCuSOn, ESCPropMax);
  RightMotor.attach(RightMotorPin, ESCuSOn, ESCPropMax);

  //Engage the ESCs and delay for 3 seconds
  LeftMotor.writeMicroseconds(ESCuSOn);
  RightMotor.writeMicroseconds(ESCuSOn);
  delay(3000);

  //Get inital values for yaw and pitch
  yaw = analogRead(Yaw);
  pitch = (analogRead(Pitch)-70);
  
  //Start the serial communication... NOTE: the 115200 bits per seconds speed
  Serial.begin(115200); 
  
  //startESC(11);
  //startESC(12);
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
#define setpoint 20
#define kp 7
#define kd 50

void loop() {
 //Get data
 currentvalue = analogRead(Pitch)-70; //70 is the minimum value from the pitch potentiometer
 dPitch = currentvalue - pitch;
 pitch = currentvalue; //bacause currentvalue is just a temporary storage variable
 
 //Calculate new output
 micro_on = ESCPropMin + kp * (pitch - setpoint) + kd * dPitch; //Note: rising physical pitch causes a drop in pitch value

 //Ensure micro_on is within range, otherwise clamp to max or min
 if (ESCPropMax < micro_on){micro_on = ESCPropMax;}
 if (micro_on < ESCPropMin) {micro_on = ESCPropMin;}
 
 //Operate the motors
 LeftMotor.writeMicroseconds(micro_on - 150);
 RightMotor.writeMicroseconds(micro_on);
 
 //send data to computer (this doesn't take very much time)
 sendPlotData("Power", micro_on);
// sendPlotData("Yaw", yaw);
 sendPlotData("Pitch", pitch);
 sendPlotData("kp", kp * (pitch - setpoint));
 sendPlotData("kd", kd * dPitch);
 
 //Wait for the system to change a bit, current frequency is 20 Hz (50 millisecond delay between updates)
 delay(50);
 
}
