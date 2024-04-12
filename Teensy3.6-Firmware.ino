
/************************************************************************************************
Libraries
************************************************************************************************/
#include "MegunoLink.h"
#include "CommandHandler.h" 
#include <Tlc59711.h>
#include "ArduinoTimer.h"
#include <PID_v1.h>
#include "Filter.h"
#include <ezTime.h>
#include "TeensyStep.h"
#include <ADC.h>


/************************************************************************************************
Initialise Time plot for PD and Temp Data
************************************************************************************************/
// The plot we are sending data to. A TimePlot is used here 
TimePlot PD1Plot("PD1"), PD2Plot("PD2"), PD3Plot("PD3"), PD4Plot("PD4"), Temp1Plot("Temp1"), Temp2Plot("Temp2"), Sensor1Plot("Sensor1"), Sensor2Plot("Sensor2");
InterfacePanel MyPanel;
/************************************************************************************************
Initialise ADC Class and variables
************************************************************************************************/
ADC *adc = new ADC(); // adc object;
int ADCAverage = 512; //number of ADC measurments taken for a given sample 
int BitShift = log(ADCAverage)/log(2); //Number of bits to shift for averaging 
int ADCResolution = 12; //ADC resotuon in bits. 10,12 or 16 bit. 

//Initialise active photodiode readings, 1 active, 2,3,4 innactive. 
int PD1Active = 1;
int PD2Active = 1;
int PD3Active = 1;
int PD4Active = 0;
int Sensor1Active = 0;
int Sensor2Active = 0;
/************************************************************************************************
Initialise Timers for data sending
************************************************************************************************/
ArduinoTimer DataSendTimer; // Timer Class for sending data over serial
//Interval (milliseconds) between sending analog data
unsigned DataSendInterval = 50; // [ms] (20Hz)
int DataSendFreq = 1000/DataSendInterval; // convert to Hz 

//loop timing variables 
unsigned long PrevLoopEndTime;
unsigned long LoopEndTime; //
float LoopFrequency; //
/************************************************************************************************
Initialise Logging Capabilities
************************************************************************************************/ 
Message LogCSVData("All_Data");

/************************************************************************************************
Initialise Stepper Motors 
************************************************************************************************/
// Define pin connections for Stepper Motor 
//Stepper 1
#define enablePinStepper1 0
#define currentPin1 3
#define stepPinStepper1 6
#define dirPinStepper1 9
//Stepper 2
#define enablePinStepper2 1
#define currentPin2 4
#define stepPinStepper2 7
#define dirPinStepper2 10
//Stepper 3
#define enablePinStepper3 2
#define currentPin3 5
#define stepPinStepper3 8
#define dirPinStepper3 12

//Initialise Variables
int Motor1CurrentPWM = 230; //Teensy PWM value - Sets current at approx 80mA
int Motor2CurrentPWM = 230; //Teensy PWM value - Sets current at approx 80mA
int Motor3CurrentPWM = 230; //Teensy PWM value - Sets current at approx 80mA

int Motor1MaxOutSpeed = 100; //Maximum possible RPM 
int Motor2MaxOutSpeed = 100; //Maximum possible RPM 
int Motor3MaxOutSpeed = 100; //Maximum possible RPM 

float Motor1OutSpeed = 10;   //Actual output speed initilaised to 10 RPM
float Motor2OutSpeed = 10;   //Actual output speed initilaised to 10 RPM
float Motor3OutSpeed = 10;   //Actual output speed initilaised to 10 RPM


float Motor1onTime = 30;   //on time in mins
float Motor2onTime = 30;   //on time in mins
float Motor3onTime = 30;   //on time in mins

float Motor1offTime = 30;   //off time in mins
float Motor2offTime = 30;   //off time in mins
float Motor3offTime = 30;   //off time in mins


float Motor1Interval = Motor1onTime*60;   //Set interval to current on time in seconds
float Motor2Interval = Motor2onTime*60;   //Set interval to current on time in seconds
float Motor3Interval = Motor3onTime*60;   //Set interval to current on time in seconds


int Motor1Reps = 10; //Reps
int Motor2Reps = 10; //Reps
int Motor3Reps = 10; //Reps

int Motor1CurrentRep = 0; //Reps
int Motor2CurrentRep = 0; //Reps
int Motor3CurrentRep = 0; //Reps

//Timers 
ArduinoTimer Motor1Timer;
ArduinoTimer Motor2Timer;
ArduinoTimer Motor3Timer;

// Blood Sample Timers 
float PrimeTime = 30;   //Priming time in seconds 
float RunTime = 270; //Blood sample run time in seconds
int BloodSampleLoaded = 0; // Flag for sample loaded 
int BloodSamplePrimed = 0; // Flag for completion of priming 
ArduinoTimer BloodSampleTimer; //Timer for running the blood sample sequence 


//Variables to hold pump speeds for printing (Only print the held pump speed if pump enabled) 
float Motor1OutSpeedTrue = 0;   
float Motor2OutSpeedTrue = 0;   
float Motor3OutSpeedTrue = 0;   

float Motor1SpeedFactor; //Speed factor = Motor1OutSpeed/Motor1MaxOutSpeed used to adjust speed of the motor 
float Motor2SpeedFactor;
float Motor3SpeedFactor;

int StepsPerRev = 200; //Assueme all motors are the same 
int GearboxRatio = 19;
int MicrostepLevel = 8; 
//Convert output RPM to Step Speed (step/s)
int OutputInputConversion = GearboxRatio * StepsPerRev * MicrostepLevel; 
long Motor1MaxStepSpeed = Motor1MaxOutSpeed * OutputInputConversion /60L;
long Motor2MaxStepSpeed = Motor2MaxOutSpeed * OutputInputConversion /60L;
long Motor3MaxStepSpeed = Motor3MaxOutSpeed * OutputInputConversion /60L;

int Motor1Direction = 0; // CW = 1, CCW = 0
int Motor2Direction = 0; // CW = 1, CCW = 0
int Motor3Direction = 0; // CW = 1, CCW = 0

// Initialise Stepper Interfaces. Seperate motor and controller interfaces allows indipendant control 
Stepper motor_1(stepPinStepper1, dirPinStepper1);   
RotateControl controller_1;

Stepper motor_2(stepPinStepper2, dirPinStepper2);   
RotateControl controller_2;

Stepper motor_3(stepPinStepper3, dirPinStepper3);   
RotateControl controller_3;

/************************************************************************************************
Initialise LED Varialbes and TLC driver board coms 
************************************************************************************************/
//Initialise Variables
long Brightness1 = 1000; //Brightness of LED1 using 16 bit PWM (MAX 65535)
long Brightness2 = 1000; //Brightness of LED2 using 16 bit PWM (MAX 65535)
long Brightness3 = 1000; //Brightness of LED3 using 16 bit PWM (MAX 65535)
long Brightness4 = 1000; //Brightness of LED4 using 16 bit PWM (MAX 65535)

//Set SPI pins for TLC board
const int NUM_TLC = 1; // # of TLC boards
const int clkPin = 13;
const int dataPin = 11;

//Initialise TLC board 
Tlc59711 tlc(NUM_TLC, clkPin, dataPin);

/************************************************************************************************
Initialise Heater Variables
************************************************************************************************/
//Initialise Variables
//Heater 1
double CurrentTemp1 = 0;
double FilteredCurrentTemp1 = 0;
double PIDOutput1 = 0;// manually set output PWM to 0 so heater is intially off
double TargetTemp1 = 40; //Variables for PID loop

PID Heater1PID(&FilteredCurrentTemp1, &PIDOutput1, &TargetTemp1, 2,2,6, DIRECT);
//PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)

//Set Heater PWM Pin and NTC analogue read pin
const int Heater1Pin = 29;
const int NTC1Pin = A20;

int Heater1State = 0; //Variable to turn heater on (1) or off (0)

int NTC1Raw = 0; //Variable to store raw adc reading for ntc 1

//Initialise NTC filter 
int SmoothingWeight = 10;
ExponentialFilter<float> FilterTemp1(SmoothingWeight, 0); //(filter weighting % (higher% = less filtering/more responsive), Filter start Value)

////////////////////////////////////////////////////////////////////////////////////////////
//Heater 2
double CurrentTemp2 = 0;
double FilteredCurrentTemp2 = 0;
double PIDOutput2 = 0;// manually set output PWM to 0 so heater is intially off
double TargetTemp2 = 40; //Variables for PID loop

PID Heater2PID(&FilteredCurrentTemp2, &PIDOutput2, &TargetTemp2, 2,2,6, DIRECT);
//PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)

//Set Heater PWM Pin and NTC analogue read pin
const int Heater2Pin = 30;
const int NTC2Pin = A19;

int Heater2State = 0; //Variable to turn heater on (1) or off (0)

int NTC2Raw = 0; //Variable to store raw adc reading for ntc 1
//Initialise NTC filter 
ExponentialFilter<float> FilterTemp2(SmoothingWeight, 0); //(filter weighting % (higher% = less filtering/more responsive), Filter start Value)


/************************************************************************************************
Initialise MegunoLink Commands
************************************************************************************************/
CommandHandler<30,40> SerialCommandHandler; //<#commands, #max character lenght of command> 

/************************************************************************************************
Define Functions for Megunolink control
************************************************************************************************/

//Function Defentitions 

//Command to sync time on connection  
void Cmd_connectInitialise(CommandParameter &Parameters)
{
  unsigned long ReceivedTime = Parameters.NextParameterAsUnsignedLong();
  setTime(ReceivedTime);
  Serial.print("Time Synced:");
  }


//Command to set ADC Parameters
void Cmd_SetADCParameters(CommandParameter &Parameters)
{
  ADCResolution = Parameters.NextParameterAsInteger();
  ADCAverage = Parameters.NextParameterAsInteger();
  BitShift = log(ADCAverage)/log(2); //calculate bit shifting requirments 
  Serial.print("Resolution: ");
  Serial.println(ADCResolution);
  //Update ADCO only as this is used for photodiodes 
  adc->adc0->setResolution(ADCResolution); // set bits of resolution   
}

//Command to set which PD's are active
void Cmd_PDActive(CommandParameter &Parameters)
{
  PD1Active = Parameters.NextParameterAsInteger();
  PD2Active = Parameters.NextParameterAsInteger();
  PD3Active = Parameters.NextParameterAsInteger();
  PD4Active = Parameters.NextParameterAsInteger();
  Sensor1Active = Parameters.NextParameterAsInteger();
  Sensor2Active = Parameters.NextParameterAsInteger();

  Serial.print("Active Photodiodes: ");
  Serial.println(PD1Active);
  Serial.println(PD2Active);
  Serial.println(PD3Active);
  Serial.println(PD4Active);
  Serial.println(Sensor1Active);
  Serial.println(Sensor2Active);
}

void Cmd_Brightness(CommandParameter &Parameters)
{
 Brightness1 = Parameters.NextParameterAsLong();//16 bit PWM requires use of longs for full scale 
 Brightness2 = Parameters.NextParameterAsLong();//16 bit PWM requires use of longs for full scale 
 Brightness3 = Parameters.NextParameterAsLong();//16 bit PWM requires use of longs for full scale 
 Brightness4 = Parameters.NextParameterAsLong();//16 bit PWM requires use of longs for full scale 
//Set LED brightness 
 tlc.setChannel(0, Brightness1); //Set LED 1 Brightness (connected to R0 channel 0)
 tlc.setChannel(5, Brightness2); //Set LED 2 Brightness (connected to B1 channel 5)
 tlc.setChannel(6, Brightness3); //Set LED 3 Brightness (connected to B2 channel 6)
 tlc.setChannel(11, Brightness4); //Set LED 4 Brightness (connected to B3 channel 11)
 tlc.write();
} 

void Cmd_BloodSample(CommandParameter &Parameters)
{
 Motor1Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
 Motor1OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
 PrimeTime = Parameters.NextParameterAsDouble();
 RunTime = Parameters.NextParameterAsDouble()*60; //multiply the incoming time in minutes by 60 to get seconds 
 //Reset the Blood Sample Timer (Will first be used to count the Prime Time)
BloodSampleTimer.Reset();
BloodSampleLoaded=1;//Set the sample loaded flag to 1 
//Calculate speed scale factor (IF CCW direction is sent (-1) the speed will be negative and the motor will run backwards)
Motor1SpeedFactor = 1.0 *Motor1Direction* Motor1OutSpeed / Motor1MaxOutSpeed; 
} 


void Cmd_DataSendFreq(CommandParameter &Parameters)
{
DataSendFreq = Parameters.NextParameterAsInteger(DataSendFreq);
DataSendInterval = 1000/DataSendFreq; //convert to ms timing delay
} 

void Cmd_Unknown()
{Serial.println(F("I don't understand"));}

//Motor 1 timer control 
void Cmd_Motor1TimerControl(CommandParameter &Parameters)
{
  Serial.print("\nEnable Motor 1 Timer");
  Motor1Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor1OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  Motor1onTime = Parameters.NextParameterAsDouble(); 
  Motor1offTime = Parameters.NextParameterAsDouble(); 
  Motor1Reps = Parameters.NextParameterAsInteger();
//Set interval and ensure rep number is initialised to 0 before starting 
  Motor1Interval = Motor1onTime*60; //Interval in seconds
  Motor1CurrentRep = 0;
  //Calculate speed scale factor (IF CCW direction is sent (-1) the speed will be negative and the motor will run backwards)
  Motor1SpeedFactor = 1.0 *Motor1Direction* Motor1OutSpeed / Motor1MaxOutSpeed; 
  //Enable motor, set speed and run
  digitalWrite(enablePinStepper1, LOW); //enable pin set low to enable driver
  controller_1.rotateAsync(motor_1); //Begin rotating motor
  controller_1.overrideSpeed(Motor1SpeedFactor); //Apply on the fly speed factor
  //Reset Motor1 timer 
  Motor1Timer.Reset();
  //Set current rep to 1 as now started
  Motor1CurrentRep = 1;

    //Print speeds for confimration 
  Serial.print("\nTIme Control Active - Motor Out Speed [RPM]= "); Serial.println(Motor1OutSpeed);
  Serial.print("\nMotor Speed Scale Factor= "); Serial.println(Motor1SpeedFactor);
  Serial.print("\nNext interval in seconds= "); Serial.println(Motor1Interval);
  Serial.print("\nCurrent Repetition = "); Serial.println(Motor1CurrentRep);
  }

  void Cmd_Motor2TimerControl(CommandParameter &Parameters)
{
  Serial.print("\nEnable Motor 2 Timer");
  Motor2Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor2OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  Motor2onTime = Parameters.NextParameterAsDouble(); 
  Motor2offTime = Parameters.NextParameterAsDouble(); 
  Motor2Reps = Parameters.NextParameterAsInteger();
//Set interval and ensure rep number is initialised to 0 before starting 
  Motor2Interval = Motor2onTime*60; //Interval in seconds
  Motor2CurrentRep = 0;
  //Calculate speed scale factor (IF CCW direction is sent (-1) the speed will be negative and the motor will run backwards)
  Motor2SpeedFactor = 1.0 *Motor2Direction* Motor2OutSpeed / Motor2MaxOutSpeed; 
  //Enable motor, set speed and run
  digitalWrite(enablePinStepper2, LOW); //enable pin set low to enable driver
  controller_2.rotateAsync(motor_2); //Begin rotating motor
  controller_2.overrideSpeed(Motor2SpeedFactor); //Apply on the fly speed factor
  //Reset Motor2 timer 
  Motor2Timer.Reset();
  //Set current rep to 1 as now started
  Motor2CurrentRep = 1;

    //Print speeds for confimration 
  Serial.print("\nTIme Control Active - Motor Out Speed [RPM]= "); Serial.println(Motor2OutSpeed);
  Serial.print("\nMotor Speed Scale Factor= "); Serial.println(Motor2SpeedFactor);
  Serial.print("\nNext interval in seconds= "); Serial.println(Motor2Interval);
  Serial.print("\nCurrent Repetition = "); Serial.println(Motor2CurrentRep);
  }
  

    void Cmd_Motor3TimerControl(CommandParameter &Parameters)
{
  Serial.print("\nEnable Motor 3 Timer");
  Motor3Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor3OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  Motor3onTime = Parameters.NextParameterAsDouble(); 
  Motor3offTime = Parameters.NextParameterAsDouble(); 
  Motor3Reps = Parameters.NextParameterAsInteger();
//Set interval and ensure rep number is initialised to 0 before starting 
  Motor3Interval = Motor3onTime*60; //Interval in seconds
  Motor3CurrentRep = 0;
  //Calculate speed scale factor (IF CCW direction is sent (-1) the speed will be negative and the motor will run backwards)
  Motor3SpeedFactor = 1.0 *Motor3Direction* Motor3OutSpeed / Motor3MaxOutSpeed; 
  //Enable motor, set speed and run
  digitalWrite(enablePinStepper3, LOW); //enable pin set low to enable driver
  controller_3.rotateAsync(motor_3); //Begin rotating motor
  controller_3.overrideSpeed(Motor3SpeedFactor); //Apply on the fly speed factor
  //Reset Motor3 timer 
  Motor3Timer.Reset();
  //Set current rep to 1 as now started
  Motor3CurrentRep = 1;

    //Print speeds for confimration 
  Serial.print("\nTIme Control Active - Motor Out Speed [RPM]= "); Serial.println(Motor3OutSpeed);
  Serial.print("\nMotor Speed Scale Factor= "); Serial.println(Motor3SpeedFactor);
  Serial.print("\nNext interval in seconds= "); Serial.println(Motor3Interval);
  Serial.print("\nCurrent Repetition = "); Serial.println(Motor3CurrentRep);
  }


void Cmd_SetMotor1Enable(CommandParameter &Parameters)
{
  Serial.print("\nEnable Motor 1");
  Motor1Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor1OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  Serial.print("\nMotor 1 direction= "); Serial.println(Motor1Direction);
 //Calculate speed scale factor (IF CCW direction is sent (-1) the speed will be negative and the motor will run backwards)
  Motor1SpeedFactor = 1.0 *Motor1Direction* Motor1OutSpeed / Motor1MaxOutSpeed; 
  //Enable motor, set speed and run
  digitalWrite(enablePinStepper1, LOW); //enable pin set low to enable driver
  controller_1.rotateAsync(motor_1); //Begin rotating motor
  controller_1.overrideSpeed(Motor1SpeedFactor); //Apply on the fly speed factor
    //Print speeds for confimration 
  Serial.print("\nInfinite Rotation - Motor Out Speed [RPM]= "); Serial.println(Motor1OutSpeed);
  Serial.print("\nMotor Speed Scale Factor= "); Serial.println(Motor1SpeedFactor);
}

void Cmd_SetMotor2Enable(CommandParameter &Parameters)
{
 Serial.print("\nEnable Motor 2");
  Motor2Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor2OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  Serial.print("\nMotor 2 direction= "); Serial.println(Motor2Direction);
 //Calculate speed scale factor (IF CCW direction is sent (-1) the speed will be negative and the motor will run backwards)
  Motor2SpeedFactor = 1.0 *Motor2Direction* Motor2OutSpeed / Motor2MaxOutSpeed; 
  //Enable motor, set speed and run
  digitalWrite(enablePinStepper2, LOW); //enable pin set low to enable driver
  controller_2.rotateAsync(motor_2); //Begin rotating motor
  controller_2.overrideSpeed(Motor2SpeedFactor); //Apply on the fly speed factor
    //Print speeds for confimration 
  Serial.print("\nInfinite Rotation - Motor Out Speed [RPM]= "); Serial.println(Motor2OutSpeed);
  Serial.print("\nMotor Speed Scale Factor= "); Serial.println(Motor2SpeedFactor);
  
}

void Cmd_SetMotor3Enable(CommandParameter &Parameters)
{
  Serial.print("\nEnable Motor 3");
  Motor3Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor3OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  Serial.print("\nMotor 3 direction= "); Serial.println(Motor3Direction);
 //Calculate speed scale factor (IF CCW direction is sent (-1) the speed will be negative and the motor will run backwards)
  Motor3SpeedFactor = 1.0 *Motor3Direction* Motor3OutSpeed / Motor3MaxOutSpeed; 
  //Enable motor, set speed and run
  digitalWrite(enablePinStepper3, LOW); //enable pin set low to enable driver
  controller_3.rotateAsync(motor_3); //Begin rotating motor
  controller_3.overrideSpeed(Motor3SpeedFactor); //Apply on the fly speed factor
    //Print speeds for confimration 
  Serial.print("\nInfinite Rotation - Motor Out Speed [RPM]= "); Serial.println(Motor3OutSpeed);
  Serial.print("\nMotor Speed Scale Factor= "); Serial.println(Motor3SpeedFactor);
  
}

void Cmd_SetMotor1Disable(CommandParameter &Parameters)
{
  Serial.print("\nDisable Motor 1");
  controller_1.stopAsync(); //Stop Motor - non blocking fucntion 
  digitalWrite(enablePinStepper1, HIGH); //enable pin set high to disable driver
  Motor1CurrentRep = 0; // Also reset timing loops to prevent motor restarting 
     //reset Flags
   BloodSamplePrimed=0;
   BloodSampleLoaded=0;
}

void Cmd_SetMotor2Disable(CommandParameter &Parameters)
{
  Serial.print("\nDisable Motor 2");
  controller_2.stopAsync(); //Stop Motor - non blocking fucntion 
  digitalWrite(enablePinStepper2, HIGH); //enable pin set high to disable driver
    Motor2CurrentRep = 0; // Also reset timing loops to prevent motor restarting 

}

void Cmd_SetMotor3Disable(CommandParameter &Parameters)
{
  Serial.print("\nDisable Motor 3");
  controller_3.stopAsync(); //Stop Motor - non blocking fucntion 
  digitalWrite(enablePinStepper3, HIGH); //enable pin set high to disable driver
    Motor3CurrentRep = 0; // Also reset timing loops to prevent motor restarting 

}


void Cmd_SetMotor1Speed(CommandParameter &Parameters)
{  
  Motor1Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor1OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  //Calculate speed scale factor 
  Motor1SpeedFactor = 1.0 *Motor1Direction* Motor1OutSpeed / Motor1MaxOutSpeed; 
  controller_1.overrideSpeed(Motor1SpeedFactor); //Apply on the fly speed factor
  //Print speeds for confimration 
  Serial.print("\nMotor 1 Output Speed changed to [RPM] "); Serial.print(Motor1OutSpeed);
  Serial.print("\nMotor Speed Scale Factor= "); Serial.println(Motor1SpeedFactor);   
}

void Cmd_SetMotor2Speed(CommandParameter &Parameters)
{ 
 Motor2Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor2OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  //Calculate speed scale factor 
  Motor2SpeedFactor = 1.0 *Motor2Direction* Motor2OutSpeed / Motor2MaxOutSpeed; 
  controller_2.overrideSpeed(Motor2SpeedFactor); //Apply on the fly speed factor
  //Print speeds for confimration 
  Serial.print("\nMotor 2 Output Speed changed to [RPM] "); Serial.print(Motor2OutSpeed);
  Serial.print("\nMotor Speed Scale Factor= "); Serial.println(Motor2SpeedFactor);   
}

void Cmd_SetMotor3Speed(CommandParameter &Parameters)
{ 
  Motor3Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor3OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  //Calculate speed scale factor 
  Motor3SpeedFactor = 1.0 *Motor3Direction* Motor3OutSpeed / Motor3MaxOutSpeed; 
  controller_3.overrideSpeed(Motor3SpeedFactor); //Apply on the fly speed factor
  //Print speeds for confimration 
  Serial.print("\nMotor 3 Output Speed changed to [RPM] "); Serial.print(Motor3OutSpeed);
  Serial.print("\nMotor Speed Scale Factor= "); Serial.println(Motor3SpeedFactor);   
}

void Cmd_SetMotor1Current(CommandParameter &Parameters)
{
  Motor1CurrentPWM  = Parameters.NextParameterAsInteger();
  Serial.print("\nMotor 1 Current Control set to = ");Serial.println(Motor1CurrentPWM);
}

void Cmd_SetMotor2Current(CommandParameter &Parameters)
{
  Motor2CurrentPWM  = Parameters.NextParameterAsInteger();
  Serial.print("\nMotor 2 Current Control set to = ");Serial.println(Motor2CurrentPWM);
}

void Cmd_SetMotor3Current(CommandParameter &Parameters)
{
  Motor3CurrentPWM  = Parameters.NextParameterAsInteger();
  Serial.print("\nMotor 3 Current Control set to = ");Serial.println(Motor3CurrentPWM);
}

void Cmd_SetTargetTemp(CommandParameter &Parameters)
{
  TargetTemp1  = Parameters.NextParameterAsDouble();
  TargetTemp2  = Parameters.NextParameterAsDouble();
  Serial.print("\nNew Heater 1 Target Temp in Celcius = ");Serial.println(TargetTemp1);
  Serial.print("\nNew Heater 2 Target Temp in Celcius = ");Serial.println(TargetTemp2);
}

void Cmd_HeaterState(CommandParameter &Parameters)
{
  Heater1State = Parameters.NextParameterAsInteger();
  Heater2State = Parameters.NextParameterAsInteger();

   //Heater 1 Logic
  if (Heater1State == 1){ //Turn Heater PID ON
    Heater1PID.SetMode(AUTOMATIC); //Turn PID Control ON  
    }
  else if (Heater1State == 0){ //Turn Heater PID OFF
    Heater1PID.SetMode(MANUAL); //Turn PID Control OFF
    PIDOutput1 = 0; //Also manually set output PWM to 0 
    }
  Serial.print("\nHeater 1 State = ");Serial.println(Heater1State);

  //Heater 2Logic
  if (Heater2State == 1){ //Turn Heater PID ON
    Heater2PID.SetMode(AUTOMATIC); //Turn PID Control ON  
    }
  else if (Heater2State == 0){ //Turn Heater PID OFF
    Heater2PID.SetMode(MANUAL); //Turn PID Control OFF
    PIDOutput2 = 0; //Also manually set output PWM to 0 
    }
  Serial.print("\nHeater 2 State = ");Serial.println(Heater2State);
}

//Define function to take raw ntc reading from ADC and convert it to temperature 
float MeasureTemperature(int nRawThermistor)
{
    /* Constants to convert the raw analogue measurement into 
   * temperature in degrees Celcius
    */
  const float ThermistorResistance = 4700; // Thermistor resistance at some nominal temperature
  const float NominalTemperature = 25; // The nominal temperature where resistance is known. 
  const float BCoefficient = 3830; // The beta coefficient of the thermistor (from data-sheet)
  const float Vsupply = 3.3;  // The supply voltage for the voltage divider. 
  const float Vref = 3.3; // Analogue reference voltage. 
  const float Rtop = 1100; // Bias resistance for the voltage divider. 

  // Calculate the output voltage of the voltage divider; it depends on temperature. 
  float Vout = (float)nRawThermistor * Vref/4095.0;
  
  // Calculate the thermistor resistance. 
  float Temp = Vout/Vsupply;
  float Rtherm = Rtop*(1-Temp)/Temp;
 
  // Convert thermistor resistance into temperature using the Steinhart equation
  float Temperature;
  Temperature = Rtherm / ThermistorResistance;
  Temperature = log(Temperature);
  Temperature /= BCoefficient;
  Temperature += 1.0 / (NominalTemperature + 273.15);
  Temperature = 1.0 / Temperature;
  
  Temperature -= 273.15; // convert to C
 
  return Temperature;
}


/************************************************************************************************
General Setup
************************************************************************************************/
void setup() {

  //Setup TLC communication
  tlc.beginFast();

  //Setup Serial communication
  Serial.begin(115200);
  Serial.println(F("Serial Comms Established"));
  Serial.println(F("=========="));

//  //Setup Stepper1 
   motor_1
    .setMaxSpeed(Motor1MaxStepSpeed)  // steps/s
    .setAcceleration(2500); // steps/s^2 
    pinMode(enablePinStepper1, OUTPUT); //Configure stepper driver enable pin as output 
    digitalWrite(enablePinStepper1, HIGH);//Set enable pin high to disable driver

    //  //Setup Stepper2
   motor_2
    .setMaxSpeed(Motor2MaxStepSpeed)  // steps/s
    .setAcceleration(2500); // steps/s^2 
    pinMode(enablePinStepper2, OUTPUT); //Configure stepper driver enable pin as output 
    digitalWrite(enablePinStepper2, HIGH);//Set enable pin high to disable driver
    
    //  //Setup Stepper3
   motor_3
    .setMaxSpeed(Motor3MaxStepSpeed)  // steps/s
    .setAcceleration(2500); // steps/s^2 
    pinMode(enablePinStepper3, OUTPUT); //Configure stepper driver enable pin as output 
    digitalWrite(enablePinStepper3, HIGH);//Set enable pin high to disable driver

  //Turn on LED's
 tlc.setChannel(0, Brightness1); //Set LED 1 Brightness (connected to R0 channel 0)
 tlc.setChannel(5, Brightness2); //Set LED 2 Brightness (connected to B1 channel 5)
 tlc.setChannel(6, Brightness3); //Set LED 3 Brightness (connected to B2 channel 6)
 tlc.setChannel(11, Brightness4); //Set LED 4 Brightness (connected to B3 channel 11)
 tlc.write();

// Set anlogue resolution and averagin for both ADC's;
    adc->adc0->setResolution(ADCResolution); // set bits of resolution
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS); // change the conversion speed it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed
    
    adc->adc1->setResolution(12); // set bits of resolution Heater only on this ADC
    adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS); // change the conversion speed it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
    adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed

//analogReadResolution(ADCResolution);

 //Initilaise filtered temp value
  NTC1Raw = adc->adc1->analogRead(NTC1Pin); //Read using ADC library 
  CurrentTemp1 = MeasureTemperature(NTC1Raw);
  FilterTemp1.SetCurrent(CurrentTemp1);

  NTC2Raw = adc->adc1->analogRead(NTC2Pin); //Read using ADC library 
  CurrentTemp2 = MeasureTemperature(NTC2Raw);
  FilterTemp2.SetCurrent(CurrentTemp2);

  // Setup the serial commands we can repond to

  //LED Commands
  //(F("Command message sent over serial"), Cmd_NameOfFunctionToCall);
  SerialCommandHandler.AddCommand(F("Brightness"), Cmd_Brightness);
  //Sampling Commands
  SerialCommandHandler.AddCommand(F("DataSendFreq"), Cmd_DataSendFreq);
   SerialCommandHandler.AddCommand(F("Motor1TimerControl"), Cmd_Motor1TimerControl);

  SerialCommandHandler.AddCommand(F("connectInitialise"), Cmd_connectInitialise);
  SerialCommandHandler.AddCommand(F("SetADCParameters"), Cmd_SetADCParameters);
  SerialCommandHandler.AddCommand(F("PDActive"), Cmd_PDActive);
  //Stepper Commands
  SerialCommandHandler.AddCommand(F("Motor1Enable"), Cmd_SetMotor1Enable);
  SerialCommandHandler.AddCommand(F("Motor1Disable"), Cmd_SetMotor1Disable);
  SerialCommandHandler.AddCommand(F("SetMotor1Speed"), Cmd_SetMotor1Speed);
  SerialCommandHandler.AddCommand(F("SetMotor1Current"), Cmd_SetMotor1Current);
  SerialCommandHandler.AddCommand(F("BloodSample"), Cmd_BloodSample);

  SerialCommandHandler.AddCommand(F("Motor2Enable"), Cmd_SetMotor2Enable);
  SerialCommandHandler.AddCommand(F("Motor2Disable"), Cmd_SetMotor2Disable);
  SerialCommandHandler.AddCommand(F("SetMotor2Speed"), Cmd_SetMotor2Speed);
  SerialCommandHandler.AddCommand(F("SetMotor2Current"), Cmd_SetMotor2Current);

  SerialCommandHandler.AddCommand(F("Motor3Enable"), Cmd_SetMotor3Enable);
  SerialCommandHandler.AddCommand(F("Motor3Disable"), Cmd_SetMotor3Disable);
  SerialCommandHandler.AddCommand(F("SetMotor3Speed"), Cmd_SetMotor3Speed);
  SerialCommandHandler.AddCommand(F("SetMotor3Current"), Cmd_SetMotor3Current);
  //Heater Commands
  SerialCommandHandler.AddCommand(F("SetTargetTemp"), Cmd_SetTargetTemp);
  SerialCommandHandler.AddCommand(F("HeaterState"), Cmd_HeaterState);  
}

/************************************************************************************************
Main Program
************************************************************************************************/
void loop() {
  // Check for serial commands and dispatch them.
  SerialCommandHandler.Process();
/************************************************************************************************
Main program Stepper
************************************************************************************************/
  //Current Crontrol
analogWrite(currentPin1, Motor1CurrentPWM);


//Motor1 Timer 
//If in the correct rep range 
if ((Motor1CurrentRep>=1) & (Motor1CurrentRep<=Motor1Reps)){
  //Iftime passed change state of motor and switch timer, and  incriment rep
  if (Motor1Timer.TimePassed_Seconds(Motor1Interval)){
    //Interval passed detect if motor on or off
    
    //If motor is currently on
    if(digitalRead(enablePinStepper1) == LOW){  
      Serial.print("\n Motor Currently on, about to disable");
      //Disable motor
       controller_1.stopAsync(); //Stop Motor - non blocking fucntion 
       digitalWrite(enablePinStepper1, HIGH); //enable pin set high to disable driver
       //reset timer and set interval to off time
       Motor1Timer.Reset();
       Motor1Interval = Motor1offTime*60;
       Serial.print("\nInterval timer set to OFF state (s) "); Serial.println(Motor1Interval);
                        // iNCRIMENT REP NUMBER 
        Motor1CurrentRep++;
       }

//If motor is currently off
     else if(digitalRead(enablePinStepper1) == HIGH){  
      Serial.print("\nMotor Currently off, about to enable");
      
       //Enable motor, set speed, direction and run
       Serial.print("\nEnable Motor 1: Rep#");Serial.println(Motor1CurrentRep);
            digitalWrite(enablePinStepper1, LOW); //enable pin set low to enable driver
            controller_1.rotateAsync(motor_1); //Begin rotating motor
            controller_1.overrideSpeed(Motor1SpeedFactor); //Apply on the fly speed factor
            //reset timer and set interval to on time
           Motor1Timer.Reset();
           Motor1Interval = Motor1onTime*60;
           Serial.print("\nInterval timer set to ON state (s) "); Serial.println(Motor1Interval);
         }
  }
  }
if ((Motor1CurrentRep>Motor1Reps)){
// Reset rep to 0
Motor1CurrentRep = 0;
 }


 //Blood Sample Timer
 //If in priming stage check if priming time passed
 if ((BloodSampleLoaded==1)& (BloodSampleTimer.TimePassed_Seconds(PrimeTime, false) ) & (BloodSamplePrimed==0) ){ //Use false in the timer to prevent autorest 
    //If it has eneable the motor 
    digitalWrite(enablePinStepper1, LOW); //enable pin set low to enable driver
    controller_1.rotateAsync(motor_1); //Begin rotating motor
    controller_1.overrideSpeed(Motor1SpeedFactor); //Apply on the fly speed factor
    //reset timer to now count the run phase
    BloodSampleTimer.Reset();
    //Set primmed flag
    BloodSamplePrimed=1;
    }
 
  //If prime complete and the motor is running check if the run time has been reached
  if  ( (BloodSampleLoaded==1) & (BloodSampleTimer.TimePassed_Seconds(RunTime, false)) & (BloodSamplePrimed==1) ){ //Use false in the timer to prevent autorest 
       controller_1.stopAsync(); //Stop Motor - non blocking fucntion 
       digitalWrite(enablePinStepper1, HIGH); //enable pin set high to disable driver
       //reset Flags
       BloodSamplePrimed=0;
       BloodSampleLoaded=0;
       BloodSampleTimer.Reset();
       }
  
  


/************************************************************************************************
Main program Measure NTC
************************************************************************************************/
  //Read NTC1 convert to temperature and smooth
  NTC1Raw = adc->adc1->analogRead(NTC1Pin); //Read using ADC library 
  CurrentTemp1 = MeasureTemperature(NTC1Raw);
  FilterTemp1.Filter(CurrentTemp1);
  FilteredCurrentTemp1 = FilterTemp1.Current();

  //Read NTC2 convert to temperature and smooth
  NTC2Raw = adc->adc1->analogRead(NTC2Pin); //Read using ADC library 
  CurrentTemp2 = MeasureTemperature(NTC2Raw);
  FilterTemp2.Filter(CurrentTemp2);
  FilteredCurrentTemp2 = FilterTemp2.Current();
  /************************************************************************************************
Main program Heater PID control 
************************************************************************************************/
   //Only allow heater to turn on if temperature is above 0. Aiming to protect against NTC failing or disconeccting as this results in a temperature of -273 being reported. 
 //Heater 1
  if (FilteredCurrentTemp1 > 0){
  Heater1PID.Compute();
  }
  else{
  PIDOutput1 = 0;
  }
  analogWrite(Heater1Pin,PIDOutput1);

  //Heater 2
  if (FilteredCurrentTemp2 > 0){
  Heater2PID.Compute();
  }
  else{
  PIDOutput2 = 0;
  }
  analogWrite(Heater2Pin,PIDOutput2);
/************************************************************************************************
Main program PD and Temperature Plot
************************************************************************************************/

if (DataSendTimer.TimePassed_Milliseconds(DataSendInterval))
  {
  unsigned long PD1Total = 0UL;
  unsigned long PD1Average = 0UL;
  
  unsigned long PD2Total = 0UL;
  unsigned long PD2Average = 0UL;

  unsigned long PD3Total = 0UL;
  unsigned long PD3Average = 0UL;

  unsigned long PD4Total = 0UL;
  unsigned long PD4Average = 0UL;

    unsigned long Sensor1Total = 0UL;
  unsigned long Sensor1Average = 0UL;

    unsigned long Sensor2Total = 0UL;
  unsigned long Sensor2Average = 0UL;
/************************************************************************************************/

  if(PD1Active == 1){
  for(int i = 0; i < ADCAverage; i++) {
    PD1Total += adc->adc0->analogRead(1);
     if(i==(ADCAverage-1)){
  PD1Average = PD1Total >> BitShift;
    }
    }
  PD1Plot.SendData("PD1", PD1Average);
  }
/************************************************************************************************/

  if(PD2Active == 1){
  for(int i = 0; i < ADCAverage; i++) {
    PD2Total += adc->adc0->analogRead(2);
     if(i==(ADCAverage-1)){
  PD2Average = PD2Total >> BitShift;
    }
    }
  PD2Plot.SendData("PD2", PD2Average);
  }
/************************************************************************************************/
  
  if(PD3Active == 1){
  for(int i = 0; i < ADCAverage; i++) {
    PD3Total += adc->adc0->analogRead(3);
     if(i==(ADCAverage-1)){
  PD3Average = PD3Total >> BitShift;
    }
    }
  PD3Plot.SendData("PD3", PD3Average);
  }
/************************************************************************************************/

  if(PD4Active == 1){
  for(int i = 0; i < ADCAverage; i++) {
    PD4Total += adc->adc0->analogRead(4);
     if(i==(ADCAverage-1)){
  PD4Average = PD4Total >> BitShift;
    }
    }
  PD4Plot.SendData("PD4", PD4Average);
  }
/************************************************************************************************/

    if(Sensor1Active == 1){
  for(int i = 0; i < ADCAverage; i++) {
    Sensor1Total += adc->adc0->analogRead(9);
     if(i==(ADCAverage-1)){
  Sensor1Average = Sensor1Total >> BitShift;
    }
    }
  Sensor1Plot.SendData("Sensor1", Sensor1Average);
  }

/************************************************************************************************/

      if(Sensor2Active == 1){
  for(int i = 0; i < ADCAverage; i++) {
    Sensor2Total += adc->adc0->analogRead(8);
     if(i==(ADCAverage-1)){
  Sensor2Average = Sensor2Total >> BitShift;
    }
    }
  Sensor2Plot.SendData("Sensor2", Sensor2Average);
  }
/************************************************************************************************/

  
  Temp1Plot.SendData("Temp1", FilteredCurrentTemp1);
  Temp2Plot.SendData("Temp2", FilteredCurrentTemp2);
/************************************************************************************************/

 //Set motor speeds for readout 
 //If motor enabled write motor speed to output variable
 if(digitalRead(enablePinStepper1) == LOW){
    Motor1OutSpeedTrue = Motor1OutSpeed;
 }
    else{
  Motor1OutSpeedTrue = 0;
    }
/************************************************************************************************/

 if(digitalRead(enablePinStepper2) == LOW){
    Motor2OutSpeedTrue = Motor2OutSpeed;
 }
    else{
  Motor2OutSpeedTrue = 0;
    }
/************************************************************************************************/

 if(digitalRead(enablePinStepper3) == LOW){
    Motor3OutSpeedTrue = Motor2OutSpeed;
 }
    else{
  Motor3OutSpeedTrue = 0;
    }
 /************************************************************************************************/

  
 
  LogCSVData.Begin();
  Serial.print(dateTime("Y-m-d H:i:s.v"));
  Serial.print(",");
  Serial.print(ADCResolution);
  Serial.print(",");
  Serial.print(ADCAverage);
  Serial.print(",");
  Serial.print(PD1Average);
  Serial.print(",");
  Serial.print(PD2Average);
  Serial.print(",");
  Serial.print(PD3Average);
  Serial.print(",");
  Serial.print(PD4Average);
  Serial.print(",");
  Serial.print(Brightness1);
  Serial.print(",");
  Serial.print(Brightness2);
  Serial.print(",");
  Serial.print(Brightness3);
  Serial.print(",");
  Serial.print(Brightness4);
  Serial.print(",");
  Serial.print(Sensor1Average);
  Serial.print(",");
  Serial.print(Sensor2Average);
  Serial.print(",");
  Serial.print(FilteredCurrentTemp1);
  Serial.print(",");
  Serial.print(FilteredCurrentTemp2);
  Serial.print(",");
  Serial.print(Motor1OutSpeedTrue);
  Serial.print(",");
  Serial.print(Motor2OutSpeedTrue);
  Serial.print(",");
  Serial.print(Motor3OutSpeedTrue);
  LogCSVData.End();
  
/************************************************************************************************
Main measure loop speed and send frequency back to megunolink
************************************************************************************************/
  PrevLoopEndTime = LoopEndTime;
  LoopEndTime = millis();
  LoopFrequency = (1000.0/(LoopEndTime - PrevLoopEndTime));
  MyPanel.SetText(F("CurrentFrequency"), LoopFrequency);

  }


}



/************************************************************************************************
Functions
************************************************************************************************/
      
