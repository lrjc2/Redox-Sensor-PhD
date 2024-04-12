/************************************************************************************************
University of Southampton Microfluidic Sensor Firmware Deployment
  - by Teensy 4.1
  - modified UoS Microfluidic Sensors Rev1.0 PCB (Rev2.0)
  - supports 2 pumps, 2 heaters, 2 flowcells & 2 sensors
Rev2.1:
  - set all un-used I/O as output low
  - change motor timer to ellapse minute
  - add PD3 & PD4
  - HIH8000 address change from 0x28 to 0x27 
  
Firmware: Rev2.1
Date: Nov 2022
Writer: Original software by Liam Carter, updated by Ken Yeung
************************************************************************************************/

/************************************************************************************************
Libraries
************************************************************************************************/
//MegunoLink
#include <MegunoLink.h>
#include <CommandHandler.h> 
#include <ArduinoTimer.h>
//PID (Heater & NTC)
#include <PID_v1.h>
#include <Filter.h>
//Steeper motor
#include <Arduino.h>
#include <teensystep4.h>    //work for Teensy4.1 (Writer said have bugs)
//ADC
#include <ADC.h>
#include <ADC_util.h>
//Real Time Clock
#include <ezTime.h>
//TLC59711 (LED)
#include <Adafruit_TLC59711.h>
//HIH8000 series temperature & humidity sensors
#include <HIH8000_I2C.h>  
//SD card
#include <Storage.h>
#include <BufferedPrint.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <RingBuf.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <sdios.h>

/*
//MTP mode
#include <SPI.h>
#include <SD.h>
#include <MTP.h>
*/

/************************************************************************************************
Define Pins
************************************************************************************************/
// Define pin connections for Stepper Motor 
//Stepper 1
#define dirPinStepper1 1
#define stepPinStepper1 2
#define currentPin1 0
#define enablePinStepper1 4
//Stepper 2
#define dirPinStepper2 5
#define stepPinStepper2 6
#define currentPin2 7
#define enablePinStepper2 8

/****************************************************/
//current hardware not support
//Stepper 3
#define enablePinStepper3 12
#define currentPin3 10
#define stepPinStepper3 9
#define dirPinStepper3 24
//Stepper 4
#define enablePinStepper4 28
#define currentPin4 30
#define stepPinStepper4 29
#define dirPinStepper4 27
/****************************************************/

//Photodiodes pins
#define PD1Pin  A0
#define PD2Pin  A1
#define PD3Pin  A2
#define PD4Pin  A3
//Sensors pins
#define Sensor1Pin A9
#define Sensor2Pin A8
//TLC59711
#define clkPin 13
#define dataPin 11

//Heater & NTC
//(Heater 1 using Heater 3 connections)
#define Heater1Pin  37  
#define NTC1Pin A15 
#define Heater2Pin 36
#define NTC2Pin A17

/****************************************************/
//current hardware not support
#define Heater3Pin  35
#define NTC3Pin 34 
#define Heater4Pin 38
#define NTC4Pin 40
/****************************************************/

//I2C0 (HIH8000 Series)
#define SCL0Pin 19
#define SDA0Pin 18

//MTP
//#define GRN_LED 25  //Green LED (Active LOW)
#define SDIndicatorPin 25
#define MTP_Pin 3   //Extrnal resistor pull up with jumper

//Unused pins
#define Unused20Pin 20
#define Unused21Pin 21
#define Unused26Pin 26
#define Unused31Pin 31
#define Unused32Pin 32
#define Unused33Pin 33
#define Unused34Pin 34
#define Unused35Pin 35
#define Unused38Pin 38
#define Unused40Pin 40

/************************************************************************************************
Initialise Time plot for PD and Temp Data
************************************************************************************************/
// The plot we are sending data to. A TimePlot is used here 
//TimePlot PD1Plot("PD1"), PD2Plot("PD2"), Temp1Plot("Temp1"), Temp2Plot("Temp2"), Sensor1Plot("Sensor1"), Sensor2Plot("Sensor2");
//TimePlot PD1Plot("PD1"), PD2Plot("PD2"), PD3Plot("PD3"), PD4Plot("PD4"), Temp1Plot("Temp1"), Temp2Plot("Temp2"), Temp3Plot("Temp3"), Temp4Plot("Temp4"), Sensor1Plot("Sensor1"), Sensor2Plot("Sensor2");
TimePlot PD1Plot("PD1"), PD2Plot("PD2"), PD3Plot("PD3"), PD4Plot("PD4"), Temp1Plot("Temp1"), Temp2Plot("Temp2"), Sensor1Plot("Sensor1"), Sensor2Plot("Sensor2");
InterfacePanel MyPanel;

/************************************************************************************************
Initialise ADC Class and variables
************************************************************************************************/
ADC *adc = new ADC(); // adc object;
uint16_t ADCAverage = 256; //number of ADC measurments taken for a given sample 
uint16_t BitShift = log(ADCAverage)/log(2); //Number of bits to shift for averaging 
uint8_t ADCResolution = 12; //ADC resotuon in bits. 10 or 12 bits for Teensy 4.1 

/************************************************************************************************
Initialise Photodiodes & Sensors
************************************************************************************************/
bool PD1Active = 1;
bool PD2Active = 1;
bool PD3Active = 1;
bool PD4Active = 1;
bool Sensor1Active = 0;
bool Sensor2Active = 0;

/************************************************************************************************
Initialise Timers for data sending
************************************************************************************************/
ArduinoTimer DataSendTimer; // Timer Class for sending data over serial
//Interval (milliseconds) between sending analog data
uint16_t DataSendInterval = 50; // [ms] (20Hz)
//uint16_t DataSendInterval = 1000; // [ms] (1Hz)
uint8_t DataSendFreq = 1000/DataSendInterval; // convert to Hz 

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
//Initialise Variables
bool  enableStepper1 = 0;  //0=stop; 1=running
bool  enableStepper2 = 0;  //0=stop; 1=running
bool  enableStepper3 = 0;  //0=stop; 1=running
bool  enableStepper4 = 0;  //0=stop; 1=running

uint8_t Motor1CurrentPWM = 230; //Teensy PWM value - Sets current at approx 80mA
uint8_t Motor2CurrentPWM = 230; //Teensy PWM value - Sets current at approx 80mA
uint8_t Motor3CurrentPWM = 230; //Teensy PWM value - Sets current at approx 80mA
uint8_t Motor4CurrentPWM = 230; //Teensy PWM value - Sets current at approx 80mA

uint8_t ReturnMotor1CurrentPWM = 230; //Return to MegunoLink parameter (Stepper Current)
uint8_t ReturnMotor2CurrentPWM = 230; //Return to Megunolink parameter
uint8_t ReturnMotor3CurrentPWM = 230; //Return to MegunoLink parameter
uint8_t ReturnMotor4CurrentPWM = 230; //Return to Megunolink parameter

uint8_t Motor1MaxOutSpeed = 100; //Maximum possible RPM 
uint8_t Motor2MaxOutSpeed = 100; //Maximum possible RPM 
uint8_t Motor3MaxOutSpeed = 100; //Maximum possible RPM 
uint8_t Motor4MaxOutSpeed = 100; //Maximum possible RPM

uint8_t Motor1OutSpeed = 10;   //Actual output speed initilaised to 10 RPM
uint8_t Motor2OutSpeed = 10;   //Actual output speed initilaised to 10 RPM
uint8_t Motor3OutSpeed = 10;   //Actual output speed initilaised to 10 RPM
uint8_t Motor4OutSpeed = 10;   //Actual output speed initilaised to 10 RPM

/*Rev2.1
float Motor1onTime = 30;   //on time in mins
float Motor2onTime = 30;   //on time in mins
float Motor3onTime = 30;   //on time in mins
float Motor4onTime = 30;   //on time in mins

float Motor1offTime = 30;   //off time in mins
float Motor2offTime = 30;   //off time in mins
float Motor3offTime = 30;   //off time in mins
float Motor4offTime = 30;   //off time in mins

float Motor1Interval = Motor1onTime*60;   //Set interval to current on time in seconds
float Motor2Interval = Motor2onTime*60;   //Set interval to current on time in seconds
float Motor3Interval = Motor3onTime*60;   //Set interval to current on time in seconds
float Motor4Interval = Motor4onTime*60;   //Set interval to current on time in seconds
*/
//Rev2.1 uint32_t?
uint32_t Motor1onTime = 30;   //on time in mins
uint32_t Motor2onTime = 30;   //on time in mins
uint32_t Motor3onTime = 30;   //on time in mins
uint32_t Motor4onTime = 30;   //on time in mins

uint32_t Motor1offTime = 30;   //off time in mins
uint32_t Motor2offTime = 30;   //off time in mins
uint32_t Motor3offTime = 30;   //off time in mins
uint32_t Motor4offTime = 30;   //off time in mins

uint32_t Motor1Interval = Motor1onTime;   //Set interval to current ON/OFF time in minutes
uint32_t Motor2Interval = Motor2onTime;   //Set interval to current ON/OFF time in minutes
uint32_t Motor3Interval = Motor3onTime;   //Set interval to current ON/OFF time in minutes
uint32_t Motor4Interval = Motor4onTime;   //Set interval to current ON/OFF time in minutes


uint16_t Motor1Reps = 10; //Reps
uint16_t Motor2Reps = 10; //Reps
uint16_t Motor3Reps = 10; //Reps
uint16_t Motor4Reps = 10; //Reps

uint16_t Motor1CurrentRep = 0; //Reps
uint16_t Motor2CurrentRep = 0; //Reps
uint16_t Motor3CurrentRep = 0; //Reps
uint16_t Motor4CurrentRep = 0; //Reps

bool Motor1Oscillation = 0; //flag for osscilation (1) or continous direction (0)
bool Motor2Oscillation = 0; //Reps
bool Motor3Oscillation = 0; //Reps
bool Motor4Oscillation = 0; //Reps

//Timers 
ArduinoTimer Motor1Timer;
ArduinoTimer Motor2Timer;
//ArduinoTimer Motor3Timer;
//ArduinoTimer Motor4Timer;

//Variables to hold pump speeds for printing (Only print the held pump speed if pump enabled) 
float Motor1OutSpeedTrue = 0;   
float Motor2OutSpeedTrue = 0;   
//float Motor3OutSpeedTrue = 0; 
//float Motor4OutSpeedTrue = 0;  

float Motor1SpeedFactor; //Speed factor = Motor1OutSpeed/Motor1MaxOutSpeed used to adjust speed of the motor 
float Motor2SpeedFactor;
//float Motor3SpeedFactor;
//float Motor4SpeedFactor;

uint8_t StepsPerRev = 200; //(Step/rev) Assueme all motors are the same
uint8_t GearboxRatio = 19;
uint8_t MicrostepLevel = 8; //MP6500 best resolution is eighth-step (32 micro steps)
//int MicrostepLevel = 8; 
//Convert output RPM to Step Speed (step/s)
uint16_t OutputInputConversion = GearboxRatio * StepsPerRev * MicrostepLevel; //200*19*8=30400

uint32_t Motor1MaxStepSpeed = Motor1MaxOutSpeed * OutputInputConversion /60;
uint32_t Motor2MaxStepSpeed = Motor2MaxOutSpeed * OutputInputConversion /60;
//long Motor3MaxStepSpeed = Motor3MaxOutSpeed * OutputInputConversion /60L;
//long Motor4MaxStepSpeed = Motor4MaxOutSpeed * OutputInputConversion /60L;

/*
long Motor1MaxStepSpeed = Motor1MaxOutSpeed * OutputInputConversion /60L;
long Motor2MaxStepSpeed = Motor2MaxOutSpeed * OutputInputConversion /60L;
long Motor3MaxStepSpeed = Motor3MaxOutSpeed * OutputInputConversion /60L;
long Motor4MaxStepSpeed = Motor4MaxOutSpeed * OutputInputConversion /60L;
*/
int8_t Motor1Direction = 1; // CW = 1, CCW = -1
int8_t Motor2Direction = -1; // CW = 1, CCW = -1
int8_t Motor3Direction = 1; // CW = 1, CCW = -1
int8_t Motor4Direction = -1; // CW = 1, CCW = -1

int32_t Motor1StepSpeed = (OutputInputConversion * Motor1OutSpeed * Motor1Direction)/60;
int32_t Motor2StepSpeed = (OutputInputConversion * Motor2OutSpeed * Motor2Direction)/60;
//long Motor3StepSpeed = (OutputInputConversion * Motor3OutSpeed * Motor3Direction)/60;
//long Motor4StepSpeed = (OutputInputConversion * Motor4OutSpeed * Motor4Direction)/60;

/*
long Motor1StepSpeed = (OutputInputConversion * Motor1OutSpeed * Motor1Direction)/60;
long Motor2StepSpeed = (OutputInputConversion * Motor2OutSpeed * Motor2Direction)/60;
long Motor3StepSpeed = (OutputInputConversion * Motor3OutSpeed * Motor3Direction)/60;
long Motor4StepSpeed = (OutputInputConversion * Motor4OutSpeed * Motor4Direction)/60;
*/
// Initialise Stepper Interfaces. Seperate motor and controller interfaces allows indipendant control 
//TeensyStep4 lib
using namespace TS4;
Stepper motor_1(stepPinStepper1, dirPinStepper1);   
Stepper motor_2(stepPinStepper2, dirPinStepper2);   
//Stepper motor_3(stepPinStepper3, dirPinStepper3);   
//Stepper motor_4(stepPinStepper4, dirPinStepper4);   

/************************************************************************************************
Initialise LED Varialbes and TLC59711
************************************************************************************************/
//Initialise Variables
uint16_t Brightness1 = 1000; //Brightness of LED1 using 16 bit PWM (MAX 256)
uint16_t Brightness2 = 1000; //Brightness of LED2 using 16 bit PWM (MAX 256)
uint16_t Brightness3 = 1000; //Brightness of LED3 using 16 bit PWM (MAX 256)
uint16_t Brightness4 = 1000; //Brightness of LED4 using 16 bit PWM (MAX 256)

//Set SPI pins for TLC59711
const int NUM_TLC = 1;

//Initialise TLC59711 
Adafruit_TLC59711 tlc(NUM_TLC, clkPin, dataPin);
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
bool Heater1State = 0; //Variable to turn heater on (1) or off (0)
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

bool Heater2State = 0; //Variable to turn heater on (1) or off (0)
int NTC2Raw = 0; //Variable to store raw adc reading for ntc 1
//Initialise NTC filter 
ExponentialFilter<float> FilterTemp2(SmoothingWeight, 0); //(filter weighting % (higher% = less filtering/more responsive), Filter start Value)

//Registers for Heater 3 & 4 (When unused)
bool Heater3State = 0; //Variable to turn heater on (1) or off (0)
double TargetTemp3 = 40; //Variables for PID loop
bool Heater4State = 0; //Variable to turn heater on (1) or off (0)
double TargetTemp4 = 40; //Variables for PID loop

/*
////////////////////////////////////////////////////////////////////////////////////////////
//Heater 3
double CurrentTemp3 = 0;
double FilteredCurrentTemp3 = 0;
double PIDOutput3 = 0;// manually set output PWM to 0 so heater is intially off
double TargetTemp3 = 40; //Variables for PID loop

PID Heater3PID(&FilteredCurrentTemp3, &PIDOutput3, &TargetTemp3, 2,2,6, DIRECT);
//PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)

bool Heater3State = 0; //Variable to turn heater on (1) or off (0)
int NTC3Raw = 0; //Variable to store raw adc reading for ntc 1
//Initialise NTC filter 
ExponentialFilter<float> FilterTemp3(SmoothingWeight, 0); //(filter weighting % (higher% = less filtering/more responsive), Filter start Value)

////////////////////////////////////////////////////////////////////////////////////////////
//Heater 4
double CurrentTemp4 = 0;
double FilteredCurrentTemp4 = 0;
double PIDOutput4 = 0;// manually set output PWM to 0 so heater is intially off
double TargetTemp4 = 40; //Variables for PID loop

PID Heater4PID(&FilteredCurrentTemp4, &PIDOutput4, &TargetTemp4, 2,2,6, DIRECT);
//PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)

bool Heater4State = 0; //Variable to turn heater on (1) or off (0)
int NTC4Raw = 0; //Variable to store raw adc reading for ntc 1
//Initialise NTC filter 
ExponentialFilter<float> FilterTemp4(SmoothingWeight, 0); //(filter weighting % (higher% = less filtering/more responsive), Filter start Value)
*/

/************************************************************************************************
HIH8000 series humidity and temperature sensors
************************************************************************************************/

HIH8000_I2C hihSensor = HIH8000_I2C(0x00);  //Sensors temp address
HIH8000_I2C HIH27 = HIH8000_I2C(0x27);  //Sensors address (On PCB)
//HIH8000_I2C HIH28 = HIH8000_I2C(0x28);  //Sensors address (On Box)

bool trigSuccess = false;
bool trigSuccess_HIH27 = false;
byte I2CStatus = 0;

float HIH27_Temperature = 0;
float HIH27_Humidity = 0;
/*
bool trigSuccess = false;
bool trigSuccess_HIH28 = false;
byte I2CStatus = 0;

float HIH28_Temperature = 0;
float HIH28_Humidity = 0;
*/
/************************************************************************************************
Initialise SD Setup use Teensy 3.6 SDIO
************************************************************************************************/

#define SD_CONFIG  SdioConfig(FIFO_SDIO)
// Interval between points for 25 ksps.
//#define LOG_INTERVAL_USEC 40

// Size to log 10 byte lines at 25 kHz for more than ten minutes.
#define LOG_FILE_SIZE 1048576  // 150,000,000 bytes.
#define DATA_FILE_SIZE 10*25000*600  // 150,000,000 bytes.

// Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define RING_BUF_CAPACITY 400*512

SdFs sd;
FsFile file;

//SD Card Variables
char fileName[32];
char dateBuffer[32];
uint8_t SDDataLog = 0; //SD Card Data Log: 0=no; 1=start; 2=logging; 3=end

ArduinoTimer SDDataLogTimer;
uint16_t SDDataLogInterval = 30; //Timer for data log peroid (minutes)*/
bool SDIndicator = 0;            //SD Card Running: 0=No; 1=Yes

/************************************************************************************************
Initialise MTP mode use Teensy 3.6
************************************************************************************************/
/*
bool  MTPMode = 0; //0=normal mode; 1=MTP mode

#define USE_SD  1         // SDFAT based SDIO and SPI

//#define USE_LFS_RAM 0     // T4.1 PSRAM (or RAM)
//#define USE_LFS_QSPI 0    // T4.1 QSPI
//#define USE_LFS_PROGM 0   // T4.1 Progam Flash

#define USE_LFS_SPI 0     // SPI Flash

#if USE_EVENTS==1
  extern "C" int usb_init_events(void);
#else
  int usb_init_events(void) {}
#endif

#if USE_LFS_RAM==1 ||  USE_LFS_PROGM==1 || USE_LFS_QSPI==1 || USE_LFS_SPI==1
  #include "LittleFS.h"
#endif

#if defined(__IMXRT1062__)
  // following only as long usb_mtp is not included in cores
  #if !__has_include("usb_mtp.h")
    #include "usb1_mtp.h"
  #endif
#else
  #ifndef BUILTIN_SDCARD 
    #define BUILTIN_SDCARD 254
  #endif
  void usb_mtp_configure(void) {}
#endif
*/

/****  Start device specific change area  ****/
// SDClasses 
/*
#if USE_SD==1
  // edit SPI to reflect your configuration (following is for T4.1)
  
//  #define SD_MOSI 11
//  #define SD_MISO 12
//  #define SD_SCK  13

  #define SPI_SPEED SD_SCK_MHZ(33)  // adjust to sd card 

  #if defined (BUILTIN_SDCARD)
    const char *sd_str[]={"sdio","sd1"}; // edit to reflect your configuration
    const int cs[] = {BUILTIN_SDCARD,10}; // edit to reflect your configuration
  #else
    const char *sd_str[]={"sd1"}; // edit to reflect your configuration
    const int cs[] = {10}; // edit to reflect your configuration
  #endif
  const int nsd = sizeof(sd_str)/sizeof(const char *);

SDClass sdx[nsd];
#endif


//LittleFS classes
#if USE_LFS_RAM==1
  const char *lfs_ram_str[]={"RAM1","RAM2"};     // edit to reflect your configuration
  const int lfs_ram_size[] = {2'000'000,4'000'000}; // edit to reflect your configuration
  const int nfs_ram = sizeof(lfs_ram_str)/sizeof(const char *);

  LittleFS_RAM ramfs[nfs_ram]; 
#endif

#if USE_LFS_QSPI==1
  const char *lfs_qspi_str[]={"QSPI"};     // edit to reflect your configuration
  const int nfs_qspi = sizeof(lfs_qspi_str)/sizeof(const char *);

  LittleFS_QSPIFlash qspifs[nfs_qspi]; 
#endif

#if USE_LFS_PROGM==1
  const char *lfs_progm_str[]={"PROGM"};     // edit to reflect your configuration
  const int lfs_progm_size[] = {1'000'000}; // edit to reflect your configuration
  const int nfs_progm = sizeof(lfs_progm_str)/sizeof(const char *);

  LittleFS_Program progmfs[nfs_progm]; 
#endif

#if USE_LFS_SPI==1
  const char *lfs_spi_str[]={"nand1","nand2","nand3","nand4"}; // edit to reflect your configuration
  const int lfs_cs[] = {3,4,5,6}; // edit to reflect your configuration
  const int nfs_spi = sizeof(lfs_spi_str)/sizeof(const char *);

LittleFS_SPIFlash spifs[nfs_spi];
#endif
*/

/************************************************************************************************
Initialise MegunoLink Commands
************************************************************************************************/
//org CommandHandler<20,60> SerialCommandHandler; //<#commands, #max character lenght of command> 
CommandHandler<30,70> SerialCommandHandler; //<#commands, #max character lenght of command> 
//CommandHandler<20,60> SerialCommandHandler; //<#commands, #max character lenght of command> 
/************************************************************************************************
Function Routines
************************************************************************************************/
/***** Command handle function routines *****/
//Command to sync time on connection  
void Cmd_connectInitialise(CommandParameter &Parameters)
{
  unsigned long ReceivedTime = Parameters.NextParameterAsUnsignedLong();
  setTime(ReceivedTime);
  //Serial.print("Time Synced:");
}

//Command to set ADC Parameters
void Cmd_SetADCParameters(CommandParameter &Parameters)
{
  ADCResolution = Parameters.NextParameterAsInteger();
  ADCAverage = Parameters.NextParameterAsInteger();
  BitShift = log(ADCAverage)/log(2); //calculate bit shifting requirments 
  //Serial.print("Resolution: ");
  //Serial.println(ADCResolution);
  //Update ADCO only as this is used for photodiodes 
  if (ADCResolution > 12)   //Maximum for Teensy 4.1
    ADCResolution = 12;
  adc->adc0->setResolution(ADCResolution); // set bits of resolution   
  //Teensy 4.1
  MyPanel.SetListValue("ADCResolution", ADCResolution); 
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
  //Teensy 4.1
/* Rev2.1 enable PD3 & PD4
  PD3Active = 0;
  PD4Active = 0;
  MyPanel.SetCheck("PD3Active", PD3Active);
  MyPanel.SetCheck("PD4Active", PD4Active);
 */
}

void Cmd_Brightness(CommandParameter &Parameters)
{
 Brightness1 = Parameters.NextParameterAsLong();//16 bit PWM requires use of longs for full scale 
 Brightness2 = Parameters.NextParameterAsLong();//16 bit PWM requires use of longs for full scale 
 Brightness3 = Parameters.NextParameterAsLong();//16 bit PWM requires use of longs for full scale 
 Brightness4 = Parameters.NextParameterAsLong();//16 bit PWM requires use of longs for full scale 
 LED_Brightness();
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
  Motor1Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor1OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  Motor1onTime = Parameters.NextParameterAsDouble(); 
  Motor1offTime = Parameters.NextParameterAsDouble(); 
  Motor1Reps = Parameters.NextParameterAsInteger();
  Motor1Oscillation = Parameters.NextParameterAsInteger();
//Set interval and ensure rep number is initialised to 0 before starting 
//rev2.1  Motor1Interval = Motor1onTime*60; //Interval in seconds
  Motor1Interval = Motor1onTime;
  Motor1CurrentRep = 0;
  SetMotor1Enable();
  //Reset Motor1 timer 
  Motor1Timer.Reset();
  //Set current rep to 1 as now started
  Motor1CurrentRep = 1;
}

//Motor 2 timer control 
void Cmd_Motor2TimerControl(CommandParameter &Parameters)
{
  Motor2Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor2OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  Motor2onTime = Parameters.NextParameterAsDouble(); 
  Motor2offTime = Parameters.NextParameterAsDouble(); 
  Motor2Reps = Parameters.NextParameterAsInteger();
  Motor2Oscillation = Parameters.NextParameterAsInteger();
//Set interval and ensure rep number is initialised to 0 before starting 
//rev2.1  Motor2Interval = Motor2onTime*60; //Interval in seconds
  Motor2Interval = Motor2onTime;
  Motor2CurrentRep = 0;
  SetMotor2Enable();
  //Reset Motor2 timer 
  Motor2Timer.Reset();
  //Set current rep to 1 as now started
  Motor2CurrentRep = 1;
}

//Motor 3 timer control 
void Cmd_Motor3TimerControl(CommandParameter &Parameters)
{/*
  Motor3Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor3OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  Motor3onTime = Parameters.NextParameterAsDouble(); 
  Motor3offTime = Parameters.NextParameterAsDouble(); 
  Motor3Reps = Parameters.NextParameterAsInteger();
  Motor3Oscillation = Parameters.NextParameterAsInteger();
//Set interval and ensure rep number is initialised to 0 before starting 
  Motor3Interval = Motor3onTime*60; //Interval in seconds
  Motor3CurrentRep = 0;
  SetMotor3Enable();
  //Reset Motor3 timer 
  Motor3Timer.Reset();
  //Set current rep to 1 as now started
  Motor3CurrentRep = 1;
*/
}

//Motor 4 timer control 
void Cmd_Motor4TimerControl(CommandParameter &Parameters)
{/*
  Motor4Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor4OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  Motor4onTime = Parameters.NextParameterAsDouble(); 
  Motor4offTime = Parameters.NextParameterAsDouble(); 
  Motor4Reps = Parameters.NextParameterAsInteger();
  Motor4Oscillation = Parameters.NextParameterAsInteger();
//Set interval and ensure rep number is initialised to 0 before starting 
  Motor4Interval = Motor4onTime*60; //Interval in seconds
  Motor4CurrentRep = 0;
  SetMotor4Enable();
  //Reset Motor4 timer 
  Motor4Timer.Reset();
  //Set current rep to 1 as now started
  Motor4CurrentRep = 1;
*/
}

void Cmd_SetMotor1Enable(CommandParameter &Parameters)
{
  Motor1Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor1OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  SetMotor1Enable();
}

void Cmd_SetMotor2Enable(CommandParameter &Parameters)
{
  Motor2Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor2OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  SetMotor2Enable();
}

void Cmd_SetMotor3Enable(CommandParameter &Parameters)
{/*
  Motor3Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor3OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  SetMotor3Enable();
*/
}

void Cmd_SetMotor4Enable(CommandParameter &Parameters)
{/*
  Motor4Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor4OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  SetMotor4Enable();
*/
}

void Cmd_SetMotor1Disable(CommandParameter &Parameters){
  SetMotor1Disable ();
  Motor1CurrentRep = 0; // Also reset timing loops to prevent motor restarting
  }
void Cmd_SetMotor2Disable(CommandParameter &Parameters){
  SetMotor2Disable ();
  Motor2CurrentRep = 0; // Also reset timing loops to prevent motor restarting
  }
void Cmd_SetMotor3Disable(CommandParameter &Parameters){
/*  SetMotor3Disable ();
  Motor3CurrentRep = 0; // Also reset timing loops to prevent motor restarting
  */
  }
void Cmd_SetMotor4Disable(CommandParameter &Parameters){
/*  SetMotor4Disable ();
  Motor4CurrentRep = 0; // Also reset timing loops to prevent motor restarting
  */}

//Motors Speed
void Cmd_SetMotor1Speed(CommandParameter &Parameters)
{
  Motor1Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor1OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  if (enableStepper1)
      SetMotor1Enable();
}

void Cmd_SetMotor2Speed(CommandParameter &Parameters)
{ 
  Motor2Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor2OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  if (enableStepper2)
      SetMotor2Enable();
}

void Cmd_SetMotor3Speed(CommandParameter &Parameters)
{/*
  Motor3Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor3OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  if (enableStepper3)
      SetMotor3Enable();
*/}

void Cmd_SetMotor4Speed(CommandParameter &Parameters)
{ /*
  Motor4Direction = Parameters.NextParameterAsInteger(); //Set direction from message data 
  Motor4OutSpeed = Parameters.NextParameterAsDouble(); //Set outputspeed from message data 
  if (enableStepper4)
      SetMotor4Enable();
*/}

void Cmd_SetMotor1Current(CommandParameter &Parameters){
  Motor1CurrentPWM  = Parameters.NextParameterAsInteger();}
void Cmd_SetMotor2Current(CommandParameter &Parameters){
  Motor2CurrentPWM  = Parameters.NextParameterAsInteger();}
void Cmd_SetMotor3Current(CommandParameter &Parameters){
  /*Motor3CurrentPWM  = Parameters.NextParameterAsInteger();
    */
   }
void Cmd_SetMotor4Current(CommandParameter &Parameters){
  /*Motor4CurrentPWM  = Parameters.NextParameterAsInteger();
   */}

void Cmd_SetTargetTemp(CommandParameter &Parameters)
{
  TargetTemp1  = Parameters.NextParameterAsDouble();
  TargetTemp2  = Parameters.NextParameterAsDouble();
  TargetTemp3  = Parameters.NextParameterAsDouble();
  TargetTemp4  = Parameters.NextParameterAsDouble();
}

void Cmd_HeaterState(CommandParameter &Parameters)
{
  Heater1State = Parameters.NextParameterAsInteger();
  Heater2State = Parameters.NextParameterAsInteger();
  Heater3State = Parameters.NextParameterAsInteger();
  Heater4State = Parameters.NextParameterAsInteger();
  
  Heater3State = 0;   //Teensy 4.1
  Heater4State = 0;   //Teensy 4.1
  
  HeaterState();

  MyPanel.SetCheck("Heater3Active", Heater3State);  //Teensy 4.1
  MyPanel.SetCheck("Heater4Active", Heater4State);  //Teensy 4.1
}

void Cmd_SetTime(CommandParameter &Parameters)
{//set the RTC
  unsigned long ReceivedTime = Parameters.NextParameterAsUnsignedLong();
  Teensy3Clock.set(ReceivedTime);
  setTime(ReceivedTime);
  //Serial.println("Current Time Set!");
}

void Cmd_SyncParameters(CommandParameter &Parameters)
{
  SyncMegunoLink();
}

void Cmd_SDDataLog(CommandParameter &Parameters)
{ //SDDataLogTimer 
  SDDataLogInterval  = Parameters.NextParameterAsInteger();
  if (SDDataLog == 0){
    SDDataLog = 1;   
    SDDataLogTimer.Reset();  
    }
  if (SDDataLog == 2){
    SDDataLog = 3;
  }
}

//Motors Enable
void SetMotor1Enable()
{//Calculate speed scale factor (IF CCW direction is sent (-1) the speed will be negative and the motor will run backwards)
  Motor1StepSpeed = (OutputInputConversion * Motor1OutSpeed * Motor1Direction)/60;
  //Enable motor, set speed and run
  enableStepper1 = 1;  
  digitalWrite(enablePinStepper1, LOW); //enable pin set low to enable driver
  motor_1.rotateAsync(Motor1StepSpeed); //Begin rotating motor
}
void SetMotor2Enable()
{ //Calculate speed scale factor (IF CCW direction is sent (-1) the speed will be negative and the motor will run backwards)
  Motor2StepSpeed = (OutputInputConversion * Motor2OutSpeed * Motor2Direction)/60;
  //Enable motor, set speed and run
  enableStepper2 = 1;  
  digitalWrite(enablePinStepper2, LOW); //enable pin set low to enable driver
  motor_2.rotateAsync(Motor2StepSpeed); //Begin rotating motor
}
/*
void SetMotor3Enable()
{ //Calculate speed scale factor (IF CCW direction is sent (-1) the speed will be negative and the motor will run backwards)
  Motor3StepSpeed = (OutputInputConversion * Motor3OutSpeed * Motor3Direction)/60;
  //Enable motor, set speed and run
  enableStepper3 = 1;  
  digitalWrite(enablePinStepper3, LOW); //enable pin set low to enable driver
  motor_3.rotateAsync(Motor3StepSpeed); //Begin rotating motor
}
void SetMotor4Enable()
{ //Calculate speed scale factor (IF CCW direction is sent (-1) the speed will be negative and the motor will run backwards)
  Motor4StepSpeed = (OutputInputConversion * Motor4OutSpeed * Motor4Direction)/60;
  //Enable motor, set speed and run
  enableStepper4 = 1;  
  digitalWrite(enablePinStepper4, LOW); //enable pin set low to enable driver
  motor_4.rotateAsync(Motor4StepSpeed); //Begin rotating motor
}
*/
//Motors diable
void SetMotor1Disable()
{  
  if (enableStepper1){//can't stop more than 1 time
  motor_1.stopAsync(); //Stop Motor - non blocking fucntion 
  enableStepper1 = 0; 
  digitalWrite(enablePinStepper1, HIGH); //enable pin set high to disable driver
  pinMode(currentPin1, OUTPUT); //Configure stepper driver current pin as output 
  digitalWrite(currentPin1, LOW);//Set current pin LOW
  }
}
void SetMotor2Disable()
{  
  if (enableStepper2){//can't stop more than 1 time
  motor_2.stopAsync(); //Stop Motor - non blocking fucntion 
  enableStepper2 = 0;
  digitalWrite(enablePinStepper2, HIGH); //enable pin set high to disable driver
  pinMode(currentPin2, OUTPUT); //Configure stepper driver current pin as output 
  digitalWrite(currentPin2, LOW);//Set current pin LOW
  }
}
/*
void SetMotor3Disable()
{
  motor_3.stopAsync(); //Stop Motor - non blocking fucntion 
  enableStepper3 = 0;
  digitalWrite(enablePinStepper3, HIGH); //enable pin set high to disable driver
  pinMode(currentPin3, OUTPUT); //Configure stepper driver current pin as output 
  digitalWrite(currentPin3, LOW); //Set current pin LOW 
}
void SetMotor4Disable()
{
  motor_4.stopAsync(); //Stop Motor - non blocking fucntion 
  enableStepper4 = 0;
  digitalWrite(enablePinStepper4, HIGH); //enable pin set high to disable driver
  pinMode(currentPin4, OUTPUT); //Configure stepper driver current pin as output 
  digitalWrite(currentPin4, LOW);  //Set current pin LOW  
}
*/
//Control LED brightness
void LED_Brightness()
{
 tlc.setLED(0, Brightness1, 0, 0); //Set LED 1 Brightness (connected to R0 channel 0)
 tlc.setLED(1, 0, 0, Brightness2); //Set LED 2 Brightness (connected to B1 channel 5)
 tlc.setLED(2, 0, 0, Brightness3); //Set LED 3 Brightness (connected to B2 channel 8)
 tlc.setLED(3, 0, 0, Brightness4); //Set LED 4 Brightness (connected to B3 channel 11)
 tlc.write();
}

//Heater State Control
void HeaterState()
{
   //Heater 1 Logic
  if (Heater1State == 1){ //Turn Heater PID ON
    Heater1PID.SetMode(AUTOMATIC); //Turn PID Control ON  
    }
  else{
    Heater1PID.SetMode(MANUAL); //Turn PID Control OFF
    PIDOutput1 = 0; //Also manually set output PWM to 0 
    }

  //Heater 2 Logic
  if (Heater2State == 1){ //Turn Heater PID ON
    Heater2PID.SetMode(AUTOMATIC); //Turn PID Control ON  
    }
  else{
    Heater2PID.SetMode(MANUAL); //Turn PID Control OFF
    PIDOutput2 = 0; //Also manually set output PWM to 0 
    }
/*
   //Heater 3 Logic
  if (Heater3State == 1){ //Turn Heater PID ON
    Heater3PID.SetMode(AUTOMATIC); //Turn PID Control ON  
    }
  else{
    Heater3PID.SetMode(MANUAL); //Turn PID Control OFF
    PIDOutput3 = 0; //Also manually set output PWM to 0 
    }

  //Heater 4 Logic
  if (Heater4State == 1){ //Turn Heater PID ON
    Heater4PID.SetMode(AUTOMATIC); //Turn PID Control ON  
    }
  else{
    Heater4PID.SetMode(MANUAL); //Turn PID Control OFF
    PIDOutput4 = 0; //Also manually set output PWM to 0 
    }
*/    
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

//Teensy 3.6 RTC
time_t getTeensy3Time()
{//Function to fetch teensy RTC Time, Teensy RTC is set to current date and time when code is compiled and uploaded or "set current time" by MegunoLink
  return Teensy3Clock.get();
}

//Synchronization with Meguno Link parameters
void SyncMegunoLink()
{ //Sampling Frequency
 MyPanel.SetNumber("DataSendFreq", LoopFrequency);
 
 //LED Controller
 MyPanel.SetNumber("numBrightness1", Brightness1);
 MyPanel.SetNumber("numBrightness2", Brightness2); 
 MyPanel.SetNumber("numBrightness3", Brightness3);
 MyPanel.SetNumber("numBrightness4", Brightness4); 

 //Heater Controller
 MyPanel.SetNumber("TargetTemp1", TargetTemp1);
 MyPanel.SetNumber("TargetTemp2", TargetTemp2); 
 MyPanel.SetNumber("TargetTemp3", TargetTemp3);
 MyPanel.SetNumber("TargetTemp4", TargetTemp4); 
 MyPanel.SetCheck("Heater1Active", Heater1State);
 MyPanel.SetCheck("Heater2Active", Heater2State); 
 MyPanel.SetCheck("Heater3Active", Heater3State);
 MyPanel.SetCheck("Heater4Active", Heater4State); 

 //Stepper Controller
  MyPanel.SetNumber("Motor1OutSpeed", Motor1OutSpeed);
  MyPanel.SetNumber("Motor2OutSpeed", Motor2OutSpeed);
  MyPanel.SetNumber("Motor3OutSpeed", Motor3OutSpeed);
  MyPanel.SetNumber("Motor4OutSpeed", Motor4OutSpeed);  

  ReturnMotor1CurrentPWM = (0.3577-sqrt(0.3577*0.3577-4*0.0006*(256.6-Motor1CurrentPWM)))/(2*0.0006);
  ReturnMotor2CurrentPWM = (0.3577-sqrt(0.3577*0.3577-4*0.0006*(256.6-Motor2CurrentPWM)))/(2*0.0006);
  ReturnMotor3CurrentPWM = (0.3577-sqrt(0.3577*0.3577-4*0.0006*(256.6-Motor3CurrentPWM)))/(2*0.0006);
  ReturnMotor4CurrentPWM = (0.3577-sqrt(0.3577*0.3577-4*0.0006*(256.6-Motor4CurrentPWM)))/(2*0.0006);

  MyPanel.SetNumber("Stepper1Current", ReturnMotor1CurrentPWM);
  MyPanel.SetNumber("Stepper2Current", ReturnMotor2CurrentPWM); 
  MyPanel.SetNumber("Stepper3Current", ReturnMotor3CurrentPWM);
  MyPanel.SetNumber("Stepper4Current", ReturnMotor4CurrentPWM); 

  MyPanel.SetNumber("M1OnTime", Motor1onTime);
  MyPanel.SetNumber("M1OffTime", Motor1offTime); 
  MyPanel.SetNumber("M1Reps", Motor1Reps);
  MyPanel.SetCheck("M1oscillation", Motor1Oscillation); 

  MyPanel.SetNumber("M2OnTime", Motor2onTime);
  MyPanel.SetNumber("M2OffTime", Motor2offTime); 
  MyPanel.SetNumber("M2Reps", Motor2Reps);
  MyPanel.SetCheck("M2oscillation", Motor2Oscillation); 

  MyPanel.SetNumber("M3OnTime", Motor3onTime);
  MyPanel.SetNumber("M3OffTime", Motor3offTime); 
  MyPanel.SetNumber("M3Reps", Motor3Reps);
  MyPanel.SetCheck("M3oscillation", Motor3Oscillation);  

  MyPanel.SetNumber("M4OnTime", Motor4onTime);
  MyPanel.SetNumber("M4OffTime", Motor4offTime); 
  MyPanel.SetNumber("M4Reps", Motor4Reps);
  MyPanel.SetCheck("M4oscillation", Motor4Oscillation); 

  // Motor direction: CW = 1, CCW = -1
  if (Motor1Direction == 1)
  {
    MyPanel.SetCheck("CCWDir1", 0);
    MyPanel.SetCheck("CWDir1", 1);      
  }
  else
  {
    MyPanel.SetCheck("CCWDir1", 1);
    MyPanel.SetCheck("CWDir1", 0); 
  }

  if (Motor2Direction == 1)
  {  
    MyPanel.SetCheck("CCWDir2", 0);
    MyPanel.SetCheck("CWDir2", 1);
  }
  else
  {
    MyPanel.SetCheck("CCWDir2", 1);
    MyPanel.SetCheck("CWDir2", 0); 
  }

  if (Motor3Direction == 1)
  {
    MyPanel.SetCheck("CCWDir3", 0);
    MyPanel.SetCheck("CWDir3", 1);      
  }
  else
  {
    MyPanel.SetCheck("CCWDir3", 1);
    MyPanel.SetCheck("CWDir3", 0); 
  }

  if (Motor4Direction == 1)
  {  
    MyPanel.SetCheck("CCWDir4", 0);
    MyPanel.SetCheck("CWDir4", 1);
  }
  else
  {
    MyPanel.SetCheck("CCWDir4", 1);
    MyPanel.SetCheck("CWDir4", 0); 
  }

//PD & Sensors
  MyPanel.SetCheck("PD1Active", PD1Active);
  MyPanel.SetCheck("PD2Active", PD2Active); 
  MyPanel.SetCheck("PD3Active", PD3Active); 
  MyPanel.SetCheck("PD4Active", PD4Active);  
  MyPanel.SetCheck("Sensor1Active", Sensor1Active); 
  MyPanel.SetCheck("Sensor2Active", Sensor2Active);

  MyPanel.SetListValue("ADCResolution", ADCResolution); 
  MyPanel.SetListValue("ADCAverage", ADCAverage); 

  MyPanel.SetNumber("SDDataLogTimer", SDDataLogInterval);
  MyPanel.SetIndicator(F("SDIndicator"), SDIndicator); 

  if (SDIndicator){
    MyPanel.SetIndicatorColor(F("SDIndicator"), F("Green")); 
  }
}

//HIH8000 series functions
bool trigMeas()
{
  I2CStatus = hihSensor.triggerMeasurement();
  return true;
}

void fetchMeas()
{
  I2CStatus = hihSensor.fetchMeasurement();
}

/************************************************************************************************
General Setup
************************************************************************************************/
void setup() {
  
  //Initialisation of I/O Pins
  //Motor enable
  pinMode(enablePinStepper1, OUTPUT); //Configure stepper driver enable pin as output 
  digitalWrite(enablePinStepper1, HIGH);//Set enable pin high to disable driver
  pinMode(enablePinStepper2, OUTPUT); //Configure stepper driver enable pin as output 
  digitalWrite(enablePinStepper2, HIGH);//Set enable pin high to disable driver
/*  pinMode(enablePinStepper3, OUTPUT); //Configure stepper driver enable pin as output 
  digitalWrite(enablePinStepper3, HIGH);//Set enable pin high to disable driver
  pinMode(enablePinStepper4, OUTPUT); //Configure stepper driver enable pin as output 
  digitalWrite(enablePinStepper4, HIGH);//Set enable pin high to disable driver
*/  //Motor Current
  pinMode(currentPin1, OUTPUT); //Configure stepper driver current pin as output 
  digitalWrite(currentPin1, LOW);//Set current pin LOW to reduce current at power ON
  pinMode(currentPin2, OUTPUT); //Configure stepper driver current pin as output 
  digitalWrite(currentPin2, LOW);//Set current pin LOW to reduce current at power ON
/*  pinMode(currentPin3, OUTPUT); //Configure stepper driver current pin as output 
  digitalWrite(currentPin3, LOW);//Set current pin LOW to reduce current at power ON
  pinMode(currentPin4, OUTPUT); //Configure stepper driver current pin as output 
  digitalWrite(currentPin4, LOW);//Set current pin LOW to reduce current at power ON  
*/  
  //Heater
  pinMode(Heater1Pin, OUTPUT);
  digitalWrite(Heater1Pin, LOW);
  pinMode(Heater2Pin, OUTPUT);
  digitalWrite(Heater2Pin, LOW);
//unused Heater pins
  pinMode(Heater3Pin, OUTPUT);
  digitalWrite(Heater3Pin, LOW);
  pinMode(Heater4Pin, OUTPUT);
  digitalWrite(Heater4Pin, LOW);

  //Unused
  pinMode(enablePinStepper3, OUTPUT);
  digitalWrite(enablePinStepper3, LOW);
  pinMode(currentPin3, OUTPUT);
  digitalWrite(currentPin3, LOW);
  pinMode(stepPinStepper3, OUTPUT);
  digitalWrite(stepPinStepper3, LOW);
  pinMode(dirPinStepper3, OUTPUT);
  digitalWrite(dirPinStepper3, LOW);
  pinMode(enablePinStepper4, OUTPUT);
  digitalWrite(enablePinStepper4, LOW);
  pinMode(currentPin4, OUTPUT);
  digitalWrite(currentPin4, LOW);
  pinMode(stepPinStepper4, OUTPUT);
  digitalWrite(stepPinStepper4, LOW);
  pinMode(dirPinStepper4, OUTPUT);
  digitalWrite(dirPinStepper4, LOW);

  pinMode(Unused20Pin, OUTPUT);
  digitalWrite(Unused20Pin, LOW);
  pinMode(Unused21Pin, OUTPUT);
  digitalWrite(Unused21Pin, LOW);
  pinMode(Unused26Pin, OUTPUT);
  digitalWrite(Unused26Pin, LOW);
  pinMode(Unused31Pin, OUTPUT);
  digitalWrite(Unused31Pin, LOW);  
  pinMode(Unused32Pin, OUTPUT);
  digitalWrite(Unused32Pin, LOW);  
  pinMode(Unused33Pin, OUTPUT);
  digitalWrite(Unused33Pin, LOW);  
  pinMode(Unused34Pin, OUTPUT);
  digitalWrite(Unused34Pin, LOW);  
  pinMode(Unused35Pin, OUTPUT);
  digitalWrite(Unused35Pin, LOW);  
  pinMode(Unused38Pin, OUTPUT);
  digitalWrite(Unused38Pin, LOW);  
  pinMode(Unused40Pin, OUTPUT);
  digitalWrite(Unused40Pin, LOW); 

//SD Indicator
  pinMode(SDIndicatorPin, OUTPUT);
  digitalWrite(SDIndicatorPin, !SDIndicator);
   
  //Setup TLC communication
  tlc.begin();
  tlc.write();  

  //Setup Serial communication
  Serial.begin(115200);
  Serial.println(F("Serial Comms Established"));
  Serial.println(F("=========="));

//TeensyStep4 lib
TS4::begin();
  //Setup Motor 1    
   motor_1
    .setMaxSpeed(Motor1MaxStepSpeed)  // steps/s
    .setAcceleration(2500); // steps/s^2 

   //Setup Motor 2
   motor_2
    .setMaxSpeed(Motor2MaxStepSpeed)  // steps/s
    .setAcceleration(2500); // steps/s^2
/*    
   //Setup Motor 3
   motor_3
    .setMaxSpeed(Motor3MaxStepSpeed)  // steps/s
    .setAcceleration(2500); // steps/s^2 

   //Setup Motor 4
   motor_4
    .setMaxSpeed(Motor4MaxStepSpeed)  // steps/s
    .setAcceleration(2500); // steps/s^2 
*/  
  //Turn on LED's
  LED_Brightness();

// Set anlogue resolution and averagin for both ADC's;
    adc->adc0->setResolution(ADCResolution); // set bits of resolution
//    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS); // change the conversion speed it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed
    
    adc->adc1->setResolution(12); // set bits of resolution Heater only on this ADC
    adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
    adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed

 //Initilaise filtered temp value
  NTC1Raw = adc->adc1->analogRead(NTC1Pin); //Read using ADC library 
  CurrentTemp1 = MeasureTemperature(NTC1Raw);
  FilterTemp1.SetCurrent(CurrentTemp1);

  NTC2Raw = adc->adc1->analogRead(NTC2Pin); //Read using ADC library 
  CurrentTemp2 = MeasureTemperature(NTC2Raw);
  FilterTemp2.SetCurrent(CurrentTemp2);
/*
  NTC3Raw = adc->adc1->analogRead(NTC3Pin); //Read using ADC library 
  CurrentTemp3 = MeasureTemperature(NTC3Raw);
  FilterTemp3.SetCurrent(CurrentTemp3);

  NTC4Raw = adc->adc1->analogRead(NTC4Pin); //Read using ADC library 
  CurrentTemp4 = MeasureTemperature(NTC4Raw);
  FilterTemp4.SetCurrent(CurrentTemp4);  
*/
//HIH8000 series 
  Wire.setSDA(SDA0Pin);
  Wire.setSCL(SCL0Pin);
  Wire.begin();
  Wire.setClock(400000);
  
  hihSensor = HIH27;
  trigSuccess_HIH27 = trigMeas();  
/*
  hihSensor = HIH28;
  trigSuccess_HIH28 = trigMeas();  
*/ 
// Setup the serial commands we can repond to
  //Connection Initialise
  SerialCommandHandler.AddCommand(F("connectInitialise"), Cmd_connectInitialise);    
  //sync with MegunoLink
  SyncMegunoLink();
  SerialCommandHandler.AddCommand(F("SyncParameters"), Cmd_SyncParameters);   
  SerialCommandHandler.AddCommand(F("SetTime"), Cmd_SetTime);  
  SerialCommandHandler.AddCommand(F("DataSendFreq"), Cmd_DataSendFreq);
  SerialCommandHandler.AddCommand(F("SDDataLog"), Cmd_SDDataLog);    
  //LED Commands
  SerialCommandHandler.AddCommand(F("Brightness"), Cmd_Brightness);
  //ADC & PD Commands
  SerialCommandHandler.AddCommand(F("SetADCParameters"), Cmd_SetADCParameters);
  SerialCommandHandler.AddCommand(F("PDActive"), Cmd_PDActive);
  //Stepper Commands
  //Motor 1
  SerialCommandHandler.AddCommand(F("Motor1Enable"), Cmd_SetMotor1Enable);
  SerialCommandHandler.AddCommand(F("Motor1Disable"), Cmd_SetMotor1Disable);
  SerialCommandHandler.AddCommand(F("SetMotor1Speed"), Cmd_SetMotor1Speed);
  SerialCommandHandler.AddCommand(F("SetMotor1Current"), Cmd_SetMotor1Current);
  SerialCommandHandler.AddCommand(F("Motor1TimerControl"), Cmd_Motor1TimerControl);  
  //Motor 2
  SerialCommandHandler.AddCommand(F("Motor2Enable"), Cmd_SetMotor2Enable);
  SerialCommandHandler.AddCommand(F("Motor2Disable"), Cmd_SetMotor2Disable);
  SerialCommandHandler.AddCommand(F("SetMotor2Speed"), Cmd_SetMotor2Speed);
  SerialCommandHandler.AddCommand(F("SetMotor2Current"), Cmd_SetMotor2Current);
  SerialCommandHandler.AddCommand(F("Motor2TimerControl"), Cmd_Motor2TimerControl);
  //Motor 3    
  SerialCommandHandler.AddCommand(F("Motor3TimerControl"), Cmd_Motor3TimerControl);  
  SerialCommandHandler.AddCommand(F("Motor3Enable"), Cmd_SetMotor3Enable);
  SerialCommandHandler.AddCommand(F("Motor3Disable"), Cmd_SetMotor3Disable);
  SerialCommandHandler.AddCommand(F("SetMotor3Speed"), Cmd_SetMotor3Speed);
  SerialCommandHandler.AddCommand(F("SetMotor3Current"), Cmd_SetMotor3Current);
  //Motor 4
  SerialCommandHandler.AddCommand(F("Motor4TimerControl"), Cmd_Motor4TimerControl);
  SerialCommandHandler.AddCommand(F("Motor4Enable"), Cmd_SetMotor4Enable);
  SerialCommandHandler.AddCommand(F("Motor4Disable"), Cmd_SetMotor4Disable);
  SerialCommandHandler.AddCommand(F("SetMotor4Speed"), Cmd_SetMotor4Speed);
  SerialCommandHandler.AddCommand(F("SetMotor4Current"), Cmd_SetMotor4Current);
  
  //Heater Commands
  SerialCommandHandler.AddCommand(F("SetTargetTemp"), Cmd_SetTargetTemp);
  SerialCommandHandler.AddCommand(F("HeaterState"), Cmd_HeaterState); 

  //Initialisation of Timer
//  Motor1Timer.Reset();
//  Motor2Timer.Reset();
//  Motor3Timer.Reset();
//  Motor4Timer.Reset();    
//  SDDataLogTimer.Reset();
//  DataSendTimer.Reset();
  
}//end of Setup()

/************************************************************************************************
Main Program
************************************************************************************************/
void loop() {
  // Check for serial commands and dispatch them.
  SerialCommandHandler.Process();

/************************************************************************************************
  Motors Control
************************************************************************************************/
  //Motors Current Crontrol
if (enableStepper1)
  analogWrite(currentPin1, Motor1CurrentPWM);
     
if (enableStepper2)
  analogWrite(currentPin2, Motor2CurrentPWM);
/*
if (enableStepper3)
  analogWrite(currentPin3, Motor3CurrentPWM);
        
if (enableStepper4)
  analogWrite(currentPin4, Motor4CurrentPWM);
*/
  //Motor1 Timer 
  //If in the correct rep range 
if ((Motor1CurrentRep>=1) && (Motor1CurrentRep<=Motor1Reps)){
  //Iftime passed change state of motor and switch timer, and  incriment rep
//rev2.1 if (Motor1Timer.TimePassed_Seconds(Motor1Interval)){
  if (Motor1Timer.TimePassed_Minutes(Motor1Interval)){    
    //Interval passed detect if motor on or off
    //If motor is currently on
    if(enableStepper1){   
      //Serial.print("\n Motor Currently on, about to disable");
      //Disable motor
      SetMotor1Disable();      
      //reset timer and set interval to off time
//rev2.1 Motor1Interval = Motor1offTime*60;
      Motor1Interval = Motor1offTime;
      Motor1Timer.Reset();
      // iNCRIMENT REP NUMBER 
      Motor1CurrentRep++;
       }

  //If motor is currently off
     else {//if(enableStepper1 == 0){   
      // Reverse Motor speed factor if osscilation is selected 
      if(Motor1Oscillation == 1)
         Motor1Direction = Motor1Direction*-1;
       //Enable motor, set speed, direction and run
       SetMotor1Enable();
       //reset timer and set interval to on time
//rev2.1 Motor1Interval = Motor1onTime*60;
       Motor1Interval = Motor1onTime;
       Motor1Timer.Reset();
      }
  }
  }
  else if ((Motor1CurrentRep>Motor1Reps)){
  // Reset rep to 0
  Motor1CurrentRep = 0;
 }

  //Motor2 Timer 
  //If in the correct rep range 
if ((Motor2CurrentRep>=1) && (Motor2CurrentRep<=Motor2Reps)){
  //Iftime passed change state of motor and switch timer, and  incriment rep
//rev2.1  if (Motor2Timer.TimePassed_Seconds(Motor2Interval)){
  if (Motor2Timer.TimePassed_Minutes(Motor2Interval)){
    //Interval passed detect if motor on or off
    //If motor is currently on
    if(enableStepper2){ 
      //Disable motor
      SetMotor2Disable();  
      //reset timer and set interval to off time
      Motor2Interval = Motor2offTime;
      Motor2Timer.Reset();
      // iNCRIMENT REP NUMBER 
      Motor2CurrentRep++;
       }
  //If motor is currently off
      else {
      // Reverse Motor speed factor if osscilation is selected 
      if(Motor2Oscillation == 1)
         Motor2Direction = Motor2Direction*-1;
       //Enable motor, set speed, direction and run
       SetMotor2Enable();
       //reset timer and set interval to on time
       Motor2Interval = Motor2onTime;
       Motor2Timer.Reset();
       }
  }
  }
  else if ((Motor2CurrentRep>Motor2Reps)){
  // Reset rep to 0
  Motor2CurrentRep = 0;
 }
/*
  //Motor3 Timer 
  //If in the correct rep range 
if ((Motor3CurrentRep>=1) & (Motor3CurrentRep<=Motor3Reps)){
  //Iftime passed change state of motor and switch timer, and  incriment rep
  if (Motor3Timer.TimePassed_Seconds(Motor3Interval)){
    //Interval passed detect if motor on or off
    
    //If motor is currently on
    if(enableStepper3){  
      Serial.print("\n Motor 3 Currently on, about to disable");
      //Disable motor
      SetMotor3Disable();  
      //reset timer and set interval to off time
      Motor3Interval = Motor3offTime*60;
      Motor3Timer.Reset();     
      Serial.print("\nInterval timer set to OFF state (s) "); Serial.println(Motor3Interval);
      // iNCRIMENT REP NUMBER 
      Motor3CurrentRep++;
       }

  //If motor is currently off
     //else if(digitalRead(enablePinStepper3) == HIGH){
     else {  
      Serial.print("\nMotor 3 Currently off, about to enable");

      // Reverse Motor speed factor if osscilation is selected 
      if(Motor3Oscillation == 1)
         Motor3Direction = Motor3Direction*-1;

       //Enable motor, set speed, direction and run
       Serial.print("\nEnable Motor 3: Rep#");Serial.println(Motor3CurrentRep);
       SetMotor3Enable();
        //reset timer and set interval to on time
       Motor3Interval = Motor3onTime*60;
       Motor3Timer.Reset();       
       Serial.print("\nInterval timer set to ON state (s) "); Serial.println(Motor3Interval);
      }
  }
  }
  else if ((Motor3CurrentRep>Motor3Reps)){
   // Reset rep to 0
  Motor3CurrentRep = 0;
 }

  //Motor4 Timer 
  //If in the correct rep range 
if ((Motor4CurrentRep>=1) & (Motor4CurrentRep<=Motor4Reps)){
  //Iftime passed change state of motor and switch timer, and  incriment rep
  if (Motor4Timer.TimePassed_Seconds(Motor4Interval)){
    //Interval passed detect if motor on or off
    
    //If motor is currently on
    if(enableStepper4){  
      Serial.print("\n Motor Currently on, about to disable");
      //Disable motor
      SetMotor4Disable();  
      //reset timer and set interval to off time
      Motor4Interval = Motor4offTime*60;      
      Motor4Timer.Reset();
      Serial.print("\nInterval timer set to OFF state (s) "); Serial.println(Motor4Interval);
      // iNCRIMENT REP NUMBER 
      Motor4CurrentRep++;
       }

  //If motor is currently off
//     else if(digitalRead(enablePinStepper4) == HIGH){ 
      else { 
      Serial.print("\nMotor Currently off, about to enable");

      // Reverse Motor speed factor if osscilation is selected 
      if(Motor4Oscillation == 1)
         Motor4Direction = Motor4Direction*-1;

       //Enable motor, set speed, direction and run
       Serial.print("\nEnable Motor 4: Rep#");Serial.println(Motor4CurrentRep);
       SetMotor4Enable();
       //reset timer and set interval to on time
       Motor4Interval = Motor4onTime*60;       
       Motor4Timer.Reset();
       Serial.print("\nInterval timer set to ON state (s) "); Serial.println(Motor4Interval);
      }
  }
  }
  else if ((Motor4CurrentRep>Motor4Reps)){
  // Reset rep to 0
  Motor4CurrentRep = 0;
 } 
*/

/************************************************************************************************
  Measure NTC
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
/*
  //Read NTC3 convert to temperature and smooth
  NTC3Raw = adc->adc1->analogRead(NTC3Pin); //Read using ADC library 
  CurrentTemp3 = MeasureTemperature(NTC3Raw);
  FilterTemp3.Filter(CurrentTemp3);
  FilteredCurrentTemp3 = FilterTemp3.Current();

  //Read NTC4 convert to temperature and smooth
  NTC4Raw = adc->adc1->analogRead(NTC4Pin); //Read using ADC library 
  CurrentTemp4 = MeasureTemperature(NTC4Raw);
  FilterTemp4.Filter(CurrentTemp4);
  FilteredCurrentTemp4 = FilterTemp4.Current();
*/
/************************************************************************************************
  Heater PID control 
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
/*
 //Heater 3
  if (FilteredCurrentTemp3 > 0){
  Heater3PID.Compute();
  }
  else{
  PIDOutput3 = 0;
  }
  analogWrite(Heater3Pin,PIDOutput3);

  //Heater 4
  if (FilteredCurrentTemp4 > 0){
  Heater4PID.Compute();
  }
  else{
  PIDOutput4 = 0;
  }
  analogWrite(Heater4Pin,PIDOutput4);
*/
  if (SDDataLogTimer.TimePassed_Minutes(SDDataLogInterval, false) && SDDataLog == 2)
  {
    SDDataLog=3;
    //Serial.println("SDTimer***:");
  }

/************************************************************************************************
  PD and Temperature Plot
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
      PD1Total += adc->adc0->analogRead(PD1Pin);
      if(i==(ADCAverage-1)){
          PD1Average = PD1Total >> BitShift;
          }
    }
  PD1Plot.SendData("PD1", PD1Average);
  }
/************************************************************************************************/

  if(PD2Active == 1){
  for(int i = 0; i < ADCAverage; i++) {
      PD2Total += adc->adc0->analogRead(PD2Pin);
      if(i==(ADCAverage-1)){
          PD2Average = PD2Total >> BitShift;
          }
    }
  PD2Plot.SendData("PD2", PD2Average);
  }
/************************************************************************************************/
  
  if(PD3Active == 1){
  for(int i = 0; i < ADCAverage; i++) {
    PD3Total += adc->adc0->analogRead(PD3Pin);
     if(i==(ADCAverage-1)){
  PD3Average = PD3Total >> BitShift;
    }
    }
  PD3Plot.SendData("PD3", PD3Average);
  }
/************************************************************************************************/

  if(PD4Active == 1){
  for(int i = 0; i < ADCAverage; i++) {
    PD4Total += adc->adc0->analogRead(PD4Pin);
     if(i==(ADCAverage-1)){
  PD4Average = PD4Total >> BitShift;
    }
    }
  PD4Plot.SendData("PD4", PD4Average);
  }
/************************************************************************************************/

  if(Sensor1Active == 1){
  for(int i = 0; i < ADCAverage; i++) {
      Sensor1Total += adc->adc0->analogRead(Sensor1Pin);
      if(i==(ADCAverage-1)){
          Sensor1Average = Sensor1Total >> BitShift;
          }
    }
  Sensor1Plot.SendData("Sensor1", Sensor1Average);
  }

/************************************************************************************************/

  if(Sensor2Active == 1){
  for(int i = 0; i < ADCAverage; i++) {
      Sensor2Total += adc->adc0->analogRead(Sensor2Pin);
      if(i==(ADCAverage-1)){
          Sensor2Average = Sensor2Total >> BitShift;
          }
    }
  Sensor2Plot.SendData("Sensor2", Sensor2Average);
  }
/************************************************************************************************/
  Temp1Plot.SendData("Temp1", FilteredCurrentTemp1);
  Temp2Plot.SendData("Temp2", FilteredCurrentTemp2);
//  Temp3Plot.SendData("Temp3", FilteredCurrentTemp3);
//  Temp4Plot.SendData("Temp4", FilteredCurrentTemp4);
/************************************************************************************************/

// HIH8000 series get reading

    if (trigSuccess_HIH27) {
      hihSensor = HIH27;
      fetchMeas();
      HIH27_Temperature = hihSensor.getTemperature();
      HIH27_Humidity = hihSensor.getHumidity();
      }
  
    hihSensor = HIH27;
    trigSuccess_HIH27 = trigMeas();
/*
    if (trigSuccess_HIH28) {
      hihSensor = HIH28;
      fetchMeas();
      HIH28_Temperature = hihSensor.getTemperature();
      HIH28_Humidity = hihSensor.getHumidity();
      }
  
    hihSensor = HIH28;
    trigSuccess_HIH28 = trigMeas();
*/

/*
 //Set motor speeds for readout 
 //If motor enabled write motor speed to output variable
 if(digitalRead(enablePinStepper1) == LOW){
    Motor1OutSpeedTrue = Motor1OutSpeed;
 }
    else{
  Motor1OutSpeedTrue = 0;
    }*/
/************************************************************************************************/
/*
 if(digitalRead(enablePinStepper2) == LOW){
    Motor2OutSpeedTrue = Motor2OutSpeed;
 }
    else{
  Motor2OutSpeedTrue = 0;
    }*/
/************************************************************************************************/
/*
 if(digitalRead(enablePinStepper3) == LOW){
    Motor3OutSpeedTrue = Motor3OutSpeed;
 }
    else{
  Motor3OutSpeedTrue = 0;
    }*/
 /************************************************************************************************/
/*
 if(digitalRead(enablePinStepper4) == LOW){
    Motor4OutSpeedTrue = Motor4OutSpeed;
 }
    else{
  Motor4OutSpeedTrue = 0;
    }*/
/************************************************************************************************/
  
//Meguno Link data logging 
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
/*  Serial.print(FilteredCurrentTemp3);
  Serial.print(",");
  Serial.print(FilteredCurrentTemp4);
  Serial.print(",");
*/
  Serial.print(Motor1OutSpeed);
  Serial.print(",");
  Serial.print(Motor2OutSpeed);
  Serial.print(",");
/*  Serial.print(Motor3OutSpeed);
  Serial.print(",");
  Serial.print(Motor4OutSpeed);
  Serial.print(",");
*/  
  Serial.print(String(HIH27_Temperature, 2));
  Serial.print(",");
  Serial.print(String(HIH27_Humidity, 2));
/*  Serial.print(",");
  Serial.print(String(HIH28_Temperature, 2));
  Serial.print(",");
  Serial.print(String(HIH28_Humidity, 2));
*/
  LogCSVData.End();


  //SD Card Data Logging
  if ( SDDataLog == 1)  //create & open file 
  {
  // Initialize the SD
  if (sd.begin(SD_CONFIG)) { 

/*   
  if (!sd.begin(SD_CONFIG)) {   
    //SD Card Ad-normal/Removed
   
    Heater1State = 0;
    Heater1PID.SetMode(MANUAL); //Turn PID Control OFF
    PIDOutput1 = 0; //Also manually set output PWM to 0 
  
    Heater2State = 0;
    Heater2PID.SetMode(MANUAL); //Turn PID Control OFF
    PIDOutput2 = 0; //Also manually set output PWM to 0 
  
    //Motor 1 & 2
    motor_1.stopAsync(); //Stop Motor - non blocking fucntion 
    digitalWrite(enablePinStepper1, HIGH); //enable pin set high to disable driver
    
    motor_2.stopAsync(); //Stop Motor - non blocking fucntion 
    digitalWrite(enablePinStepper2, HIGH); //enable pin set high to disable driver

    //Update MegunoLink
    SyncMegunoLink();

    sd.initErrorHalt(&Serial); 
    }
*/ 
    SDIndicator = !SDIndicator;   
    digitalWrite(SDIndicatorPin, !SDIndicator);
    MyPanel.SetIndicator(F("SDIndicator"), SDIndicator); 
    MyPanel.SetIndicatorColor(F("SDIndicator"), F("Green")); 
    
  if (!file.open("Datalog.txt", O_RDWR | O_CREAT | O_TRUNC))
  {//Can't open file
    Serial5.println("open failed\n");
    SDIndicator = !SDIndicator;    
    digitalWrite(SDIndicatorPin, !SDIndicator);
    MyPanel.SetIndicator(F("SDIndicator"), SDIndicator); 
    SDDataLog = 0;
    return;
  }
  
  //Start RTC Timing
  Timezone myTZ;
  myTZ.setTime(getTeensy3Time());

  if (timeStatus()!= timeSet) {         //Check for RTC sync
    file.println("Unable to sync with the RTC");
  } else {
    file.println("RTC has set the system time");
  }
//    Serial.println(myTZ.dateTime());
    file.println(myTZ.dateTime());

  //Startup Process
  file.println("Analysis System Starting...");
//  Serial.println("Standby");
  file.println("Standby");
  file.println();
  
  file.println("Initial values");
  file.print("ADCResolution: ");
  file.println(ADCResolution);
  file.print("ADCAverage: ");
  file.println(ADCAverage);
  file.print("Brightness1: ");
  file.println(Brightness1);
  file.print("Brightness2: ");
  file.println(Brightness2);
  file.print("Brightness3: ");
  file.println(Brightness3);
  file.print("Brightness4: ");
  file.println(Brightness4);  
  file.print("Heater1State: ");
  file.println(Heater1State);
  file.print("TargetTemp1: ");
  file.println(TargetTemp1 );  
  file.print("Heater2State: ");
  file.println(Heater2State);
  file.print("TargetTemp2: ");
  file.println(TargetTemp2 );
/*  file.print("Heater3State: ");
  file.println(Heater3State);
  file.print("TargetTemp3: ");
  file.println(TargetTemp3 );  
  file.print("Heater4State: ");
  file.println(Heater4State);
  file.print("TargetTemp4: ");
  file.println(TargetTemp4 );
*/
  file.print("Motor1CurrentPWM: ");
  file.println(Motor1CurrentPWM);
  file.print("Motor1Direction(1=CW): ");
  file.println(Motor1Direction);
  file.print("Motor1OutSpeed(RPM): ");
  file.println(Motor1OutSpeed);
  file.print("Motor2CurrentPWM: ");
  file.println(Motor2CurrentPWM);
  file.print("Motor2Direction(1=CW): ");
  file.println(Motor2Direction);  
  file.print("Motor2OutSpeed(RPM): ");
  file.println(Motor2OutSpeed);  
/*  file.print("Motor3CurrentPWM: ");
  file.println(Motor3CurrentPWM);
  file.print("Motor3Direction(1=CW): ");
  file.println(Motor3Direction);
  file.print("Motor3OutSpeed(RPM): ");
  file.println(Motor3OutSpeed);
  file.print("Motor4CurrentPWM: ");
  file.println(Motor4CurrentPWM);
  file.print("Motor4Direction(1=CW): ");
  file.println(Motor4Direction);  
  file.print("Motor4OutSpeed(RPM): ");
  file.println(Motor4OutSpeed);  
*/
  file.print("DataSendInterval(ms): ");
  file.println(DataSendInterval); 
  
  file.close();

  //Create .csv file
  char dateBuffer[32];
  dateTime("d-M-H-i").toCharArray(dateBuffer,32);
  sprintf(fileName, "%s.csv", dateBuffer);
  Serial.println(fileName);
  if (!file.open(fileName, O_RDWR | O_CREAT | O_TRUNC))
  {//Can't open file
    Serial.println("open failed\n");
    SDIndicator = !SDIndicator;
    digitalWrite(SDIndicatorPin, !SDIndicator);
    MyPanel.SetIndicator(F("SDIndicator"), SDIndicator); 
    SDDataLog = 0;    
    return;
  }
//   file.println("Time,PD1AVG,PD2AVG,PD3AVG,PD4AVG,Sensor1AVG,Sensor2AVG,Heater1Temp,Heater2Temp,Heater3Temp,Heater4Temp,M1Speed,M2Speed,M3Speed,M4Speed,Box_Temp(C),Box_Hum(%)");
   file.println("Time,PD1AVG,PD2AVG,PD3AVG,PD4AVG,Sensor1AVG,Sensor2AVG,Heater1Temp,Heater2Temp,M1Speed,M2Speed,Box_Temp(C),Box_Hum(%)");
   SDDataLog = 2;
  }
  else //without SD Card
    SDDataLog = 0;    
  }
   
  if  (SDDataLog == 2)    //data logging
  { 
  file.print(dateTime("d-M-y H:i:s.v"));
  file.print(",");
  file.print(PD1Average);
  file.print(",");
  file.print(PD2Average);
  file.print(",");
  file.print(PD3Average);
  file.print(",");
  file.print(PD4Average);
  file.print(",");
  file.print(Sensor1Average);
  file.print(",");
  file.print(Sensor2Average);  
  file.print(",");
  file.print(FilteredCurrentTemp1);
  file.print(",");
  file.print(FilteredCurrentTemp2);
  file.print(",");
/*  file.print(FilteredCurrentTemp3);
  file.print(",");
  file.print(FilteredCurrentTemp4); 
  file.print(",");
*/
  file.print(Motor1OutSpeed);
  file.print(",");
  file.print(Motor2OutSpeed);
  file.print(",");
/*  file.print(Motor3OutSpeed);
  file.print(",");
  file.print(Motor4OutSpeed);   
  file.print(",");
*/
  file.print(String(HIH27_Temperature, 2));
  file.print(",");            
  file.println(String(HIH27_Humidity, 2));
/*  file.print(",");
  file.print(String(HIH28_Temperature, 2));
  file.print(",");            
  file.println(String(HIH28_Humidity, 2));
*/
/*
  DataSamples +=1;
  if (DataSamples == 36000)  //20Hz: 1 min=1200; 30 min=36000 (Field test)
//    if (DataSamples == 600)  //10Hz: 1 min=600; 30 min=18000 (Testing)
//    if (DataSamples == 1200)  //20Hz: 1 min=1200; 30 min=36000 (Testing)
    SDDataLog=3;
*/    
  }

  if (SDDataLog == 3)   //Close file
  {
  file.println(F("Stopping Analysis"));
  file.close();

  SDDataLogTimer.Reset();

  /************************************************************************************************
  - Disable Heater 1, 2, 3, 4
  - Disable motor 1, 2, 3, 4
  ************************************************************************************************/
  Heater1State = 0; 
  Heater2State = 0;  
//  Heater3State = 0;
//  Heater4State = 0;

  HeaterState();

  SetMotor1Disable();  
//  Motor1Timer.Reset();
  Motor1CurrentRep = 0; // Also reset timing loops to prevent motor restarting
  SetMotor2Disable();  
//  Motor2Timer.Reset();
  Motor2CurrentRep = 0; // Also reset timing loops to prevent motor restarting

//  SetMotor3Disable();  
//  Motor3CurrentRep = 0; // Also reset timing loops to prevent motor restarting
//  SetMotor4Disable();
//  Motor3CurrentRep = 0; // Also reset timing loops to prevent motor restarting
  
  SDIndicator = !SDIndicator;
  digitalWrite(SDIndicatorPin, !SDIndicator);
  SDDataLog = 0;
  //sync with MegunoLink
  SyncMegunoLink();
  }
   
  
/************************************************************************************************
Main measure loop speed and send frequency back to megunolink
************************************************************************************************/
  PrevLoopEndTime = LoopEndTime;
  LoopEndTime = millis();
  LoopFrequency = (1000.0/(LoopEndTime - PrevLoopEndTime));
  MyPanel.SetText(F("CurrentFrequency"), LoopFrequency);

  //Temperature & RH send to MegunoLink
  MyPanel.SetText(F("Temp1"), HIH27_Temperature,1);
  MyPanel.SetText(F("RH1"), HIH27_Humidity,1);
/*
    //Temperature & RH send to MegunoLink
  MyPanel.SetText(F("Temp1"), HIH28_Temperature,1);
  MyPanel.SetText(F("RH1"), HIH28_Humidity,1);*/
  }

}
