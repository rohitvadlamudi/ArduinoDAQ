float AcclEarthsGravity = 9.81;// m/s^2
////////////////   DATA LOGGING   //////////////////
#include <SPI.h> //CH "Load SPI (Serial Peripheral Interface) library"
#include <Adafruit_GPS.h> //CH "Install the adafruit GPS library"
#include <SD.h> //CH "Load SD card library"
#include <SoftwareSerial.h>
#include <avr/sleep.h> //CH Loads the AVR Sleep library (This allows AVR device [Mega] to sleep at times to reduce power consumption)
#define mySerial Serial1 //Serial CH1 Arduino Mega - SD module connects to Serial 1 of Arduino Mega
Adafruit_GPS GPS(&mySerial);// The SD module has a GPS but this functionality is not used in this project
#define LOG_FIXONLY false  // Set to 'true' if you want to debug and listen to the raw GPS sentences. We do not use the GPS functionality
boolean usingInterrupt = false;// this keeps track of whether we're using the interrupt. The GPS sentences from the module are read using interrupts.GPS is not required 
#define chipSelect 10 // Define the chip select pin. Chipselect pin should be set to 10 for Arduino Mega.
#define ledPin 13 // The LED blink everytime the loop is completed
File logfile;


////////////////   AIRSPEED   //////////////////
float Velocity; // Float used in airspeed calculations
char c;
int sensorPin = A15;   // Define the Airspeed sensor's Analog pin on Arduino Mega
int sensorValue = 0; // Define the sensorvalue as zero.
float sensorValue2;  //  Float used in Airspeed calculations
float Vout;  //   Float used in Airspeed calculations
float P;    //Float used in Airspeed calculations

////////////////   FUELFLOW RATE   //////////////////
float flowrate;// Float used in fuelflowrate calculations
float flowrate2;// Float used in fuelflowrate calculations
#include <SoftwareSerial.h> // Library to define additonal serial communication pins.
SoftwareSerial ArduinoNanoFuelFlow(52, 53); // RX, TX  -- The Arduino Nano reading pulse signal from the fuel flow sensor connects to this software serial port. Specific pins on the Arduino can for serial communication can be defined. 


////////////////   CURRENT SHUNT AMPLIFIER   //////////////////
#include <Adafruit_ADS1015.h> // Install the ADS1015/ADS1115 Library
#include <Wire.h>  // Install the wire library for IIC communication
Adafruit_ADS1115 ads; // Define a reference name for the ADS1015/ADS1115
double multi = 0.0078125F; // This value is defined by the selected gain.
double impliedMv; //  Float used in current draw calculations
double ampsPerMv = 4; // Float used in current draw calculations
int16_t result; //Float used in current draw calculations
float amps; // Float used in current draw calculations


////////////////   RPM   //////////////////
float RPM =0;  // The RPM is set to zero
int blades=2; // Define the Propeller blade count




////////////////   THRUST AND TORQUE   //////////////////
#include "HX711.h"  // Load cell Amplifier Library
HX711 scale2(25, 23); // Define the pins for interfacing the HX711 Library
HX711 scale(24, 22); //Define the pins for interfacing the HX711 Library
long calibration_factor =  210000; // Define the calibration factor for Thrust Loadcell. This value can be changed with the HMI device.
long calibration_factor2= 210000;  //Define the calibration factor for Torque Loadcell. This value can be changed with the HMI device.
float distance_from_Base_to_Shaft = 1.50; // Define the vertical distance from the test bench's base to the crank shaft in meters. This can be changed from the HMI device
float distance_from_Shaft_to_Loadcell = 0.80; // Define the horizontal distance from the base to the Torque load cell in meters. This can be changed from the HMI device
float xx = distance_from_Shaft_to_Loadcell;
float propellerdiameter = 11.00; // Define the Propeller's diamter in inches.
float pd2 = propellerdiameter*0.0254; // Define the Propeller's diamter in meters.
long y;  // 
float output; // Float used in Thrust caluculation
float output2; // Float used in Torque caluculation
int x = 0;  // Selects thrust load in calibration page by default
////////////////   HMI Touch Screen device   //////////////////
#include <Nextion.h>
String command;  // The HMI device cannot display floats. Thus, all the values must be converted to strings
String command2;
String command3;
String command4;
String command5;
String command6;
String command7;
String command8;
String command9;
String propellerdiameter1;
String blades1;
String distance_from_Base_to_Shaft1;  
String distance_from_Shaft_to_Loadcell1;

int testtype= 0;// Static or Dynamic - Defines the flight environement as static by default
int fueltype= 0;//  Battery or Gasoline- Selects battery as default

////////// GUAGES //////////// 
// The below variables are used for mapping the performance parameters in degress from 0 to 360 to dispaly them on the guages.
int x1;
int x2;
int x3;
int x4;
int x5;
int x6;
int x7;
int x8;
int x9;
int x10;
int x11;


///
int CurrentPage = 0;  // Create a variable to store which page is currently loaded
NexDSButton b12 = NexDSButton(1, 2, "b12");     // Dual state button added
NexPage page0 = NexPage(0, 0, "page0");         // Page added as a touch event
NexPage page1 = NexPage(1, 0, "page1");         // Page added as a touch event
NexPage page2 = NexPage(2, 0, "page2");         // Page added as a touch event
NexPage page3 = NexPage(3, 0, "page3");         // Page added as a touch event
NexPage page4 = NexPage(4, 0, "page4");         // Page added as a touch event
NexPage page5 = NexPage(5, 0, "page5");         // Page added as a touch event
NexPage page6 = NexPage(6, 0, "page6");         // Page added as a touch event
NexPage page7 = NexPage(7, 0, "page7");         // Page added as a touch event
NexPage page8 = NexPage(8, 0, "page8");         // Page added as a touch event
NexPage page9 = NexPage(9, 0, "page9");         // Page added as a touch event
NexPage page10 = NexPage(10, 0, "page10");      // Page added as a touch event
NexPage page11 = NexPage(11, 0, "page11");      // Page added as a touch event
NexButton b13 = NexButton(1, 8, "b13");    // Button added
NexButton b14 = NexButton(1, 9, "b14");    // Button added
NexButton b15 = NexButton(1, 10, "b15");   // Button added
NexButton b16 = NexButton(1, 11, "b16");   // Button added
NexButton b17 = NexButton(1, 12, "b17");   // Button added
NexButton b19 = NexButton(1, 13, "b19");   // Button added
NexButton b20 = NexButton(1, 14, "b20");   // Button added
NexButton b21 = NexButton(1, 15, "b21");   // Button added
NexButton b22 = NexButton(1, 16, "b22");   // Button added
NexButton b23 = NexButton(1, 17, "b23");   // Button added
NexButton b18 = NexButton(1, 18, "b18");   // Button added
NexButton b24 = NexButton(4, 12, "b24");   // Button added
NexButton b25 = NexButton(4, 13, "b25");   // Button added
NexButton b27 = NexButton(5, 3, "b27");    // Button added
NexButton b28 = NexButton(5, 4, "b28");    // Button added
NexButton b29 = NexButton(9, 5, "b29");    // Button added
NexButton b30 = NexButton(9, 6, "b30");    // Button added
NexButton b31 = NexButton(9, 9, "b31");    // Button added
NexButton b32 = NexButton(9, 10, "b32");   // Button added
NexButton b33 = NexButton(10, 4, "b33");   // Button added
NexButton b34 = NexButton(10, 8, "b34");   // Button added
NexButton b35 = NexButton(10, 9, "b35");   // Button added
NexButton b36 = NexButton(10, 10, "b36");  // Button added
NexButton b37 = NexButton(11, 7, "b37");   // Button added
NexButton b38 = NexButton(11, 6, "b38");   // Button added
NexButton b39 = NexButton(2, 4, "b39");    // Button added
NexButton b40 = NexButton(2, 5, "b40");    // Button added

NexTouch *nex_listen_list[] = 
{
  &b12,  // Button added
  &b13,  // Button added
  &b14,  // Button added
  &b15,  // Button added
  &b16,  // Button added
  &b17,  // Button added
  &b18,  // Button added
  &b19,  // Button added
  &b20,  // Button added
  &b21,  // Button added
  &b22,  //Button added
  &b23,  // Button added
  &b24,  // Button added
  &b25,  // Button added
  &b27,  // Button added
  &b28,  // Button added
  &b29,  // Button added
  &b30,  // Button added
  &b31,  // Button added
  &b32,  // Button added
  &b33,  // Button added
  &b34,  // Button added
  &b35,  // Button added 
  &b36,  // Button added
  &b37,  // Button added
  &b38,  // Button added
  &b39,  // Button added
  &b40,  // Button added
  &page0,    // Page added as a touch event
  &page1,    // Page added as a touch event
  &page2,    // Page added as a touch event
  &page3,    // Page added as a touch event
  &page4,    // Page added as a touch event
  &page5,    // Page added as a touch event
  &page6,    // Page added as a touch event
  &page7,    // Page added as a touch event
  &page8,    // Page added as a touch event
  &page9,    // Page added as a touch event
  &page10,   // Page added as a touch event
  &page11,   // Page added as a touch event
  NULL  // String terminated
};  // End of touch event list




//// Page events////// -  These events let the Arduino know what page you are on the HMI display.
void page0PushCallback(void *ptr)  
{
  CurrentPage = 0; // page0
}  
void page1PushCallback(void *ptr) 
{
  CurrentPage = 1; 
} 
void page2PushCallback(void *ptr) 
{
  CurrentPage = 2; 
} 

void page3PushCallback(void *ptr) 
{
  CurrentPage = 3; 
} 
void page4PushCallback(void *ptr) 
{
  CurrentPage = 4; 
} 
void page5PushCallback(void *ptr) 
{
  CurrentPage = 5; 
} 

void page6PushCallback(void *ptr) 
{
  CurrentPage = 6; 
} 
void page7PushCallback(void *ptr) 
{
  CurrentPage = 7; 
} 
void page8PushCallback(void *ptr) 
{
  CurrentPage = 8; 
} 
void page9PushCallback(void *ptr) 
{
  CurrentPage = 9; 
} 
void page10PushCallback(void *ptr) 
{
  CurrentPage = 10; 
} 
void page11PushCallback(void *ptr) 
{
  CurrentPage = 11; 
} 


void b12PopCallback(void *ptr) //  Select the load cell for calibration
{
  uint32_t number5 = 0;  
  b12.getValue(&number5);  

  if(number5 == 1){ 
    x =1 ;//Thrust laod cell  selected for calibration
  }else{  
   x =0 ;  // Torque load cell selected for calibration
  }
}




/////////////////// Code for adjusting the calibration factor of load cells /////////////////////////////////////


void b13PushCallback(void *ptr) // Increase the calibartion factor by 1
{
  if (x == 0){
    calibration_factor = calibration_factor + 1; // Increases Torque loadcell's calibration factor by 1
  }
  else {
    calibration_factor2 = calibration_factor2 +1;  // Increases Thrust loadcell's calibration factor by 
  }
  
} 

void b14PushCallback(void *ptr) 
{
  if (x == 0){
    calibration_factor = calibration_factor + 10;  // 
  }
  else {
    calibration_factor2 = calibration_factor2 +10;
  }
  
} 

void b15PushCallback(void *ptr) 
{
  if (x == 0){
    calibration_factor = calibration_factor + 100;
  }
  else {
    calibration_factor2 = calibration_factor2 +100;
  }
  
} 

void b16PushCallback(void *ptr) 
{
  if (x == 0){
    calibration_factor = calibration_factor + 1000;
  }
  else {
    calibration_factor2 = calibration_factor2 +1000;
  }
  
} 

void b17PushCallback(void *ptr) 
{
  if (x == 0){
    calibration_factor = calibration_factor + 10000;
  }
  else {
    calibration_factor2 = calibration_factor2 +10000;
  }
  
} 

void b18PopCallback(void *ptr) // Resests the loadcell values to zero.
{
    if (x == 0){
     scale.tare(); // Thrust loadcell value set to zero
    }
   if (x == 1){
     scale2.tare(); // Torque loadcell value set to zero

   }
}

void b19PushCallback(void *ptr) 
{
  if (x == 0){
    calibration_factor = calibration_factor -1;
  }
  else {
    calibration_factor2 = calibration_factor2 -1;
  }
  
} 

void b20PushCallback(void *ptr) 
{
  if (x == 0){
    calibration_factor = calibration_factor -10;
  }
  else {
    calibration_factor2 = calibration_factor2 -10;
  }
  
} 

 void b21PushCallback(void *ptr) 
{
  if (x == 0){
    calibration_factor = calibration_factor -100;
  }
  else {
    calibration_factor2 = calibration_factor2 -100;
  }
  
} 

void b22PushCallback(void *ptr) 
{
  if (x == 0){
    calibration_factor = calibration_factor -1000;
  }
  else {
    calibration_factor2 = calibration_factor2 -1000;
  }
  
} 

void b23PushCallback(void *ptr) 
{
  if (x == 0){
    calibration_factor = calibration_factor -10000;
  }
  else {
    calibration_factor2 = calibration_factor2 -10000;
  }
  
} 
 ///// Load cell calibaration code ends here////////
 
void b24PopCallback(void *ptr) // Thrust load cell set to zero
{
  scale.tare(); 
  
}  

 void b25PopCallback(void *ptr)  // Torque load cell set to zero
{
   scale2.tare(); 
  
} 

void b27PopCallback(void *ptr) // Thrust load cell set to zero
{
  scale.tare(); 
  
}  

void b28PopCallback(void *ptr) // Torque load cell set to zero
{
   scale2.tare(); 
  
} 
void b29PopCallback(void *ptr)  // Increase the blade counbt by 1
{
   blades = blades+1;
  
} 
void b30PopCallback(void *ptr) // Decrease the blade count by 1.
{
  if (blades <= 2){
    blades = 2;
  }
  else{
   blades = blades -1;
  
} }

void b31PopCallback(void *ptr) // Increase the propellers diameter by 0.1 inches
{
   propellerdiameter = propellerdiameter+0.1;
  
} 
void b32PopCallback(void *ptr) // Decreases the propellers diameter by 0.1 inches
{
  if (propellerdiameter <= 0.1){
    propellerdiameter = 0.1;
  }
  else{
   propellerdiameter = propellerdiameter-0.1;
  
} }

/////  Code for adjusting the distances for torque calculation///////////
void b33PopCallback(void *ptr)  // Incresese the vertical distance by 0.01 meters
{
   distance_from_Base_to_Shaft = distance_from_Base_to_Shaft+0.01;
} 
void b34PopCallback(void *ptr) // Decreases the vertical distance by 0.01 meters
{
  if (distance_from_Base_to_Shaft <= 0.01){
    distance_from_Base_to_Shaft = 0.01;
  }
  else{
   distance_from_Base_to_Shaft = distance_from_Base_to_Shaft-0.01;
} 
}
void b35PopCallback(void *ptr) // Incresese the Horizontal distance by 0.01 meters
{
   distance_from_Shaft_to_Loadcell = distance_from_Shaft_to_Loadcell+0.01;
} 
void b36PopCallback(void *ptr)//Decreases the horizontal distance by 0.01 meters
{
  if (distance_from_Shaft_to_Loadcell <= 0.01){
    distance_from_Shaft_to_Loadcell = 0.01;
  }
  else{
   distance_from_Shaft_to_Loadcell = distance_from_Shaft_to_Loadcell-0.01;
} 
}
void b37PopCallback(void *ptr) 
{
  testtype = 0; // static test mode selected
  
}  

void b38PopCallback(void *ptr) 
{
   testtype =1; // dynamic test mode selected
  
} 
void b39PopCallback(void *ptr) 
{
  fueltype = 0; // battery selected
  
}  

void b40PopCallback(void *ptr) 
{
   fueltype =1; // alcohol-derived fuel selected
} 





/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  Serial.begin(115200);   // Set the Baudrate for SerialPort0 - Communication with PC
  Serial1.begin(115200);  // Set the Baudrate for SerialPort1 - Communication with SD moduler
  Serial2.begin(115200);  // Set the Baudrate for SerialPort1 - Communication with HMI device
  Serial3.begin(115200);  // Set the Baudrate for SerialPort3 - Communication with Arduino Nano - RPM
  ArduinoNanoFuelFlow.begin(9600); // Set the Baudrate for SerialPort3 - Communication with Arduino Nano - FuelFlow

ads.setGain(GAIN_SIXTEEN); // Define the gain of the current shunt amplifier
  ads.begin(); // begin the ADS1015/ADS1115 sensor
 pinMode(ledPin, OUTPUT); // Pin 13 set as output
 pinMode(10, OUTPUT);// Pin 10 set as output

       if (!SD.begin(chipSelect, 11, 12, 13)) { // This line must be used for Arudino Mega
        //if (!SD.begin(chipSelect)) {      // if you're using an UNO, you can use this line instead
       
        
    }
    char filename[15];
    strcpy(filename, "GPSLOG00.CSV"); //Define the name of the logfile
    for (uint8_t i = 0; i < 100; i++) { //creates the log file
        filename[6] = '0' + i/10;
        filename[7] = '0' + i%10;
        if (! SD.exists(filename)) {
            break;
        }
    }
    logfile = SD.open(filename, FILE_WRITE); //opens the file
    
    logfile.flush();
    GPS.begin(9600);

  scale.set_scale(); // Arduino starts reading the Thrust loadcell
  scale2.set_scale();  // Arduino starts reading the Torque loadcell
  scale.tare();  //set the Thrust load cell to zero before the loop starts
  scale2.tare(); // set the Torque load cell to zero before the loop starts
 

  b12.attachPop(b12PopCallback, &b12);  // Dual state button bt0 press
  page0.attachPush(page0PushCallback);  // Page press event
  page1.attachPush(page1PushCallback);  // Page press event
  page2.attachPush(page2PushCallback);  // Page press event
  page3.attachPush(page3PushCallback);  // Page press event
  page4.attachPush(page4PushCallback);  // Page press event
  page5.attachPush(page5PushCallback);  // Page press event
  page6.attachPush(page6PushCallback);  // Page press event
  page7.attachPush(page7PushCallback);  // Page press event
  page8.attachPush(page8PushCallback);  // Page press event
  page9.attachPush(page9PushCallback);  // Page press event
  page10.attachPush(page10PushCallback);  // Page press event
  page11.attachPush(page11PushCallback);  // Page press event
  b13.attachPush(b13PushCallback, &b13);  // Dual state button bt0 press
  b14.attachPush(b14PushCallback, &b14);  // Dual state button bt0 press
  b15.attachPush(b15PushCallback, &b15);  // Dual state button bt0 press
  b16.attachPush(b16PushCallback, &b16);  // Dual state button bt0 press
  b17.attachPush(b17PushCallback, &b17);  // Dual state button bt0 press
  b18.attachPop(b18PopCallback, &b18);    // Dual state button bt0 press
  b19.attachPush(b19PushCallback, &b19);  // Dual state button bt0 press
  b20.attachPush(b20PushCallback, &b20);  // Dual state button bt0 press
  b21.attachPush(b22PushCallback, &b21);  // Dual state button bt0 press
  b22.attachPush(b22PushCallback, &b22);  // Dual state button bt0 press
  b23.attachPush(b23PushCallback, &b23);  // Dual state button bt0 press
  b24.attachPop(b24PopCallback, &b24);  // Dual state button bt0 press
  b25.attachPop(b25PopCallback, &b25);  // Dual state button bt0 press
  b27.attachPop(b27PopCallback, &b27);  // Dual state button bt0 press
  b28.attachPop(b28PopCallback, &b28);  // Dual state button bt0 press
  b29.attachPop(b29PopCallback, &b29);  // Dual state button bt0 press
  b30.attachPop(b30PopCallback, &b30);  // Dual state button bt0 press
  b31.attachPop(b31PopCallback, &b31);  // Dual state button bt0 press
  b32.attachPop(b32PopCallback, &b32);  // Dual state button bt0 press 
  b33.attachPop(b33PopCallback, &b33);  // Dual state button bt0 press
  b34.attachPop(b34PopCallback, &b34);  // Dual state button bt0 press 
  b35.attachPop(b35PopCallback, &b35);  // Dual state button bt0 press
  b36.attachPop(b36PopCallback, &b36);  // Dual state button bt0 press 
  b37.attachPop(b37PopCallback, &b37);  // Dual state button bt0 press
  b38.attachPop(b38PopCallback, &b38);  // Dual state button bt0 press 
  b39.attachPop(b39PopCallback, &b39);  // Dual state button bt0 press
  b40.attachPop(b40PopCallback, &b40);  // Dual state button bt0 press 
}





void loop() {
  scale.set_scale(calibration_factor); //Adjust to this calibration factor
  scale2.set_scale(calibration_factor2); //Adjust to this calibration factor

  result = ads.readADC_Differential_0_1(); // reads the differential voltage from the current shunt
  impliedMv =result * multi; // Gain 16 corresponds to the resolution of 0.0078125mV per bit
  amps = impliedMv * ampsPerMv; //The 300A current shunt gives 75mV at full load capacity. Thus the impliedmV must be multiplied by 4

    
    
  
   sensorValue2 = analogRead(sensorPin); // reads the analog value from the defined sensor pin from the Airspeed signal
   Vout=(5*sensorValue2)/1024.0;
   float Vout2 = floorf(Vout * 100) / 100;   // round the voltage to 2 decinmal places
   P=Vout2-2.5; // This sensor gives 2.5mV at zero airspeed. 
   Velocity = sqrt (2000*P/1.225);  // Gets velocity from bernoullis principle
   
    

  ///////
  float volt = analogRead(A8);// reads the analog signal from the voltage divider
  float voltage = volt * 0.02332; // gives the actual voltage  from the measured analog signal
  ///////

 
  if ( Serial3.available()) // Check to see if at least one character is available from the ArduinoNano that measures RPM
  {
   String str = Serial3.readStringUntil('\n');
  RPM = (str.toFloat()*60)/blades; // Get the RPM from the measured frequency
  }
  
  if (ArduinoNanoFuelFlow.available()) // Check to see if at least one character is available from the ArduinoNano that measures FuelFlow
  {
  String str2 = ArduinoNanoFuelFlow.readStringUntil('\n');
  flowrate = ((str2.toFloat()-0)/14000)*3600; // This gives the Fuel flow rate in L/hr
  flowrate2 = (flowrate*1000)/60;

  }
  


  if(CurrentPage == 9){
  blades1 = "t0.txt=\""+String(blades)+"\"";
  propellerdiameter1 = "t1.txt=\""+String(propellerdiameter)+"\"";
  Serial2.print(blades1);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print(propellerdiameter1);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  
  }
  if(CurrentPage == 10){
  distance_from_Base_to_Shaft1 = "t0.txt=\""+String(distance_from_Base_to_Shaft)+"\"";
  distance_from_Shaft_to_Loadcell1 = "t1.txt=\""+String(distance_from_Shaft_to_Loadcell)+"\"";
  Serial2.print(distance_from_Base_to_Shaft);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print(distance_from_Shaft_to_Loadcell1);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  
  }
  
  if(CurrentPage == 1){
    if (x ==0){
      output=scale.get_units(), 1;
    
       y = calibration_factor;
      }
  if(x ==1){
      output2=scale2.get_units(), 1;

      y = calibration_factor2;
  }
  command = "force.txt=\""+String(output)+"\"";
  command2 = "calib.txt=\""+String(y)+"\"";

  Serial2.print(command);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print(command2);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
 
}

  if(CurrentPage == 4){ // Battery mode selected
   
  output=scale.get_units(), 2;
 
  output2= scale2.get_units(), 2;        

  command = "t0.txt=\""+String(output)+"\"";
  command2 = "t1.txt=\""+String(output2)+"\"";
  command3= "t5.txt=\""+String(amps)+"\"";
  command4= "t4.txt=\""+String(voltage)+"\"";
  command5= "t2.txt=\""+String(RPM)+"\"";
  command6= "t3.txt=\""+String(Velocity)+"\"";
  command8 = "t6.txt=\""+String(voltage*amps)+"\"";
  Serial2.print(command);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print(command2);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print (command3);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print (command4);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print (command5);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print (command6);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print (command8);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
if(testtype == 0){ // data logging for static testing

  logfile.print(abs(output*9.81)); // Logging Thrust in Newtons
  logfile.print(",");
  logfile.print(abs(output2*9.81*xx));// Logging Torque in Newton
  logfile.print(",");
  logfile.print(RPM); //Rpm
  logfile.print(",");
  logfile.print(RPM/60); // n
  logfile.print(",");
  logfile.print(abs(output2*9.81*xx)*2*(RPM/60)*3.14); // Power
  logfile.print(",");
  logfile.print((abs(output2*9.81*xx)*2*(RPM/60)*3.14)/(1.225*((RPM*RPM*RPM/216000))*(pd2*pd2*pd2*pd2*pd2))); // Cof of P
  logfile.print(",");
  logfile.print((abs(output*9.81))/(1.225*((RPM*RPM/3600))*(pd2*pd2*pd2*pd2))); // Cof of T
  logfile.print(",");
  logfile.print(voltage); //voltage
  logfile.print(",");
  logfile.print(amps);  //current draw
  logfile.print(",");
  logfile.print(amps*voltage);  //Power electric
  logfile.print(",");
  logfile.print((abs(output2*9.81*xx)*2*(RPM/60)*3.14))/(amps*voltage);  // Motor efficiency
  logfile.print(",");
  logfile.print((abs(output))/(abs(output2*9.81*xx)*2*(RPM/60)*3.14));  //Propeller Mech, efficiency
  logfile.print(",");
  logfile.println(((abs(output))/(abs(output2*9.81)*2*(RPM/60)*3.14))*(((abs(output2*9.81)*2*(RPM/60)*3.14))/(amps*voltage)));  //Overall efficiency
  logfile.flush();

 }
 if(testtype == 1){ // data logging for dynamic testing
  float J = (Velocity*60)/(RPM*pd2);
      float CT = ((abs(output*9.81))/(1.225*((RPM*RPM/3600))*(pd2*pd2*pd2*pd2)));
       float CP = ((abs(output2*9.81*xx)*2*(RPM/60)*3.14)/(1.225*(RPM*RPM*RPM/216000)*(pd2*pd2*pd2*pd2*pd2)));
  logfile.print(abs(output*9.81)); // Logging Thrust in Newtons
  logfile.print(",");
  logfile.print(abs(output2*9.81*xx));// Logging Torque in Newton
  logfile.print(",");
  logfile.print(RPM); //rpm 
  logfile.print(",");
  logfile.print(RPM/60);//n
  logfile.print(",");
  logfile.print(abs(output2*9.81*xx)*2*(RPM/60)*3.14); // P
  logfile.print(",");
  logfile.print((abs(output2*9.81*xx)*2*(RPM/60)*3.14)/(1.225*((RPM*RPM*RPM/216000))*(pd2*pd2*pd2*pd2*pd2))); //Cof of P
  logfile.print(",");
  logfile.print((abs(output*9.81))/(1.225*((RPM*RPM/3600))*(pd2*pd2*pd2*pd2))); // Cof of T
  logfile.print(",");
  logfile.print(voltage); // voltage
  logfile.print(",");
  logfile.print(amps);  //Current draw
  logfile.print(",");
  logfile.print(amps*voltage);  // electric power
  logfile.print(",");
  logfile.print((abs(output2*9.81*xx)*2*(RPM/60)*3.14))/(amps*voltage);  //  Motor efficiency
  logfile.print(",");
  logfile.print((abs(output))/(abs(output2*9.81*xx)*2*(RPM/60)*3.14));  // propeller mech efficiency
  logfile.print(",");
  logfile.print(Velocity);  // Velocity
  logfile.print(",");
  logfile.print((Velocity*60)/(RPM*pd2));  //J
  logfile.print(",");
  logfile.println(J*CT/CP); // Efficiency
  logfile.flush();
}
 
 if(CurrentPage == 5){ //Gasoline selected
  output=scale.get_units(), 2;

  output2=scale2.get_units(), 2;    
   
  command = "t0.txt=\""+String(output)+"\"";
  command2 = "t1.txt=\""+String(output2)+"\"";
  command5 = "t2.txt=\""+String(RPM)+"\"";
  command6= "t3.txt=\""+String(Velocity)+"\"";
  command7= "t4.txt=\""+String(flowrate)+"\"";
  command9= "t5.txt=\""+String(flowrate2)+"\"";
  Serial2.print(command);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print(command2);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print (command5);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print (command6);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print (command7);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print (command9);  // This is the value you want to send to that object and atribute mentioned before.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
   if(testtype == 0){ // data logging for static testing
   
  logfile.print(abs(output)); //Thrust in Kgf
  logfile.print(",");
  logfile.print(abs(output2));// Torque in Kgf
  logfile.print(",");
  logfile.print(abs(output*9.81));// Thrust in Newtons
  logfile.print(",");
  logfile.print(abs(output2*9.81*xx));// Torque in Newtona
  logfile.print(",");
  logfile.print(RPM); //Rpm
  logfile.print(",");
  logfile.print(RPM/60); //n
  logfile.print(",");
  logfile.print(abs(output2*9.81*xx)*2*(RPM/60)*3.14); //Power
  logfile.print(",");
  logfile.print((abs(output2*9.81*xx)*2*(RPM/60)*3.14)/(1.225*((RPM*RPM*RPM/216000))*(pd2*pd2*pd2*pd2*pd2))); //Coef of Power
  logfile.print(",");
  logfile.print((abs(output*9.81))/(1.225*((RPM*RPM/3600))*(pd2*pd2*pd2*pd2))); //coefficent of Thrust
  logfile.print(",");
  logfile.print((abs(output))/(abs(output2*9.81*xx)*2*(RPM/60)*3.14));  // Propeller mech efficiency
  logfile.print(",");
  logfile.println(flowrate2);  // Fuel flow rate in mL/min
  logfile.flush();
  
 }}
  if(testtype == 1){// data logging for dynamic testing
     float J = (Velocity*60)/(RPM*pd2);
      float CT = ((abs(output*9.81))/(1.225*((RPM*RPM/3600))*(pd2*pd2*pd2*pd2)));
       float CP = ((abs(output2*9.81*xx)*2*(RPM/60)*3.14)/(1.225*((RPM*RPM*RPM/216000))*(pd2*pd2*pd2*pd2*pd2)));
  logfile.print(abs(output)); // Thrust in Kgf
  logfile.print(",");
  logfile.print(abs(output2)); // Torque in Kgf
  logfile.print(",");
  logfile.print(abs(output*9.81)); // Thrust in N
  logfile.print(",");
  logfile.print(abs(output2*9.81*xx)); //Torque in N-m
  logfile.print(",");
  logfile.print(RPM);//RPM
  logfile.print(",");
  logfile.print(RPM/60); // n
  logfile.print(",");
  logfile.print(abs(output2*9.81*xx)*2*(RPM/60)*3.14); //Power
  logfile.print(",");
  logfile.print((abs(output2*9.81*xx)*2*(RPM/60)*3.14)/(1.225*((RPM*RPM*RPM/216000))*(pd2*pd2*pd2*pd2*pd2))); //Coef of P
  logfile.print(",");
  logfile.print((abs(output*9.81))/(1.225*((RPM*RPM/3600))*(pd2*pd2*pd2*pd2)));//Coef of Thrust
  logfile.print(",");
  logfile.print(((abs(output))/(abs(output2*9.81*xx)*2*(RPM/60)*3.14)));  // Prop Mech Efficiency
  logfile.print(",");
  logfile.print(Velocity);  // velocity
  logfile.print(",");
  logfile.print((Velocity*60)/(RPM*pd2));   //J
  logfile.print(",");
  logfile.print(J*CT/CP);
  logfile.print(",");
  logfile.println(flowrate2); // FLow rate in mL/min
  logfile.flush();
  
 }}
 if(CurrentPage == 6){ // Guages selected
  // below code converts the important performance parameters to degress to represent them in guages
  x1 =scale.get_units();
  x2 = abs(6*x1);
  x5 = 180 + x2;
  if (x5 > 360){
    x5 = x5 -360; 
  }
  x3 = scale2.get_units();
  x4 = abs(6*x3);
  x6 = 180 + x4;
  if (x6 > 360){
    x6 = x6 -360; 
  }
  x7 = amps + 180;
  if (x7 > 360){
    x7 = x7 -360; 
  }
  x8 = voltage*10;
  x8 = x8+180;
  if (x8 > 360){
    x8 = x8 -360; 
  }
  x9 = RPM*7.5;
  x9 = x9+180;
  if (x9 > 360){
    x9 = x9 -360; 
  }
  x10 = Velocity*6;
  x10 = x10+180;
   if (x10 > 360){
    x10 = x10 -360; 
  }
  Serial2.print("z0.val=");
  Serial2.print(x5);
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("z1.val=");
  Serial2.print(x6);
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("z5.val=");
  Serial2.print(x7);
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("z4.val=");
  Serial2.print(x8);
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("z2.val=");
  Serial2.print(x9);
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("z3.val=");
  Serial2.print(x10);
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
 }
 
  if(CurrentPage == 7){
  x1 =scale.get_units();
  x2 = abs(6*x1);
  x5 = 180 + x2;
  if (x5 > 360){
    x5 = x5 -360; 
  }
  x3 = scale2.get_units();
  x4 = abs(6*x3);
  x6 = 180 + x4;
  if (x6 > 360){
    x6 = x6 -360; 
  }
  x9 = RPM*7.5;
  x9 = x9+180;
  if (x9 > 360){
    x9 = x9 -360; 
  }
  x10 = Velocity*6;
  x10 = x10+180;
   if (x10 > 360){
    x10 = x10 -360; 
  }
  x11 = flowrate * 60;
  x11 = x11+180;
  if (x11 > 360){
    x11 = x11 -360; 
  }
  Serial2.print("z0.val=");
  Serial2.print(x5);
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("z3.val=");
  Serial2.print(x6);
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("z4.val=");
  Serial2.print(x9);
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("z2.val=");
  Serial2.print(x10);
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("z1.val=");
  Serial2.print(x11);
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
 }
  nexLoop(nex_listen_list);  // Check for any touch event 
}








                                
