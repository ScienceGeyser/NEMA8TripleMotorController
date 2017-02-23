/*
  The user interface requires the following for communication:
  There are three command letters
    A - Sweep operational parameters will follow
    B - Begin Sweep Operation
    C - Turn on battery Charger
    
  Expected parameters for A Comma Delimitted
    start frequency - range 1000 to 100000
    frequency increment - range 1 to 100000
    number of increments - range 0 to 511
    impedance range - range 1-5
    runWithAutoRange - boolean
    target - range 1-4
    continuous - boolean
    timer - range 0-32768
  
  Program actions
  When program starts
    set Arduinos IO ports to control the power board
    Turn on the battery charger
    Wait for a command from the serial port
    
  Program Actions Required for Command A
  wait for serial buffer and write parameters to variables as they come in. Each time a comma is encountered move to the next variable
  When the last variable is set return to waiting for commands
  
  Program Actions Required for Command B
  Turn off battery charger
  Turn on TEER power
  Write TEER Calibration variables throught I2C
  Initialize AD5933 and allow to oscillator to settle (Control Mode 1)
  Apply frequency to load (Control Mode 2)
  Check Status Register
    if DFT complete Read values from Data Registers and store (write to serial port)
  Check for sweep completion
    if No - step the frequency (Control Mode 3) and run again from Start Frequency
    if Yes - put AD5933 in Stand-by (Control Mode 11) and continue
  Write TEER Operational variables through I2C
  Initialize AD5933 and allow to oscillator to settle (Control Mode 1)
  Apply frequency to load (Control Mode 2)
  Check Status Register
    if DFT complete Read values from Data Registers and store (write to serial port)
  Check for sweep completion
    if No - step the frequency (Control Mode 3) and run again from Start Frequency
    if Yes - put AD5933 in Power Down (Control Mode 10) and continue program
  Send operation complete signal to serial port and return to waiting for commands
    
  Program Actions Required for Command C
    Turn off voltage regulator and turn on Battery Charger
    return to waiting for commands
  
  Add programming command P to allow other AD5933 operational parameters to be changed
    Read serial port for parameters
    return to waiting for commands
    
  Add command to request stored data from SD card
  
*/
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

//SD setup variables
Sd2Card card;
SdVolume volume;
SdFile root;
// CS is on pin D8
// Pin D10 should be made an Output for library compatibility
// MOSI = D11
// MISO = D12
//  CLK = D13

// Defines for Hardware
byte OSCaddr = 0x17;
#define controlReg 0x80;
#define freqReg 0x82;
#define freqIncReg 0x85;
#define numIncReg 0x88;
#define settleCyclesReg 0x8A;
#define statusReg 0x8F;
#define tempDataReg 0x92;
#define realDataReg 0x94;
#define ImgDataReg 0x96;
#define initStartCom 0x10;
#define sweepCom 0x20;
#define incFreqCom 0x30;
#define freqRepeatCom 0x40;
#define tempMeasureCom 0x90;
#define powerDnCom 0xA0;
#define stbyCom 0xB0;
#define outRange1 0x00 // Output Range 2V P-P
#define outRange2 0x03 // Output Range 1V P-P
#define outRange3 0x02 // Output Range 400mV P-P
#define outRange4 0x01 // Output Range 200mV P-P

byte charge = 5; //Pin number for the charger control line
byte power = 7; //Pin number for the regulator control line
byte regulatorERR = 6; //connected to an open collector output with 330K pullup (LOW = Regulator output error)
byte chargeStatus = 4; //connected to LED & Current Limit resistor on charger status (= 5V or 5V-LEDVf) most likely useless
                       //Try tying this to an analog input if monitoring is needed
int channel = 0;
int error = 0;
byte portA1Val = 0;
byte portA2Val = 0;
byte portB1Val = 0;
byte portB2Val = 0;
int checkVal = 0;
int startFreq = 7000;
unsigned long startFreqVal = 0;
int freqInc = 250;
unsigned long freqIncVal = 0;
unsigned int numInc = 250;
int range = 4;
int autoRangeMode = 0;
int channelNum = 1;
int repeatMode = 0;
int interval = 0;
byte calibrate = 0;
byte statusVal = 0x00;
unsigned int settleCycles = 200;
unsigned int sweepRepeat = 0;
boolean tempValid = 0;
boolean dataValid = 0;
byte sweepDone = 0;
byte inByte = 0;
byte inByte1 = 0;
byte inByte2 = 0;
byte outByte = 0;
int num = 0;
int16_t real = 0;
int16_t imag = 0;
byte echo = 0;

void setup() {
  pinMode(8, OUTPUT);
  pinMode(10, OUTPUT);
  Wire1.begin();
  delay(10);
  Serial.begin(115200);
  Serial.flush();
  Serial.println("TEER Interface V6.0"); /*For debug RSR*/
  setTEERInterface();
  checkSD();
  setChargeoff();
  delay(10);
  valueBattery();
  // Check battery voltage
  // checkBattery(1900); // Sent number equals millivolts
  // Check regulator status
  setPoweron();
  delay(1);
  while (regulatorStatus() == 0)
  {
    // Print Error every 30 seconds
    delay(30000);
  }
  // Set up OSC
  Serial.println("Ready");
  setPoweroff();
  setChargeon(); 

}

void loop() {
  // Setup SD Card
  
  // Wait for commands
  Serial.println("Enter a Command Code Letter");
  while(Serial.available() <= 0)
  {
    delay(500); // sit around doing nothing
  }
  while(Serial.available() > 0)
  {
    byte command = Serial.read();
    
    //Case structure to decide which command to execute
    switch (command) 
    {
      case 'A':
      // Wait for Operational Parameters
      getValues();
      break;
      case 'B':
      // Start Operation
      runScan();
      break;
      case 'C':
      // Start Batery Charger
      setChargeon();
      delay(10);
      break;
      case 'c':
      // Stop Batery Charger
      setChargeoff();
      delay(10);
      break;
      case 'D':
      // Power Down
      setPoweroff();
      delay(10);
      break;
      case 'U':
      // Power Up
      setPoweron();
      delay(10);
      break;
      case 'O':
      // Power Up
      OSCon();
      delay(10);
      break;
      case 'P':
      // Progam Other Parameters
      programAD5933();
      delay(10);
      default: 
      // statements
      delay(10);
      break;
      }
  }
}

void setTEERInterface()
{
  pinMode(charge, OUTPUT);
  pinMode(power, OUTPUT);
  pinMode(regulatorERR, INPUT);
  pinMode(chargeStatus, INPUT);
  digitalWrite(charge, LOW); //Low logic level for the battery charger to be on
  digitalWrite(power, LOW); //LOW logic level for the TEER power to be off
}

void getValues()
{
  Serial.println("Begin getValues()");
  // Wait for Operational Parameters
      Serial.println("Enter Parameters");
      while(Serial.available() <= 0)
      {
        delay(5); // sit around doing nothing
      }
      while(Serial.available() > 0)
      {
        startFreq = Serial.parseInt();
        freqInc = Serial.parseInt();
        numInc = Serial.parseInt();
        range = Serial.parseInt();
        autoRangeMode = Serial.parseInt();
        channelNum = Serial.parseInt();
        repeatMode = Serial.parseInt();
        interval = Serial.parseInt();
        /*if (Serial.read() == '\n')
        {*/
          Serial.println("The following operational values have been entered:");
          Serial.print("Start Frequency of ");
          Serial.print(startFreq);
          Serial.println("Hz");
          startFreqVal = ((float)startFreq/4001728)*134217728;
          Serial.print("Hex Value is ");
          Serial.println(startFreqVal, HEX);
          Serial.print("Frequency Increment of ");
          Serial.print(freqInc);
          Serial.println("Hz");
          freqIncVal = ((float)freqInc/4000000)*134217728;
          Serial.print("Hex Value is ");
          Serial.println(freqIncVal, HEX);
          Serial.print("Increment Number of ");
          Serial.print(numInc);
          Serial.println(" steps");
          Serial.println(numInc, HEX);
          Serial.print("Settle Cycles: ");
          Serial.println(settleCycles);
          Serial.println(settleCycles, HEX);
          Serial.print("Impedance Range ");
          Serial.print(range);
          Serial.println(" has been chosen");
          Serial.print("Autoranging mode is ");
          Serial.print(autoRangeMode);
          Serial.println(" ");
          Serial.print("Measurement will be made on Channel ");
          Serial.print(channelNum);
          Serial.println(" ");
          //channel = channel << range-1;
          break;
        /*}*/
        Serial.println("ERROR");
      }
      
      
}
void configIO(byte rangeVal, byte channelVal, byte calibrateVal)
{
   //Serial.println("Begin configIO()");
   
   // Set all expander ports as outputs  
   writePCA5555(0x20, 0x06, 0x00, 0x00);
   writePCA5555(0x22, 0x06, 0x00, 0x00);
   
   // Set up each byte based on current config
   // Reset Peak detector should always be low for a running config
   // Bit 7 of port A1 should be low when running calibration and high when making a measurement
   // port B2 selects channel
   if (calibrateVal == 0) // Experimental Run
   {
     portA1Val = (0x01<<(rangeVal-1))+128; //Current Resistors
     portB1Val = 0x00<<(rangeVal-1);   //Calibration Resistors
     portA2Val = 0x01<<(rangeVal-1);   //Safety Resistors
     portB2Val = 0x01<<(channelVal-1); //Channel Select
   }
   else if (calibrateVal ==1) //Calibration Run
   {
     portA1Val = 0x01<<(rangeVal-1);
     portB1Val = 0x01<<(rangeVal-1);
     portA2Val = 0x01<<(rangeVal-1);
     portB2Val = 0x00<<(channelVal-1);
     
   }
   else if (calibrateVal == 2) // Autoranging Run
   {
     portA1Val = (0x01<<(rangeVal-1));
     portB1Val = 0x00<<(rangeVal-1);
     portA2Val = 0x01<<(rangeVal-1);
     portB2Val = 0x01<<(channelVal-1);
     
   }
   else
   {
     Serial.println("Setup Error");
   }

   // Send portVals to PCA5555
   writePCA5555(0x20, 0x02, portA1Val, portB1Val);
   writePCA5555(0x22, 0x02, portA2Val, portB2Val);

   //Serial.println("configIO Done!");

}

void runScan()
{
  //Serial.println("Begin runScan()");
  // Start Operation
      // Turn off Charger
      setChargeoff();
      //Serial.println("Charger Off!");
      // Power up board
      setPoweron();
      //Serial.println("Power On!");
      // Setup Port Expanders for Calibration Run
      calibrate = 1;
      configIO(range, channelNum, calibrate);
      OSCon();
      Serial.println("Cal Data Start");
      setupAD5933();
      // Run Calibration Scan
      //delay(5);
      initAD5933();
      delay(3);
      startAD5933();
      // while checkAD5933() < 4 Just Keep Checking in Sweep Loop
      // if checkAD5933() = 2 Read the Real and Imag registers and store the contents to SD Card or print to serial port. Continue in Sweep Loop
      // When checkAD5933 >= 4 Read the Real and Imag registers and break out of Sweep Loop
      sweepAD5933();
      Serial.println("Cal Data Stop");
      setupAD5933();
      Serial.print("Delay 10 Seconds #1");
      //delay(10000);
      Serial.print("Delay 10 Seconds #2");
      //delay(10000);
      //Reconfigure Port Expanders for Live Measurement
      calibrate = 0;
      configIO(range, channelNum, calibrate);
      Serial.println("Delay 10 Seconds Last One");
      //delay(10000);
      Serial.println("Exp Data Start");
      // Run Sweep
      setupAD5933();
      // Run Calibration Scan
      //delay(5);
      initAD5933();
      delay(3);
      startAD5933();
      sweepAD5933();
      Serial.println("Exp Data Stop");
      // Return to Wait for Command
      delay(10);
      setupAD5933();
      setChargeon();
}

void setChargeon() 
{
  if (echo) Serial.println("setChargeon()");
  // Turn on Battery Charger
  digitalWrite(charge, LOW); //Low logic level for the battery charger to be on

}
void setChargeoff() 
{
  if (echo) Serial.println("setChargeoff()");
  // Turn off Battery Charger
  digitalWrite(charge, HIGH); //HIGH logic level for the battery charger to be off

}

void setPoweroff()
{
  if (echo) Serial.println("setPoweroff()");
  // Turn off Regulator
  digitalWrite(power, LOW); //LOW logic level for the TEER power to be off
}

void setPoweron()
{
   if (echo) Serial.println("setPoweron()");
 // Turn on Regulator
  digitalWrite(power, HIGH); //HIGH logic level for the TEER power to be on
}

void programAD5933()
{
  byte index = 0;
  char inData[10]; // Longest Value is 10 bytes
  
  Serial.println("You have entered I2C Programming Mode!");
  Serial.println("Enter the 7-bit device address that you wish to program");
  while(Serial.available() <= 0)
  {
    delay(5); // sit around doing nothing
  }
  while(Serial.available() > 0)
  {
    char inByte = Serial.read();
    if(inByte == '\n')
    {
      // End of value
      break;
      
    }
    else
    {
      inData[index]=inByte;
      index++;
      inData[index]='\0';
    }
  }
  // Ask for address?
  // Ask for register?
  // Ask for Command?
  // Ask for values?
      Wire1.beginTransmission(0x0D); // Address for AD5933
      Wire1.write(0xB0); // Set Register Pointer
      Wire1.write(0x82); // Register to write to
      Wire1.write(0xA1); // Set Register Pointer
      Wire1.write(0xA0); // Register to write to
      Wire1.requestFrom(0x0D, 10, 1);
      while(Wire1.available())
      {
        char c = Wire1.read();
        Serial.print(c, HEX);
      }
      Serial.println(" Old Register Contents");
  
}

byte readI2Cbyte (byte addr, byte reg, byte echo)
{
  Wire1.beginTransmission(addr); // Address for AD5933
  Wire1.write(0xB0); // Set Register Pointer
  Wire1.write(reg); // Register Address
  Wire1.endTransmission();
  Wire1.requestFrom(0x0D, 1, 1);
      while(Wire1.available())
      {
        char c = Wire1.read();
        if (echo)
        {
          Serial.print(c, HEX);
        }
        return c;
      }

}

void printI2C (void)
{
      readI2Cbyte(0x0D, 0x80, 1);
      readI2Cbyte(0x0D, 0x81, 1);
      Serial.println(" ");
      readI2Cbyte(0x0D, 0x82, 1);
      readI2Cbyte(0x0D, 0x83, 1);
      readI2Cbyte(0x0D, 0x84, 1);
      Serial.println(" ");
      readI2Cbyte(0x0D, 0x85, 1);
      readI2Cbyte(0x0D, 0x86, 1);
      readI2Cbyte(0x0D, 0x87, 1);
      Serial.println(" ");
      readI2Cbyte(0x0D, 0x88, 1);
      readI2Cbyte(0x0D, 0x89, 1);
      Serial.println(" ");
      readI2Cbyte(0x0D, 0x8A, 1);
      readI2Cbyte(0x0D, 0x8B, 1);
      Serial.println(" ");
      readI2Cbyte(0x0D, 0x8F, 1);
      Serial.println(" ");

}

byte checkAD5933 (void)
{
  //Serial.println("checkAD5933()");
  // Check the Status Register and take appropriate action
  
  // Set Pointer to Status Register
      Wire1.beginTransmission(0x0D); // Address for AD5933
      Wire1.write(0xB0); // Set Register Pointer
      Wire1.write(0x8F); // Register to write to
      Wire1.endTransmission();
  
  // Read Status Register
      Wire1.requestFrom(0x0D, 1, 1); // Address for AD5933
      statusVal = (Wire1.read() & 0x06); // Register Data - Initialize with Start Frequency
  //Serial.print("Status Value = ");
  //Serial.println(statusVal, HEX);
  return statusVal;

}

void setupAD5933 (void)
{
  if (echo) Serial.println("setupAD5933()");

      Wire1.beginTransmission(0x0D); // Address for AD5933
      Wire1.write(0x80); //Register Address
      Wire1.write(0xB0); // Register Data - Standby Mode
      Wire1.endTransmission();
      readI2Cbyte(0x0D, 0x80, 0);
      //Serial.println(" ");
      //Serial.println("Stby!");
      Wire1.beginTransmission(0x0D); // Address for AD5933
      Wire1.write(0x81); // Register to write to
      Wire1.write(0x10); // Register Data - Reset Frequency Sweep
      Wire1.endTransmission();
      Wire1.beginTransmission(0x0D); // Address for AD5933
      Wire1.write(0x81); // Register to write to
      Wire1.write(0x08); // Register Data - Set Ext OSC on
      Wire1.endTransmission();
      //Serial.println("Reset!");
      //delay(1);
      //printI2C();
      //Serial.println(" ");
      Wire1.beginTransmission(0x0D); // Address for AD5933
      Wire1.write(0xB0); // Set Register Pointer
      Wire1.write(0x82); // Register to Address
      Wire1.endTransmission();
      //delay(1);
      Wire1.beginTransmission(0x0D); // Address for AD5933
      Wire1.write(0xA0); // Block Write
      Wire1.write(0x0A); // Number of Bytes in Block
      Wire1.write((startFreqVal>>16) & 0xFF);
      Wire1.write((startFreqVal>>8) & 0xFF);
      Wire1.write((startFreqVal) & 0xFF);
      Wire1.write(freqIncVal>>16 & 0xFF);
      Wire1.write((freqIncVal>>8) & 0xFF);
      Wire1.write((freqIncVal) & 0xFF);
      Wire1.write((numInc>>8) & 0xFF);
      Wire1.write((numInc) & 0xFF);
      Wire1.write((settleCycles>>8) & 0xFF); // Settle Cycles
      Wire1.write((settleCycles) & 0xFF); // Settle Cycles
      Wire1.endTransmission();
      //delay(1);
      //printI2C();
      //Serial.println(" ");
      Wire1.beginTransmission(0x0D); // Address for AD5933
      Wire1.write(0x80); // Register
      Wire1.write(0xB0); // Register Data - Standby Mode
      Wire1.endTransmission();
      if (echo) Serial.println("Setup Done!");

}

void initAD5933 (void)
{
  Wire1.beginTransmission(0x0D); // Address for AD5933
  Wire1.write(0x80); // Register
  Wire1.write(0x10); // Register Data - Initialize with Start Frequency
  Wire1.endTransmission();
  //Serial.println("Init!");
}

void startAD5933 (void)
{
  Wire1.beginTransmission(0x0D); // Address for AD5933
  Wire1.write(0x80); // Register to write to
  Wire1.write(0x20); // Register Data - Initialize Sweep
  Wire1.endTransmission();
  //Serial.println("Start!");
}

void incrementAD5933 (void)
{
  Wire1.beginTransmission(0x0D); // Address for AD5933
  Wire1.write(0x80); // Register to write to
  Wire1.write(0x30); // Register Data - Increment Sweep
  Wire1.endTransmission();
  //Serial.println("Increment!");
}

void repeatAD5933 (void)
{
  Wire1.beginTransmission(0x0D); // Address for AD5933
  Wire1.write(0x80); // Register to write to
  Wire1.write(0x40); // Register Data - Repeat Sweep Frequency
  Wire1.endTransmission();
  //Serial.println("Repeat!");
}

void tempAD5933 (void)
{
  Wire1.beginTransmission(0x0D); // Address for AD5933
  Wire1.write(0x80); // Register to write to
  Wire1.write(0x90); // Register Data - Measure Temperature
  Wire1.endTransmission();
  //Serial.println("Temp!");
}

int readAD5933temp (void)
{
  if (echo) Serial.println("tempValid");
  // Read Temp Register
  int tempByte = readI2Cbyte(0x0D, 0x92, 0);
  int tempRaw = (tempByte<<8);
  tempByte = readI2Cbyte(0x0D, 0x93, 0);
  tempRaw = tempRaw + tempByte;
  return tempRaw;
    
}

int readAD5933Real (void)
{
  //Serial.print("Real Hex ");
  real = readI2Cbyte(0x0D, 0x94, 0);
  real = real<<8;
  real = real + readI2Cbyte(0x0D, 0x95, 0);
}

int readAD5933Imag (void)
{
  //Serial.print("Real Hex ");
  imag = readI2Cbyte(0x0D, 0x96, 0);
  imag = imag<<8;
  imag = imag + readI2Cbyte(0x0D, 0x97, 0);
}

byte getSerialByte (void)
{
        Serial.println("Enter Parameters");
      while(Serial.available() <= 0)
      {
        delay(5); // sit around doing nothing
      }
      while(Serial.available() > 0)
      {
        inByte = Serial.read();
        return inByte;
      }

}

byte getSerialHEX (void)
{
  // get a hexidecimal value from the serial port and return a single byte value
  // for example: 
  // Two characters are typed in the serial interface - 1C and the byte 00011100 is returned
  while(Serial.available() <= 0)
  {
    // Just wait around
    delay(5);
  }
  while(Serial.available() > 0)
  {
    // Read in the characters and use the laast two as hex values
    num = 0;
    inByte = Serial.read();
    if(inByte>=48 && inByte<=57) num = inByte-48;
    else if(inByte>=65 && inByte<=70) num = inByte-55;
    else if(inByte>=97 && inByte<=102) num = inByte-87;
    else num = 0;
    outByte = num<<4;
    num = 0;
    byte inByte = Serial.read();
    if(inByte>=48 && inByte<=57) num = inByte-48;
    else if(inByte>=65 && inByte<=70) num = inByte-55;
    else if(inByte>=97 && inByte<=102) num = inByte-87;
    else num = 256;
    outByte = outByte + num;
    return outByte;
  }
}

void writePCA5555(byte addressI2C, byte commandI2C, byte inByte1, byte inByte2)
{
   if (echo) Serial.print("Write to PCA5555 Address ");
   if (echo) Serial.print(addressI2C, HEX);
   if (echo) Serial.print(" ");
   if (echo) Serial.print(commandI2C, HEX);
   if (echo) Serial.print(" ");
   if (echo) printByteBits(inByte1);
   if (echo) Serial.print(" ");
   if (echo) printByteBits(inByte2);
   if (echo) Serial.println(" ");
   Wire1.beginTransmission(addressI2C);
   Wire1.write(commandI2C);
   Wire1.write(inByte1);
   Wire1.write(inByte2);
   Wire1.endTransmission();
}

void printByteBits(byte myByte){
 for(byte mask = 0x80; mask; mask >>= 1){
   if(mask  & myByte)
       Serial.print('1');
   else
       Serial.print('0');
 }
}

void printIntBits(int myInt){
 for(int mask = 0x8000; mask; mask >>= 1){
   if(mask  & myInt)
       Serial.print('1');
   else
       Serial.print('0');
 }
}


void sweepAD5933 (void)
{
      sweepDone = 0;
      while(sweepDone == 0)
      {
        delay(1);
        statusVal = checkAD5933(); // check the status byte on the AD5933 every pass
        if (statusVal == 0)
        {
          // Nothing needs to be done, just stay in the loop
          sweepDone = 0;
        }
        else if (statusVal == 2)
        {
          // Read the Real and Imag Registers, Increment Sweep and stay in the loop
          readAD5933Real();
          readAD5933Imag();
          incrementAD5933();
          Serial.print(real);
          Serial.print(", ");
          Serial.println(imag);
          sweepDone = 0;
        }
        else if (statusVal == 4)
        {
          // Break out of the loop
          sweepDone = 1;
        }
         // Read the Real and Imag Registers and break out of the loop
         else if (statusVal == 6)
        {
          readAD5933Real();
          readAD5933Imag();
          Serial.print(real);
          Serial.print(", ");
          Serial.println(imag);
          sweepDone = 1;
        }
       
      }
}

boolean regulatorStatus (void)
{
  // Check regulator status line on pin 6 (regulatorERR)
  if (digitalRead(regulatorERR))
  {
    //Serial.println("Regulator OK");
    return 1;
  }
  else
  {
    Serial.println("Regulator Error");
    return 0;
    
  }
}

void OSCon (void)
{
  Wire1.beginTransmission(OSCaddr);
  Wire1.write(0xDE);
  Wire1.write(0xBC);
  Wire1.endTransmission();
  delay(1000);

}

boolean checkBattery (int compareVal)  // Usage: while(checkBat == 0){wait in loop}
{                                  //    or: if (checkBat == 0) {report error and break from loop}
  // Test voltage on Analog Pin 0
  // Voltage is 1/2 Vbb and should be at least 1.9V before a sweep is made
  int readBat = analogRead(0);
  int scaledBat = map(readBat, 0, 1023, 0, 6600);
  if (compareVal <= scaledBat)
  {
    return 1;
  }
  else
  {
    // False
    return 0;
  }
}

int valueBattery (void)
{
  // Print the value of the Vattery Voltage
  int valBat = analogRead(0);
  valBat = map(valBat, 0, 1023, 0, 6600);
  Serial.print("Battery Voltage is ");
  Serial.print(valBat);
  Serial.println("mV");
}

void checkSD (void)
{
  // See if the SD card is present and working
  int chipSelect = 8;
  if (card.init(SPI_HALF_SPEED, chipSelect)==0)
  {
    Serial.println("No SD Card Detected");
    return;
  }
  else Serial.println("SD Card Ready");
}
