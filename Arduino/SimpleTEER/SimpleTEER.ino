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

// Defines for Hardware
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
int portAVal = 0;
int portBVal = 0;
int checkVal = 0;
int startFreq = 7000;
unsigned long startFreqVal = 0;
int freqInc = 250;
unsigned long freqIncVal = 0;
int numInc = 250;
int range = 4;
int autoRangeMode = 0;
int channelNum = 1;
int repeatMode = 0;
int interval = 0;
byte calibrate = 0;
int statusVal = 0x08;
                       
void setup() {
  Wire1.begin();
  delay(10);
  Serial.begin(115200);
  Serial.flush();
  Serial.print("Set Up Arduino to TEER Interface"); /*For debug RSR*/
  setTEERInterface();
  Serial.println(" - Done!");

}

void loop() {
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
      case 'P':
      // Progam Other Parameters
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
          startFreqVal = ((float)startFreq/4000000)*134217728;
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
void configIO(int rangeVal, int channelVal, boolean calibrateVal)
{
  	switch(rangeVal)
	{
		case 1:
			portAVal = 0x0101 + (32768*calibrateVal); 
 			portBVal = 0x0001 + (256 << channelVal); 
                break;
		case 2:
			portAVal = 0x0202 + (32768*calibrateVal); 
                        portBVal = 0x0002 + (256 << channelVal); 
		break;             
		case 3:
			portAVal = 0x0404 + (32768*calibrateVal);
                        portBVal = 0x0004 + (256 << channelVal);
		break;
		case 4:
			portAVal = 0x0808 + (32768*calibrateVal);
                        portBVal = 0x0008 + (256 << channelVal);
		break;
		case 5:
			portAVal = 0x1010 + (32768*calibrateVal);
                        portBVal = 0x0010 + (256 << channelVal);
		break;
		default:
			error = error +1;
		break;		
	}  
      Wire1.beginTransmission(0x20);
      Wire1.write(0x02); //Send to Output Register 1 
      Wire1.write(portAVal);
      //Wire1.write(0x02);
      //checkVal = Wire1.read();
      Wire1.endTransmission();
      Serial.println("Expander 1 Done!");
      Wire1.beginTransmission(0x22);
      Wire1.write(0x02); //Send to Output Register 1 
      Wire1.write(portBVal);
      //Wire1.write(0x02);
      //checkVal = Wire1.read();
      Wire1.endTransmission();
      Serial.println("Expander 2 Done!");

}

void runScan()
{
  // Start Operation
      // Turn off Charger
      setChargeoff();
      //Serial.println("Charger Off!");
      // Power up board
      setPoweron();
      //Serial.println("Power On!");
      // Setup Port Expanders
      calibrate = 1;
      configIO(range, channelNum, calibrate);
      Serial.println("Run Cal!");
      //delay(10);
      // Setup Oscillator
        // Not Used Right Now
      // Clear Peak Detector
        // Not Used Right Now
      // Run Calibration Scan
      Wire1.beginTransmission(0x0D); // Address for AD5933
      Wire1.write(0xB0); // Set Register Pointer
      Wire1.write(0x80); // Register to write to
      Wire1.write(0xB1); // Register Data - Standby Mode
      Wire1.endTransmission();
      Serial.println("Stby!");
      Wire1.beginTransmission(0x0D); // Address for AD5933
      Wire1.write(0xB0); // Set Register Pointer
      Wire1.write(0x81); // Register to write to
      Wire1.write(0x10); // Register Data - Reset Frequency Sweep
      Wire1.endTransmission();
      delay(1);
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
      Serial.println("Reset!");
      Wire1.beginTransmission(0x0D); // Address for AD5933
      Wire1.write(0xB0); // Set Register Pointer
      Wire1.write(0x82); // Register to write to
      Wire1.endTransmission();
      delay(1);
      Wire1.beginTransmission(0x0D); // Address for AD5933
      Wire1.write(0xA0); // Block Write
      Wire1.write(0x0A); // Number of Bytes in Block
      Wire1.write(startFreqVal & 0xFF);
      Wire1.write((startFreqVal>>8) & 0xFF);
      Wire1.write((startFreqVal>>16) & 0xFF);
      Wire1.write(freqIncVal & 0xFF);
      Wire1.write((freqIncVal>>8) & 0xFF);
      Wire1.write((freqIncVal>>16) & 0xFF);
      Wire1.write(numInc);
      Wire1.write(0x64); // Settle Cycles
      Wire1.endTransmission();
      delay(1);
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
      Serial.println(" New Register Contents");
      //Serial.println("Param!");
      Wire1.beginTransmission(0x0D); // Address for AD5933
      Wire1.write(0xB0); // Set Register Pointer
      Wire1.write(0x80); // Register to write to
      Wire1.write(0x11); // Register Data - Initialize with Start Frequency
      Wire1.endTransmission();
      delay(1);
      Serial.println("Init!");
      Wire1.beginTransmission(0x0D); // Address for AD5933
      Wire1.write(0xB0); // Set Register Pointer
      Wire1.write(0x80); // Register to write to
      Wire1.write(0x21); // Register Data - Initialize Sweep
      Wire1.endTransmission();
      delay(10);
      
      //Loop Check and Step
      Serial.println(statusVal, BIN);
      Wire1.beginTransmission(0x0D); // Address for AD5933
      Wire1.write(0xB0); // Set Register Pointer
      Wire1.write(0x8F); // Register to write to
      Wire1.endTransmission();
      Wire1.requestFrom(0x0D, 1); // Address for AD5933
      statusVal = Wire1.read(); // Register Data - Initialize with Start Frequency
      //Wire1.endTransmission();
      Serial.println(statusVal, BIN);
      
      // if statusVal = 2 then read 4 bytes from 0x94
      //
      // if status < 4 then continue loop

      delay(10);
      Serial.println("Calibration Done!");
      delay(10);
      calibrate = 0;
      configIO(range, channelNum, calibrate);
      delay(10);
      
      // Setup AD5933 for Sweep
      Serial.println("Data Begins");
      // Run Sweep
      // Return to Wait for Command
      delay(10);
}

void setChargeon() 
{
  // Turn on Battery Charger
  digitalWrite(charge, LOW); //Low logic level for the battery charger to be on

}
void setChargeoff() 
{
  // Turn off Battery Charger
  digitalWrite(charge, HIGH); //HIGH logic level for the battery charger to be off

}

void setPoweroff()
{
  // Turn off Regulator
  digitalWrite(power, LOW); //LOW logic level for the TEER power to be off
}

void setPoweron()
{
  // Turn on Regulator
  digitalWrite(power, HIGH); //HIGH logic level for the TEER power to be on
}

