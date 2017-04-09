// Arduino Code for integrating Feather32u4 and Android app
// Author: Seokchan Yoo
// Date: April-09-2017
// Reference: AD5933 Library codes are based on Il-Taek Kwon's work
// URL Link: https://github.com/WuMRC/drive

#include "AD5933.h"
#include <Wire.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE     Perform a factory reset when running this sketch
   
                            Enabling this will put your Bluefruit LE module
                            in a 'known good' state and clear any config
                            data set in previous sketches or projects, so
                            running this at least once is a good idea.
   
                            When deploying your project, however, you will
                            want to disable factory reset by setting this
                            value to 0.  If you are making changes to your
                            Bluefruit LE device via AT commands, and those
                            changes aren't persisting across resets, this
                            is the reason why.  Factory reset will erase
                            the non-volatile memory where config data is
                            stored, setting it back to factory default
                            values.
       
                            Some sketches that require you to bond to a
                            central device (HID mouse, keyboard, etc.)
                            won't work at all with this feature enabled
                            since the factory reset will clear all of the
                            bonding data stored on the chip, meaning the
                            central device won't be able to reconnect.
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE      1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/



#define cycles_base 15
#define cycles_multiplier 1
#define start_frequency 1000          // Modify this to set start frequency
#define cal_samples 1                 // Modify this to set # of samples to be measured for taking the average (works only for calibration.)
#define step_size 1000                // Modify this to set step size of frequency increment (This should be any real number.)


// Create the Bluefruit object using hardware SPI (for Bluefruit LE feather).
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// Function prototypes and data over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);
extern uint8_t packetbuffer[];


// Constant Variable Declarations
const int numofIncrement = 90;        // Modify this to set # of frequency increment. Limited to 90 due to the memory. (Max=511)
const double calResistance = 25000;   // Modify this to be matched to the value of calibration resistance
char state, state2;
double temp;
double gainF[numofIncrement+1];
double phShift[numofIncrement+1];

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(115200);  // start serial for output

  // Basic AD5933 register setup commands
  //setByte(0x81, 0x18); // Reset & Use Ext. Clock - 0001 1000
  AD5933.setExtClock(false);
  AD5933.resetAD5933();
  AD5933.setStartFreq(start_frequency);
  AD5933.setSettlingCycles(cycles_base, cycles_multiplier);
  AD5933.setStepSize(step_size);
  AD5933.setNumofIncrement(numofIncrement);
  AD5933.setPGA(GAIN_1);
  AD5933.setRange(RANGE_3);
  AD5933.tempUpdate();

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);
  
  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=NetAnalyzer_AD5933")) ) {
    error(F("Could not set device name?"));
  }

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }


  // Automatic Calibration upon boot
  Serial.println(F("Please setup for calibration. If completed, press p and Enter>"));
  while( Serial.read() != 'p');
    
  AD5933.getGainFactorsSweep(calResistance,cal_samples, gainF, phShift);
  //printGainFactor();  // Check gain factor 
  Serial.println(F("Calibration Done!"));
}

void loop()
{
  AD5933.tempUpdate();  // Update temperature without reading it in order to increase accuracy
  
 if(Serial.available()>0) {
      state = Serial.read();

      //FSM
      switch(state) {
        case 'A':  //Program Registers
          AD5933.tempUpdate();
          AD5933.setCtrMode(TEMP_MEASURE);
          delay(10);
          temp = AD5933.getTemperature();
          Serial.println(temp);
          delay(10);
          AD5933.setCtrMode(STAND_BY);
          break;
          
        case 'B':
          AD5933.getComplexSweep(gainF, phShift, impVal, phVal);
          printImpedanceData();
          delay(1000);
          break;
      }
      Serial.flush();
    }
}


// Print Impedance Data with 5 digits precision
void printImpedanceData(){
  int cfreq = start_frequency;
  for(int i =0; i < numofIncrement+1; i++, cfreq++){
            cfreq += (step_size*i);
            Serial.println(cfreq);
            //Serial.println(F("kHz"));
            //Serial.print(F("|Z|="));
            Serial.println(impVal[i],5);
            //Serial.print(F("Phi="));
            Serial.println(phVal[i],5);   // in Radian
            delay(10);
   }
}

bool getUserInput(char buffer[], uint8_t maxSize)
{
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while( (!Serial.available()) && !timeout.expired() ) { delay(1); }

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count=0;
  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && (Serial.available()) );

  return true;
}