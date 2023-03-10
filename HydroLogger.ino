/* HydroLogger
 created from Hydrobot by
 Laura Youse 01/14/23
 Last edit 03/09/23 
 for Arduino mega2560
*/

#include "Wire.h"
#include <elapsedMillis.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <string.h>
#include <LiquidCrystal.h>
#include <BH1750.h>
#include <RTClib.h>
#include <SD.h>
#include <OneWire.h>
#include <DS18B20.h>
#include <DHT.h>
#include <DHT_U.h>

// Active sensor flags
// set any unused items to false
bool DHT_sensor_flag    = true;   // DHT22 or AM2302 I2C
bool light_meter_flag  = true;    // BH1750 I2C
bool water_sensor_flag = true;    // analog soil (for soil plants)
bool water_level_flag = true;    // analog water (for hydroponics)
bool water_temp_flag = true;      // DS18B20 one-wire

/*
 * Arduino Mega 2560 board pinouts
 * DS3231 RTC on I2C SCL pin 21, SDA pin 20
* BH1750 light sensor on I2C
* DHT22 temp/humidity sensor on 1-wire pin 45
* uSD card reader/writer on pins CS 4, SCK 52, MOSI 51, MISO 50
* DS18B20 water temp sensor on  1-wire pin 44
* Soil water sensor on A0 analog
* Water level sensor on A1 analog
*/

RTC_DS3231 rtc;

// LIGHT SENSOR - BH1750 (GY30) on I2C
BH1750 lightMeter(0x23);
// uint16_t LIGHT_TIME_OFF = 8; // Unused light off time in hours
float lux = 0;
float pre_lux = 0;
float pre_pre_lux = 0;
float sum_lux = 0;           // Used for fan control
const int hilux = 200;       // high limit for turning fan on
const int lolux = 10;        // low limit for turning fan off

// DIGITAL TEMP/HUMID SENSOR PIN and TYPE
#define DHTPIN 45
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
const bool inFahrenheit_f = true;
uint16_t dHum, dTemp;   // keep DHT and DS18 temps separate
uint16_t oldHum, oldTemp;

// DS18B20 alarms set to 10C (50F) and 35C (95F)
#define LOW_ALARM 10
#define HIGH_ALARM 35

DS18B20 ds(44);
uint8_t address[] = {40,159,231,129,227,104,60,68};
uint8_t selected;
uint16_t wTemp;

// Soil and water sensors on analog pins
#define watersensorPin A0
#define waterlevelPin A2    // A1 is defective on mega2560 board
int sensorValue = 0;        // A0 read of soil
int levelValue = 0;         // A2 read of water
int percentWsValue = 0;     // soil water %
int percentWlValue = 0;     // hydroponics water %
const int drysoil = 625; // value for dry soil sensor
const int wetsoil = 315; // value for wet soil sensor
const int dry = 1023; // value for dry sensor
const int wet = 162; // value for wet sensor
uint16_t pre_water_sensor = 0;
uint16_t pre_pre_water_sensor = 0;
uint16_t pre_water_level = 0;
uint16_t pre_pre_water_level = 0;

// pushbutton switch pins for datalog control
// connected to digital pins 2 and 3 on arduino
// for Set and Advance to next parameter
// ** enable pullups on inputs to use **
#define B0  2 
#define B1  3
volatile int buttonState0 = 0;     // variable for reading pushbutton0 status
volatile int buttonState1 = 0;     // variable for reading pushbutton1 status

/* 
 LiquidCrystal display with:
 LCD 1 (0V) to arduino ground pin 
 LCD 2 (5V) to arduino +5 V
 LCD 3 (V0) to output from potentiometer 10kOhm connected between +5 V and GND
 LCD 4 (RS) to arduino pin 12
 LCD 5 (R/W) to arduino ground
 LCD 6 (E) to arduino pin 11
 LCD 11 (DB4), 12 (DB5), 13 (DB6), 14 (DB7) on arduino pins 7, 8, 9, 10 
 LCD15 (LED+) through resistor 12 Ohm to arduino +5 V
 LCD16 (LED-) to arduino ground 
*/

LiquidCrystal lcd(12, 11, 7, 8, 9, 10);  // rs, enable, d4, d5, d6, d7

//reallocates the printing area on the SC2004CSWB by Usmar A Padow 
char buffer[20*4+1]="                                                                                ";//20*4 spaces
int pos=0;

int percentValue = 50; // default to 50% duty cycle
int pumpPeriod = 3; // default period pump cycle time in minutes
int pumpSec = 180;  // default pump cycle time in seconds
// charBuf1 and charBuf2 seem to be used by LiquidCrystal library
char charBuf1[3];
char charBuf2[3];
char charBuf3[6];
char charBuf4[9];
char charBuf5[4];

// variables for loop delay for changing pump states
unsigned long previousMillis = millis(); 
unsigned long intervalMillis = 30000;    // 30 sec in milliseconds
elapsedMillis timer0;
unsigned long minOn = 4;    // Sets fixed 4 minute on time **
unsigned long minOff = 11;  // Sets fixed 11 minute off time **
unsigned long minSet = 0;  // variable to pass ISR1 button presses to ISR0
unsigned long intervalOn = 60000 * minOn; // default to pump 2 min on time
unsigned long intervalOff = 60000* minOff; // default off time = 2 * 60 * 1000ms
int ivaltoggle = HIGH;  // default to intervalOn state

// output pin - relay board inverts - active with a low input
int pumpOutPin = 30;  // output to relay board for 120vac water pump
// output pin - MOSFET is non-inverting - active with a high input
int fanOutPin = 31;   // output to MOSFET for 12v dc fan

// sd card to Arduino mega2560 pins
// SCK on 52
// MOSI on 51
// MISO on 50
const int chipSelect = 4; // for Arduino CS

// logging control variables
bool logControl = true;            //logging on by default
volatile int logPeriod = 60;       //default log period of 60 minutes
unsigned long  unixTime = 0;
unsigned long  previousUnixTime = 0;
unsigned long  intervalUnixTime = 3600000;    // 60 min in milliseconds

// Create a file to store the logged data
File myFile;

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}


void setup()
{
  // Start serial comm for debugging, I2C and light meter
  // DS2321 rtc on I2C address of 0x68
  Serial.begin(9600);
  Wire.begin();
  
  // setup rtc and send current data to DateTime
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }
  
  if (! rtc.lostPower()) {
    Serial.println("RTC is NOT running, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
 
  delay(50); // allow time for serial output to settle

  // Initialize light meter
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println(F("BH1750 Advanced begin"));
  } else {
    Serial.println(F("Error initialising BH1750"));
  }

  delay(300); // allow >120ms for measurement
  
  dht.begin();

  // declare output pins and set items off
  pinMode(pumpOutPin,OUTPUT);
  digitalWrite(pumpOutPin,HIGH);
  pinMode(fanOutPin,OUTPUT);
  digitalWrite(fanOutPin,LOW);  
  delay(50);   //delay to reduce current spikes

 
  // Check disable sensors if reads fail 

  // initial read of BH1750
  if (isnan(lightMeter.readLightLevel())) {
    Serial.print(F("Failed to read from Light sensor!"));
    light_meter_flag = false;
  }
     
  if ((isnan(dht.readHumidity())) || isnan(dht.readTemperature(true))) {
    Serial.print(F("Failed to read from DHT sensor!"));
     DHT_sensor_flag = false;
  }
   
  // DS18B20 water temp sensor 
  selected = ds.select(address);
  if (selected) {
    ds.setAlarms(LOW_ALARM, HIGH_ALARM);
  } else {
    Serial.println("Water Temp sensor not found!");
    water_temp_flag = false;
  }
  
  if (isnan(analogRead(watersensorPin))) {
    Serial.println("Water Soil sensor not found!");
    water_sensor_flag = false;
  }   
  if (isnan(analogRead(waterlevelPin))) {
    Serial.println("Water Level sensor not found!");
    water_level_flag = false;
  } 


  // setup for the SD card
  Serial.print(F("Initializing SD card..."));

  if(!SD.begin(chipSelect)) {
    Serial.println(F("SD initialization failed!"));
    return;
  }
  Serial.println(F("SD initialization done."));
    
  //open file
  myFile=SD.open("DATA.txt", FILE_WRITE);

  // if the file opened ok, write header to it:
  if (myFile) {
    Serial.println(F("File opened ok"));
    // print the headings for our data
    myFile.println("Date,Time,Temp,Hum,Light,WTemp,Soil,Water");
  }
  myFile.close();


  // setup LED pin 13 as output
  pinMode(LED_BUILTIN,OUTPUT);

  // Get lcd ready to start displaying characters
  lcd.begin(20, 4); //lcd.begin(columns, rows)

  // set pushbutton switch inputs with pullups
  // for datalog off/on/interval control
  // not tested to work as of 2/9/23
  pinMode(B0,INPUT_PULLUP);   // Set button
  pinMode(B1,INPUT_PULLUP);   // Select button
  delay(50);  // delay 

  displayPumpSettings();
  setPumpCycle();
}

void displayPumpSettings()
{
  pumpPeriod = (int)((intervalOn + intervalOff)/60000);
  
  // put minOn into character buffer
  String strMinOn = "";
  strMinOn += minOn;
  strMinOn.toCharArray(charBuf1, 3);
  Serial.print("minOn = ");
  Serial.println(minOn);
  
  // put minOff into character buffer
  String strMinOff = "";
  strMinOff += minOff;
  strMinOff.toCharArray(charBuf2, 3);
  Serial.print("minOff = ");
  Serial.println(minOff); 
 
 // continue with more status info - 1st line of display
 delay(500);
 lcd.setCursor(0,0); // Position cursor on col x=1, line y=1
 lcd.print("Pump On "); // Print a message to the LCD
 lcd.setCursor(8,0); // Print message on 1st line (col,row are n-1)
 lcd.print(charBuf1); // minOn
 lcd.setCursor(10,0); // Print more of message on 1st line
 lcd.print("Off ");
 lcd.print(charBuf2); // minOff
 lcd.print(" Min"); // Print last of 1st line message to the LCD
 
// lcd.setCursor(0,1); // Position cursor on col x=1, line y=2  
// lcd.print(charBuf2); // Print duty cycle message on 2nd line
  delay(250);          // Delay for lcd to finish printing
}

void setPumpCycle()
{
  pumpPeriod = (int)((intervalOn + intervalOff)/60000);
  
  // put minOn into character buffer
  String strMinOn = "";
  strMinOn += minOn;
  strMinOn.toCharArray(charBuf1, 3);
  Serial.print("minOn = ");
  Serial.println(minOn);
  
  // put minOff into character buffer
  String strMinOff = "";
  strMinOff += minOff;
  strMinOff.toCharArray(charBuf2, 3);
  Serial.print("minOff = ");
  Serial.println(minOff); 
 
  // continue with more status info - 1st line of display
  delay(500);
  lcd.setCursor(0,0); // Position cursor on col x=1, line y=1
  lcd.print("Pump On "); // Print a message to the LCD
  lcd.setCursor(8,0); // Print message on 1st line (col,row are n-1)
  lcd.print(charBuf1); // minOn
  lcd.setCursor(10,0); // Print more of message on 1st line
  lcd.print("Off ");
  lcd.print(charBuf2); // minOff
  lcd.print(" Min"); // Print last of 1st line message to the LCD
 
  delay(250);          // Delay for lcd to finish printing

  // Turn on pump for start of cycle after setting period & duty 
  PumpOn();
  ivaltoggle = HIGH;  // default to intervalOn state
  timer0 = 0; // clear the timer at the end of setup
}
  
void PumpOn()
{
  digitalWrite(pumpOutPin, LOW);  // turn pump on with relay board
  lcd.setCursor(0,1); // Print message on 2nd line
  lcd.print("Pump:ON  "); // Print a message to the LCD
}

void PumpOff()
{
  digitalWrite(pumpOutPin, HIGH);  // turn pump off with relay board
  lcd.setCursor(0,1); // Print message on 2nd line
  lcd.print("Pump:OFF "); // Print a message to the LCD
}

void FanOn()
{
  digitalWrite(fanOutPin, HIGH);  // turn fan on with MOSFET
  lcd.setCursor(10,1); // Print more of message on 2nd line
  lcd.print("Fan:ON "); // Print a message to the LCD
}

void FanOff()
{
  digitalWrite(fanOutPin, LOW);  // turn fan off with MOSFET
  lcd.setCursor(10,1); // Print more of message on 2nd line
  lcd.print("Fan:OFF "); // Print a message to the LCD
}

void LightMeter()
{
  if (lightMeter.measurementReady()) {
    lux = lightMeter.readLightLevel();
    Serial.print(F("Light: "));
    Serial.println(lux);
    //Display Light Sensor information to LCD    
    lcd.setCursor(0,2); // Print message on 3rd line
    lcd.print("L:");   
    lcd.setCursor(2,2); // Print message on 3rd line, 3rd column
    if (lux < 10)
    {
    lcd.print("    ");
    } else
    if (lux < 100)
    {
    lcd.print("   ");
    } else
    if (lux < 1000)
    {
    lcd.print("  ");
    } else
    if (lux < 10000)
    {
    lcd.print(" ");
    }
    lcd.print(int(lux));
    lcd.print(" ");   
  }
  
  // if the lux has changed  
  if (lux != pre_lux) {
    //Insert new reading, drops last reading
    pre_lux = lux;
    pre_pre_lux = pre_lux;
    
    //Calculates the running average of the lum value using past 2 readings
    //sum_lux is declared as a global float variable to use for fan on/off control
    sum_lux = (lux + pre_lux + pre_pre_lux)/3;

   //Display Light Sensor information(BH1750) to Serial Terminal
   Serial.print("Light Average: ");
   Serial.print(sum_lux, DEC);
   Serial.println("  | ");
    }      
}

void TempHumidity()
{
// for DHT22 (AM2302) 
 //Gather humidity and temperature readings from sensor
  dHum = dht.readHumidity();
  dTemp = dht.readTemperature(inFahrenheit_f);
  
  if (dHum != oldHum || dTemp != oldTemp) {      // if the humidity or Temperature has changed
      
    oldHum = dHum;             // save the value for next comparison 
    oldTemp = dTemp;           // save the value for next comparison
  }
    
    //Print out sensor information to LCD.
    lcd.setCursor(9,2); // Print more of message on 3rd line
    lcd.print("H:");
    lcd.print(dHum, 10);
    lcd.print("% T:");
    lcd.print(dTemp, 10);
    if (inFahrenheit_f)
      lcd.print("F");
    else
      lcd.print("C");

    //Print out sensor information to Serial Output
    Serial.print("Humidity: ");
    Serial.print(dHum);
    Serial.print("% Temperature ");
    if (inFahrenheit_f)
      Serial.print("(F)");
    else
      Serial.print("(C)");
    Serial.print(": ");
    Serial.print(dTemp);
    Serial.print(" | ");       
}

void WaterTemp()
{
  // DS18B20 code here
  if (selected) {
    wTemp = ds.getTempC();
    lcd.setCursor(0,3); // Print message on 4th line
    lcd.print("WT:");
    lcd.print(wTemp, 10);
    if (ds.hasAlarm()) {
      Serial.print("Warning! Temperature is ");
      Serial.print(ds.getTempC());
      Serial.println(" C");
    }
  } else {
    Serial.println("Water Temp sensor not found!");  
  }
}

void WaterSensor()
{
  // read the soil moisture from the sensor, dry is highest voltage
  sensorValue = analogRead(watersensorPin);
  Serial.print(F("WaterSensor ADC value for calibration = "));
  Serial.println(sensorValue);
  percentWsValue = map(sensorValue, wetsoil, drysoil, 100, 0); 
  // wait before more calculations
  delay(50);
  lcd.setCursor(6,3); // Print more of message on 4th line
  lcd.print("WS:");
  lcd.print(percentWsValue, 10); // 10 is Decimal
}

void WaterLevel()
{
  // read the water level from the sensor, dry is highest voltage
  levelValue = analogRead(waterlevelPin);
  Serial.print(F("WaterLevel ADC value for calibration = "));
  Serial.println(levelValue);
  percentWlValue = map(levelValue, wet, dry, 100, 0);
  // wait before more calculations
  delay(50);
  lcd.setCursor(14,3); // Print last of message on 4th line
  lcd.print("WL:");
  lcd.print(percentWlValue, 10); // 10 is Decimal
}

// this routine isn't used - left in for reference
void loggingTime()
{
  DateTime now = rtc.now();
  myFile = SD.open("DATA.txt", FILE_WRITE);
  if (myFile) {
    myFile.print(now.year(), DEC);
    myFile.print('/');
    myFile.print(now.month(), DEC);
    myFile.print('/');
    myFile.print(now.day(), DEC);
    myFile.print(',');
    myFile.print(now.hour(), DEC);
    myFile.print(':');
    myFile.print(now.minute(), DEC);
    myFile.print(':');
    myFile.print(now.second(), DEC);
    myFile.print(",");
  }
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.println(now.day(), DEC);
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.println(now.second(), DEC);
  myFile.close();
  delay(1000);  
}


void loop()
{
  // Wait 30 sec between time display update by checking on time milliseconds
  // created 2005 by David A. Mellis, modified 8 Feb 2010 by Paul Stoffregen
  // from tutorial Blink without Delay on arduino.cc
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > intervalMillis)
  {
    // save the last time
    previousMillis = currentMillis;
    //displayTime(); // display the real-time clock data on the Serial Monitor
    // pump on time set by duty cycle
    Serial.print("    ");
    Serial.print(F("timer0: "));
    Serial.print(timer0, DEC);
    Serial.println(" ");
    if ((ivaltoggle == HIGH) && (timer0 > intervalOn))     
    {
      Serial.print(F("high to low"));
      Serial.print(ivaltoggle);
      Serial.println(timer0, DEC);
      ivaltoggle = LOW; // switch to off cycle check on next loop
      timer0 -= intervalOn; //reset the timer
      //turn off at end of intervalOn
      PumpOff();
    }
    else if ((ivaltoggle == LOW) && (timer0 > intervalOff))
    {
      Serial.print(F("low to high"));
      Serial.print(ivaltoggle);
      Serial.println(timer0, DEC);  
      ivaltoggle = HIGH; // switch to on cycle check on next loop
      timer0 -= intervalOff; //reset the timer
      //turn on at end of intervalOff     
      PumpOn();
    } 
  }
  Alarm.delay(500);  // required for water temp lo/hi - usually set to 1000

  Serial.println(F("Loop start light meter for fan control"));
  if (light_meter_flag == true) {
    LightMeter();
    } else {
      Serial.println(F("No Light Sensor | "));
      lcd.setCursor(0,2); // Print message on 3rd line
      lcd.print("L: OFF ");
      light_meter_flag = false;
  }
  Serial.println(F("Loop light - fan control"));
  if (light_meter_flag == true) {
    // Turn fan on immediately when light is turned on 
    if (lux > (hilux && !digitalRead(fanOutPin))) {
      FanOn();      
        }       
   
    // Turn fan off after light goes off
    if (lux < (lolux && digitalRead(fanOutPin))) {
      FanOff(); 
        }      
  }
   
  if (light_meter_flag == false) {
    FanOn();  // default to fan always on if light sensor not working
  }   

// Check if Set button (B0) has been held down for controlling logging
// not working, need to check wiring - fixed to 60 minutes 3/8/23
  Serial.println(F("Loop before button pressed checking"));
  while(!digitalRead(B0)) {
    if(!digitalRead(B1)) {
    // increment with switch-case 20,30,60 mins or off
    for (int i = 0; i < 4; i++) {
    switch (i) {
      case 0:
        Serial.print("20 min");
        logPeriod = 20;
        logControl = true;
        Serial.println(i);
        break;
      case 1:
        Serial.print("30 min");
        logPeriod = 30;
        logControl = true;
        Serial.println(i);
        break;
      case 2:
        Serial.print("60 min");
        logPeriod = 60;
        logControl = true;
        Serial.println(i);
        break;
      default:
        Serial.print("Off");
        logPeriod = 0;
        logControl = false;
        Serial.println(i);
        break;
    }
  }
    }
  }
  
  // Print logPeriod in minutes on far right after pump & fan status
  lcd.setCursor(18,1); // Print message on end of 2nd line
  lcd.print(logPeriod, 10);

  // rem out to check for error 3/8/23
  // intervalUnixTime = logPeriod * 60 * 1000;   // min*60*1000 milleseconds
  Serial.print(F("intervalUnixTime for logging = "));
  Serial.println(intervalUnixTime);

// Measure light, temp, hum, water temp, soil and plant water levels
// Display messages if temps are high, hum or water levels are low

//========== LIGHT METER - START ============
  Serial.println(F("Loop start light meter"));
  if (light_meter_flag == true) {
    LightMeter();
  } else {
    Serial.print(F("No Light Sensor | "));
    lcd.setCursor(0,2); // Print message on 3rd line
    lcd.print("L: OFF ");
  }
//========== LIGHT SENSOR - END  =============

//========== DHT SENSOR - START ==============
  Serial.println(F("Loop start DHT"));
  if (DHT_sensor_flag == true) {
     TempHumidity();
  } else {
    Serial.println(F("No DHT Sensor | "));
    lcd.setCursor(7,2); // Print more of message on 3rd line
    lcd.print("T & H:OFF ");
  }
//========== DHT SENSOR - END  ===============

//========== DS18B20 WATER TEMP - START  ===============
  Serial.println(F("Loop start water temp"));
  if (water_temp_flag == true) {
     WaterTemp();
  } else {
    Serial.println(F("No Water Temp Sensor | "));
    lcd.setCursor(0,3); // Print message on 4th line
    lcd.print("WT:OFF ");
  }

//========== DS18B20 WATER TEMP - END  ===============

//======= WATER SENSOR - START =======
  Serial.println(F("Loop start water sensor for soil"));
  if (water_sensor_flag == true) {
    
    //Gather water sensor reading from sensor and converts it to percentage 
    sensorValue = analogRead(watersensorPin);
    Serial.print(F("WaterSensor ADC value for calibration = "));
    Serial.println(sensorValue); 
    uint16_t cur_water_sensor = map(sensorValue, wetsoil, drysoil, 100, 0);

    //Calculates the running average of the water sensor value using past 2 readings
    uint16_t sum_water_sensor = (cur_water_sensor + pre_water_sensor + pre_pre_water_sensor)/3;

    //Insert new reading, drops last reading
    pre_water_sensor = cur_water_sensor;
    pre_pre_water_sensor = pre_water_sensor;
 
    //Display Water Sensor 1 information to LCD    
    lcd.setCursor(7,3); // Print message on 4th line
    lcd.print("WS:");
    lcd.print(sum_water_sensor);
    lcd.print("% ");
    
    //Display Water Level information to Serial Terminal
    Serial.print(F("Water Sensor : "));
    Serial.print(sum_water_sensor, DEC);
    Serial.print("% | ");
    
  } else {
    Serial.print(F("No WS Sensor | "));
    lcd.setCursor(6,3); // Print message on 4th line
    lcd.print("WS:OFF ");
  }
  
//======= WATER SENSOR - END   =======

//======= WATER LEVEL - START =======
  Serial.println(F("Loop start water level for hydro"));
  if (water_level_flag == true) {
    
    //Gather water sensor reading from sensor and converts it to percentage 
    levelValue = analogRead(waterlevelPin);
    Serial.print(F("WaterLevel ADC value for calibration = "));
    Serial.println(levelValue);
    uint16_t cur_water_level = map(levelValue, wet, dry, 100, 0);
     
    //Calculates the running average of the water sensor value using past 2 readings
    uint16_t sum_water_level = (cur_water_level + pre_water_level + pre_pre_water_level)/3;
    
    //Insert new reading, drops last reading
    pre_water_level = cur_water_level;
    pre_pre_water_level = pre_water_level;
 
    //Display Water Sensor 1 information to LCD    
    lcd.setCursor(14,3); // Print message on 4th line
    lcd.print("WL:");
    lcd.print(sum_water_level);
    lcd.print("% ");
    
    //Display Water Level information to Serial Terminal
    Serial.print(F("Water Level : "));
    Serial.print(sum_water_level, DEC);
    Serial.print("% | ");
    
  } else {
    Serial.print(F("No WL Sensor | "));
    lcd.setCursor(14,3); // Print message on 4th line
    lcd.print("WL:OFF");
  }
  
//======= WATER LEVEL - END   =======

 // rewrite first line in case WL steps on it
 lcd.setCursor(0,0); // Position cursor on col x=1, line y=1
 lcd.print("Pump On "); // Print a message to the LCD
 lcd.setCursor(8,0); // Print message on 1st line (col,row are n-1)
 lcd.print(charBuf1); // minOn
 lcd.setCursor(10,0); // Print more of message on 1st line
 lcd.print("Off ");
 lcd.print(charBuf2); // minOff
 lcd.print(" Min"); // Print last of 1st line message to the LCD


//======= LOGGING - START =======  
// Build log string for data below if log period has passed
// Date, Time, Light, Air Temp, Humidity, Water Temp, Soil Moisture, Water Level 
  DateTime now = rtc.now();
  unixTime = now.unixtime();
  Serial.print(F("Logging unixTime is "));
  Serial.println(unixTime, DEC);
  if(unixTime - previousUnixTime > intervalUnixTime)
    {
      // save the last time
      previousUnixTime = unixTime;
  if (logControl) {
  Serial.println(F("Open log to write data"));   
  myFile = SD.open("DATA.txt", FILE_WRITE);
  if (myFile) {    
    myFile.print(now.year(), DEC);
    myFile.print('/');
    myFile.print(now.month(), DEC);
    myFile.print('/');
    myFile.print(now.day(), DEC);
    myFile.print(',');
    myFile.print(now.hour(), DEC);
    myFile.print(':');
    myFile.print(now.minute(), DEC);
    myFile.print(':');
    myFile.print(now.second(), DEC);
    myFile.print(", ");
    myFile.print(lux);
    myFile.print(",");
    myFile.print(dTemp);
    myFile.print(",");
    myFile.print(dHum);
    myFile.print(",");
    myFile.print(wTemp);
    myFile.print(",");
    myFile.print(percentWsValue);
    myFile.print(",");
    myFile.println(percentWlValue);
  }
  myFile.close();
  }         
    }
//======= LOGGING - END =======    
}
