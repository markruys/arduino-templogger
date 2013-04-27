/////////////////////////////////////////////////////////////////////////////////////////////////
//  Temperature logger
//  Mark Ruys, mark.ruys@peercode.nl, April 2013
//
//  Shows the current temperatur reading(s) in your display. Periodically takes a sampe and
//  stores it to either volatile RAM, persistent EEPROM, or persistent SD-card.
//  Stored samples can be dumped to the serial console. Supports RTC if available.
//
//  Needed hardware:
//   - Arduino board
//   - One or more TMP36 analog temperature sensors
//   - SD card shield (optional)
//   - RTC cock (optional)
//   - 16x2 display (optional)

// Define how often you want to store a sample
#define SAMPLE_PERIOD_IN_MINUTES 15

// Choose the type of memory you want to use to store samples:
//#define STORAGE_VOLATILE
//#define STORAGE_EEPROM
#define STORAGE_SDCARD
// If you choose STORAGE_VOLATILE, read NOTE 1 at the bottom of this file.
// Unless you choose STORAGE_SDCARD, we use a button to print all stored
// samples to the serial console. If you press & hold this button for a few
// seconds, the storage will be erased. Useful for discarding the contents i
// EEPROM memory.

// If the board has an RTC, uncomment the HAS_RTC define
#define HAS_RTC

// If you have attached an LCD, define HAS_LCD:
#define HAS_LCD

// Enabling serial gives us useful status information via the serial USB.
// Disabling serial gives us the possibility to use the pins 0 and 1 for
// other purposes like card detect. Note that to program the Arduino, these
// pins may not be connected to anything (so use jumpers or a switch).
// #define USE_SERIAL

//////////////////////////////////////////////////////////////////////////////////////////////////
// Includes

#ifdef HAS_LCD
  #include <LiquidCrystal.h>
#endif

#include <Wire.h>
#include <RTClib.h> # from https://github.com/adafruit/RTClib

#ifdef STORAGE_SDCARD
  #include <SPI.h>
  #include <SD.h>
#endif

#ifdef STORAGE_EEPROM
  #include <EEPROM.h>
#endif

// Help IDE compiling our code properly, see http://arduino.cc/forum/index.php?topic=87155.0:
char foo;

//////////////////////////////////////////////////////////////////////////////////////////////////
// Pin assignents

// Analog pin assignments:
//
//  A0 temperature sensor B
//  A1 temperature sensor A
//  A2 analog noise for random seed
//  A3
//  A4 TWI SDA    RTC SDA (i2c data)
//  A5 TWI SCL    RTC SCL (i2c clock)

// Digital pin assignments:
//
//   0 Serial RX (receive)
//   1 Serial TX (transmit)
//   2 LCD data 7
//   3 LCD data 6
//   4 LCD data 5
//   5 LCD data 4
//   6 green heartbeat LED
//   7 red error LED
//   8 LCD EN (enable write)
//   9 LCD RS (register select)
//  10 SPI SS      SD card CS (slave select -> card/chip select)
//  11 SPI MOSI    SD card DI (master out slave in -> data in)
//  12 SPI MISO    SD card DO (master in salve out -> data out)
//  13 SPI SCK     SD card CLK (clock)

#ifndef USE_SERIAL
  #define PIN_SDCARD_WP 0
  #define PIN_SDCARD_CD 1
#endif

#ifdef HAS_LCD
  #define PIN_LCD_D7 2
  #define PIN_LCD_D6 3
  #define PIN_LCD_D5 4
  #define PIN_LCD_D4 5
  #define PIN_LCD_EN 8
  #define PIN_LCD_RS 9
#endif

// Comment out if these LED's do not exist:
#define PIN_LED_HEARTBEAT 6
//#define PIN_LED_ERROR 7

// Without shield, on a Arduino Uno, uncomment:
#define PIN_LED_ERROR 13

#ifdef STORAGE_SDCARD
// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
  #define PIN_SDCARD_CS 10
#else
  #define PIN_BUTTON 10
#endif

#define PIN_TEMPERATURE_SENSOR_A A1
#define PIN_TEMPERATURE_SENSOR_B A0

// At some point, we need a random generator and this helps to add some noise:
#define PIN_ANALOG_UNUSED A2

//////////////////////////////////////////////////////////////////////////////////////////////////
// To get a better accuracy, set AREF to 3.3V and use this voltage for the TMP36:
//  http://learn.adafruit.com/tmp36-temperature-sensor/using-a-temp-sensor

//#define AREF_VOLTAGE 3.3     // measure with a multimeter & adjust this value, typically 3.3 (Volts)
#define AREF_VOLTAGE 3.304     // measure with a multimeter & adjust this value, typically 3.3 (Volts)

//#define OFFSET_SENSOR_A -1.4
#define OFFSET_SENSOR_A -0.5
#define OFFSET_SENSOR_B 0

//////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBALS

DateTime now;
int prev_sec;

boolean halted = false;

//////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP

void setup() {

  #ifdef PIN_LED_ERROR
    pinMode(PIN_LED_ERROR, OUTPUT); 
    digitalWrite(PIN_LED_ERROR, LOW);
  #endif
  
  #ifdef PIN_SDCARD_CD
    pinMode(PIN_SDCARD_CD, INPUT);
  #endif
  
  #ifdef PIN_SDCARD_WP
    pinMode(PIN_SDCARD_WP, INPUT);
  #endif
  
  #ifdef USE_SERIAL
    Serial.begin(57600);
  #endif

  readingsSetup();
  
  heartbeatLedSetup();
  
  buttonSetup();

  lcdSetup();

  rtcSetup();

  // Initialize random generator
  randomSeed(analogRead(PIN_ANALOG_UNUSED) ^ now.unixtime());

  analogReference(EXTERNAL);

  storageSetup();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// LOOP

void loop() {

  heartbeatLedOff();

  if ( halted ) {
    return; // Application crashed
  }
  
  #ifdef PIN_SDCARD_CD
    if ( digitalRead(PIN_SDCARD_CD) ) {
      error("Insert SD card");
      return; // exit loop
    }
  #endif

  #ifdef PIN_SDCARD_WP
    if ( digitalRead(PIN_SDCARD_WP) ) {
      error("Unlock SD card");
      return; // exit loop
    }
  #endif

  rtcUpdate();

  if ( prev_sec != now.second() ) {
    prev_sec = now.second();

    readingsLoop();

    if ( now.minute() % SAMPLE_PERIOD_IN_MINUTES == 0 && now.second() == 0 ) {
      readingsStore();
    }

    heartbeatLedOn();  
  }

  buttonLoop();  
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Readings

#if defined(PIN_TEMPERATURE_SENSOR_B)
  #define NUMBER_OF_SENSORS 2
  const int temperatureSensorPins[] = {PIN_TEMPERATURE_SENSOR_A, PIN_TEMPERATURE_SENSOR_B};
  const float temperatureSensorOffsets[] = {OFFSET_SENSOR_A, OFFSET_SENSOR_B};
#elif defined(PIN_TEMPERATURE_SENSOR_A)
  #define NUMBER_OF_SENSORS 1
  const int temperatureSensorPins[] = {PIN_TEMPERATURE_SENSOR_A};
  const float temperatureSensorOffsets[] = {OFFSET_SENSOR_A};
#else
  #define NUMBER_OF_SENSORS 0
  const int temperatureSensorPins[] = {};
  const float temperatureSensorOffsets[] = {};
#endif

float summedTemperatures[NUMBER_OF_SENSORS];
int numberOfTemperatures;

void readingsSetup() {
  int i;
  for ( i = 0; i < NUMBER_OF_SENSORS; i++ ) {
    summedTemperatures[i] = 0;
 }
  numberOfTemperatures = 0;
}

inline void readingsLoop() {
  float temperatures[NUMBER_OF_SENSORS];
  
  int i;
  for ( i = 0; i < NUMBER_OF_SENSORS; i++ ) {
    analogRead(temperatureSensorPins[i]);
    delay(10); // Give high impendance TMP36 sensor time to get used to the ADC multiplexer
    
    float voltage = analogRead(temperatureSensorPins[i]) * AREF_VOLTAGE / 1024.0;
    temperatures[i] = (voltage - 0.5) * 100.0 + temperatureSensorOffsets[i]; 

    summedTemperatures[i] += temperatures[i];

    loggerPrint(temperatures[i]);
    loggerPrint(" ");
  }

  loggerPrintln("");

  numberOfTemperatures++;
    
  lcdLoop(temperatures);
}

inline void readingsStore() {
    storageLog(now, summedTemperatures, numberOfTemperatures);

    readingsSetup();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// RTC

#ifdef HAS_RTC

  RTC_DS1307 RTC;
  
  inline void rtcSetup() {
    Wire.begin();
    RTC.begin();
  
    if ( ! RTC.isrunning() ) {
      loggerPrintln("RTC is NOT running!");
      // following line sets the RTC to the date & time this sketch was compiled
      RTC.adjust(DateTime(__DATE__, __TIME__));
    }
    
    now = RTC.now();
  }

#else

  RTC_Millis RTC;
  
  inline void rtcSetup() {
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.begin(DateTime(__DATE__, __TIME__));
    
    now = RTC.now();
  }

#endif /*HAS_RTC*/

inline void rtcUpdate() {
  now = RTC.now();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// LCD

#ifdef HAS_LCD
  
  LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

  inline void lcdSetup() {
    lcd.begin(16, 2); // 16 characters per line, two lines
  }

  /* digits()
     Number of digits before the floating point (not included). Negative sign also counts as a digit.
   */

  int digits(int i) {
    if ( i < 0 ) {
      return digits(-i) + 1;
    }
    if ( i < 10 ) {
      return 1;
    }
    return digits(i / 10) + 1;  
  }

  int digits(float f) {
    if ( f < 0 ) {
      return digits(-f) + 1;
    }
    
    f = round(f * 10.0) / 10.0;
    
    return digits((int)f);
  }

  inline void lcdLoop(float *temperature) {
#if NUMBER_OF_SENSORS == 1
    lcd.setCursor(1, 0);
    lcd.print("Temp: ");
    lcd.print(temperature[0], 1);
    lcd.print(" 'C ");
#elif NUMBER_OF_SENSORS == 2
    lcd.setCursor(3 - digits(temperature[0]), 0);
    lcd.print(" ");
    lcd.print(temperature[0], 1);

    lcd.setCursor(12 - digits(temperature[1]), 0);
    lcd.print(" ");
    lcd.print(temperature[1], 1);
#else
    lcd.home();
    lcd.print(NUMBER_OF_SENSORS);
    lcd.print(" sensors unsupported");
#endif

#ifdef STORAGE_SDCARD
    lcd.setCursor(0, 1);
    lcd.print(FormattedTime(now));
    lcd.print(" ");
    
    int used = storageUsed();

    lcd.setCursor(14 - digits(used), 1);
    lcd.print(" #");
    lcd.print(used);
#else
    lcd.setCursor(0, 1);
    lcd.print(FormattedTime(now));    
    lcd.print(" ");

    int used = storageUsed();
    int free = storageFree();

    lcd.setCursor(13 - digits(used) - digits(free), 1);
    lcd.print(" #");
    lcd.print(used);
    lcd.print("/");
    lcd.print(free);    
#endif

  }
  
  inline void lcdError(char *message) {
    lcd.clear();
    lcd.print(message);
  }
  
#else 

  inline void lcdSetup() { }
  inline void lcdLoop(float) { } 
  inline void lcdError(char *) { }
  
#endif

/////////////////////////////////////////////////////////////////////////
// Heartbeat

#ifdef PIN_LED_HEARTBEAT

  unsigned long heartbeatLedMillis;
  
  inline void heartbeatLedSetup() {
    pinMode(PIN_LED_HEARTBEAT, OUTPUT); 
    digitalWrite(PIN_LED_HEARTBEAT, HIGH);
  }

  inline void heartbeatLedOn() {
    digitalWrite(PIN_LED_HEARTBEAT, HIGH);
    heartbeatLedMillis = millis();
  }

  inline void heartbeatLedOff() {
    // Turn off the LED only after some 100 milleseconds delay
    if ( millis() > heartbeatLedMillis + 100 ) {
      digitalWrite(PIN_LED_HEARTBEAT, LOW);
      heartbeatLedMillis = 0;
    }
  }
  
#else

  inline void heartbeatLedSetup() { }
  inline void heartbeatLedOn() { }
  inline void heartbeatLedOff() { }

#endif

/////////////////////////////////////////////////////////////////////////
// Handle button presses

#ifdef PIN_BUTTON

  boolean buttonState = LOW;
  unsigned long buttonChangeUnixtime;

  inline void buttonSetup() {
    pinMode(PIN_BUTTON, INPUT);
  }

  inline void buttonLoop() {
    int reading = digitalRead(PIN_BUTTON);
    if (reading != buttonState) {
      buttonChangeUnixtime = now.unixtime();
      if ( reading == HIGH ) {
        loggerPrintln("Results:");
  
        storageReadInit();
  
        DateTime dt;
        float temperatures[NUMBER_OF_SENSORS];
        while ( storageReadNext(&dt, temperatures) ) {
          loggerPrint(FormattedDate(dt));
          loggerPrint(" ");
          loggerPrint(FormattedTime(dt));
          
          int i;
          for ( i = 0 ; i < NUMBER_OF_SENSORS; i++ ) {
            loggerPrint("\t");
            loggerPrint(temperatures[i]);
          }
          loggerPrintln("");
        }
      }
    }
    buttonState = reading;
    
    if ( buttonState && now.unixtime() - buttonChangeUnixtime == 5 ) {
      storageErase();
      loggerPrintln("Results erased");
      buttonChangeUnixtime = 0;
    }
  }

#else

  inline void buttonSetup() { }
  inline void buttonLoop() { }

#endif


/////////////////////////////////////////////////////////////////////////
// Logging to the serial console (if enabled)

#ifdef USE_SERIAL
  inline void loggerPrint(char *s) {
    Serial.print(s);
  }
  
  inline void loggerPrint(int i) {
    Serial.print(i);
  }
  
  inline void loggerPrint(float f) {
    Serial.print(f, 1);
  }
  
  inline void loggerPrintln(char *s) {
    Serial.println(s);
  }
  
  inline void loggerPrintln(int i) {
    Serial.println(i);
  }
  
  inline void loggerPrintln(float f) {
    Serial.println(f, 1);
  }
#else
  inline void loggerPrint(char *s) { }
  inline void loggerPrint(int i) { }
  inline void loggerPrint(float f) { }
  inline void loggerPrintln(char *s) { }
  inline void loggerPrintln(int i) { }
  inline void loggerPrintln(float f) { }
#endif

/////////////////////////////////////////////////////////////////////////
// Error handler

void error(char *message) {

  #ifdef PIN_LED_ERROR
    digitalWrite(PIN_LED_ERROR, HIGH);
  #endif
  
  loggerPrintln(message);

  lcdError(message);
  
  halted = true;
}

/////////////////////////////////////////////////////////////////////////
// Date and time helper functions

char buffer[11];

char *FormattedTime(DateTime dt) {
  sprintf(buffer, "%d:%02d:%02d", dt.hour(), dt.minute(), dt.second());

  return buffer;
}

char *FormattedDate(DateTime dt) {
  sprintf(buffer, "%d-%02d-%02d", dt.year(), dt.month(), dt.day());

  return buffer;
}

/////////////////////////////////////////////////////////////////////////
// Storage

#ifdef STORAGE_SDCARD

#define FILENAME "datalog.csv"

//------------------------------------------------------------------------------
// call back for file timestamps
void dateTime(uint16_t* date, uint16_t* time) {

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

int storageTemperatureCount;

inline int storageUsed() {
  return storageTemperatureCount;
}

inline void storageSetup() {
  loggerPrint("Initializing SD card... ");

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if ( ! SD.begin(PIN_SDCARD_CS)) {
    error("Card failed, or not present");
    return;
  }
  loggerPrintln("done.");

  // Set date time callback function
  SdFile::dateTimeCallback(dateTime); 
  
  storageTemperatureCount = 0;
  
  File dataFile = SD.open(FILENAME, FILE_READ);
  if ( dataFile ) {
    while ( dataFile.available() ) {
      if ( dataFile.read() == 13 ) {
        storageTemperatureCount++;
      }
    }    
    dataFile.close();
  }
}

inline void storageErase() {
  if ( ! SD.remove(FILENAME) ) {
    error("Failed SD.remove()");
  }

  storageSetup();
}

void storageLog(DateTime dt, float *summedTemperatures, int numberOfTemperatures) {

  // Open the file. Note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(FILENAME, FILE_WRITE);

  if (dataFile) {
    dataFile.print(FormattedDate(dt));
    dataFile.print(" ");
    dataFile.print(FormattedTime(dt));
    int i;
    for ( i = 0; i < NUMBER_OF_SENSORS; i++ ) {
      dataFile.print("\t");
      dataFile.print(summedTemperatures[i] / numberOfTemperatures, 1);
    }
    dataFile.println();
    dataFile.close();
    
    storageTemperatureCount++;
  }  
  else {
    error("Error SD.open()");
  } 
}

#else /* ! STORAGE_SDCARD */

/*
 The TMP36 measures temperatures ranging from -40°C upto 150°C. We want to save
 temperatures at a resolution of 0.1°C (although the TMP36 is not that precise.)

 As RAM and EEPROM storage is rather limited, we want to store temperatures as 
 compact as possible. We need values upto (40 + 150) * 10 = 1900. For this we need
 11 bits (2 ^ 11 = 2048). So we store two temperature readings in 3 bytes (24 bits.)
 The first two (most significant) bits define the contents of the other 22 bits:
   11: free, unallocated memory
   10: a date time value (see below)
   01: both 11 bits slots are filled with temperatures
   00: only the first slot is filled with a temperature
 To erae EEPROM, we fill it completely with 255 (0xff.)

 We store dates from 2013-1-1. Suppose you want to store dates up to 2021-1-1,
 you need to sample no more than once a seconds in order to be able to save it
 in a 22 bit slot: 
   8 * 365 * 24 * 60 / (2 ^ 22) minutes = 1 minute
 */

#define STORAGE_PACK_TEMP_1   B00
#define STORAGE_PACK_TEMP_2   B01
#define STORAGE_PACK_DATETIME B10
#define STORAGE_PACK_FREE     B11

int storageTemperatureCount;
boolean storageDateTimeWritten;

int storageSize;
int storageAddress;

void _storageSetup() {
  storageAddress = storageTemperatureCount = 0;
  storageDateTimeWritten = false;
}

// summedTemperatures holds for each sensor the summed readings.
// We have to divided it to the number of readings to get the average.

inline int storageUsed() {
  return storageTemperatureCount;
}

inline int storageFree() {
  return (storageSize - storageAddress) * 2 / 3 / NUMBER_OF_SENSORS;
}

void storageLog(DateTime dt, float *summedTemperatures, int numberOfTemperatures) {

  if ( ! storageDateTimeWritten && storageAddress + 3 <= storageSize ) {
    packDateTime(dt, &storageAddress);
    storageDateTimeWritten = true;
    loggerPrintln("Date and time saved");
  }

  loggerPrint(++storageTemperatureCount);
  loggerPrint(": ");

  int i = 0;
  while ( i < NUMBER_OF_SENSORS ) {

    if ( storageAddress + 3 > storageSize ) {
      error("Out of memory");
      return;
    }
    
    float temperature1 = summedTemperatures[i++] / numberOfTemperatures;
    loggerPrint(temperature1);
    loggerPrint(" ");    

    boolean first = storageRead(storageAddress) >> 6 != STORAGE_PACK_TEMP_1;
    if ( first && i < NUMBER_OF_SENSORS ) {
      // If we have another sensor reading we need to store and have space in this
      // slot, store it now. For EEPROM memory doesn't like to be written to often 
      // and also is slow on writes.
      
      float temperature2 = summedTemperatures[i++] / numberOfTemperatures;
      loggerPrint(temperature2);
      loggerPrint(" ");

      packTwoTemperatures(temperature1, temperature2, &storageAddress);
    }
    else {
      packTemperature(first, temperature1, &storageAddress);
    }
  }
  
  loggerPrintln("'C");
}

/////////////////////////////////////////////////////////////////////////

int storageReadAddress;
boolean storageReadFirst;

void storageReadInit() {
  storageReadAddress = 0;
  storageReadFirst = true;
}

boolean storageReadNext(DateTime *dt, float *temperatures) {

  int i = 0;
  
  while ( i < NUMBER_OF_SENSORS ) {
    int type = storageRead(storageReadAddress) >> 6;

    switch ( type ) {
    case STORAGE_PACK_FREE:
      return false; // no more data

    case STORAGE_PACK_DATETIME:
      *dt = unpackDateTime(&storageReadAddress);
      *dt = DateTime((*dt).unixtime() - SAMPLE_PERIOD_IN_MINUTES * 60);
      break;

    default:
      temperatures[i++] = unpackTemperature(storageReadFirst, &storageReadAddress);

      if ( type == STORAGE_PACK_TEMP_2 ) {
        storageReadFirst = ! storageReadFirst;
      }
    }
  }

  *dt = DateTime((*dt).unixtime() + SAMPLE_PERIOD_IN_MINUTES * 60);

  return true; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Routines the pack and unpack DateTime and temperature readings.

#define SECONDS_FROM_1970_TO_2013 1356998400L

void packDateTime(DateTime dt, int *address) {
  unsigned long t = (dt.unixtime() - SECONDS_FROM_1970_TO_2013) / (SAMPLE_PERIOD_IN_MINUTES * 60);

  storageWrite((*address)++, (STORAGE_PACK_DATETIME << 6) | (t >> 16 & B00111111));
  storageWrite((*address)++, t >> 8);
  storageWrite((*address)++, t); 
}

DateTime unpackDateTime(int *address) {
  unsigned long t = 0;

  t |= (unsigned long)(storageRead((*address)++) & B00111111) << 16;
  t |= (unsigned long)(storageRead((*address)++)) << 8;
  t |= (unsigned long)(storageRead((*address)++));

  return DateTime(t * SAMPLE_PERIOD_IN_MINUTES * 60 + SECONDS_FROM_1970_TO_2013);
}

void packTemperature(boolean first, float temperature, int *address) {
  unsigned long t = (temperature + 50.0) * 10.0;
  // t will be maximum (150 + 50) * 10 = 2000 < 2048 = 2^11

  // byte0: .. bit10---bit5
  // byte1: bit4--bit0 bit10-8
  // byte2: bit7--bit0

  if ( first ) {
    storageWrite(*address, (STORAGE_PACK_TEMP_1 << 6) | (t >> 5));
    storageWrite(*address + 1, (t & B00011111) << 3);
  }
  else {
    storageWrite(*address, (storageRead(*address) & B00111111) | (STORAGE_PACK_TEMP_2 << 6)); 
    (*address)++;
    storageWrite(*address, storageRead(*address) | t >> 8); 
    (*address)++;
    storageWrite(*address, t & B11111111); 
    (*address)++;
  }
}

void packTwoTemperatures(float temperature1, float temperature2, int *address) {
  unsigned long t1 = (temperature1 + 50.0) * 10.0;
  unsigned long t2 = (temperature2 + 50.0) * 10.0;
  // t will be maximum (150 + 50) * 10 = 2000 < 2048 = 2^11

  // byte0: .. bit10---bit5
  // byte1: bit4--bit0 bit10-8
  // byte2: bit7--bit0

  storageWrite((*address)++, (STORAGE_PACK_TEMP_2 << 6) | (t1 >> 5));
  storageWrite((*address)++, ((t1 & B00011111) << 3) | (t2 >> 8)); 
  storageWrite((*address)++, t2 & B11111111);
}

float unpackTemperature(boolean first, int *address) {
  unsigned long t;

  // byte0: .. bit10---bit5
  // byte1: bit4--bit0 bit10-8
  // byte2: bit7--bit0

  byte value0 = storageRead(*address);
  byte value1 = storageRead(*address + 1);

  if ( first ) {
    t = (value0 & B00111111) << 5 | (value1 >> 3); 
  }
  else {
    t = (value1 & B00000111) << 8 | storageRead(*address + 2);
  }

  if ( ! first || (value0 >> 6 == STORAGE_PACK_TEMP_1) ) {
    *address += 3;
  }

  return t / 10.0 - 50.0;
}

#endif /* else STORAGE_SDCARD */

#ifdef STORAGE_EEPROM

int randomOffsetAddress;

void storageErase() {
  int i;
  for ( i = 0; i < storageSize; i += 3 ) {
    if ( storageRead(i) >> 6 != STORAGE_PACK_FREE ) {
      storageWrite(i, STORAGE_PACK_FREE << 6);
    }
  }

  storageSetup();
}

void storageSetup() {
  _storageSetup();

  storageSize = (E2END + 1) / 3 * 3; // 1 KB for Uno

  randomOffsetAddress = random(storageSize / 3) * 3;
  boolean inside_prev = EEPROM.read((storageSize / 3 - 1) * 3) >> 6 != STORAGE_PACK_FREE;
  while ( storageAddress < storageSize ) {
    boolean inside_current = EEPROM.read(storageAddress) >> 6 != STORAGE_PACK_FREE;
    if ( ! inside_prev && inside_current ) {
      randomOffsetAddress = storageAddress;
      break;
    }
    inside_prev = inside_current;
    storageAddress += 3;
  }
  storageAddress = 0;
  loggerPrint("Random offset set to ");
  loggerPrintln(randomOffsetAddress);

  boolean scanning = true;

  while ( scanning && storageAddress + 3 <= storageSize ) {
    switch ( storageRead(storageAddress) >> 6 ) {
    case STORAGE_PACK_FREE:
      scanning = false;
      break;
    case STORAGE_PACK_TEMP_2:
      storageTemperatureCount++;
      // fall through
    case STORAGE_PACK_TEMP_1:
      storageTemperatureCount++;
      // fall through
    case STORAGE_PACK_DATETIME:
      storageAddress += 3;
      break;
    } 
  }

  loggerPrint("Loaded ");
  loggerPrint(storageTemperatureCount);
  loggerPrintln(" samples");  
}

void storageWrite(int address, byte value) {
//  loggerPrint(" [write addr: ");
//  loggerPrint(address);
//  loggerPrint(", value: ");
//  loggerPrint(value);
//  loggerPrintln("]");
  
  EEPROM.write((address + randomOffsetAddress) % storageSize, value);
}

byte storageRead(int address) {

  byte value = EEPROM.read((address + randomOffsetAddress) % storageSize);
  
//  loggerPrint(" [read addr: ");
//  loggerPrint(address);
//  loggerPrint(", value: ");
//  loggerPrint(value);
//  loggerPrintln("]");

  return value;
}

#endif /*STORAGE_EEPROM*/

#ifdef STORAGE_VOLATILE

byte *storageMemory = 0;

void storageErase() {
  _storageSetup();

  if ( storageMemory ) {
    memset(storageMemory, 0xff, storageSize);
  }
}

void storageSetup() {
  _storageSetup();

  storageSize = constrain(30, freeRam() - 500, 9999); // Try to reserve some bytes on the heap for other usage  
  if ( (storageMemory = (byte *)malloc(storageSize)) ) {
    memset(storageMemory, 0xff, storageSize);
  }
  else {
    error("Out of memory");
  }
}

int freeRam () {
  // cf http://playground.arduino.cc/Code/AvailableMemory
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

byte storageRead(int address) {
  byte value = storageMemory && address >=0 && address < storageSize ? storageMemory[address] : 0xff;

//  loggerPrint(" [read addr: ");
//  loggerPrint(address);
//  loggerPrint(", value: ");
//  loggerPrint(value);
//  loggerPrintln("]");

  return value;
}

void storageWrite(int address, byte value) {
//  loggerPrint(" [write addr: ");
//  loggerPrint(address);
//  loggerPrint(", value: ");
//  loggerPrint(value);
//  loggerPrintln("]");
  
  if ( storageMemory && address >=0 && address < storageSize ) {
    storageMemory[address] = value;
  }
}

#endif /*STORAGE_VOLATILE*/


//////////////////////////////////////////////////////////////////////////////////////////////////
// NOTE 1
//////////////////////////////////////////////////////////////////////////////////////////////////
//  In case you want to use violatile RAM as your storages, you do not want your program to reset
//  as soon as you enable the serial console. The prevent this, check out:
//
//  http://electronics.stackexchange.com/questions/24743/arduino-resetting-while-reconnecting-the-serial-terminal
//   
//  Connect a 22uF capacitor between RESET and GND (on the POWER header). You will need to 
//  disconnect it to program the board, but it doesn't involve breaking the board at all. 
//  Just plug it in between those two connections in the header, making sure the - side of 
//  the capacitor goes to ground. This works by holding the reset line high enough to stop 
//  the chip from resetting even when driven low by the USB chip. The reset switch should 
//  still work, but you may need to hold it in for a bit longer than normal.


