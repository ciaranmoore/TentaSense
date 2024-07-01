#include <Wire.h>
#include <SoftWire.h>
#include "AHT20.h"
#include "AHT201.h" // A hack to force AHT20 to use Wire1 interface.
#include "AHT202.h" // Another hack to force AHT20 to use SoftWire interface.
// #include <SensirionI2CSht4x.h>
#include <BleSerial.h> // From https://github.com/avinabmalla/ESP32_BleSerial.git
#include <ESP32Time.h>
#include <Arduino.h>

// SoftWire WireSW(4, 17);  // DEBUG


// #define BOARD_TYPE_PROGRAMMER
#define BOARD_TYPE_THING_PLUS
#define SENSOR1_ENABLED
#define SENSOR2_ENABLED
#define SENSOR3_ENABLED
// #define SHT45_SENSOR     // This is disabled by default. Will take some untangling to enable, as some code for SHT45 tied up in SENSOR3_ENABLED flags.
#define LED_ENABLED
#define BUTTONS_ENABLED
#define SERIAL_ENABLED // NB: Serial is always enabled to report setup messages, but can disable logging data from sensors to serial port with this flag.
#define BLE_ENABLED
#define SD_ENABLED
#define SAMPLE_PERIOD_S 10
#define BUTTON_DEBOUNCE_PERIOD_MS 100

#define BUILD_EPOCH __DATE__ __TIME__

// RTC code from https://www.theelectronics.co.in/2022/04/how-to-use-internal-rtc-of-esp32.html
//ESP32Time rtc;
ESP32Time rtc(0);  // offset in seconds: 43200 for GMT+12, but don't need this if using local __DATE__ and __TIME__ values.

#define MESSAGE_LENGTH 255 // Max length of BLE/sensor messages. Must be <= 255.

String strDeviceName  = "TENTASense-01";
char DeviceName[MESSAGE_LENGTH];

// Programmer board:
#ifdef BOARD_TYPE_PROGRAMMER
  #define HEARTBEAT 5
  #define SENSOR_VDD 32
  #define SENSOR_GND 33
  #define SCL_pin 25
  #define SDA_pin 26
#endif
#ifdef BOARD_TYPE_THING_PLUS 
  #define HEARTBEAT 13

  // Sensor 1 connected to  primary I2C bus (Wire)
  #define SCL_pin 22
  #define SDA_pin 21
  // Sensor 2 connected to secondary I2C bus (Wire1)
  #define SCL1_pin 33
  #define SDA1_pin 15
  // Sensor 3 connected to software I2C bus (WireSW)
  #define SCLSW_pin 17
  #define SDASW_pin 4 

  #define BUTTON_MONTH_pin 14 
  #define BUTTON_DAY_pin 32 
  #define BUTTON_HOUR_pin 27
  #define BUTTON_MINUTE_pin 12
  
  struct Button {
    const uint8_t pin;
    bool flag;
  };
  Button MonthButton = {BUTTON_MONTH_pin, false};
  Button DayButton = {BUTTON_DAY_pin, false};
  Button HourButton = {BUTTON_HOUR_pin, false};
  Button MinuteButton = {BUTTON_MINUTE_pin, false};

  #define SD_CS_PIN 5
#endif

#ifdef SENSOR1_ENABLED
  AHT20 AHT1;
#endif
#ifdef SENSOR2_ENABLED
  AHT201 AHT2;
#endif
#ifdef SENSOR3_ENABLED
  // SensirionI2CSht4x sht4x;
  SoftWire WireSW(SDASW_pin, SCLSW_pin);
  char swTxBuffer[MESSAGE_LENGTH];  // These buffers must be at least as large as the largest read or write you perform.
  char swRxBuffer[MESSAGE_LENGTH];
  AsyncDelay readInterval; // DEBUG: Do I need this?
  AHT202 AHT3;
#endif
#ifdef BLE_ENABLED
  BleSerial ble;
#endif
#ifdef SD_ENABLED
  #include "SD_helpers.h"
  bool SDWorking = true;
  char SDlogFile[MESSAGE_LENGTH];
  String strSDlogFile;
#endif

// Define hardware timers:
#define mmentPrescalar 64000
#define mmentPeriod (1250 * SAMPLE_PERIOD_S) // 80 MHz / 64000 / 1250 = 1 s
hw_timer_t * mmentTimer = NULL;
volatile bool mmentTimerFlag = false;

void ARDUINO_ISR_ATTR mmentTimerISR(){
  // xSemaphoreGiveFromISR(mmentSemaphore, NULL);
  mmentTimerFlag = true;
}
#define buttonPrescalar 80
#define buttonDebouncePeriod (1000 * BUTTON_DEBOUNCE_PERIOD_MS) // 80 MHz / 80 / 1000 = 1 ms
hw_timer_t * buttonTimer = NULL;
volatile bool buttonTimerFlag = false;

void ARDUINO_ISR_ATTR buttonTimerISR(){
  buttonTimerFlag = true;
}

void IRAM_ATTR MonthAdjISR() {
// void ARDUINO_ISR_ATTR MonthAdjISR() {
    MonthButton.flag = true;
}
void IRAM_ATTR DayAdjISR() {
  DayButton.flag = true;
}
void IRAM_ATTR HourAdjISR() {
  HourButton.flag = true;
}
void IRAM_ATTR MinuteAdjISR() {
  MinuteButton.flag = true;
}

void setup() {
  // Find Date and Time:
  // Use compile date and time as a reference for RTC module:
  // __DATE__ gives MMM DD YYYY, __TIME__ give HH:MM:SS. Start counting characters from 0.
  int build_year = ((BUILD_EPOCH[7] - '0') * 1000 + (BUILD_EPOCH[8] - '0') * 100 + (BUILD_EPOCH[9] - '0') * 10 + (BUILD_EPOCH[10] - '0'));
  int build_month = ( \
          (BUILD_EPOCH[0] == 'J' && BUILD_EPOCH[1] == 'a')  ? 1 : \
          (BUILD_EPOCH[0] == 'F')                           ? 2 : \
          (BUILD_EPOCH[0] == 'M' && BUILD_EPOCH[2] == 'r')  ? 3 : \
          (BUILD_EPOCH[1] == 'p')                           ? 4 : \
          (BUILD_EPOCH[2] == 'y')                           ? 5 : \
          (BUILD_EPOCH[1] == 'u' && BUILD_EPOCH[2] == 'n')  ? 6 : \
          (BUILD_EPOCH[2] == 'l')                           ? 7 : \
          (BUILD_EPOCH[2] == 'g')                           ? 8 : \
          (BUILD_EPOCH[0] == 'S')                           ? 9 : \
          (BUILD_EPOCH[0] == 'O')                           ? 10 : \
          (BUILD_EPOCH[0] == 'N')                           ? 11 : \
          (BUILD_EPOCH[0] == 'D')                           ? 12 : 0);
  // First character of day (i.e. BUILD_EPOCH[4]) may be '1', '2', '3' or ' '. 
  int build_day;
  if (BUILD_EPOCH[4] == ' ')
    build_day = (BUILD_EPOCH[5] - '0');
  else
    build_day =  ((BUILD_EPOCH[4] -'0') * 10 + (BUILD_EPOCH[5] - '0'));
  int build_hour = ((BUILD_EPOCH[11] - '0') * 10 + (BUILD_EPOCH[12] - '0'));
  int build_min = ((BUILD_EPOCH[14] - '0') * 10 + (BUILD_EPOCH[15] - '0'));
  int build_sec = ((BUILD_EPOCH[17] - '0') * 10 + (BUILD_EPOCH[18] - '0'));
  rtc.setTime(build_sec, build_min, build_hour, build_day, build_month, build_year);

  String strLogFileName = strDeviceName + " Log\n";
  #ifdef SENSOR1_ENABLED
    Wire.begin(SDA_pin, SCL_pin);  
    AHT1.begin();
  #endif
  #ifdef SENSOR2_ENABLED
    Wire1.begin(SDA1_pin, SCL1_pin);
    AHT2.begin();
  #endif
  #ifdef SENSOR3_ENABLED
    WireSW.setTxBuffer(swTxBuffer, sizeof(swTxBuffer));
    WireSW.setRxBuffer(swRxBuffer, sizeof(swRxBuffer));
     
    WireSW.begin();
    AHT3.begin();
  // sht4x.begin(Wire);
//  AsyncDelay samplingInterval;
  #endif
  #ifdef BUTTONS_ENABLED
    pinMode(MonthButton.pin, INPUT_PULLUP);
    pinMode(DayButton.pin, INPUT_PULLUP);
    pinMode(HourButton.pin, INPUT_PULLUP);
    pinMode(MinuteButton.pin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(MonthButton.pin), MonthAdjISR, FALLING); // FALLING  // DEBUG
    attachInterrupt(digitalPinToInterrupt(DayButton.pin), DayAdjISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(HourButton.pin), HourAdjISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(MinuteButton.pin), MinuteAdjISR, FALLING);
    
  #endif
  #ifdef LED_ENABLED
    pinMode(HEARTBEAT, OUTPUT);
  #endif
  
  strDeviceName.toCharArray(DeviceName, MESSAGE_LENGTH);

  Serial.begin(115200);
  // Start BLE Serial connection
  #ifdef BLE_ENABLED
    ble.begin(DeviceName);
  #endif
  #ifdef SD_ENABLED
    strSDlogFile = "/" + strDeviceName + "_log.txt";
    strSDlogFile.toCharArray(SDlogFile, MESSAGE_LENGTH);
  
    if(!SD.begin(SD_CS_PIN)){
      Serial.println("Card Mount Failed");
      SDWorking = false;
    } else {
      uint8_t cardType = SD.cardType();

      if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        SDWorking = false;
      } else {
        Serial.print("SD Card Type: ");
        if(cardType == CARD_MMC){
            Serial.println("MMC");
        } else if(cardType == CARD_SD){
            Serial.println("SDSC");
        } else if(cardType == CARD_SDHC){
            Serial.println("SDHC");
        } else {
            Serial.println("UNKNOWN");
        }

        uint64_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.printf("SD Card Size: %lluMB\n", cardSize);
        writeFile(SD, SDlogFile, SDlogFile);      // Write filename as first line in file.
        writeFile(SD, SDlogFile, "\n");
        SDWorking = true;
      }
    }
  #endif
  
  // Use 0th timer of 4-1. Base frequency = 80 MHz by default. Prescalar = 80 gives 1 MHz timer.
  // Max prescalar = 65536 -> 1220.7 kHz min clock: Choose prescalar = 64000 for 1.25 kHz timer. 
  mmentTimer = timerBegin(0, mmentPrescalar, true);
  timerAttachInterrupt(mmentTimer, &mmentTimerISR, true); // timer, ISR, autoreload
  // period (s) = 80 MHz / mmentPrescalar / mmentPeriod
  // For 1 s period: mmentPrescalar = 80, mmentPeriod = 1000000
  //              or mmentPrescalr = 64000, mmentPeriod = 1250
  timerAlarmWrite(mmentTimer, mmentPeriod, true); 
  timerAlarmEnable(mmentTimer);

  // Use 1th time of 4-1 for button debouncing:
  // Want 10 ms debounce
  buttonTimer = timerBegin(1, buttonPrescalar, true);
  timerAttachInterrupt(buttonTimer, &buttonTimerISR, true); // timer, ISR, autorelaod
  // DebouncePeriod (s) = 80 MHz / buttonPrescalar / buttonDebouncePeriod
  // For 10 ms debounce: buttonPrescalar = 80, buttonDebouncePeriod = 10000
  //                  or buttonPrescalar = 64000, buttonDebouncePeriod = 12.5
  timerAlarmWrite(buttonTimer, buttonDebouncePeriod, true);
  timerAlarmEnable(buttonTimer);

  // esp_sleep_enable_timer_wakeup(SAMPLE_PERIOD_S * US_PER_S);
  //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);

  Serial.println("Time set to: " + rtc.getTime("%d%m%y%H%M"));
  Serial.println("ESP32-WROOM-32 programmed!");
}
void loop() {
  static float AHThumi, AHTtemp;
  static float SHThumi, SHTtemp;

  static int AHTret;
  static uint16_t SHTerror;
  static char SHTerrorMessage[MESSAGE_LENGTH];
  static String strTime;
  static String strMeasurement;
  static String strTimeUpdate;
  static char measurementMessage[MESSAGE_LENGTH];
  static char timeUpdateMessage[MESSAGE_LENGTH];
  static bool TimeChanged = false;
  struct tm time;

  if (buttonTimerFlag == true) {
    // Serial.println(MonthButton.flag + DayButton.flag + HourButton.flag + MinuteButton.flag); // DEBUG
    if ((MonthButton.flag == true) && (TimeChanged == false)) {
      TimeChanged = true;
      time = rtc.getTimeStruct();
      time.tm_mon = time.tm_mon + 1;
      rtc.setTimeStruct(time);
      MonthButton.flag = false;
    }
    if ((DayButton.flag == true)  && (TimeChanged == false)) {
      TimeChanged = true;
      time = rtc.getTimeStruct();
      time.tm_mday = time.tm_mday + 1;
      rtc.setTimeStruct(time);
      DayButton.flag = false;
    }
    if ((HourButton.flag == true)  && (TimeChanged == false)) {
      TimeChanged = true;
      time = rtc.getTimeStruct();
      time.tm_hour = time.tm_hour + 1;
      rtc.setTimeStruct(time);
      HourButton.flag = false;
    }
    if ((MinuteButton.flag == true)  && (TimeChanged == false)) {
      TimeChanged = true;
      Serial.println("Minute++"); // DEBUG
      time = rtc.getTimeStruct();
      time.tm_min = time.tm_min + 1;
      rtc.setTimeStruct(time);
      MinuteButton.flag = false;
    }

    // Print new date & time
    if (TimeChanged == true) {
      strTime = rtc.getTime("%d%m%H%M"); 
      strTimeUpdate = "Date and time updated to " + strTime;
      strTimeUpdate.toCharArray(timeUpdateMessage, MESSAGE_LENGTH);
      #ifdef SERIAL_ENABLED
        Serial.println(strTimeUpdate);
      #endif
      #ifdef BLE_ENABLED
        ble.print(timeUpdateMessage);
        ble.print(" ");
      #endif
      #ifdef SD_ENABLED
        if (SDWorking) {
          appendFile(SD, SDlogFile, measurementMessage);
          appendFile(SD, SDlogFile, "\n");
        }
      #endif
      TimeChanged = false;
    }
    buttonTimerFlag = false;
  }
  // if mmentTimer has fired:
  if (mmentTimerFlag == 1) {
    mmentTimerFlag = 0;
    #ifdef LED_ENABLED
      digitalWrite(HEARTBEAT, HIGH);
    #endif
    // Sensor working:
    #ifdef SENSOR1_ENABLED
      AHTret = AHT1.getSensor(&AHThumi, &AHTtemp);
      if (AHTret) {
        strTime = rtc.getTime("%d%m%H%M"); 
        strMeasurement = strTime + "S1H" + AHThumi * 100 + "T" + AHTtemp;
        strMeasurement.toCharArray(measurementMessage, MESSAGE_LENGTH);
        #ifdef SERIAL_ENABLED
          Serial.println(strMeasurement);
        #endif
        #ifdef BLE_ENABLED
          ble.print(measurementMessage);
        #endif
        #ifdef SD_ENABLED
          if (SDWorking) {
            appendFile(SD, SDlogFile, measurementMessage);
            appendFile(SD, SDlogFile, "\n");
          }
        #endif
      } else  {
        #ifdef SERIAL_ENABLED
          Serial.println("GET DATA FROM AHT20 #1 FAIL");
        #endif
        #ifdef BLE_ENABLED
        strTime = rtc.getTime("%d%m%H%M"); 
        strMeasurement = strTime + "S1H" + "FailTFail";
        strMeasurement.toCharArray(measurementMessage, MESSAGE_LENGTH);
          ble.print(measurementMessage);
        #endif
        #ifdef SD_ENABLED
          if (SDWorking)
            appendFile(SD, SDlogFile, "GET DATA FROM AHT20 #1 FAIL\n");
        #endif
      }
    #endif
    #ifdef SENSOR2_ENABLED
      AHTret = AHT2.getSensor(&AHThumi, &AHTtemp);
      if (AHTret) {
        strTime = rtc.getTime("%d%m%H%M"); 
        strMeasurement = strTime + "S2H" + AHThumi * 100 + "T" + AHTtemp;
        strMeasurement.toCharArray(measurementMessage, MESSAGE_LENGTH);
        #ifdef SERIAL_ENABLED
          Serial.println(strMeasurement);
        #endif
        #ifdef BLE_ENABLED
          ble.print(measurementMessage);
        #endif
        #ifdef SD_ENABLED
          if (SDWorking) {
            appendFile(SD, SDlogFile, measurementMessage);
            appendFile(SD, SDlogFile, "\n");
          }
        #endif
      } else  {
        #ifdef SERIAL_ENABLED
          Serial.println("GET DATA FROM AHT20 #2 FAIL");
        #endif
        #ifdef BLE_ENABLED
          ble.print("GET DATA FROM AHT20 #2 FAIL");
        #endif  
        #ifdef SD_ENABLED
          if (SDWorking)
            appendFile(SD, SDlogFile, "GET DATA FROM AHT20 #2 FAIL\n");
        #endif
      }
    #endif
    #ifdef SENSOR3_ENABLED
      AHTret = AHT3.getSensor(&AHThumi, &AHTtemp);
      if (AHTret) {
        strTime = rtc.getTime("%d%m%H%M"); 
        strMeasurement = strTime + "S3H" + AHThumi * 100 + "T" + AHTtemp;
        strMeasurement.toCharArray(measurementMessage, MESSAGE_LENGTH);
        #ifdef SERIAL_ENABLED
          Serial.println(strMeasurement);
        #endif
        #ifdef BLE_ENABLED
          ble.print(measurementMessage);
        #endif
        #ifdef SD_ENABLED
          if (SDWorking) {
            appendFile(SD, SDlogFile, measurementMessage);
            appendFile(SD, SDlogFile, "\n");
          }
        #endif
      } else  {
        #ifdef SERIAL_ENABLED
          Serial.println("GET DATA FROM AHT20 #3 FAIL");
        #endif
        #ifdef BLE_ENABLED
          ble.print("GET DATA FROM AHT20 #3 FAIL\n");
        #endif  
        #ifdef SD_ENABLED
          if (SDWorking)
            appendFile(SD, SDlogFile, "GET DATA FROM AHT20 #3 FAIL\n");
        #endif
      }
    #endif
    

    #ifdef SHT45_SENSOR
      SHTerror = sht4x.measureMediumPrecision(SHTtemp, SHThumi);
      if (SHTerror) {
        errorToString(SHTerror, SHTerrorMessage, 256);
        #ifdef SERIAL_ENABLED
          Serial.println("GET DATA FROM SHT45 FAIL");
          Serial.println(SHTerrorMessage);
        #endif
        #ifdef BLE_ENABLED
          ble.print("GET DATA FROM SHT45 FAIL");
          ble.print(SHTerrorMessage);
        #endif
        #ifdef SD_ENABLED
          if (SDWorking) {
            appendFile(SD, SDlogFile, "GET DATA FROM SHT45 FAIL\n");
            appendFile(SD, SDlogFile, SHTerrorMessage);
          }
        #endif
      } else {
        strTime = rtc.getTime("%d%m%H%M"); 
        strMeasurement = strTime + "S3H" + SHThumi + "T" + SHTtemp;
              strMeasurement.toCharArray(measurementMessage, MESSAGE_LENGTH);
        #ifdef SERIAL_ENABLED
          Serial.println(strMeasurement);
        #endif
        #ifdef BLE_ENABLED
          ble.print(measurementMessage);
      //    ble.print("\n");
        #endif
        #ifdef SD_ENABLED
          if (SDWorking) {
            appendFile(SD, SDlogFile, measurementMessage);
            appendFile(SD, SDlogFile, "\n");
          }
        #endif
      }
    #endif

    #ifdef SERIAL_ENABLED
      Serial.println();
    #endif
    #ifdef BLE_ENABLED
      ble.print(" ");
    #endif
    #ifdef SD_ENABLED
      if (SDWorking)
        appendFile(SD, SDlogFile, "\n");
    #endif
    #ifdef LED_ENABLED
      digitalWrite(HEARTBEAT, LOW);
    #endif
  }

  // Store data in circular buffer:

}