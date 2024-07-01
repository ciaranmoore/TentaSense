#include <Wire.h>
#include "AHT20.h"
#include "AHT201.h" // A hack to force AHT20 to use Wire1 interface.
#include <SensirionI2CSht4x.h>
#include <BleSerial.h> // From https://github.com/avinabmalla/ESP32_BleSerial.git
#include <ESP32Time.h>
#include <Arduino.h>

// #define BOARD_TYPE_PROGRAMMER
#define BOARD_TYPE_THING_PLUS
#define SERIAL_ENABLED // NB: Serial is always enabled to report setup messages, but can disable logging data from sensors to serial port with this flag.
#define BLE_ENABLED
#define SD_ENABLED
#define LED_ENABLED
#define SAMPLE_PERIOD_S 10

// Needed for SD card:
#ifdef SD_ENABLED
  #include "SD_helpers.h"
#endif

AHT201 AHT1;
AHT20 AHT2;
SensirionI2CSht4x sht4x;
BleSerial ble;

bool SDWorking = true;

// RTC code from https://www.theelectronics.co.in/2022/04/how-to-use-internal-rtc-of-esp32.html
//ESP32Time rtc;
ESP32Time rtc(0);  // offset in seconds: 43200 for GMT+12, but don't need this if using local __DATE__ and __TIME__ values.

#define MESSAGE_LENGTH 256 // Max length of BLE/sensor messages

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
  #define SENSOR1_VDD 32 // Connected to secondary I2C bus (Wire1)
  #define SENSOR1_GND 12
  #define SENSOR2_VDD 14 // Connected to primary I2C bus (Wire)
  #define SENSOR3_VDD 15 // Connected to primary I2C bus (Wire)
  #define SCL_pin 22
  #define SDA_pin 21
  #define SCL1_pin 27
  #define SDA1_pin 33
  #define SD_CS_PIN 5
#endif

// Use compile date and time as a reference for RTC module:
// __DATE__ gives MMM DD YYYY, __TIME__ give HH:MM:SS. Start counting characters from 0.
#define BUILD_EPOCH __DATE__ __TIME__
#define BUILD_YEAR ((BUILD_EPOCH[7] - '0') * 1000 + (BUILD_EPOCH[8] - '0') * 100 + (BUILD_EPOCH[9] - '0') * 10 + (BUILD_EPOCH[10] - '0'))
#define BUILD_MONTH ( \
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
        (BUILD_EPOCH[0] == 'D')                           ? 12 : 0)
#define BUILD_DAY  ((BUILD_EPOCH[4] -'0') * 10 + (BUILD_EPOCH[5] - '0'))
#define BUILD_HOUR ((BUILD_EPOCH[11] - '0') * 10 + (BUILD_EPOCH[12] - '0'))
#define BUILD_MIN  ((BUILD_EPOCH[14] - '0') * 10 + (BUILD_EPOCH[15] - '0'))
#define BUILD_SEC  ((BUILD_EPOCH[17] - '0') * 10 + (BUILD_EPOCH[18] - '0'))

// Define hardware timer:
#define mmentPrescalar 64000
#define mmentPeriod (1250 * SAMPLE_PERIOD_S) // 80 MHz / 64000 / 1250 = 1 s
hw_timer_t * mmentTimer = NULL;
// volatile SemaphoreHandle_t mmentSemaphore;
volatile bool mmentTimerFlag = 0;
// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void ARDUINO_ISR_ATTR mmentTimerISR(){
  // xSemaphoreGiveFromISR(mmentSemaphore, NULL);
  mmentTimerFlag = 1;
}

#ifdef SD_ENABLED
  char SDlogFile[MESSAGE_LENGTH];
  String strSDlogFile;
#endif

void setup() {
  pinMode(HEARTBEAT, OUTPUT);
  pinMode(SENSOR1_VDD, OUTPUT);
  pinMode(SENSOR1_GND, OUTPUT);
  pinMode(SENSOR2_VDD, OUTPUT);
  pinMode(SENSOR3_VDD, OUTPUT);
  
  digitalWrite(SENSOR1_GND, LOW);
  digitalWrite(SENSOR1_VDD, HIGH);
  digitalWrite(SENSOR2_VDD, HIGH);
  digitalWrite(SENSOR3_VDD, HIGH);
  
  Wire.begin(SDA_pin, SCL_pin);
  Wire1.begin(SDA1_pin, SCL1_pin);
  AHT1.begin();
  AHT2.begin();
  sht4x.begin(Wire);

  // Start wired serial connection
  Serial.begin(115200);
  // Start BLE Serial connection
  #ifdef BLE_ENABLED
    ble.begin("TENTAsense-01");
  #endif

  #ifdef SD_ENABLED
    strSDlogFile = "/TENTAsense-01_log.txt";
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
        writeFile(SD, SDlogFile, "TENTAsense-01 Log\n");
        SDWorking = true;
      }
    }
  #endif
  
  rtc.setTime(BUILD_SEC, BUILD_MIN, BUILD_HOUR, BUILD_DAY, BUILD_MONTH, BUILD_YEAR);

  // Use 0th timer of 4-1. Base frequency = 80 MHz by default. Prescalar = 80 gives 1 MHz timer.
  // Max prescalar = 65536 -> 1220.7 kHz min clock: Choose prescalar = 64000 for 1.25 kHz timer. 
  mmentTimer = timerBegin(0, mmentPrescalar, true);
  timerAttachInterrupt(mmentTimer, &mmentTimerISR, true); // timer, ISR, autoreload
  // period (s) = 80 MHz / mmentPrescalar / mmentPeriod
  // For 1 s period: mmentPrescalar = 80, mmentPeriod = 1000000
  //              or mmentPrescalr = 64000, mmentPeriod = 1250
  timerAlarmWrite(mmentTimer, mmentPeriod, true); 
  timerAlarmEnable(mmentTimer);

  // Serial working:
  Serial.println("ESP32-WROOM-32 was successfully programmed!");
}
void loop() {
  static float AHThumi, AHTtemp;
  static float SHThumi, SHTtemp;

  static int AHTret;
  static uint16_t SHTerror;
  static char SHTerrorMessage[MESSAGE_LENGTH];
  static String strTime;
  static String strMeasurement;
  static char measurementMessage[MESSAGE_LENGTH];

  // GPIO working:
  // if mmentTimer has fired:
  if (mmentTimerFlag == 1) {
    mmentTimerFlag = 0;
    #ifdef LED_ENABLED
      digitalWrite(HEARTBEAT, HIGH);
    #endif
    // Sensor working:
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
        if (SDWorking)
          appendFile(SD, SDlogFile, measurementMessage);
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
        if (SDWorking)
          appendFile(SD, SDlogFile, measurementMessage);
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
        if (SDWorking)
          appendFile(SD, SDlogFile, measurementMessage);
      #endif
    }

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