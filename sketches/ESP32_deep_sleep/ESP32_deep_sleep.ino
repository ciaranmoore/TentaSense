#include <Arduino.h>
#include <Wire.h>
#include <SoftWire.h>
#include "AHT20.h"  // Used by Sensor 1.
#include "AHT201.h" // A hack to force AHT20 to use Wire1 interface. Used by Sensor 2.
#include "AHT202.h" // Another hack to force AHT20 to use SoftWire interface. Used by Sensor 3.
#include <BleSerial.h> // From https://github.com/avinabmalla/ESP32_BleSerial.git
#include "SD_helpers.h"
#include <ESP32Time.h>
#include "driver/rtc_io.h" // Needed for rtc_gpio_pulldown_en()

#define DEBUG
#define SERIAL_RATE 115200
#define DEVICE_NAME "TENTASense-12-F"
#define ARB_BUILD_TIME
#ifdef ARB_BUILD_TIME
  #define ARB_BUILD_SEC 0
  #define ARB_BUILD_MIN 0
  #define ARB_BUILD_HOUR 9
  #define ARB_BUILD_DAY 1
  #define ARB_BUILD_MONTH 11
  #define ARB_BUILD_YEAR 2023
#endif


// #define BOARD_TYPE_PROGRAMMER
#define BOARD_TYPE_THING_PLUS

#define SAMPLE_PERIOD_BT_S 10         // Period when sending samples to bluetooth in 'configure' mode.
#define SAMPLE_PERIOD_S 3600      // Period when sending samples to SD card in 'running' mode.
#define MS_PER_S 1000
#define US_PER_S 1000000
#define BUTTON_DEBOUNCE_PERIOD_MS 200
#define MAX_MMENT_COUNT_BEFORE_SLEEP 5

// Programmer board:
#ifdef BOARD_TYPE_PROGRAMMER
  // Allows for alternative pin definitions/routing.
#endif
#ifdef BOARD_TYPE_THING_PLUS 
  // Sensor 1 connected to  primary I2C bus (Wire)
  #define SCL1_pin GPIO_NUM_21
  #define SDA1_pin GPIO_NUM_4
  // Sensor 2 connected to secondary I2C bus (Wire1)
  #define SCL2_pin GPIO_NUM_22
  #define SDA2_pin GPIO_NUM_17
  // Sensor 3 connected to software I2C bus (WireSW)
  #define SCL3_pin GPIO_NUM_14
  #define SDA3_pin GPIO_NUM_16

  #define BATT_LVL_pin A0
  #define LED_pin GPIO_NUM_13
  #define RGB_LED_pin GPIO_NUM_2

  #define BUTTON_MONTH_pin GPIO_NUM_15 
  #define BUTTON_DAY_pin GPIO_NUM_33
  #define BUTTON_HOUR_pin GPIO_NUM_27
  #define BUTTON_MINUTE_pin GPIO_NUM_12
  #define WAKE_BUTTON_MASK ((1 << BUTTON_MINUTE_pin))

  #define SD_CS_PIN GPIO_NUM_5
#endif

// Provision Sensors:
uint32_t battery_level;
AHT20 AHT1;
AHT201 AHT2;
SoftWire WireSW(SDA3_pin, SCL3_pin);
#define MESSAGE_LENGTH 255 // Max length of BLE/sensor messages. Must be <= 255.
char swTxBuffer[MESSAGE_LENGTH];
char swRxBuffer[MESSAGE_LENGTH];
// AsyncDelay readInterval; // DEBUG: Do I need this?
AHT202 AHT3;
float AHThumi, AHTtemp;
int AHTret;

// Bluetooth:
BleSerial ble;

// SD card:
bool SDWorking; 
char SDlogFile[MESSAGE_LENGTH]; 
String strSDlogFile;

// Buttons to change date and time:
struct Button {
  const uint8_t pin;
  volatile bool flag;
};
volatile Button MonthButton = {BUTTON_MONTH_pin, false};
volatile Button DayButton = {BUTTON_DAY_pin, false};
volatile Button HourButton = {BUTTON_HOUR_pin, false};
volatile Button MinuteButton = {BUTTON_MINUTE_pin, false};

// Record time when code is compiled:
#define BUILD_EPOCH __DATE__ __TIME__
// RTC code from https://www.theelectronics.co.in/2022/04/how-to-use-internal-rtc-of-esp32.html
ESP32Time rtc(0);  // offset in seconds: 43200 for GMT+12, but don't need this if using local __DATE__ and __TIME__ values.
static bool TimeChanged = false;
static String strDeviceName  = DEVICE_NAME;
char DeviceName[MESSAGE_LENGTH];
String strBattery;
char batteryMessage[MESSAGE_LENGTH];
String strMeasurement;
char measurementMessage[MESSAGE_LENGTH];
String strTime;
String strTimeUpdate;
char timeUpdateMessage[MESSAGE_LENGTH];
String  strLullaby;
char lullabyMessage[MESSAGE_LENGTH];


// Define hardware timers: two timer groups, each group has two timers. 
// All timers have 64 bits counters, 16 bit prescalars. Prescalars can have any value from 2 to 65536.
// Un-scaled timer frequency is 80 MHz. 
#define APB_CLK 80000000
#define MMENT_PRESCALAR 64000
#define MMENT_PERIOD_BT_S (APB_CLK / MMENT_PRESCALAR * SAMPLE_PERIOD_BT_S)
hw_timer_t * mmentTimer = NULL;
volatile bool mmentTimerFlag = false;
volatile uint8_t mmentTimerCount;

void ARDUINO_ISR_ATTR mmentTimerISR(){
  mmentTimerCount--;
  mmentTimerFlag = true;
}
#define buttonPrescalar 80
#define buttonDebouncePeriod (APB_CLK / buttonPrescalar * BUTTON_DEBOUNCE_PERIOD_MS / MS_PER_S)
hw_timer_t * buttonTimer = NULL;
volatile bool buttonTimerFlag = false;

void ARDUINO_ISR_ATTR buttonTimerISR(){
  buttonTimerFlag = true;
}

void ARDUINO_ISR_ATTR MonthAdjISR() {
    MonthButton.flag = true;
}
void ARDUINO_ISR_ATTR DayAdjISR() {
  DayButton.flag = true;
} 
void ARDUINO_ISR_ATTR HourAdjISR() {
  HourButton.flag = true;
}
void ARDUINO_ISR_ATTR MinuteAdjISR() {
  MinuteButton.flag = true;
}

void setup() {
  uint8_t cardType;
  Serial.begin(SERIAL_RATE);

  pinMode(BATT_LVL_pin, INPUT);
  // uint32_t battery_level = analogReadMilliVolts(BATT_LVL_pin);
  // if (battery_level < BATT_LOW_LVL_mV) {
  //   #ifdef DEBUG
  //     Serial.print("Battery level is ");
  //     Serial.print(battery_level);
  //     Serial.print(" mV\n");
  //   #endif
  //   pinMode(LED_pin, OUTPUT);
  //   #ifdef DEBUG
  //     Serial.println("Low battery");
  //   #endif
  //   for (uint8_t i = 0; i < 5; i++) {
  //     digitalWrite(LED_pin, HIGH);
  //     delay(200);
  //     digitalWrite(LED_pin, LOW);
  //     delay(800);
  //   }
  //   // Go back to sleep
  //   rtc_gpio_pullup_dis(BUTTON_MINUTE_pin);
  //   rtc_gpio_pulldown_en(BUTTON_MINUTE_pin);
  //   esp_sleep_enable_ext0_wakeup(BUTTON_MINUTE_pin, HIGH);
  //   ESP.deepSleep(LOW_BATT_PERIOD_S * US_PER_S);
  // }

  // Initialise sensors:
  Wire.begin(SDA1_pin, SCL1_pin);
  AHT1.begin();
  Wire1.begin(SDA2_pin, SCL2_pin);
  AHT2.begin();
  WireSW.setTxBuffer(swTxBuffer, sizeof(swTxBuffer));
  WireSW.setRxBuffer(swTxBuffer, sizeof(swRxBuffer));
  WireSW.begin();
  AHT3.begin();
  
  // Initialise SD card:
  if(!SD.begin(SD_CS_PIN)){
    Serial.println("Card Mount Failed");
    // pinMode(RGB_LED_pin, OUTPUT);
    // digitalWrite(RGB_LED_pin, HIGH);
    // delay(500);
    // digitalWrite(RGB_LED_pin, LOW);
    SDWorking = false;
  } else {
    cardType = SD.cardType();
    if(cardType == CARD_NONE) SDWorking = false;
    else SDWorking = true;
  }
  // Set up SD card:
  // strTime = rtc.getTime("%d%m%H%M");
  strSDlogFile = "/";
  strSDlogFile = strSDlogFile + strDeviceName;
  // strSDlogFile = strSDlogFile + "_";
  // strSDlogFile = strSDlogFile + strTime;
  strSDlogFile = strSDlogFile + ".txt";
  strSDlogFile.toCharArray(SDlogFile, MESSAGE_LENGTH);

  // Set up device name, needed by BT.
  strDeviceName.toCharArray(DeviceName, MESSAGE_LENGTH);

  // Check wake-up reason
  esp_sleep_wakeup_cause_t wake_up_source = esp_sleep_get_wakeup_cause();
  switch (wake_up_source) {
    // Running mode: log new measurements, go back to sleep 
    case ESP_SLEEP_WAKEUP_TIMER:    
      #ifdef DEBUG
        Serial.println("Waking up for scheduled measurement.");
      #endif

      if(!SD.begin(SD_CS_PIN)){
        Serial.println("Card Mount Failed");
        // pinMode(RGB_LED_pin, OUTPUT);
        // digitalWrite(RGB_LED_pin, HIGH);
        // delay(500);
        // digitalWrite(RGB_LED_pin, LOW);
        SDWorking = false;
      } else {
        cardType = SD.cardType();
        if(cardType == CARD_NONE){
          Serial.println("No SD card attached");
          SDWorking = false;
        } else { // SDWorking
          strTime = rtc.getTime("%d%m%H%M");
          // Read sensors & log to SD:
          battery_level = analogReadMilliVolts(BATT_LVL_pin);
          strBattery = "Battery: ";
          strBattery = strBattery + battery_level + " mV\n";
          strBattery.toCharArray(batteryMessage, MESSAGE_LENGTH);
          appendFile(SD, SDlogFile, batteryMessage);
          // appendFile(SD, SDlogFile, "\n");
          #ifdef DEBUG
            Serial.print(strBattery);
          #endif
          AHTret = AHT1.getSensor(&AHThumi, &AHTtemp);
          if (AHTret) {
            strMeasurement = strTime + " S1 H:" + AHThumi * 100 + " T: " + AHTtemp + "\n";
            strMeasurement.toCharArray(measurementMessage, MESSAGE_LENGTH);
            appendFile(SD, SDlogFile, measurementMessage);        
            #ifdef DEBUG
              Serial.print(strMeasurement);
            #endif
          }
          else {
            #ifdef DEBUG
              Serial.print("S1 fail.");
            #endif
          }
          AHTret = AHT2.getSensor(&AHThumi, &AHTtemp);
          if (AHTret) {
            strMeasurement = strTime + " S2 H:" + AHThumi * 100 + " T: " + AHTtemp + "\n";
            strMeasurement.toCharArray(measurementMessage, MESSAGE_LENGTH);
            appendFile(SD, SDlogFile, measurementMessage);
            #ifdef DEBUG
              Serial.print(strMeasurement);
            #endif
          }
          else {
            #ifdef DEBUG
              Serial.print("S2 fail.");
            #endif
          }
          AHTret = AHT3.getSensor(&AHThumi, &AHTtemp);
          if ((AHTret) && (AHTtemp >= -49)) {   // S3 uses WireSW bus, which sends erroneous data (T=-50) if no sensor connected.
            strMeasurement = strTime + " S3 H:" + AHThumi * 100 + " T: " + AHTtemp + "\n";
            strMeasurement.toCharArray(measurementMessage, MESSAGE_LENGTH);
            appendFile(SD, SDlogFile, measurementMessage);
            #ifdef DEBUG
              Serial.print(strMeasurement);
            #endif
          }
          else {
            #ifdef DEBUG
              Serial.print("S3 fail.");
            #endif
          }
        }
      }
      // Go back to sleep
      #ifdef DEBUG
        Serial.println("Scheduled measurement complete. Going to sleep.");
        Serial.flush();
        Serial.end(true);
      #endif

      Wire.end();
      Wire1.end();
      WireSW.end();
      SD.end();
      rtc_gpio_pullup_dis(BUTTON_MINUTE_pin);
      rtc_gpio_pulldown_en(BUTTON_MINUTE_pin);
      esp_sleep_enable_ext0_wakeup(BUTTON_MINUTE_pin, HIGH);
      ESP.deepSleep(SAMPLE_PERIOD_S * US_PER_S);
    break; 
    
    // Reset not caused by exit from deep sleep: i.e. fresh code or power cycle.
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    {
      #ifdef DEBUG
        Serial.println("Initialising real-time clock and creating new log file.");
      #endif
      // Find Date and Time:
      #ifdef ARB_BUILD_TIME
        rtc.setTime(ARB_BUILD_SEC, ARB_BUILD_MIN, ARB_BUILD_HOUR, ARB_BUILD_DAY, ARB_BUILD_MONTH, ARB_BUILD_YEAR);
      #else
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
      #endif
      if(!SD.begin(SD_CS_PIN)){
        Serial.println("Card Mount Failed");
        // pinMode(RGB_LED_pin, OUTPUT);
        // digitalWrite(RGB_LED_pin, HIGH);
        // delay(500);
        // digitalWrite(RGB_LED_pin, LOW);
        SDWorking = false;
      } else {
        cardType = SD.cardType();
        if(cardType == CARD_NONE){
          Serial.println("No SD card attached");
          SDWorking = false;
        } 
        else {
          uint64_t cardSize = SD.cardSize() / (1024 * 1024);
          Serial.printf("SD Card Size: %lluMB\n", cardSize);
          if (readFile(SD, SDlogFile) == 0) {
          writeFile(SD, SDlogFile, SDlogFile);      // Write filename as first line in file.
          }
          else {
          appendFile(SD, SDlogFile, SDlogFile);
          }
          appendFile(SD, SDlogFile, "\n");
          #ifdef DEBUG
            Serial.println("New log file created: ");
            Serial.printf(SDlogFile);
            Serial.printf("\n");
          #endif
          SDWorking = true;
        }
      } 
    }
    // Setup mode: use push buttons to set time, verify via BT. 
    // Entered automatically after Epoch found & SD log file created.
    case ESP_SLEEP_WAKEUP_EXT0: 
      #ifdef DEBUG
        Serial.println("Waking up due to button push.");
      #endif
      // Initialise bluetooth
      ble.begin(DeviceName);

      // Initialise interrupts for measurement timer and buttons.     
      pinMode(MonthButton.pin, INPUT_PULLDOWN);
      pinMode(DayButton.pin, INPUT_PULLDOWN);
      pinMode(HourButton.pin, INPUT_PULLDOWN);
      pinMode(MinuteButton.pin, INPUT_PULLDOWN);
      attachInterrupt(digitalPinToInterrupt(MonthButton.pin), MonthAdjISR, ONHIGH); // Could also use RISING
      attachInterrupt(digitalPinToInterrupt(DayButton.pin), DayAdjISR, ONHIGH);
      attachInterrupt(digitalPinToInterrupt(HourButton.pin), HourAdjISR, ONHIGH);
      attachInterrupt(digitalPinToInterrupt(MinuteButton.pin), MinuteAdjISR, ONHIGH);

      mmentTimerCount = MAX_MMENT_COUNT_BEFORE_SLEEP;

      // Use 0th timer of 4-1. Base frequency = 80 MHz by default. Prescalar = 80 gives 1 MHz timer.
      // Max prescalar = 65536 -> 1220.7 kHz min clock: Choose prescalar = 64000 for 1.25 kHz timer. 
      mmentTimer = timerBegin(0, MMENT_PRESCALAR, true);
      timerAttachInterrupt(mmentTimer, &mmentTimerISR, true); // timer, ISR, autoreload
      // period (s) = 80 MHz / MMENT_PRESCALAR / MMENT_PERIOD_BT_S
      // For 1 s period: MMENT_PRESCALAR = 80, MMENT_PERIOD_BT_S = 1000000
      //              or mmentPrescalr = 64000, MMENT_PERIOD_BT_S = 1250
      timerAlarmWrite(mmentTimer, MMENT_PERIOD_BT_S, true); 
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

      if(!SD.begin(SD_CS_PIN)){
        Serial.println("Card Mount Failed");
        SDWorking = false;
      } else {
        cardType = SD.cardType();
        if(cardType == CARD_NONE){
          Serial.println("No SD card attached");
          SDWorking = false;
        } else { // SDWorking
          SDWorking = true;  
        }
      }
    break; 
  }
}

void loop() {
  struct tm time;
  volatile static bool TimeChanged = false;

  if (mmentTimerFlag == true) {
    // Take a measurement, transmit via BT
    strTime = rtc.getTime("%d%m%H%M");
    // Read sensors & log to BT:
    battery_level = analogReadMilliVolts(BATT_LVL_pin);
    strBattery = "Battery: ";
    strBattery = strBattery + battery_level + " mV\n";
    strBattery.toCharArray(batteryMessage, MESSAGE_LENGTH);
    ble.print(batteryMessage);
    #ifdef DEBUG
      Serial.print(strBattery);
    #endif
    AHTret = AHT1.getSensor(&AHThumi, &AHTtemp);
    if (AHTret) {
      strMeasurement = strTime + "S1H" + AHThumi * 100 + "T" + AHTtemp;
      strMeasurement.toCharArray(measurementMessage, MESSAGE_LENGTH);
      ble.print(measurementMessage);
      #ifdef DEBUG
        Serial.println(strMeasurement);
      #endif
    }
    else {
      #ifdef DEBUG
        Serial.print("S1 fail.");
      #endif
    }
    AHTret = AHT2.getSensor(&AHThumi, &AHTtemp);
    if (AHTret) {
      strMeasurement = strTime + "S2H" + AHThumi * 100 + "T" + AHTtemp;
      strMeasurement.toCharArray(measurementMessage, MESSAGE_LENGTH);
      ble.print(measurementMessage);
      #ifdef DEBUG
        Serial.println(strMeasurement);
      #endif
    }
    else {
      #ifdef DEBUG
        Serial.print("S2 fail.");
      #endif
    }
    AHTret = AHT3.getSensor(&AHThumi, &AHTtemp);
    if (AHTret) {
      strMeasurement = strTime + "S3H" + AHThumi * 100 + "T" + AHTtemp;
      strMeasurement.toCharArray(measurementMessage, MESSAGE_LENGTH);
      ble.print(measurementMessage);
      #ifdef DEBUG
        Serial.println(strMeasurement);
      #endif
    }
    else {
      #ifdef DEBUG
        Serial.print("S3 fail.");
      #endif
    }
    mmentTimerFlag = false;
    
    strLullaby = "Going to sleep in ";
    strLullaby = strLullaby + mmentTimerCount;
    strLullaby.toCharArray(lullabyMessage, MESSAGE_LENGTH);
    ble.print(lullabyMessage);
    #ifdef DEBUG
      Serial.println(strLullaby);
    #endif

    if (mmentTimerCount <= 0) {
      // Take a measurement, transmit via BT, log to SD card
      strTime = rtc.getTime("%d%m%H%M");
      // Read sensors & log to BT:
      battery_level = analogReadMilliVolts(BATT_LVL_pin);
      strBattery = "Battery: ";
      strBattery = strBattery + battery_level + " mV\n";
      strBattery.toCharArray(batteryMessage, MESSAGE_LENGTH);
      ble.print(batteryMessage);        
      appendFile(SD, SDlogFile, batteryMessage);
      // appendFile(SD, SDlogFile, "\n");

      #ifdef DEBUG
        Serial.print(strBattery);
      #endif
      AHTret = AHT1.getSensor(&AHThumi, &AHTtemp);
      if (AHTret) {
        strMeasurement = strTime + "S1H" + AHThumi * 100 + "T" + AHTtemp + "\n";
        strMeasurement.toCharArray(measurementMessage, MESSAGE_LENGTH);
        ble.print(measurementMessage);
        appendFile(SD, SDlogFile, measurementMessage);
        #ifdef DEBUG
          Serial.println(strMeasurement);
        #endif
      }
      AHTret = AHT2.getSensor(&AHThumi, &AHTtemp);
      if (AHTret) {
        strMeasurement = strTime + "S2H" + AHThumi * 100 + "T" + AHTtemp + "\n";
        strMeasurement.toCharArray(measurementMessage, MESSAGE_LENGTH);
        ble.print(measurementMessage);        
        appendFile(SD, SDlogFile, measurementMessage);
        #ifdef DEBUG
          Serial.println(strMeasurement);
        #endif
      }
      AHTret = AHT3.getSensor(&AHThumi, &AHTtemp);
      if (AHTret) {
        strMeasurement = strTime + "S3H" + AHThumi * 100 + "T" + AHTtemp + "\n";
        strMeasurement.toCharArray(measurementMessage, MESSAGE_LENGTH);
        ble.print(measurementMessage);        
        appendFile(SD, SDlogFile, measurementMessage);
        #ifdef DEBUG
          Serial.println(strMeasurement);
        #endif
      }      
      // Go to sleep
      timerAlarmDisable(mmentTimer);
      timerAlarmDisable(buttonTimer);
      timerDetachInterrupt(mmentTimer);
      timerDetachInterrupt(buttonTimer);
      detachInterrupt(digitalPinToInterrupt(MonthButton.pin));
      detachInterrupt(digitalPinToInterrupt(DayButton.pin));
      detachInterrupt(digitalPinToInterrupt(HourButton.pin));
      detachInterrupt(digitalPinToInterrupt(MinuteButton.pin));
      Serial.println("Good night from loop");
      Serial.flush();
      Serial.end(true);
      ble.print("Good night from loop");
      ble.end();
      Wire.end();
      Wire1.end();
      WireSW.end();
      SD.end();     
      rtc_gpio_pullup_dis(BUTTON_MINUTE_pin);
      rtc_gpio_pulldown_en(BUTTON_MINUTE_pin);
      esp_sleep_enable_ext0_wakeup(BUTTON_MINUTE_pin, HIGH);  
      ESP.deepSleep(SAMPLE_PERIOD_S * US_PER_S);
    }
  }
  if (buttonTimerFlag == true) {
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
      time = rtc.getTimeStruct();
      time.tm_min = time.tm_min + 1;
      rtc.setTimeStruct(time);
      MinuteButton.flag = false;
    }
    if (TimeChanged) { 
      #ifdef DEBUG
        Serial.println("TimeChanged!");
      #endif
      strTime = rtc.getTime("%d%m%H%M"); 
      strTimeUpdate = "Date and time updated to ";
      strTimeUpdate = strTimeUpdate + strTime;
      strTimeUpdate.toCharArray(timeUpdateMessage, MESSAGE_LENGTH);
      ble.print(timeUpdateMessage);
      Serial.println(strTimeUpdate);
      if (SDWorking) {
        appendFile(SD, SDlogFile, timeUpdateMessage);
        appendFile(SD, SDlogFile, "\n");
      }
      mmentTimerCount = MAX_MMENT_COUNT_BEFORE_SLEEP;
      TimeChanged = false;
    }
    buttonTimerFlag = false;
  }
}
