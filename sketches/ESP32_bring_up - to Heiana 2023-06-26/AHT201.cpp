// AHT201 ARDUINO LIBRARY
#include "AHT201.h"

void AHT201::begin()
{
    // Wire1.begin();                  // CM
    // Wire1.begin(SDA_pin, SCL_pin);  // CM
    
    Wire1.beginTransmission(0x38); // transmit to device #8
    Wire1.write(0xBE);
    Wire1.endTransmission();    // stop transmitting
}

bool AHT201::startSensor()
{
    Wire1.beginTransmission(0x38); // transmit to device #8
    Wire1.write(0xac);
    Wire1.write(0x33);
    Wire1.write(0x00);
    Wire1.endTransmission();    // stop transmitting

    unsigned long timer_s = millis();
    while(1)
    {
        if(millis()-timer_s > 200) return 0;        // time out
        Wire1.requestFrom(0x38, 1);

        while(Wire1.available())
        {
            unsigned char c = Wire1.read();
            // Serial.println("Wire1.available, Reading from Wire1.);
            if(c&0x80 != 0)return 1;      // busy

        }

        delay(20);
    }
}

bool AHT201::getSensor(float *h, float *t)
{
    startSensor();
    Wire1.requestFrom(0x38, 6);


    unsigned char str[6] ={0,};
    int index = 0;
    while (Wire1.available())
    {
        str[index++] = Wire1.read(); // receive a byte as character
    }
    if(index == 0 )return 0; 
    if(str[0] & 0x80)return 0;

    unsigned long __humi = 0;
    unsigned long __temp = 0;

    __humi = str[1];
    __humi <<= 8;
    __humi += str[2];
    __humi <<= 4;
    __humi += str[3] >> 4;

    *h = (float)__humi/1048576.0;

    __temp = str[3]&0x0f;
    __temp <<=8;
    __temp += str[4];
    __temp <<=8;
    __temp += str[5];

    *t = (float)__temp/1048576.0*200.0-50.0;

    return 1;

}

bool AHT201::getTemperature(float *t)
{
    float __t, __h;
    
    int ret = getSensor(&__h, &__t);
    if(0 == ret)return 0;
    
    *t = __t;
    return 1;
}

bool AHT201::getHumidity(float *h)
{
    float __t, __h;
    
    int ret = getSensor(&__h, &__t);
    if(0 == ret)return 0;
    
    *h = __h;
    return 1;
}
