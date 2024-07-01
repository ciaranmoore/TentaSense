#ifndef __AHT202_H__
#define __AHT202_H__

#include <Arduino.h>
#include <SoftWire.h>
extern SoftWire WireSW;  

class AHT202{

protected:
    
private:

    bool startSensor();
public:

    void begin();
    bool getSensor(float *h, float *t);
    bool getTemperature(float *t);
    bool getHumidity(float *h);
};

#endif