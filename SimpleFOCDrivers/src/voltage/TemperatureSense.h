#ifndef __TEMPSENSE_H__
#define __TEMPSENSE_H__

#include "./VoltageSense.h"
#include "common/foc_utils.h"

#define TEMP_LT_SIZE 4095

class TemperatureSense : public VoltageSense{
  public:
    TemperatureSense(float r1, float r25c, float beta, int pin, float gain = 1.0f, float offset = 0.0f, float Tf = 0.1f, float fullScaleVoltage = 3.3f);
    bool init(int resolution = 10, int scale = 100);
    uint16_t getTemperature();
    void calcLookupTable();
    float fullScaleVoltage;
    float temperature;

    LowPassFilter filter;
    float gain;
    float offset;

    float r1resistor;
    float rt25c; 
    float beta;

  protected:
    float readRawVoltage() override;
    int rawADC;
    int maxValue;
    int pin;

    int tempScale;
    int16_t temp_lt[TEMP_LT_SIZE];
    
};



#endif