

#include "./TemperatureSense.h"
#include "Arduino.h"

TemperatureSense::TemperatureSense(float r1, float r25c, float beta, int pin, float gain, float offset, float Tf, float fullScaleVoltage):
    filter(Tf), r1resistor(r1), rt25c(r25c), beta(beta), gain(gain), offset(offset), fullScaleVoltage(fullScaleVoltage), pin(pin) {};


bool TemperatureSense::init(int resolution, int scale){
    pinMode(pin, INPUT_ANALOG);
    maxValue = _powtwo(resolution);
    tempScale = scale; 
    calcLookupTable();
    return true;
};

bool init(int resolution = -1);

uint16_t TemperatureSense::getTemperature()
{
    readRawVoltage();
    return temp_lt[rawADC];
}

float TemperatureSense::readRawVoltage(){
    rawADC = analogRead(pin);
    return rawADC; 
};


void TemperatureSense::calcLookupTable(){
    
    for (int i = 0; i < TEMP_LT_SIZE; i++) {
        float voltage_rt = (((float)i / 4095.0) * fullScaleVoltage);
        float current_rt = ((fullScaleVoltage - voltage_rt) / r1resistor);
        float NTC_resistance_motor = voltage_rt/current_rt;
        float temp_recip = (1.0 / 298.15) - (log(rt25c/NTC_resistance_motor)/beta);
        temp_lt[i] = static_cast<uint16_t>(((1/temp_recip) - 273.15f) * tempScale);
    }
}




