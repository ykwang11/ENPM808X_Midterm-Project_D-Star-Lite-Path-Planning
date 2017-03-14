#pragma once

#include <iostream>

class AnalogSensor
{
public:
    AnalogSensor(unsigned int samples);
    ~AnalogSensor();
    int Read();
private:
    unsigned int mSamples;
};
