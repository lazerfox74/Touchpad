#pragma once


#ifndef SEGMENT_H
#define SEGMENT_H


#include "cmath"

class Segment
{
    public:

    Segment();
    ~Segment();

    void Init(float sampleRate);

    void rampTo(float value,float time,float overshootRation = 1.001);

    float process();

    void setValue(float value);
    //flag for setting true when an envelope segment finishes, allowing adsr class to advance to the next envelope stage
    bool updateState = false;

    private:

    float sampleRate_;
    double currentVal_ = 0;
    double targetVal_;
    double asymptoteVal_;
    double expValue_;
    double multiplier_ = 1;

    //true = up false = down
    bool direction = true;

};

#endif // SEGMENT_H
