#pragma once


#ifndef ENVELOPE_H
#define ENVELOPE_H

#include "Segment.h"

class Envelope
{
    public :

    //initialise object set samplerate
    void init(float sampleRate);
    //process the envelope, return value and change state if nececarry
    float process();

    //triggers the envelope, takes it from whatever its current value is to the attack 1 value state
    void trigger();

    //set decay time and caluclate the multiplication factor to use
    void setDecay(float time);
    //set the release time
    void setRelease(float time);
    //set sustain amount
    void setSustain(float sus);
    //set the lenght of the envelope and which stage should be considered off
    void setOffState(int oS);
    //used for setting values in time array and amplitude array
    void setVal(int index, float amp,float time);
    
    void startDecay();


    // void set(float time);

    

    private :

    // enum State
    // {
	// 	attackState = 0,
	// 	decayState,
	// 	releaseState,
    //     offState,
	// };
    //current envelope state
    // State state_;
    float currentValue;
    float previousValue;
    float targetValue;
    float holdTime;
    float sampleRate_;
    double increment;

    //variable for env state,
    // 0 = att
    // 1 = decay
    // 2 = release
    // 3 == off
    int eState = 0;

    //variable for storing which envelope stage is the off state
    int offState = 3;

    //array for holding difference segment decay and sustain values

    float values[8] = {1.0f,0.5f,0.5f,0.f,0.0f};

    float times[8] = {0.05f,0.2f,0.5f,2.0f,0.0f};

    Segment thisSegment;
    bool allowDecay = false;

};

#endif