#include "Segment.h"


Segment::Segment()
{
    //rada
}
Segment::~Segment()
{
    //rada
}


float Segment::process()
{

    currentVal_ = asymptoteVal_ + expValue_;

    // if(direction)

    if(currentVal_ >= targetVal_ && currentVal_ <= asymptoteVal_)
    {
        //if passed set multiplier to 1 to prevent envelope changing
        // multiplier_ = 0.0;
        
        updateState = true;
        //==dsadsadsa==

    }
    else if(currentVal_ <= targetVal_ && currentVal_ >= asymptoteVal_)
    {
        //if trending down check to see if destination value passed
        //if passed set multiplier to 1 to prevent envelope changing
        // multiplier_ = 0.0;
        updateState = true;
    }

    expValue_ *= multiplier_;

    return currentVal_;

}

void Segment::Init(float sampleRate)
{
    sampleRate_ = sampleRate;
}

// bool Segment::finished()
// {

// }

void Segment::rampTo(float value, float time,float overshootRatio)
{
    targetVal_ = value;

    float distanceToTarget = targetVal_ - currentVal_;
    asymptoteVal_ = currentVal_ + distanceToTarget * overshootRatio;

    expValue_ = currentVal_ - asymptoteVal_;

    //calculate time constant
    double tau = -1.0 * time / log(1.0 - 1.0/overshootRatio);
    
    multiplier_ = pow(exp(-1.0 / tau),1.0/sampleRate_);

    //set the direction variable for the envelope based upon if the envelope segment is trending up or down
    if(distanceToTarget > 0.0f)
    {
        direction = true;
    }
    else
    {
        direction = false;
    }

}

void Segment::setValue(float value)
{
    currentVal_ = value;
}