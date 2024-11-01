#include "Envelope.h"

void Envelope::init(float sampleRate)
{
    //initialise samplerate for segment and envelope

    sampleRate_ = sampleRate;
    thisSegment.Init(sampleRate);
    thisSegment.setValue(0.0f);
    // thisSegment.rampTo(0,0);


    //set to off state on init
    // state_ = State::offState; 
    eState = offState;

    // thisSegment.rampTo(values[eState],times[eState]);

}

float Envelope::process()
{
    // if in off state then do nothing and return 0, state will be updated externally
    if(eState == offState+1)
    {
        return 0.f;
    }
    else
    {   


        if(allowDecay == true)
        {
            eState = 3;
            thisSegment.rampTo(values[eState],times[eState],1.001);
            //set update flag back to false
            thisSegment.updateState = false;
            allowDecay = false;
        }
        //check to see if the segment needs updating
        else if(thisSegment.updateState == true && eState != 2)
        {

            eState++;            
            
            //set the segment class to ramp to the new segment
            thisSegment.rampTo(values[eState],times[eState],1.001);
            //set update flag back to false
            thisSegment.updateState = false;
        }
  
        //then process segment
        return thisSegment.process();

    }


}


void Envelope::trigger()
{
    //set state to attack
    eState = 0;
    allowDecay = false;
    // thisSegment.setValue(1.0f);
    //update segment
    // thisSegment.setValue(1.0f);
    thisSegment.rampTo(values[eState],times[eState],1.1);

}

void Envelope::setDecay(float time)
{
    times[1] = time;
    
}

void Envelope::setRelease(float time)
{
    times[2] = time;
}

void Envelope::setSustain(float sus)
{
    values[1] = sus;
}

void Envelope::setOffState(int oS)
{
    offState = oS;
}

void Envelope::setVal(int index, float amp,float time)
{
    
    values[index] = amp;
    times[index] = time;
}

void Envelope::startDecay()
{
    allowDecay = true;
}




    // //first check state
    // switch(state_)
    // {
    //     case State::offState : 
    //     //do nothing
    //     break;

    //     case State::attackState :
        
    //     break;

    //     case State::decayState :

    //     break;

    //     case State::releaseState :

    //     break;

    // }