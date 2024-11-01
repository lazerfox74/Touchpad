#ifndef LERP_H
#define LERP_H

class lerpF
{
    public:
    float process(float num)
    {
        float output = prevNum + (amt * (num - prevNum));
        // Linear interpolation is best done simply as:
        // g(x) = y[0] + x(y[1] − y[0])
        //http://yehar.com/blog/wp-content/uploads/2009/08/deip.pdf
        prevNum = output;
        return output;
    }

    void setAmt(float _amt)
    {
        amt = _amt;
        prevNum = 0.0;
    }

    private:

    float memory,amt,prevNum;
};

#endif