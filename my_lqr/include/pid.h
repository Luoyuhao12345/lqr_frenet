#ifndef PID_H
#define PID_H

#include <iostream>
using namespace std;


class PID
{
public:
    PID(float a, float b, float c);
    float pid(float steer, float e);
private:
    float total_e;
    float cur_e;
    float p,i,d;
};

#endif