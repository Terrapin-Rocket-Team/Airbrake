#include "e5.h"


E5::E5(const int chan_a, const int chan_b, const char *name) : myEnc(chan_a, chan_b)
DPS310::DPS310(const char *name) : dps()
{
    setName(name);
}

bool E5::init()
{
    return initialized = true;
}

void E5::read()
{
    currentRelativeSteps = myEnc.read();
}

