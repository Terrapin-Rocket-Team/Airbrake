#include "e5.h"


E5::E5(const int chan_a, const int chan_b, const char *name) : myEnc(chan_a, chan_b)
{
    setName(name);
}

bool E5::init()
{ 
    initialSteps = myEnc.read();
    return initialized = true;
}

void E5::read()
{
    // currentRelativeSteps = myEnc.read();

    currentRelativeSteps = myEnc.read() - initialSteps;
    Serial.print("myEnc.read():");
    Serial.println(myEnc.read());
    Serial.print("Initial Steps:");
    Serial.println(initialSteps);
    Serial.print("Currently Relative Steps:");
    Serial.println(currentRelativeSteps);
}

