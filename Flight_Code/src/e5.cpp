#include "e5.h"

{

    E5::E5(const char *name) : myEnc()
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
}