#ifndef E5_H
#define E5_H

#include "Encoder.h"
#include <MMFS.h>

class E5 : public mmfs::Encoder_MMFS
{
private:
    Encoder myEnc;

public:
    E5(const int, const int, const char *name = "E5");
    virtual bool init() override;
    virtual void read() override;
};


#endif