#ifndef MOCKBR_H
#define MOCKBR_H

#include "MMFS.h"
#include "BR.h"

namespace mmfs
{
class MockBR : public BR {
    private:
        SerialReader dataReader;

        int altitudeColIndex = -1;
        int pressureColIndex = -1;
        int temperatureColIndex = -1;
        int tiltColIndex = -1;
        int rollColIndex = -1;
        int velocityColIndex = -1;
        int accIndices[3]{-1, -1, -1};
        int gyroIndices[3]{-1, -1, -1};

        std::string altitudeColName;
        std::string pressureColName;
        std::string temperatureColName;
        std::string tiltColName;
        std::string rollColName;
        std::string velocityColName;
        std::string accColNames[3];
        std::string gyroColNames[3];

        float sdData[MAX_NUM_COLS]{0.0f};

    public:
        MockBR(const char* dataPath, const std::string& altitudeColName, const std::string& pressureColName, const std::string& temperatureColName, 
            const std::string& tiltColName, const std::string& rollColName, const std::string& velocityColName, 
            const std::string accColNames[3], const std::string gyroColNames[3]);

        bool init() override;
        void read() override;
    };
}

#endif // MOCKBR_H