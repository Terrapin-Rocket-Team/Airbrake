
#include "MockBR.h"

using namespace mmfs;
MockBR::MockBR(const char* dataPath, const std::string& altitudeColName, const std::string& pressureColName, const std::string& temperatureColName, 
    const std::string& tiltColName, const std::string& rollColName, const std::string& velocityColName, 
    const std::string accColNames[3], const std::string gyroColNames[3]): BR("MockBR"), dataReader(dataPath),
    altitudeColName(altitudeColName), pressureColName(pressureColName), temperatureColName(temperatureColName),
    tiltColName(tiltColName), rollColName(rollColName), velocityColName(velocityColName)
    {
        for (int i = 0; i < 3; i++)
        {
            this->accColNames[i] = accColNames[i];
            this->gyroColNames[i] = gyroColNames[i];

            this->accIndices[i] = -1;
            this->gyroIndices[i] = -1;
        }
    }

    bool MockBR::init()
    {
        if (!dataReader.isInit())
            return false;

        int numCols = -1;
        std::string colNames[MAX_NUM_COLS];
        dataReader.readColumnHeaders(numCols, colNames);

        if (numCols == -1 || numCols > MAX_NUM_COLS)
        {
            getLogger().recordLogData(ERROR_, 100, "[MockBR]: Invalid number of columns read: %d", numCols);
            return false;
        }

        for (int i = 0; i < numCols; i++)
        {   
            if (colNames[i] == altitudeColName)
            {
                altitudeColIndex = i;
            }
            else if (colNames[i] == pressureColName)
            {
                pressureColIndex = i;
            }
            else if (colNames[i] == temperatureColName)
            {
                temperatureColIndex = i;
            }
            else if (colNames[i] == tiltColName)
            {
                tiltColIndex = i;
            }
            else if (colNames[i] == rollColName)
            {
                rollColIndex = i;
            }
            else if (colNames[i] == velocityColName)
            {
                velocityColIndex = i;
            }
            for (int j = 0; j < 3; j++)
            {
                if (colNames[i] == accColNames[j])
                {
                    accIndices[j] = i;
                }
                else if (colNames[i] == gyroColNames[j])
                {
                    gyroIndices[j] = i;
                }
            }
        }

        // underscores indicate skipped fields
        if (altitudeColName == "_")
        {
            altitudeColIndex = MAX_NUM_COLS - 1;
        }
        if (pressureColName == "_")
        {
            pressureColIndex = MAX_NUM_COLS - 1;
        }
        if (temperatureColName == "_")
        {
            temperatureColIndex = MAX_NUM_COLS - 1;
        }
        if (tiltColName == "_")
        {
            tiltColIndex = MAX_NUM_COLS - 1;
        }
        if (rollColName == "_")
        {
            rollColIndex = MAX_NUM_COLS - 1;
        }
        if (velocityColName == "_")
        {
            velocityColIndex = MAX_NUM_COLS - 1;
        }
        for (int j = 0; j < 3; j++)
        {
            if (accColNames[j] == "_")
            {
                accIndices[j] = MAX_NUM_COLS - 1;
            }
            if (gyroColNames[j] == "_")
            {
                gyroIndices[j] = MAX_NUM_COLS - 1;
            }
        }

        if (altitudeColIndex == -1)
        {
            getLogger().recordLogData(ERROR_, 100, "[MockBR]: Failed to find altitude column index for name: %s", altitudeColName.c_str());
            return false;
        }
        if (pressureColIndex == -1)
        {
            getLogger().recordLogData(ERROR_, 100, "[MockBR]: Failed to find pressure column index for name: %s", pressureColName.c_str());
            return false;
        }
        if (temperatureColIndex == -1)
        {
            getLogger().recordLogData(ERROR_, 100, "[MockBR]: Failed to find temperature column index for name: %s", temperatureColName.c_str());
            return false;
        }
        if (tiltColIndex == -1)
        {
            getLogger().recordLogData(ERROR_, 100, "[MockBR]: Failed to find tilt column index for name: %s", tiltColName.c_str());
            return false;
        }
        if (rollColIndex == -1)
        {
            getLogger().recordLogData(ERROR_, 100, "[MockBR]: Failed to find roll column index for name: %s", rollColName.c_str());
            return false;
        }
        if (velocityColIndex== -1)
        {
            getLogger().recordLogData(ERROR_, 100, "[MockBR]: Failed to find velocity column index for name: %s", velocityColName.c_str());
            return false;
        }
        for (int i = 0; i < 3; i++)
        {
            if (accIndices[i] == -1)
            {
                getLogger().recordLogData(ERROR_, 100, "[MockBR]: Failed to find acceleration column index for name: %s", accColNames[i].c_str());
                return false;
            }

            if (gyroIndices[i] == -1)
            {
                getLogger().recordLogData(ERROR_, 100, "[MockBR]: Failed to find gyroscope column index for name: %s", gyroColNames[i].c_str());
                return false;
            }
        }

        initialized = true;
        return initialized;
    }

    void MockBR::read()
    {
        if (!dataReader.readLine(sdData))
        {
            getLogger().recordLogData(ERROR_, 100, "[MockBR]: Failed to read data from file!");
            initialized = false;
            return;
        }
        altitude = sdData[altitudeColIndex];
        pressure = sdData[pressureColIndex];
        temperature = sdData[temperatureColIndex];
        tiltAngle = sdData[tiltColIndex];
        rollAngle = sdData[rollColIndex];
        velocity= sdData[velocityColIndex];
        accelX = sdData[accIndices[0]];
        accelY = sdData[accIndices[1]];
        accelZ = sdData[accIndices[2]];
        gyroX = sdData[gyroIndices[0]];
        gyroY = sdData[gyroIndices[1]];
        gyroZ = sdData[gyroIndices[2]];
    }
