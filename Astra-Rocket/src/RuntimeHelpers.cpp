#include "RuntimeHelpers.h"

#include <cmath>

#include <AstraRocket.h>
#include <Sensors/SensorManager/SensorManager.h>
#include <Sensors/VoltageSensor/VoltageSensor.h>

#include "AirbrakeController.h"
#include "md6.h"

using namespace astra;
using namespace astra_rocket;

namespace runtime_helpers
{
void runOrientationCalibration(AstraRocket &rocket,
                               AstraRocketConfig &config,
                               bool usingHitlSensors,
                               Stream &serialOut,
                               uint32_t stationaryCalTimeMs,
                               uint32_t magCalTimeMs)
{
    return;
    auto *rocketState = rocket.getRocketState();
    auto *filter = rocketState ? rocketState->getOrientationFilter() : nullptr;
    auto *sm = config.getSensorManager();
    auto *accelSrc = sm ? sm->getAccelSource() : nullptr;
    auto *gyroSrc = sm ? sm->getGyroSource() : nullptr;
    auto *magSrc = sm ? sm->getMagSource() : nullptr;

    const bool canRunOrientation = (filter != nullptr && accelSrc != nullptr && gyroSrc != nullptr);
    if (!canRunOrientation)
    {
        serialOut.println("# Orientation calibration skipped: missing filter/accel/gyro.");
        return;
    }
    if (usingHitlSensors)
    {
        serialOut.println("# Orientation calibration skipped in HITL sensor mode.");
        return;
    }

    serialOut.println("# ==================================");
    serialOut.println("# Mahony Calibration Starting");
    serialOut.println("# Phase 1: KEEP STILL");
    serialOut.println("# ==================================");

    uint32_t startMs = millis();
    uint32_t lastMs = millis();

    while (millis() - startMs < stationaryCalTimeMs)
    {
        rocket.update();

        uint32_t now = millis();
        double dt = (now - lastMs) * 1e-3;
        lastMs = now;
        if (dt <= 0.0)
        {
            delay(1);
            continue;
        }

        Vector<3> accel = accelSrc->getAccel();
        Vector<3> gyro = gyroSrc->getAngVel();
        filter->update(accel, gyro, dt);
        delay(5);
    }

    serialOut.println("# Phase 1 Complete");

    bool magUsable = (magSrc != nullptr);
    if (magUsable && !magSrc->isHealthy())
        magUsable = false;

    if (!magUsable)
    {
        serialOut.println("# Mag unavailable/unhealthy. Using gyro+accel fallback.");
        serialOut.println("# Skipping mag calibration phase.");
    }
    else
    {
        serialOut.println("# ==================================");
        serialOut.println("# Phase 2: ROTATE BOARD IN ALL AXES");
        serialOut.println("# 30 seconds...");
        serialOut.println("# ==================================");

        startMs = millis();
        lastMs = millis();

        while (millis() - startMs < magCalTimeMs)
        {
            rocket.update();

            uint32_t now = millis();
            double dt = (now - lastMs) * 1e-3;
            lastMs = now;
            if (dt <= 0.0)
            {
                delay(1);
                continue;
            }

            Vector<3> accel = accelSrc->getAccel();
            Vector<3> gyro = gyroSrc->getAngVel();
            Vector<3> mag = magSrc->getMag();

            const bool finiteMag = std::isfinite(mag.x()) && std::isfinite(mag.y()) && std::isfinite(mag.z());
            const bool nonZeroMag = mag.magnitude() > 1e-6;

            if (magSrc->isHealthy() && finiteMag && nonZeroMag)
            {
                filter->update(accel, gyro, mag, dt);
                filter->collectMagCalibrationSample(mag);
            }
            else
            {
                filter->update(accel, gyro, dt);
            }

            if ((millis() - startMs) % 1000 < 20)
                serialOut.print(".");

            delay(5);
        }

        serialOut.println();
        serialOut.println("# Finalizing mag calibration...");
        filter->finalizeMagCalibration();

        if (filter->isMagCalibrated())
            serialOut.println("# Mag calibration SUCCESS");
        else
            serialOut.println("# Mag calibration FAILED (running accel+gyro fallback)");
    }

    serialOut.println("# Calibration complete.");
    serialOut.println("# ==================================");
}

void emitCompactHeader(Stream &out)
{
    out.println("CTLM/t_s,stage,bat_v,ab_bat_v,pz_m,vz_mps,az_mps2,lat_deg,lon_deg,ab_cmd_deg,ab_act_deg,q_re_w,q_re_x,q_re_y,q_re_z");
}

void emitCompactData(Stream &out,
                     RocketState *state,
                     AstraRocketConfig &config,
                     AirbrakeController *airbrakeCtrl,
                     VoltageSensor *voltageSensor,
                     MotorDriver *motorDriver)
{
    double pz = 0.0;
    double vz = 0.0;
    double az = 0.0;
    double stage = 0.0;
    double batV = 0.0;
    double abBatV = 0.0;
    double lat = 0.0;
    double lon = 0.0;
    double abCmd = 0.0;
    double abAct = 0.0;
    double qw = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;

    if (state)
    {
        stage = static_cast<int>(state->getFlightStage());
        const Vector<3> pos = state->getPosition();
        const Vector<3> vel = state->getVelocity();
        const Vector<3> acc = state->getAcceleration();
        pz = pos.z();
        vz = vel.z();
        az = acc.z();

        MahonyAHRS *orientation = state->getOrientationFilter();
        if (orientation)
        {
            const Quaternion q = orientation->getQuaternion();
            qw = q.w();
            qx = q.x();
            qy = q.y();
            qz = q.z();
        }
    }

    SensorManager *sm = config.getSensorManager();
    if (sm)
    {
        GPS *gps = sm->getGPSSource();
        if (gps && gps->isInitialized() && gps->getHasFix())
        {
            const Vector<3> gpsPos = gps->getPos();
            lat = gpsPos.x();
            lon = gpsPos.y();
        }
    }

    if (airbrakeCtrl)
    {
        abCmd = airbrakeCtrl->getCommandedDeployment();
        abAct = airbrakeCtrl->getCurrentDeployment();
    }
    if (voltageSensor)
    {
        batV = voltageSensor->getVoltage();
    }
    if (motorDriver)
    {
        abBatV = motorDriver->getBatteryVoltage();
    }

    const double tSec = millis() / 1000.0;

    out.print("CTLM/");
    out.print(tSec, 3);
    out.write(',');
    out.print(stage, 0);
    out.write(',');
    out.print(batV, 3);
    out.write(',');
    out.print(abBatV, 3);
    out.write(',');
    out.print(pz, 3);
    out.write(',');
    out.print(vz, 3);
    out.write(',');
    out.print(az, 3);
    out.write(',');
    out.print(lat, 7);
    out.write(',');
    out.print(lon, 7);
    out.write(',');
    out.print(abCmd, 3);
    out.write(',');
    out.print(abAct, 3);
    out.write(',');
    out.print(qw, 6);
    out.write(',');
    out.print(qx, 6);
    out.write(',');
    out.print(qy, 6);
    out.write(',');
    out.println(qz, 6);
}
} // namespace runtime_helpers
