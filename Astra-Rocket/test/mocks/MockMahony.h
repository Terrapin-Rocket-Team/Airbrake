#ifndef MOCK_MAHONY_H
#define MOCK_MAHONY_H

#include "Filters/Mahony.h"

namespace astra_mocks
{
    using namespace astra;

    class MockMahony : public MahonyAHRS
    {
    public:
        MockMahony(double Kp = 0.1, double Ki = 0.0005) : MahonyAHRS(Kp, Ki) {}

        void update(const Vector<3>& accel, const Vector<3>& gyro, double dt) {
            // Do nothing - mock doesn't update state
        }

        // Helper to manually set orientation for testing
        void setMockOrientation(const Quaternion& q) {
            mockQ = q;
            mockValid = true;
        }

        bool isReady() const override {
            return mockValid || MahonyAHRS::isReady();
        }

        Quaternion getQuaternion() const override {
            if (mockValid) {
                // Treat mockQ as board->earth so existing tests can set intuitive orientations.
                // Mahony internally tracks body->earth, so convert using current board->body mount.
                Quaternion boardToBody = getBoardToBodyQuaternion();
                return mockQ * boardToBody.conjugate();
            }
            return MahonyAHRS::getQuaternion();
        }

        Vector<3> getEarthAcceleration(const Vector<3>& accel) const override {
            // If mock acceleration is set, return it directly
            if (mockAccelValid) {
                return mockEarthAccel;
            }

            // For mocking: convert body-frame accel to earth-frame and remove gravity
            // Body frame: Z down is negative (accel sensor reading)
            // Earth frame: Z up is positive, gravity removed
            Vector<3> earthAccel = accel; // Assume aligned (mock doesn't rotate)
            earthAccel.z() += 9.81; // Remove gravity (add it back since sensor reading includes it)
            return earthAccel;
        }

        // Helper to directly set the earth-frame acceleration for testing
        void setMockEarthAcceleration(const Vector<3>& accel) {
            mockEarthAccel = accel;
            mockAccelValid = true;
        }

    private:
        Quaternion mockQ = Quaternion(1.0, 0.0, 0.0, 0.0);
        bool mockValid = false;
        Vector<3> mockEarthAccel = Vector<3>(0, 0, 0);
        bool mockAccelValid = false;
    };

} // namespace astra_mocks

#endif // MOCK_MAHONY_H
