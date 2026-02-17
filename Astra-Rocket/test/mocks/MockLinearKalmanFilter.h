#ifndef MOCK_LINEAR_KALMAN_FILTER_H
#define MOCK_LINEAR_KALMAN_FILTER_H

#include "Filters/LinearKalmanFilter.h"

namespace astra_mocks
{
    using namespace astra;

    class MockLinearKalmanFilter : public LinearKalmanFilter
    {
    public:
        MockLinearKalmanFilter(int measurementSize, int controlSize, int stateSize)
            : LinearKalmanFilter(measurementSize, controlSize, stateSize) {}

        void initialize() override {}
        Matrix getF(double dt) override { return Matrix(stateSize, stateSize); }
        Matrix getG(double dt) override { return Matrix(stateSize, controlSize); }
        Matrix getH() override { return Matrix(measurementSize, stateSize); }
        Matrix getR() override { return Matrix(measurementSize, measurementSize); }
        Matrix getQ(double dt) override { return Matrix(stateSize, stateSize); }

        void predict(double dt, Matrix control) override {
            // Do nothing
        }
        void update(Matrix measurement) override {
            // Do nothing
        }

        void setState(const Matrix& state) {
            X = state;
        }
    };

} // namespace astra_mocks

#endif // MOCK_LINEAR_KALMAN_FILTER_H
