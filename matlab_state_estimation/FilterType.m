classdef FilterType
    enumeration
        LKF, % Basic linear kalman filter
        LKFMM, % linear kalman filter with additional barometer measurements
        EKF % extended kalman filter
    end
end