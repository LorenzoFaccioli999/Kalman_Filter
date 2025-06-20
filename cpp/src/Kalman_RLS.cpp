#include "Kalman_RLS.hpp"

Kalman_RLS::Kalman_RLS(float C, float R, float x0, float P0)
    : C_m(C), R_m(R), x_m(x0), P_m(P0) {}

void Kalman_RLS::update(float z) {
    // Kalman Gain
    float K = P_m * C_m / (C_m * P_m * C_m + R_m);

    // State estimate update
    x_m = x_m + K * (z - C_m * x_m);

    // Error covariance update
    P_m = (1.0f - K * C_m) * P_m;
}

float Kalman_RLS::getEstimate() const {
    return x_m;
}

float Kalman_RLS::getCovariance() const {
    return P_m;
}
