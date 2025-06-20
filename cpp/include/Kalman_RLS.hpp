#ifndef KALMAN_RLS_HPP
#define KALMAN_RLS_HPP

class Kalman_RLS {
public:
    /**
     * @brief Constructor for the recursive least squares Kalman estimator.
     * @param C Observation matrix (e.g., sensor gain)
     * @param R Measurement noise covariance
     * @param x0 Initial estimate of the state
     * @param P0 Initial estimate covariance
     */
    Kalman_RLS(float C, float R, float x0, float P0);

    /**
     * @brief Update the state estimate with a new measurement.
     * @param z The latest sensor measurement
     */
    void update(float z);
    float getEstimate() const;
    float getCovariance() const;

    void setR(float r) { R_m = r; }


private:
    float C_m;  // Observation matrix
    float R_m;  // Measurement noise covariance
    float x_m;  // Current estimate
    float P_m;  // Estimate error covariance
};

#endif // KALMAN_RLS_HPP
