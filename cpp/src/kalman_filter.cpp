#include "kalman_filter.hpp"
#include <Eigen/Dense>
#include <vector>
#include <unsupported/Eigen/MatrixFunctions>

KalmanFilter::KalmanFilter(const Eigen::MatrixXd& A,
                           const Eigen::MatrixXd& B,
                           const Eigen::MatrixXd& C,
                           const Eigen::MatrixXd& D,
                           const Eigen::MatrixXd& L,
                           const Eigen::MatrixXd& Q,
                           const Eigen::MatrixXd& R)
    : A(A), B(B), C(C),
      D(D.size() ? D : Eigen::MatrixXd::Zero(B.rows(), B.cols())),
      L(L.size() ? L : Eigen::MatrixXd::Zero(B.rows(), B.cols())),
      Q(Q.size() ? Q : Eigen::MatrixXd::Zero(B.cols(), B.cols())),
      R(R.size() ? R : Eigen::MatrixXd::Zero(C.rows(), C.rows())) {}

void KalmanFilter::compute_discrete_matrices(double T) {
    // Matrix exponential for Phi
    Phi = (A * T).exp(); // Eigen's MatrixFunctions module provides .exp()
    Gamma = A.inverse() * (Phi - Eigen::MatrixXd::Identity(A.rows(), A.cols())) * B;
    Lambda = A.inverse() * (Phi - Eigen::MatrixXd::Identity(A.rows(), A.cols())) * L;
}

std::vector<double> KalmanFilter::recursive_least_square(const std::vector<double>& zs, double x0, double P0) {
    std::vector<double> x_estimates;
    double x = x0;
    double P = P0;
    for (const auto& z : zs) {
        double K = P * C(0,0) / (C(0,0) * P * C(0,0) + R(0,0));
        x = x + K * (z - C(0,0) * x);
        P = (1 - K * C(0,0)) * P;
        x_estimates.push_back(x);
    }
    return x_estimates;
}

Eigen::MatrixXd KalmanFilter::discrete_time_filter(const Eigen::VectorXd& x0,
                                                   const Eigen::MatrixXd& P0,
                                                   const Eigen::MatrixXd& u_seq,
                                                   const Eigen::MatrixXd& z) {
    int N_steps = u_seq.cols();
    int n = x0.size();
    Eigen::MatrixXd X_hat(n, N_steps + 1);
    X_hat.col(0) = x0;
    Eigen::MatrixXd P = P0;
    for (int k = 0; k < N_steps; ++k) {
        // Prediction
        X_hat.col(k + 1) = Phi * X_hat.col(k) + Gamma * u_seq.col(k);
        P = Phi * P * Phi.transpose() + Lambda * Q * Lambda.transpose();
        // Kalman gain
        Eigen::MatrixXd K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
        // Correction
        X_hat.col(k + 1) += K * (z.col(k) - C * X_hat.col(k + 1));
        P = (Eigen::MatrixXd::Identity(n, n) - K * C) * P;
    }
    return X_hat;
}
