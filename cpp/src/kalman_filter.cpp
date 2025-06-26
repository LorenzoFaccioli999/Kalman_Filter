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

void KalmanFilter::predict(const Eigen::VectorXd& u) {
    x_hat = Phi * x_hat + Gamma * u;
    P = Phi * P * Phi.transpose() + Lambda * Q * Lambda.transpose();
}

void KalmanFilter::update(const Eigen::VectorXd& z) {
    Eigen::MatrixXd K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    x_hat = x_hat + K * (z - C * x_hat);
    P = (Eigen::MatrixXd::Identity(P.rows(), P.cols()) - K * C) * P;
}

