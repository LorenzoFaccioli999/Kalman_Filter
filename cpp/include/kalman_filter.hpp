#pragma once
#include <Eigen/Dense>
#include <vector>

class KalmanFilter {
public:
    KalmanFilter(const Eigen::MatrixXd& A,
                 const Eigen::MatrixXd& B,
                 const Eigen::MatrixXd& C,
                 const Eigen::MatrixXd& D = Eigen::MatrixXd(),
                 const Eigen::MatrixXd& L = Eigen::MatrixXd(),
                 const Eigen::MatrixXd& Q = Eigen::MatrixXd(),
                 const Eigen::MatrixXd& R = Eigen::MatrixXd());

    void compute_discrete_matrices(double T);

    std::vector<double> recursive_least_square(const std::vector<double>& zs, double x0, double P0);

    // Step-by-step (real-time) Kalman filter API
    void predict(const Eigen::VectorXd& u);
    void update(const Eigen::VectorXd& z);

    const Eigen::VectorXd& getState() const { return x_hat; }
    const Eigen::MatrixXd& getCovariance() const { return P; }
    void setState(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) { x_hat = x0; P = P0; }

    // Matrices for discrete-time filter
    Eigen::MatrixXd Phi, Gamma, Lambda;

private:
    Eigen::MatrixXd A, B, C, D, L, Q, R;
    Eigen::VectorXd x_hat;
    Eigen::MatrixXd P;
};
