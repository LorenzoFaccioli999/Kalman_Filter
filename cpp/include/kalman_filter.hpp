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

    Eigen::MatrixXd discrete_time_filter(const Eigen::VectorXd& x0,
                                         const Eigen::MatrixXd& P0,
                                         const Eigen::MatrixXd& u_seq,
                                         const Eigen::MatrixXd& z);

    // Matrices for discrete-time filter
    Eigen::MatrixXd Phi, Gamma, Lambda;

private:
    Eigen::MatrixXd A, B, C, D, L, Q, R;
};
