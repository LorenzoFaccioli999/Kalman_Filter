#include "kalman_filter.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <filesystem>

int main() {
    // Physical parameters
    double g = 9.81;
    double l = 1.0;
    double m = 1.0;
    double zeta = 0.1;
    double omega_n = std::sqrt(g / l);
    Eigen::MatrixXd A(2,2);
    A << -2 * zeta * omega_n, -omega_n * omega_n,
          1,                 0;
    Eigen::MatrixXd B(2,1);
    B << 1.0 / (m * l * l),
         0;
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(2,2);
    Eigen::MatrixXd L(2,1); L << 1, 1;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(1,1) * 0.1;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2,2) * 0.2;

    double T = 0.01;
    KalmanFilter kf(A, B, C, Eigen::MatrixXd(), L, Q, R);
    kf.compute_discrete_matrices(T);

    // Initial conditions
    Eigen::VectorXd x0(2); x0 << 0.0, 0.1;
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(2,2) * 0.1;
    double T_total = 10.0;
    int N_steps = static_cast<int>(T_total / T);

    // Control input
    Eigen::MatrixXd u_seq(1, N_steps); u_seq.setOnes();
    for (int i = static_cast<int>(2.0 / T); i < static_cast<int>(5.0 / T); ++i) {
        u_seq(0, i) = 0.0;
    }

    // Simulate true system
    Eigen::MatrixXd X_true(2, N_steps + 1);
    X_true.col(0) = x0;
    for (int k = 0; k < N_steps; ++k) {
        X_true.col(k + 1) = kf.Phi * X_true.col(k) + kf.Gamma * u_seq.col(k);
    }

    // Add noise
    std::mt19937 gen(42);
    std::normal_distribution<double> noise_theta(0.0, std::sqrt(R(1,1)));
    std::normal_distribution<double> noise_theta_dot(0.0, std::sqrt(R(0,0)));
    Eigen::MatrixXd z(2, N_steps + 1);
    for (int k = 0; k < N_steps + 1; ++k) {
        z(0, k) = X_true(0, k) + noise_theta_dot(gen);
        z(1, k) = X_true(1, k) + noise_theta(gen);
    }
    std::normal_distribution<double> noise_u(0.0, std::sqrt(Q(0,0)));
    Eigen::MatrixXd u_seq_noisy = u_seq;
    for (int k = 0; k < N_steps; ++k) {
        u_seq_noisy(0, k) += noise_u(gen);
    }

    // Kalman filter
    Eigen::MatrixXd X_hat = kf.discrete_time_filter(x0, P0, u_seq_noisy, z.leftCols(N_steps));

    // Get absolute path to results directory relative to source
    std::filesystem::path results_dir = std::filesystem::absolute("../results");
    std::filesystem::create_directories(results_dir);
    std::filesystem::path csv_path = results_dir / "kalman_results.csv";
    std::ofstream fout(csv_path);
    fout << "Time,True_theta,True_theta_dot,Meas_theta,Meas_theta_dot,Est_theta,Est_theta_dot\n";
    for (int k = 0; k < N_steps + 1; ++k) {
        fout << k * T << ","
             << X_true(1, k) << "," << X_true(0, k) << ","
             << z(1, k) << "," << z(0, k) << ",";
        if (k < X_hat.cols())
            fout << X_hat(1, k) << "," << X_hat(0, k);
        else
            fout << ",";
        fout << "\n";
    }
    fout.close();
    std::cout << "Results saved to " << csv_path << std::endl;
    return 0;
}
