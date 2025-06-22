import numpy as np
from scipy.linalg import expm

class KalmanFilter:
    def __init__(self, A, B, C, D=None, L=None, Q=None, R=None):
        self.A = A
        self.B = B
        self.C = C
        self.D = D if D is not None else np.zeros_like(B)
        self.L = L if L is not None else np.zeros_like(B)
        self.Q = Q if Q is not None else np.zeros((B.shape[1], B.shape[1]))
        self.R = R if R is not None else np.zeros((C.shape[0], C.shape[0]))

    def compute_discrete_matrices(self, T):
        """
        Computes the discrete-time state transition matrices based on the sampling time T.
        """
        self.Phi = expm(self.A * T)
        self.Gamma = np.linalg.inv(self.A) @ (self.Phi - np.eye(self.A.shape[0])) @ self.B
        self.Lambda = np.linalg.inv(self.A) @ (self.Phi - np.eye(self.A.shape[0])) @ self.L

    def recursive_least_square(self, zs, x0, P0):
        """
        Estimates a constant value using recursive least squares (Kalman filter variant).
        Only valid for constant estimation problems with scalar system.
        """
        x_estimates = []
        x = x0
        P = P0
        for z in zs:
            K = P * self.C / (self.C * P * self.C + self.R)
            x = x + K * (z - self.C * x)
            P = (1 - K * self.C) * P
            x_estimates.append(x)
        return np.array(x_estimates)

    def discrete_time_filter(self, x0, P0, u_seq, z):
        """
        Executes a discrete-time Kalman filter over input u_seq and noisy measurements z.

        Parameters:
            x0: Initial state estimate (n x 1)
            P0: Initial state covariance (n x n)
            u_seq: Control input sequence (m x N)
            z: Measurement sequence (n x N)

        Returns:
            X_hat: Estimated state trajectory (n x N+1)
        """
        N_steps = u_seq.shape[1]
        n = x0.shape[0]
        X_hat = np.zeros((n, N_steps + 1))
        X_hat[:, 0] = x0.squeeze()
        P = np.copy(P0)

        for k in range(N_steps):
            # Prediction
            X_hat[:, k + 1] = self.Phi @ X_hat[:, k] + self.Gamma @ u_seq[:, k]
            P = self.Phi @ P @ self.Phi.T + self.Lambda @ self.Q @ self.Lambda.T

            # Kalman gain
            K = P @ self.C.T @ np.linalg.inv(self.C @ P @ self.C.T + self.R)

            # Correction
            X_hat[:, k + 1] += K @ (z[:, k] - self.C @ X_hat[:, k + 1])
            P = (np.eye(n) - K @ self.C) @ P

        return X_hat
