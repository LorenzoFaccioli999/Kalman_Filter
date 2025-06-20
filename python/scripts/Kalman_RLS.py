import numpy as np
import matplotlib.pyplot as plt

def kalman_constant_estimation(zs, C, R, x0, P0):
    """
    This function estimates a constant value from noisy measurements using the Kalman filter approach, through
    recursive least squares estimation.

    Parameters:
    zs : array-like, Measurements (noisy observations of the constant)
    C : C matrix of the system (gain factor)
    R : float, Measurement noise variance
    x0 : float, Initial estimate of the constant
    P0 : float, Initial error covariance
    """
    x_estimates = []
    x = x0
    P = P0

    for z in zs:
        # Kalman gain
        K = P * C / (C * P * C + R)

        # Update estimate with measurement z
        x = x + K * (z - C * x)

        # Update the error covariance
        P = (1 - K * C) * P

        x_estimates.append(x)

    return np.array(x_estimates)


def simulate_constant_estimation():
    true_x = 1.5  # True value of the constant to be estimated
    C = 10
    R = 1   # Measurement noise variance
    N = 100  # number of measurements

    noise = np.random.normal(0, np.sqrt(R), size=N) # Generate noise
    measurements = C * true_x + noise

    estimates = kalman_constant_estimation(measurements, C=C, R=R, x0=0, P0=0.1)

    # Plotting
    plt.figure()
    plt.plot(measurements / C, label='Noisy measurements (normalized)')
    plt.plot(estimates, label='Kalman estimate')
    plt.axhline(true_x, color='r', linestyle='--', label='True value')
    plt.legend()
    plt.xlabel('Time step')
    plt.ylabel('Estimated value')
    plt.title('Recursive Least Squares (Kalman Filter) Estimation of Constant')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    simulate_constant_estimation()