#include "kalmanFilter.h"

KalmanFilter::KalmanFilter() {
    // Initialize measurement matrix H (state to measurement m by n)
    H[0][0] = 1;
    H[0][1] = 0;

    // Covariance matrix of Wk (n by n matrix)
    Q[0][0] = 0.5; Q[0][1] = 0;
    Q[1][0] = 0; Q[1][1] = 1.5; // set to 0.05 and 0.15 by testing different values
    // lower value means we trust the measurements less

    // Measurement noise covariance R
    R = 60; // change if we got the data sheet, rn it's at 60 cuz it gives the smoothest velocity
    // If we want smoother velocity, increase but might oversmooth

    // Initial state [altitude, velocity]
    x[0] = 0;
    x[1] = 0;

    // Initial covariance matrix P
    P[0][0] = 1; P[0][1] = 0;
    P[1][0] = 0; P[1][1] = 1; // set to 1 because lower number means it trusts its own prediction more

}

void KalmanFilter::update(double z, time_t time1, long long ms) {
    const double dt = 0.011;  // Fixed dt value for all updates (smoother and closer to the real values than dynamic dt)

    // Update state transition matrix A
    A[0][0] = 1; A[0][1] = dt;
    A[1][0] = 0; A[1][1] = 1;

    // Prediction step
    double xp[2]; // Predicted state
    xp[0] = x[0] + x[1] * dt;
    xp[1] = x[1];

    // Compute predicted covariance: Pp = A * P * A' + Q
    double temp[2][2];  // Temporary storage
    double Pp[2][2];    // Predicted covariance

    matrixMultiply(A, P, temp);
    double A_transpose[2][2] = {{A[0][0], A[1][0]}, {A[0][1], A[1][1]}};
    matrixMultiply(temp, A_transpose, Pp);
    matrixAdd(Pp, Q, Pp);

    // Innovation covariance S = H * Pp * H' + R
    double S = Pp[0][0] + R;

    // Kalman Gain K = Pp * H' * inv(S)
    double K[2];
    K[0] = Pp[0][0] / S;
    K[1] = Pp[1][0] / S;

    // Update step
    double innovation = z - xp[0]; // Measurement residual
    x[0] = xp[0] + K[0] * innovation;
    x[1] = xp[1] + K[1] * innovation;

    // Update covariance P = (I - KH) * Pp
    double I[2][2] = {{1, 0}, {0, 1}}; // Identity matrix
    double KH[2][2]; // K * H
    KH[0][0] = K[0] * H[0][0]; KH[0][1] = K[0] * H[0][1];
    KH[1][0] = K[1] * H[0][0]; KH[1][1] = K[1] * H[0][1];

    double tempKH[2][2];
    matrixSubtract(I, KH, tempKH);
    matrixMultiply(tempKH, Pp, P);
}


double KalmanFilter::getPosition() const {
    return x[0];
}

double KalmanFilter::getVelocity() const {
    return x[1];
}

// Helper functions for matrix operations
void KalmanFilter::matrixMultiply(double a[2][2], double b[2][2], double result[2][2]) {
    double temp[2][2]; // Prevent overwriting result
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            temp[i][j] = 0;
            for (int k = 0; k < 2; ++k) {
                temp[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    // Copy back to result
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            result[i][j] = temp[i][j];
}

void KalmanFilter::matrixVectorMultiply(double matrix[2][2], double vector[2], double result[2]) {
    for (int i = 0; i < 2; ++i) {
        result[i] = 0;
        for (int j = 0; j < 2; ++j) {
            result[i] += matrix[i][j] * vector[j];
        }
    }
}

void KalmanFilter::matrixAdd(double a[2][2], double b[2][2], double result[2][2]) {
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            result[i][j] = a[i][j] + b[i][j];
        }
    }
}

void KalmanFilter::matrixSubtract(double a[2][2], double b[2][2], double result[2][2]) {
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            result[i][j] = a[i][j] - b[i][j];
        }
    }
}
