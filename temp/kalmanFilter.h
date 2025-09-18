#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <chrono>

class KalmanFilter {
private:
    double A[2][2]; // State transition matrix
    double H[1][2]; // Measurement matrix
    double Q[2][2]; // Process noise covariance
    double R;       // Measurement noise covariance
    double x[2];    // State [position, velocity]
    double P[2][2]; // State covariance

public:
    KalmanFilter();
    void update(double z, time_t time1, long long ms); // Now takes in a timestamp
    double getPosition() const;
    double getVelocity() const;

private:
    void matrixMultiply(double a[2][2], double b[2][2], double result[2][2]);
    void matrixVectorMultiply(double matrix[2][2], double vector[2], double result[2]);
    void matrixAdd(double a[2][2], double b[2][2], double result[2][2]);
    void matrixSubtract(double a[2][2], double b[2][2], double result[2][2]);
};

#endif // KALMAN_FILTER_H
