#pragma once

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter(
        const Eigen::MatrixXd& F,
        const Eigen::MatrixXd& B,
        const Eigen::MatrixXd& H,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& P
    );

    void Predict(const Eigen::VectorXd& u);
    void Update(const Eigen::VectorXd& z);
    
    Eigen::VectorXd state() const { return x_; }

private:
    Eigen::MatrixXd F_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd P_;
    Eigen::VectorXd x_;
};