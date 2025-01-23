#include "slam_kalman/estimation/kalman_filter.hpp"

KalmanFilter::KalmanFilter(
    const Eigen::MatrixXd& F,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P
) : F_(F), B_(B), H_(H), Q_(Q), R_(R), P_(P), x_(VectorXd::Zero(F.rows())) {}

void KalmanFilter::predict(const Eigen::VectorXd& u) {
    x_ = F_ * x_ + B_ * u;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd& z) {
    const Eigen::VectorXd y = z - H_ * x_;
    const Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    const Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    
    x_ += K * y;
    const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
    P_ = P_ - K * H_ * P_;
}