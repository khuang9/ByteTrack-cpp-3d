#include "ByteTrack/KalmanFilter.h"

#include <cmath>
#include <cstddef>

byte_track::KalmanFilter::KalmanFilter(const float& std_weight_position,
                                       const float& std_weight_velocity,
                                       const float& alpha,
                                       const float& beta) :
    std_weight_position_(std_weight_position),
    std_weight_velocity_(std_weight_velocity),
    alpha_(alpha),
    beta_(beta)
{
    constexpr size_t ndim = 3;
    constexpr float dt = 1;

    motion_mat_ = Eigen::MatrixXf::Identity(10, 10);
    update_mat_ = Eigen::MatrixXf::Identity(7, 10);

    for (size_t i = 0; i < ndim; i++)
    {
        motion_mat_(i, 4 + ndim + i) = dt;
    }
}

void byte_track::KalmanFilter::initiate(StateMean &mean, StateCov &covariance, const DetectBox &measurement)
{
    mean.block<1, 7>(0, 0) = measurement.block<1, 7>(0, 0);
    mean.block<1, 3>(0, 7) = Eigen::Vector3f::Zero();

    // Matrix defaults follow AB3DMOT
    StateMean std;
    for (int i = 0; i < 7; ++i)
        std(i) = 10 * std_weight_position_;
    for (int i = 7; i < 10; ++i)
        std(i) = 10000 * std_weight_velocity_;

    StateMean tmp = std.array().square();
    covariance = tmp.asDiagonal();
}

void byte_track::KalmanFilter::predict(StateMean &mean, StateCov &covariance)
{
    // Matrix defaults follow AB3DMOT
    StateMean std;
    for (int i = 0; i < 7; ++i)
        std(i) = std_weight_position_;
    for (int i = 7; i < 10; ++i)
        std(i) = 0.01 * std_weight_velocity_;

    StateMean tmp = std.array().square();
    StateCov motion_cov = tmp.asDiagonal();

    mean = motion_mat_ * mean.transpose();
    covariance = motion_mat_ * covariance * (motion_mat_.transpose()) + motion_cov;
}

void byte_track::KalmanFilter::update(StateMean &mean, StateCov &covariance, const DetectBox &measurement, const float &confidence)
{
    StateHMean projected_mean;
    StateHCov projected_cov;
    project(projected_mean, projected_cov, mean, covariance, confidence);

    Eigen::Matrix<float, 7, 10> B = (covariance * (update_mat_.transpose())).transpose();
    Eigen::Matrix<float, 10, 7> kalman_gain = (projected_cov.llt().solve(B)).transpose();
    Eigen::Matrix<float, 1, 7> innovation = measurement - projected_mean;

    const auto tmp = innovation * (kalman_gain.transpose());
    mean = (mean.array() + tmp.array()).matrix();
    covariance = covariance - kalman_gain * projected_cov * (kalman_gain.transpose());
}

void byte_track::KalmanFilter::project(StateHMean &projected_mean, StateHCov &projected_covariance,
                                       const StateMean& mean, const StateCov& covariance, const float &confidence)
{
    DetectBox std;
    for (int i = 0; i < 7; ++i)
        std(i) = std_weight_position_;

    projected_mean = update_mat_ * mean.transpose();
    projected_covariance = update_mat_ * covariance * (update_mat_.transpose());

    // Dynamic R scaling inspired by ByteTrackV2 paper
    Eigen::Matrix<float, 7, 7> diag = (1 + alpha_*std::pow(1 - confidence, beta_)) * (std.asDiagonal());
    projected_covariance += diag.array().square().matrix();
}
