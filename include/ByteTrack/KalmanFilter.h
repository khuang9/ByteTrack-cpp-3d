#pragma once

#include "Eigen/Dense"

#include "ByteTrack/Rect.h"

namespace byte_track
{
class KalmanFilter
{
public:
    using DetectBox = Xyzolwh<float>;

    using StateMean = Eigen::Matrix<float, 1, 10, Eigen::RowMajor>;
    using StateCov = Eigen::Matrix<float, 10, 10, Eigen::RowMajor>;

    using StateHMean = Eigen::Matrix<float, 1, 7, Eigen::RowMajor>;
    using StateHCov = Eigen::Matrix<float, 7, 7, Eigen::RowMajor>;

    KalmanFilter(const float& std_weight_position = 1.,
                 const float& std_weight_velocity = 1.,
                 const float& alpha = 0.,
                 const float& beta = 2.);

    static constexpr double pi = 3.141592653589793;

    void initiate(StateMean& mean, StateCov& covariance, const DetectBox& measurement);

    void predict(StateMean& mean, StateCov& covariance);

    float wrapAngle(float angle);

    void update(StateMean& mean, StateCov& covariance, const DetectBox& measurement, const float& confidence);

    void enable_R_scaling();

    void disable_R_scaling();

private:
    float std_weight_position_;
    float std_weight_velocity_;
    float alpha_;
    float beta_;

    Eigen::Matrix<float, 10, 10, Eigen::RowMajor> motion_mat_;
    Eigen::Matrix<float, 7, 10, Eigen::RowMajor> update_mat_;

    void project(StateHMean &projected_mean, StateHCov &projected_covariance,
                 const StateMean& mean, const StateCov& covariance, const float& confidence);
};
}
