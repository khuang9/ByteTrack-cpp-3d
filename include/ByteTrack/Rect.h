#pragma once

#include "Eigen/Dense"
#include "ByteTrack/iou_utils.h"

namespace byte_track
{
template<typename T>
using Xyzolwh = Eigen::Matrix<T, 1, 7, Eigen::RowMajor>;

template<typename T>
class Rect
{
    public:
    Xyzolwh<T> xyzolwh;

    Rect() = default;
    Rect(const T &x, const T &y, const T &z, const T &yaw, const T &length, const T &width, const T &height, const float &ciou_alpha = 0.3f);

    ~Rect();

    const T &x() const;
    const T &y() const;
    const T &z() const;
    const T &yaw() const;
    const T &length() const;
    const T &width() const;
    const T &height() const;

    T &x();
    T &y();
    T &z();
    T &yaw();
    T &length();
    T &width();
    T &height();

    Xyzolwh<T> getXyzolwh() const;

    // 3D IoU calcs
    float minEnclosingBoxDiag(const Rect<T>& other) const;
    float calcBEVIntersection(const Rect<T>& other) const;
    float calcIoU(const Rect<T>& other) const;
    float calcDIoU(const Rect<T>& other) const;
    float calcCIoU(const Rect<T>& other) const;

    float ciou_alpha_;
};
}
