#include "ByteTrack/Rect.h"

#include <algorithm>
#include <cmath>

template <typename T>
byte_track::Rect<T>::Rect(const T &x, const T &y, const T &z, const T &yaw, const T &length, const T &width, const T &height, const float &ciou_alpha) :
    xyzolwh({x, y, z, yaw, length, width, height}),
    ciou_alpha_(ciou_alpha)
{
}

template <typename T>
byte_track::Rect<T>::~Rect()
{
}

template <typename T>
const T& byte_track::Rect<T>::x() const
{
    return xyzolwh[0];
}

template <typename T>
const T& byte_track::Rect<T>::y() const
{
    return xyzolwh[1];
}

template <typename T>
const T& byte_track::Rect<T>::z() const
{
    return xyzolwh[2];
}

template <typename T>
const T& byte_track::Rect<T>::yaw() const
{
    return xyzolwh[3];
}

template <typename T>
const T& byte_track::Rect<T>::length() const
{
    return xyzolwh[4];
}

template <typename T>
const T& byte_track::Rect<T>::width() const
{
    return xyzolwh[5];
}

template <typename T>
const T& byte_track::Rect<T>::height() const
{
    return xyzolwh[6];
}

template <typename T>
T& byte_track::Rect<T>::x()
{
    return xyzolwh[0];
}

template <typename T>
T& byte_track::Rect<T>::y()
{
    return xyzolwh[1];
}

template <typename T>
T& byte_track::Rect<T>::z()
{
    return xyzolwh[2];
}

template <typename T>
T& byte_track::Rect<T>::yaw()
{
    return xyzolwh[3];
}

template <typename T>
T& byte_track::Rect<T>::length()
{
    return xyzolwh[4];
}

template <typename T>
T& byte_track::Rect<T>::width()
{
    return xyzolwh[5];
}

template <typename T>
T& byte_track::Rect<T>::height()
{
    return xyzolwh[6];
}

template <typename T>
byte_track::Xyzolwh<T> byte_track::Rect<T>::getXyzolwh() const
{
    return {
        xyzolwh[0],
        xyzolwh[1],
        xyzolwh[2],
        xyzolwh[3],
        xyzolwh[4],
        xyzolwh[5],
        xyzolwh[6],
    };
}

template<typename T>
float byte_track::Rect<T>::minEnclosingBoxDiag(const Rect<T>& other) const
{
    Box rrect{
        Point{static_cast<double>(xyzolwh[0]), static_cast<double>(xyzolwh[1])},
        static_cast<double>(xyzolwh[4]),
        static_cast<double>(xyzolwh[5]),
        static_cast<double>(xyzolwh[3])
    };
    Box other_rrect{
        Point{static_cast<double>(other.xyzolwh[0]), static_cast<double>(other.xyzolwh[1])},
        static_cast<double>(other.xyzolwh[4]),
        static_cast<double>(other.xyzolwh[5]),
        static_cast<double>(other.xyzolwh[3])
    };

    float min_diag_sq = minAreaBoxDiagSquared(rrect, other_rrect);
    float max_z = std::max(xyzolwh[2] + xyzolwh[6] / 2, other.xyzolwh[2] + other.xyzolwh[6] / 2);
    float min_z = std::min(xyzolwh[2] - xyzolwh[6] / 2, other.xyzolwh[2] - other.xyzolwh[6] / 2);
    float height = max_z - min_z;

    return std::sqrt(min_diag_sq + height*height);
}

template<typename T>
float byte_track::Rect<T>::calcBEVIntersection(const Rect<T>& other) const
{
    Box rrect{
        Point{static_cast<double>(xyzolwh[0]), static_cast<double>(xyzolwh[1])},
        static_cast<double>(xyzolwh[4]),
        static_cast<double>(xyzolwh[5]),
        static_cast<double>(xyzolwh[3])
    };
    Box other_rrect{
        Point{static_cast<double>(other.xyzolwh[0]), static_cast<double>(other.xyzolwh[1])},
        static_cast<double>(other.xyzolwh[4]),
        static_cast<double>(other.xyzolwh[5]),
        static_cast<double>(other.xyzolwh[3])
    };

    return boxIntersectArea(rrect, other_rrect);
}

template<typename T>
float byte_track::Rect<T>::calcIoU(const Rect<T>& other) const
{
    float z_top = std::min(xyzolwh[6]/2, other.xyzolwh[6]/2);
    float z_bot = std::max(-xyzolwh[6]/2, -other.xyzolwh[6]/2);
    float z_inter = std::max(0.0f, z_top - z_bot);

    float vol = xyzolwh[4] * xyzolwh[5] * xyzolwh[6];
    float other_vol = other.xyzolwh[4] * other.xyzolwh[5] * other.xyzolwh[6];
    float inter_vol = z_inter * calcBEVIntersection(other);
    float union_vol = vol + other_vol - inter_vol;

    if (union_vol <= 0.0f) return 0.0f;
    else return inter_vol / union_vol;
}

template<typename T>
float byte_track::Rect<T>::calcDIoU(const Rect<T>& other) const
{
    float dx = xyzolwh[0] - other.xyzolwh[0];
    float dy = xyzolwh[1] - other.xyzolwh[1];
    float dz = xyzolwh[2] - other.xyzolwh[2];
    float d = std::sqrt(dx*dx + dy*dy + dz*dz);
    float c = minEnclosingBoxDiag(other);
    return calcIoU(other) - (d*d) / c*c;
}

template<typename T>
float byte_track::Rect<T>::calcCIoU(const Rect<T>& other) const
{
    float theta_1 = std::atan2(xyzolwh[4], xyzolwh[5]) - std::atan2(other.xyzolwh[4], xyzolwh[5]);
    float theta_2    = std::atan2(xyzolwh[4], xyzolwh[6]) - std::atan2(other.xyzolwh[4], xyzolwh[6]);
    float pi = std::acos(-1.0);
    float v = 2*(theta_1*theta_1 + theta_2*theta_2) / (pi*pi);
    return calcDIoU(other) - ciou_alpha_*v;
}

// explicit instantiation
template class byte_track::Rect<int>;
template class byte_track::Rect<float>;
