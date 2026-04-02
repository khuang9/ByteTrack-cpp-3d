#pragma once

#include "ByteTrack/Rect.h"

#include <string>

namespace byte_track
{
struct Object
{
    Rect<float> rect;
    std::string label;
    float prob;

    Object(const Rect<float> &_rect,
           const std::string &_label,
           const float &_prob);
};
}