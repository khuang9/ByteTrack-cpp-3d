#include "ByteTrack/Object.h"

#include <string>

byte_track::Object::Object(const Rect<float> &_rect,
                           const std::string &_label,
                           const float &_prob) : rect(_rect), label(_label), prob(_prob)
{
}