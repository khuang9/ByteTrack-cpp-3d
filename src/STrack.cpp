#include "ByteTrack/STrack.h"

#include <cstddef>

byte_track::STrack::STrack(const Rect<float>& rect, const float& score, const int& class_id, const bool& use_maj_cls) :
    kalman_filter_(),
    mean_(),
    covariance_(),
    rect_(rect),
    state_(STrackState::New),
    is_activated_(false),
    use_majority_class_(use_maj_cls),
    score_(score),
    class_id_(class_id),
    track_id_(0),
    frame_id_(0),
    start_frame_id_(0),
    tracklet_len_(0)
{
    class_count_[class_id] = 1;
}

byte_track::STrack::~STrack()
{
}

const byte_track::Rect<float>& byte_track::STrack::getRect() const
{
    return rect_;
}

const byte_track::STrackState& byte_track::STrack::getSTrackState() const
{
    return state_;
}

const bool& byte_track::STrack::isActivated() const
{
    return is_activated_;
}
const bool& byte_track::STrack::useMajorityClass() const
{
    return use_majority_class_;
}
const float& byte_track::STrack::getScore() const
{
    return score_;
}
const int& byte_track::STrack::getClassId() const
{
    return class_id_;
}
const std::unordered_map<int, int>& byte_track::STrack::getClassCount() const
{
    return class_count_;
}

const size_t& byte_track::STrack::getTrackId() const
{
    return track_id_;
}

const size_t& byte_track::STrack::getFrameId() const
{
    return frame_id_;
}

const size_t& byte_track::STrack::getStartFrameId() const
{
    return start_frame_id_;
}

const size_t& byte_track::STrack::getTrackletLength() const
{
    return tracklet_len_;
}

void byte_track::STrack::updateClass(int new_class_id)
{
    if (!use_majority_class_) {
        class_id_ = new_class_id;
        return;
    }

    if (new_class_id == class_id_) {
        ++class_count_[class_id_];
        return;
    }

    auto it = class_count_.find(new_class_id);
    if (it == class_count_.end()) class_count_[new_class_id] = 1;
    else {
        ++it->second;
        if (it->second > class_count_[class_id_]) class_id_ = new_class_id;
    }
}

void byte_track::STrack::activate(const size_t& frame_id, const size_t& track_id)
{
    kalman_filter_.initiate(mean_, covariance_, rect_.getXyzolwh());

    updateRect();

    state_ = STrackState::Tracked;
    if (frame_id == 1)
    {
        is_activated_ = true;
    }
    track_id_ = track_id;
    frame_id_ = frame_id;
    start_frame_id_ = frame_id;
    tracklet_len_ = 0;
}

void byte_track::STrack::reActivate(const STrack &new_track, const size_t &frame_id, const int &new_track_id)
{
    kalman_filter_.update(mean_, covariance_, new_track.getRect().getXyzolwh(), new_track.getScore());

    updateRect();

    state_ = STrackState::Tracked;
    is_activated_ = true;
    score_ = new_track.getScore();
    updateClass(new_track.getClassId());
    if (0 <= new_track_id)
    {
        track_id_ = new_track_id;
    }
    frame_id_ = frame_id;
    tracklet_len_ = 0;
}

void byte_track::STrack::predict()
{
    kalman_filter_.predict(mean_, covariance_);
}

void byte_track::STrack::update(const STrack &new_track, const size_t &frame_id)
{
    kalman_filter_.update(mean_, covariance_, new_track.getRect().getXyzolwh(), new_track.getScore());

    updateRect();

    state_ = STrackState::Tracked;
    is_activated_ = true;
    score_ = new_track.getScore();
    updateClass(new_track.getClassId());
    frame_id_ = frame_id;
    tracklet_len_++;
}

void byte_track::STrack::markAsLost()
{
    state_ = STrackState::Lost;
}

void byte_track::STrack::markAsRemoved()
{
    state_ = STrackState::Removed;
}

void byte_track::STrack::updateRect()
{
    rect_.length() = mean_[4];
    rect_.width() = mean_[5];
    rect_.height() = mean_[6];
    rect_.x() = mean_[0];
    rect_.y() = mean_[1];
    rect_.z() = mean_[2];
    rect_.yaw() = mean_[3];
}
