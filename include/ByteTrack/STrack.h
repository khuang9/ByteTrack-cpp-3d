#pragma once

#include "ByteTrack/Rect.h"
#include "ByteTrack/KalmanFilter.h"

#include <cstddef>
#include <string>

namespace byte_track
{
enum class STrackState {
    New = 0,
    Tracked = 1,
    Lost = 2,
    Removed = 3,
};

class STrack
{
public:
    STrack(const Rect<float>& rect, const float& score, const std::string& class_id, const bool& use_maj_cls = true, const bool& use_R_scaling = false);
    ~STrack();

    const Rect<float>& getRect() const;
    const STrackState& getSTrackState() const;

    const bool& isActivated() const;
    const bool& useMajorityClass() const;
    const float& getScore() const;
    const std::string& getClassId() const;
    const std::unordered_map<std::string, int>& getClassCount() const;
    const size_t& getTrackId() const;
    const size_t& getFrameId() const;
    const size_t& getStartFrameId() const;
    const size_t& getTrackletLength() const;
    const KalmanFilter::StateMean& getMean() const;
    const KalmanFilter::StateCov& getCov() const;

    void updateClass(const std::string& new_class_id);
    void activate(const size_t& frame_id, const size_t& track_id);
    void reActivate(const STrack &new_track, const size_t &frame_id, const int &new_track_id = -1);

    void predict();
    void update(const STrack &new_track, const size_t &frame_id);

    void markAsLost();
    void markAsRemoved();

private:
    KalmanFilter kalman_filter_;
    KalmanFilter::StateMean mean_;
    KalmanFilter::StateCov covariance_;

    Rect<float> rect_;
    STrackState state_;

    bool is_activated_;
    bool use_majority_class_;
    float score_;
    std::string class_id_;
    size_t track_id_;
    size_t frame_id_;
    size_t start_frame_id_;
    size_t tracklet_len_;

    std::unordered_map<std::string, int> class_count_;

    void updateRect();
};
}
