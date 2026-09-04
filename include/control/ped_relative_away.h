#pragma once

#include <cstdint>
#include <deque>

namespace ped_relative {

enum class Side : int8_t {
    None = 0,
    Left = -1,
    Right = 1,
};

struct Sample {
    Side side = Side::None;
    float clearance_ratio = 0.0f;
    int foot_x = -1;
    int foot_y = -1;
};

class AwayTracker {
public:
    bool push(const Sample& sample, int confirm_required,
              float min_growth_ratio);
    void reset();

    int count() const { return static_cast<int>(samples_.size()); }
    Side side() const { return side_; }
    float lastClearance() const;

private:
    void startWith(const Sample& sample);

    Side side_ = Side::None;
    std::deque<Sample> samples_;
};

} // namespace ped_relative
