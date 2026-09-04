#include "control/ped_relative_away.h"

#include <algorithm>
#include <cmath>

namespace ped_relative {

void AwayTracker::reset()
{
    side_ = Side::None;
    samples_.clear();
}

void AwayTracker::startWith(const Sample& sample)
{
    side_ = sample.side;
    samples_.push_back(sample);
}

float AwayTracker::lastClearance() const
{
    return samples_.empty() ? 0.0f : samples_.back().clearance_ratio;
}

bool AwayTracker::push(const Sample& sample, int confirm_required,
                       float min_growth_ratio)
{
    const int required = std::max(3, confirm_required);
    const float growth = std::max(0.0f, min_growth_ratio);
    const float reverse_tolerance = growth * 0.5f;

    if (sample.side == Side::None ||
        !std::isfinite(sample.clearance_ratio) ||
        sample.clearance_ratio < 0.0f) {
        reset();
        return false;
    }

    if (!samples_.empty()) {
        const float delta =
            sample.clearance_ratio - samples_.back().clearance_ratio;
        if (sample.side != side_ || std::fabs(delta) > 0.50f ||
            delta < -reverse_tolerance) {
            reset();
        }
    }

    if (samples_.empty()) startWith(sample);
    else samples_.push_back(sample);

    if (static_cast<int>(samples_.size()) < required) return false;

    const bool confirmed =
        samples_.back().clearance_ratio - samples_.front().clearance_ratio >=
        growth;
    if (confirmed) return true;

    const Sample latest = samples_.back();
    reset();
    startWith(latest);
    return false;
}

} // namespace ped_relative
