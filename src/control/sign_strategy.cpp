#include "control/sign_strategy.h"

namespace sign_strategy {

Direction opposite(Direction direction)
{
    if (direction == Direction::Straight) return Direction::Right;
    if (direction == Direction::Right) return Direction::Straight;
    return Direction::None;
}

Direction parseFixedDirection(const std::string& value)
{
    if (value == "straight") return Direction::Straight;
    if (value == "right") return Direction::Right;
    return Direction::None;
}

void ComplementState::reset()
{
    hasFirst_ = false;
    secondArmed_ = false;
    consumed_ = false;
    first_ = Direction::None;
}

void ComplementState::syncEnabled(bool enabled)
{
    if (enabled_ != enabled) reset();
    enabled_ = enabled;
}

bool ComplementState::recordFirst(Direction direction)
{
    if (!enabled_ || hasFirst_ || consumed_ || direction == Direction::None)
        return false;
    first_ = direction;
    hasFirst_ = true;
    secondArmed_ = false;
    return true;
}

bool ComplementState::armSecond()
{
    if (!enabled_ || !hasFirst_ || secondArmed_ || consumed_)
        return false;
    secondArmed_ = true;
    return true;
}

Direction ComplementState::trySecond(float signScore, float minimumScore,
                                     bool confirmedFork)
{
    if (enabled_ && hasFirst_ && secondArmed_ && !consumed_ &&
        signScore > minimumScore && confirmedFork) {
        consumed_ = true;
        return opposite(first_);
    }
    return Direction::None;
}

bool ComplementState::awaitingSecond() const
{
    return enabled_ && hasFirst_ && !consumed_;
}

bool ComplementState::consumed() const
{
    return consumed_;
}

Direction ComplementState::firstDirection() const
{
    return first_;
}

}  // namespace sign_strategy
