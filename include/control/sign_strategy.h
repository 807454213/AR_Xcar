#ifndef XCAR2_CONTROL_SIGN_STRATEGY_H
#define XCAR2_CONTROL_SIGN_STRATEGY_H

#include <string>

namespace sign_strategy {

enum class Direction : unsigned char { None = 0, Straight = 1, Right = 2 };

Direction opposite(Direction direction);
Direction parseFixedDirection(const std::string& value);

class ComplementState {
public:
    void reset();
    void syncEnabled(bool enabled);
    bool recordFirst(Direction direction);
    bool armSecond();
    Direction trySecond(float signScore, float minimumScore,
                        bool confirmedFork);
    bool awaitingSecond() const;
    bool consumed() const;
    Direction firstDirection() const;

private:
    bool enabled_ = false;
    bool hasFirst_ = false;
    bool secondArmed_ = false;
    bool consumed_ = false;
    Direction first_ = Direction::None;
};

}  // namespace sign_strategy

#endif  // XCAR2_CONTROL_SIGN_STRATEGY_H
