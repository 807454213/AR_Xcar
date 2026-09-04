#include "control/sign_strategy.h"

#include <iostream>
#include <limits>

using sign_strategy::Direction;

static bool require(bool value, const char* message)
{
    if (!value) std::cerr << "FAIL: " << message << "\n";
    return value;
}

int main()
{
    bool ok = true;

    sign_strategy::ComplementState state;
    state.syncEnabled(true);
    ok &= require(state.recordFirst(Direction::Straight), "record first");
    ok &= require(state.awaitingSecond(), "await second");
    ok &= require(state.trySecond(0.90f, 0.85f, true) == Direction::None,
                  "recording first leaves second trigger unarmed");
    ok &= require(state.awaitingSecond() && !state.consumed(),
                  "unarmed trigger does not consume first direction");
    ok &= require(state.armSecond(), "arm second encounter");
    ok &= require(!state.armSecond(), "second encounter arms only once");
    ok &= require(state.trySecond(0.90f, 0.85f, false) == Direction::None,
                  "SIGN alone must not trigger");
    ok &= require(state.trySecond(0.80f, 0.85f, true) == Direction::None,
                  "fork alone must not trigger");
    ok &= require(state.trySecond(0.90f, 0.85f, true) == Direction::Right,
                  "joint trigger complements straight");
    ok &= require(state.trySecond(0.99f, 0.85f, true) == Direction::None,
                  "second trigger is one-shot");

    state.reset();
    ok &= require(state.recordFirst(Direction::Right),
                  "record first after reset");
    ok &= require(state.trySecond(0.90f, 0.85f, true) == Direction::None,
                  "reset clears second encounter arming");

    state.syncEnabled(false);
    state.syncEnabled(true);
    ok &= require(state.recordFirst(Direction::Straight),
                  "record first after enable transition");
    ok &= require(state.trySecond(0.90f, 0.85f, true) == Direction::None,
                  "enable transition clears second encounter arming");

    sign_strategy::ComplementState nanState;
    nanState.syncEnabled(true);
    ok &= require(nanState.recordFirst(Direction::Straight),
                  "record first for NaN score");
    ok &= require(nanState.armSecond(), "arm second for NaN score");
    ok &= require(nanState.trySecond(
                      std::numeric_limits<float>::quiet_NaN(), 0.85f, true) ==
                      Direction::None,
                  "NaN score must not trigger");
    ok &= require(nanState.awaitingSecond() && !nanState.consumed(),
                  "NaN score must not consume state");

    sign_strategy::ComplementState equalityState;
    equalityState.syncEnabled(true);
    ok &= require(equalityState.recordFirst(Direction::Straight) &&
                  equalityState.armSecond(), "arm equality threshold state");
    ok &= require(equalityState.trySecond(0.85f, 0.85f, true) ==
                      Direction::None,
                  "score equal to threshold must not trigger");
    ok &= require(equalityState.trySecond(0.86f, 0.85f, true) ==
                      Direction::Right,
                  "greater score triggers after equality did not consume state");

    ok &= require(sign_strategy::opposite(Direction::Right) ==
                      Direction::Straight,
                  "right complements to straight");
    ok &= require(sign_strategy::parseFixedDirection("straight") ==
                      Direction::Straight,
                  "parse fixed straight direction");
    ok &= require(sign_strategy::parseFixedDirection("right") ==
                      Direction::Right,
                  "parse fixed right direction");
    ok &= require(sign_strategy::parseFixedDirection("Straight") ==
                      Direction::None,
                  "fixed direction is case-sensitive");
    ok &= require(sign_strategy::parseFixedDirection("left") ==
                      Direction::None,
                  "invalid fixed direction stays invalid");

    return ok ? 0 : 2;
}
