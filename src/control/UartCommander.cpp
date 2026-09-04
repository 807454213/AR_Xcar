#include "control/uart_commander.h"
#include "trackcontrol.h"
#include "uart.hpp"
#include <cmath>

UartCommander& UartCommander::instance()
{
    static UartCommander inst;
    return inst;
}

bool UartCommander::setMotionMode(uint8_t mode, const char* reason, bool force)
{
    if (batch_depth_ > 0) {
        return requestMotionMode(
            mode, MotionModeOwner::Normal, reason, force);
    }
    if (!force && mode == last_motion_)
        return true;
    const bool ok = Uart::instance().send(0x02, 1, mode);
    if (ok) {
        last_motion_ = mode;
        ++motion_send_count_;
    }
    return ok;
}

bool UartCommander::requestMotionMode(uint8_t mode,
                                      MotionModeOwner owner,
                                      const char* reason,
                                      bool force)
{
    if (batch_depth_ <= 0)
        return setMotionMode(mode, reason, force);

    const bool no_winner = !pending_motion_valid_;
    const bool higher_owner =
        static_cast<uint8_t>(owner) >
        static_cast<uint8_t>(pending_motion_owner_);
    const bool same_owner = owner == pending_motion_owner_;
    if (no_winner || higher_owner || same_owner) {
        pending_motion_valid_ = true;
        pending_motion_mode_ = mode;
        pending_motion_owner_ = owner;
        pending_motion_reason_ = reason;
        pending_motion_force_ = force;
    }
    return true;
}

uint8_t UartCommander::effectiveMotionMode() const
{
    if (batch_depth_ > 0 && pending_motion_valid_)
        return pending_motion_mode_;
    return last_motion_;
}

void UartCommander::beginMotionModeBatch()
{
    if (batch_depth_ == 0) {
        pending_motion_valid_ = false;
        pending_motion_mode_ = 0;
        pending_motion_owner_ = MotionModeOwner::Normal;
        pending_motion_reason_ = nullptr;
        pending_motion_force_ = false;
    }
    ++batch_depth_;
}

void UartCommander::endMotionModeBatch()
{
    if (batch_depth_ <= 0)
        return;
    --batch_depth_;
    if (batch_depth_ > 0 || !pending_motion_valid_)
        return;

    const uint8_t mode = pending_motion_mode_;
    const char* reason = pending_motion_reason_;
    const bool force = pending_motion_force_;
    pending_motion_valid_ = false;
    pending_motion_owner_ = MotionModeOwner::Normal;
    pending_motion_reason_ = nullptr;
    pending_motion_force_ = false;
    (void)setMotionMode(mode, reason, force);
}

bool UartCommander::setProtect(uint8_t value, const char* reason, bool force)
{
    if (!force && value == last_protect_)
        return true;
    const bool ok = Uart::instance().send(0x03, 1, value);
    if (ok) {
        last_protect_ = value;
    }
    return ok;
}

bool UartCommander::setCurveFlag(uint8_t value, const char* reason)
{
    if (value == last_curve_)
        return true;
    const bool ok = Uart::instance().send(0x07, 1, value);
    if (ok) {
        last_curve_ = value;
    }
    return ok;
}

bool UartCommander::setMaxSpeed(float value, const char* reason)
{
    if (last_speed_ > -1e8f && std::fabs(value - last_speed_) < 1e-3f)
        return true;
    const bool ok = Uart::instance().send(0x08, 4, value);
    if (ok) {
        last_speed_ = value;
    }
    return ok;
}

bool UartCommander::setForkDir(uint8_t dir, const char* reason, bool force)
{
    if (!force && dir == last_fork_)
        return true;
    const bool ok = Uart::instance().send(0x0B, 1, dir);
    if (ok) {
        last_fork_ = dir;
    }
    return ok;
}

bool UartCommander::sendStateFlag(uint8_t flag, const char* reason)
{
    if (flag < 1 || flag > 9)
        return false;
    if (flag == last_state_flag_)
        return true;
    const bool ok = Uart::instance().send(0x09, 1, flag);
    if (ok) {
        last_state_flag_ = flag;
    }
    return ok;
}

void UartCommander::sendError(float error)
{
    Uart::instance().send(0x01, 4, error);
}

void UartCommander::startCar()
{
    setProtect(0, "start clear protect", true);
    setMotionMode(0, "start clear stop", true);
    const bool start_ok = Uart::instance().send(0x05, 1, static_cast<uint8_t>(1));
    if (start_ok)
        tc_notify_launch_start();
}

void UartCommander::emergencyProtect(const char* reason)
{
    setProtect(1, reason ? reason : "emergency", true);
}

void UartCommander::reset()
{
    last_motion_  = kUnset;
    last_protect_ = kUnset;
    last_curve_   = kUnset;
    last_fork_    = kUnset;
    last_state_flag_ = kUnset;
    last_speed_   = -1e9f;
    batch_depth_ = 0;
    pending_motion_valid_ = false;
    pending_motion_mode_ = 0;
    pending_motion_owner_ = MotionModeOwner::Normal;
    pending_motion_reason_ = nullptr;
    pending_motion_force_ = false;
}
