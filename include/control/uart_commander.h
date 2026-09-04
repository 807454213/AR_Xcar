#ifndef CONTROL_UART_COMMANDER_H
#define CONTROL_UART_COMMANDER_H

#include <cstdint>

//=============================================================================
// UartCommander —— 离散下行指令的唯一出口（单一真值 + 统一去重）
//
// 背景：旧工程里 0x02 运动模式被行人 / 交通灯 / OCR / 金币减速等多处分别发送，
// 各自维护一份“上次值”去重（g_ped_cmd02_last / g_ocr_*），
// 不同通道之间互不知情，存在跨通道陈旧值导致漏发的隐患。
//
// 这里把所有“状态型”离散指令收敛到一个去重出口：相同值不重复下发，
// 真正的下位机状态只有一份。每帧的 0x01 横向误差属于“流式”指令，
// 不去重，按需直发。
//
// 串口帧格式与协议保持与下位机 TC264 完全一致（见 Uart::send / TC264.md）。
//=============================================================================
enum class MotionModeOwner : uint8_t {
    Normal = 0,
    StableSpeed = 1,
    FastBack = 2,
    LeavingCar = 3,
    ReturnTrack = 4,
    Gold = 5,
    Sign = 6,
    Vehicle = 7,
    Pedestrian = 8,
};

class UartCommander {
public:
    static UartCommander& instance();

    // --- 状态型指令（去重）---
    // 0x02 运动模式：0 正常 / 1 STOP / 2 FAST / 3 减速 / 4 金币赛道外减速 / 5 绕行回赛道减速 / 6 金币边界带减速
    // 返回 true 表示“下位机现已处于该模式”（已是该值或本次发送成功）
    bool setMotionMode(uint8_t mode, const char* reason = nullptr, bool force = false);
    bool requestMotionMode(uint8_t mode,
                           MotionModeOwner owner,
                           const char* reason = nullptr,
                           bool force = false);
    // 0x03 保护：1 锁存保护停 / 0 解除
    bool setProtect(uint8_t value, const char* reason = nullptr, bool force = false);
    // 0x07 弯道标志：0 直 / 1 弯
    bool setCurveFlag(uint8_t value, const char* reason = nullptr);
    // 0x08 最大速度（>12 解除限速）
    bool setMaxSpeed(float value, const char* reason = nullptr);
    // 0x0B 分岔方向：0 none / 1 left / 2 right
    bool setForkDir(uint8_t dir, const char* reason = nullptr, bool force = false);

    // 0x09 顶层 DriveState 上报：按 DriveState 去重；reset 后首个状态也会下发
    bool sendStateFlag(uint8_t flag, const char* reason = nullptr);

    // --- 流式指令（不去重）---
    void sendError(float error);   // 0x01，每帧

    // 发车序列：0x03,0 → 0x02,0 → 0x05,1（强制发送，绕过去重以确保下位机解锁）
    void startCar();
    // 安全停车：0x03,1（强制）
    void emergencyProtect(const char* reason = nullptr);

    // 复位去重记忆（tc_init / tc_reset 调用）
    void reset();

    // 当前下位机运动模式（最近一次成功下发的 0x02 值；未发送过为 255）
    uint8_t lastMotionMode() const { return last_motion_; }
    uint8_t effectiveMotionMode() const;

    // 在单个控制帧内合并 0x02 请求，直到 endMotionModeBatch() 才真实下发。
    // 用于避免同一帧先发上一状态的恢复命令、再发当前帧最终命令。
    void beginMotionModeBatch();
    void endMotionModeBatch();

#ifdef XCAR_TESTING
    uint64_t motionModeSendCountForTest() const { return motion_send_count_; }
#endif

private:
    UartCommander() = default;
    UartCommander(const UartCommander&) = delete;
    UartCommander& operator=(const UartCommander&) = delete;

    static constexpr uint8_t kUnset = 255;

    uint8_t last_motion_  = kUnset;
    uint8_t last_protect_ = kUnset;
    uint8_t last_curve_   = kUnset;
    uint8_t last_fork_    = kUnset;
    uint8_t last_state_flag_ = kUnset; // 0x09 DriveState 上次下发值；有效值 1..9
    float   last_speed_   = -1e9f;  // 未发送过的哨兵

    int batch_depth_ = 0;
    bool pending_motion_valid_ = false;
    uint8_t pending_motion_mode_ = 0;
    MotionModeOwner pending_motion_owner_ = MotionModeOwner::Normal;
    const char* pending_motion_reason_ = nullptr;
    bool pending_motion_force_ = false;
    uint64_t motion_send_count_ = 0;
};

#endif // CONTROL_UART_COMMANDER_H
