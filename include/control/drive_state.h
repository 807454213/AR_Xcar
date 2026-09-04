#ifndef CONTROL_DRIVE_STATE_H
#define CONTROL_DRIVE_STATE_H

//=============================================================================
// DriveState —— 小车顶层运行状态机（可维护性核心）
//
// 每帧的流程：
//   1) WorldModel 更新：行人 / 车辆 / 金币 / 路牌各自的子状态机并行推进
//      （这些子状态机内部沿用原有调好的几何/标定算法，逐字保留）。
//   2) selectDriveState：按固定优先级从世界状态中选出唯一的顶层状态。
//   3) 顶层状态决定引导线来源与速度模式（经 UartCommander 下发）。
//
// 优先级（靠前优先）：
//   LAUNCH > AVOID_PED > AVOID_CAR > FORK_DECIDE(active sign session)
//   > FOLLOW_GOLD > RETURN_TRACK > LEAVING_CAR
//   > FORK_DECIDE(post-decision fork bias) > FAST_BACK > STABLE_SPEED > NORMAL
// 后决策 fork bias 只保留过岔口偏置，不再阻塞 RETURN_TRACK / LEAVING_CAR。
//
// 注意：子状态机仍每帧并行更新（保留并发感知），DriveState 只表达“当前由谁主导”。
//=============================================================================
enum class DriveState : int {
    Normal       = 1,   // 纯循迹
    FollowGold   = 2,   // 金币拉线（可能伴随金币减速 0x02=4/6）
    AvoidCar     = 3,   // 车辆绕行（CLOSING CAR）
    LeavingCar   = 4,  // 车辆绕行结束后左边界临时循迹
    ReturnTrack   = 5,  // 有效行过少：固定 error 绕行回赛道，cmd02=5
    FastBack      = 6,  // 赛道可见但本车偏出赛道：快速返回赛道，cmd02=7
    StableSpeed   = 7,  // 赛道内且无元素处理：稳定加速，cmd02=8
    AvoidPed     = 8,   // 行人避让（停车 / 绕行子态）
    ForkDecide   = 9,   // 警告路牌 OCR→LLM→分岔决策
    Launch       = 10,  // 发车后等待赛道有效行恢复；仅 HUD 显示，不下发 0x09
};

const char* driveStateName(DriveState s);

// 当前主导状态（由 tc_process 每帧计算并缓存；供 HUD 显示）
DriveState tc_currentDriveState();

#endif // CONTROL_DRIVE_STATE_H
