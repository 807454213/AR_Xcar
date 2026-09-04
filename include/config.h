#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <vector>
#include <array>

//=============================================================================
// 图像处理参数
//=============================================================================
struct ImgProcessParams {
    // ROI 区域比例
    float detectionYMedium = 0.59f;   // y_top2
    float detectionYLow    = 0.64f;   // y_top

    // 最小有效行数
    int minValidRows = 12;

    // 赛道最小宽度
    int minTrackWidth = 30;

    // 多段几何候选段的最小有效宽度（< 此值视为噪声段）
    int forkScanMinSegW = 4;

    // 时间滤波权重
    float alphaTime = 0.7f;

    // 跳过底部行数
    int bottomSkipPixels = 0;

    // 中线方差辅助（classifyTrackShape 内部仍用）
    float trackMidVarStraightMax = 400.0f;
    float trackMidDirDeltaThresh = 8.0f;
    int   trackMidMinValidRows   = 10;

    // 统一赛道模式（直道 / 弯道 / 分岔）——见 classifyTrackRoadMode
    float roadCurveVarMin          = 350.0f;
    float roadStraightVarMax       = 150.0f;
    float roadDirDeltaThresh       = 12.0f;
    int   roadForkBandMinRows      = 4;
    float roadForkBandMinRatio     = 0.38f;
    float roadForkBandFullDeltaMin = 0.12f;
    float roadForkBandHighRatio    = 0.72f;
    int   roadForkBandHighMinRows  = 14;
    int   roadForkMinSpan          = 52;
    int   roadSmEnterFrames        = 5;
    int   roadSmForkEnterFrames    = 2;  // 进入 Fork/ForkEntry 防抖（比通用更短）
    int   roadSmLeaveFrames        = 6;
    int   roadForkExitLeaveFrames  = 2;  // 出口汇合时 Fork→其他 防抖帧数（更短）
    // 弯/岔互斥：探测带多段行占比 ≥ 此值视为分岔几何，禁止仅凭中线方差判弯
    float roadCurveMaxBandRatio    = 0.42f;
    float roadCurveVarHigh         = 500.0f;
    int   roadForkMinGap           = 18;     // 分岔箭头间隙(px)，弯道通常更小
    int   roadForkScoreMin         = 3;      // 分岔证据分阈值
    int   roadForkApproachMinSpan  = 36;     // 远场/接近：双支路最小跨度
    int   roadForkApproachMinGap   = 12;     // 远场/接近：支路间最小缝
    int   roadForkApproachBandMinRows = 2;   // 远场：≥2 行多段即可
    float roadForkApproachBandMinRatio = 0.28f; // 远场多段行占比下限
    int   roadCurveScoreMin        = 4;      // 弯道证据分阈值
    // 起始大门抑制：宽 span + 大 gap + 探测带多段不集中 → 门柱+主轨，非分岔
    int   roadGateMinSpan          = 165;
    int   roadGateMinGap           = 55;
    float roadGateMaxBandRatio     = 0.55f;
    // 分岔出口汇合抑制：宽 span + 全 ROI 多段（两路箭头合并），非新分岔入口
    int   roadForkExitMinSpan      = 200;
    float roadForkExitMinFullRatio = 0.40f;
    int   roadForkReenterCooldown  = 45;   // 离开分岔后禁止再进 Fork 的帧数
    int   roadForkBiasOffHold      = 4;    // stable 非 Fork 连续帧后才清 FORK_L 偏置

    // PPSeg 语义分割循迹（推理失败或有效行 < minValidRows 时返回无赛道，不回退 HSV）
    bool        usePpSegTrack  = true;
    std::string ppsegModelPath = "AI/PPSeg/models/pp_liteseg_128.rknn";
    int         ppsegInputW    = 320;
    int         ppsegInputH    = 128;
    int         ppsegNpuCore   = 2;
    int         ppsegMaxAgeMs  = 100;  // 结果源帧允许的最大年龄
    int         ppsegMaxFidLag = 2;    // 相对当前采集帧允许的最大帧差
    // 掩码稳定化：时域 EMA+滞回（抑制边界果冻抖动）+ 小连通域过滤
    bool  ppsegMaskStabilize   = true;
    float ppsegMaskEmaAlpha    = 0.5f; // EMA 新帧权重(0.05..0.95)，越小越稳但响应越慢
    int   ppsegMaskMinBlobArea = 80;   // 小白块过滤阈值（按 320x240 掩码像素计）

    // 分岔出口汇合：左边界稳定 + 右边界向上突变 → 用近处斜率拉线替换远段右边界
    bool forkExitRepairEnabled   = true;
    int  forkExitScanStepY       = 2;    // 扫描步长
    int  forkExitLeftStableRows  = 4;    // 判左稳定：近侧连续行数
    int  forkExitLeftMaxDx       = 14;   // 左边界相邻行 |Δx| 上限
    int  forkExitRightJumpDx     = 22;   // 右边界向远突变 |Δx| 下限（R_far-R_near）
    int  forkExitLeftJumpDx      = 22;   // 左边界向远突变 |Δx| 下限（L_far-L_near，负值）
    int  forkExitRightStableRows = 4;    // 左出口：判右稳定连续行数
    int  forkExitRightMaxDx      = 14;   // 左出口：右边界相邻行 |Δx| 上限
    int  forkExitVTipMinGap      = 6;    // mask V 型内缝最小宽度（像素）
    int  forkExitVTipGrowPx      = 4;    // V 尖端近侧内缝须比尖端宽至少 px
    int  forkExitSlopeRows       = 6;    // 突变点下方用于拟合斜率的行数
    int  forkExitLineStartDownRows = 3;  // 右出口拉线起始点相对突变近侧点下移行数（y 增大）
    int  forkExitTopStableRows     = 12;  // 出口：上方连续平稳边界行数下限
    int  forkExitTopStableMaxDx    = 3;   // 出口：相邻行 |Δx| 须 <= 此值才算平稳
    int  forkExitTopAnchorBandPx   = 8;   // 出口：平稳点 |rx-anchorX| 须 <= 此值
    int  forkExitLeftLineStartExtraDownRows = 5; // 左出口：在共用 downRows 上再下移行数
    int  forkExitMinTrackWidth   = 12;   // 修补后左右最小宽度
    int  forkEntryMinGapGrowPx   = 8;    // 入口：近端内缝比远端宽至少 px（mask 内 leftR/rightL）
    int  forkExitMaxGapGrowPx    = 5;    // 出口：内缝随近端增大不超过此值
    int  forkExitMinMergeY       = 125;  // 相位判别：mergeY 低于此视为远场误检（入口常见 ~117）
    int  forkExitTrustedPadPx    = 10;   // 右出口补线/HUD：mergeY 须 >= minMergeY+pad
    int  forkExitLeftTrustedPadPx = 4;  // 左出口（回主路）可更早触发
    int  forkExitHuntClearFrames = 4;    // 出口几何消失连续 N 帧后 HUNT_OUT→HUNT_IN
    int  forkPhaseScoreMargin    = 2;    // 入口/出口得分差超过此值才切换

    // 分岔入口（PPSeg）：黑-白-黑-白-黑 双支路行 → 分界点拉线
    bool forkEntryEnabled        = true;
    int  forkEntryScanStepY      = 2;
    int  forkEntryMinSegW        = 6;    // 单侧箭头最小宽度
    int  forkEntryMinGap         = 10;   // 两支路间黑缝最小宽度
    int  forkEntryMinSpan        = 52;   // 左支左缘→右支右缘最小跨度
    int  forkEntryMinWideSpan    = 62;   // 分界点处宽赛道阈值
    int  forkEntryMinRows        = 2;    // 至少多少行出现双支路模式
    int  forkEntrySlopeRows      = 5;    // 分界点往上取点拟合斜率
    int  forkEntryMinTrackWidth  = 12;
    int  forkEntryApproachMinSegW     = 3;   // 接近入口：放宽单支宽度
    int  forkEntryApproachMinSpan     = 10;  // 接近入口：放宽跨度
    int  forkEntryApproachMinWideSpan = 14;  // 接近入口：放宽分界点宽赛道
    int  forkEntryApproachMinRows     = 1;   // 接近入口：放宽有效行数
    int  forkEntryApproachMinGap      = 3;   // 接近入口：放宽支路间缝
    int   forkEntryHuntSwitchBottomPx  = 22;   // 分界点距底边不超过此 px 才 Entry→Exit
    int  forkEntryTopStableRows        = 3;   // 入口：左支右缘平稳段最少行数
    int  forkEntryTopStableMaxDx       = 4;   // 入口：平稳段相邻 |Δx| 上限
    int  forkEntryTopAnchorBandPx      = 10;  // 入口：平稳点与锚点 x 偏差上限
    int  forkEntryApproachTopStableRows = 2;  // 接近入口：平稳段行数下限（更宽松）
    int  forkEntryMinDualRowsNear    = 1;   // 近端 mask 双支路最少行数才允许补线
    int  forkEntryDualStreakFrames   = 2;   // 连续 N 帧近端双支路才首次入口补线
    int  forkEntryClearBiasNoDualFrames = 3; // 连续无双支路则清 FORK 偏置
    float forkEntryApproachNearFrac  = 0.62f; // 接近探测仅扫描 ROI 下段（y 大=近）
    bool forkEntryEarlyFork2Enabled = true;   // 入口早期：AR Fork2 行模式 + 黑缝向上扩张
    int  forkEntryEarlyFork2YMinRatio = 40;   // 早期 Fork2 特征窗口上界（ROI 百分比）
    int  forkEntryEarlyFork2YMaxRatio = 80;   // 早期 Fork2 特征窗口下界（ROI 百分比）
    int  forkEntryEarlyFork2MinWhiteWidthPx = 8;
    int  forkEntryEarlyFork2MinBlackWidthPx = 3;
    int  forkEntryEarlyFork2FallbackMaxBlackWidthPx = 24;
    int  forkEntryEarlyFork2GrowthRows = 4;
    int  forkEntryEarlyFork2GrowthMinStepPx = 1;
    bool forkOuterSupportFilterEnabled = true; // AR forkOuter：白段需有上下邻行同侧支撑
    int  forkOuterSupportRows = 3;
    int  forkOuterMinSupportRows = 2;
    int  forkOuterEdgeBaseTolPx = 6;
    int  forkOuterEdgeTolPerRowPx = 3;
    int  forkOuterMinSupportAreaPx = 24;
    // 分岔宽度探测：y=forkWidthProbeY±band 行 mask 最大白段长度 vs 直道基线
    int   forkWidthProbeY          = 150;
    int   forkWidthProbeBand       = 15;
    int   forkEntryNormalWidthRun  = 90;   // 直道参考（normal 掩码 med≈81 max≈105）
    float forkWidthForkRatio       = 1.35f;
    int   forkWidthForkMarginPx    = 35;   // max(baseline*ratio, baseline+margin) 判分岔
    bool  forkEntryPatchHoldEnabled = true; // 补线丢失时沿用上一帧 patch 直至真出岔
};

//=============================================================================
// 循迹控制参数
//=============================================================================
struct TrackControlParams {
    // 误差计算行 & 工作区
    int errorCalcY            = 175;
    int stableSpeedErrorCalcY = 175; // STABLE_SPEED 独立误差采样行
    int workZoneHalf          = 15;
    bool encoderRawDynamicErrorYEnabled = false; // true: 固定采样行改由编码器原始 tick 动态映射
    int encoderRawDynamicErrorYMin = 115;        // 高速时更远的采样行（图像 y 小）
    int encoderRawDynamicErrorYMax = 150;        // 低速时更近的采样行（图像 y 大）
    int encoderRawDynamicErrorRawMin = 0;        // raw <= 此值使用 YMax
    int encoderRawDynamicErrorRawMax = 80;       // raw >= 此值使用 YMin
    int encoderRawDynamicErrorStaleFrames = 10;  // 超过 N 个控制帧无新编码器对则回退固定行

    int workZoneUpper() const { return errorCalcY - workZoneHalf; }
    int workZoneLower() const { return errorCalcY + workZoneHalf; }

    // --- 元素入口防抖 ---
    bool elementDebounceEnabled = false; // true 时金币/车/人/路牌连续确认后才进入处理模块
    int elementDebounceConfirmFrames = 2; // 需要连续多少个不同 AI source fid，最小为 1
    // --- 元素统一 Y 门控 ---
    // 开启后，锚点 y <= 阈值的目标不进入控制状态机。
    // GOLD 使用映射后的金币点；HUMAN 使用脚点；CAR/SIGN 使用检测框中心。
    bool elementYFilterEnabled = false;
    int elementControlMinY = 0; // GOLD/CAR/HUMAN 共用 y 下限
    int signControlMaxY = 240;  // SIGN 单独 y 上限

    // --- 金币 ---
    // 可达判定：赛道内和边界橙带默认可达；橙带外只允许落在左右独立外扩
    // [lx-goldReachableWidthAddOuterLeft, rx+goldReachableWidthAddOuterRight] 内的金币拉线。
    // RETURN_TRACK 丢线期间，allowGoldOutsideTrack=true 且 y > goldReachableBypassMinY、
    // x 落在 [goldReachableBypassMinX, goldReachableBypassMaxX] 的金币可拉线，
    // 并按赛道外金币进入 GOLD_SLOW；普通状态不使用该 bypass。
    // 已锁定金币的跨帧脚点匹配半径(px)；实际下限为 24px。
    int goldLockMatchRadiusPx = 30;
    int goldLostMax    = 6;
    bool goldFollowEnabled = true; // false 时车控忽略金币检测，不拉线/减速
    int goldFollowMinY = 170; // 跟金币拉线：检测框底边中点 y > 此值才参与循迹
    int goldXMin = 80;         // 金币 center_x 最小值，小于此不处理
    int goldXMax = 240;        // 金币 center_x 最大值，大于此不处理
    int goldMinBoxDiag = 12;    // 金币检测框左上到右下长度 <= 此值时过滤，不进入拉线/减速
    float goldMappedYHeightRatio = 1.10f; // mapped_y = box.y + height * ratio + offset
    int goldMappedYOffset = 0; // 金币映射点相对底部锚点的像素偏移
    int goldSuddenDirectMinY = -1; // >=0 时：突现锁定金币 mapped_y > 此值则绕过权重直拉
    bool allowGoldOutsideTrack = true; // true 时可达含外扩环带；false 时仅原始赛道内
    bool goldBandVisualEnabled = false; // 调试叠加：显示金币内扩/外扩/可达带
    int  goldTrackWidthAddInner = 24; // 金币减速边界带向赛道内侧扩宽(px，参考行标定)
    int  goldTrackWidthAddOuter = 24; // 金币减速边界带向赛道外侧扩宽(px，参考行标定)
    int  goldReachableWidthAddOuterLeft = 52;  // 金币可达带向左赛道边界外侧扩宽(px，参考行标定)
    int  goldReachableWidthAddOuterRight = 52; // 金币可达带向右赛道边界外侧扩宽(px，参考行标定)
    int  goldReachableBypassMinY = 150;   // RETURN_TRACK 金币可达窗口最小 y
    int  goldReachableBypassMinX = 90;    // RETURN_TRACK 金币可达窗口最小 x
    int  goldReachableBypassMaxX = 230;   // RETURN_TRACK 金币可达窗口最大 x
    int  goldTrackGuidanceWeightRef = 128;   // 赛道内/边界带金币反向权重参考距离(px)，越大越保留真实金币x
    int  goldOutsideGuidanceWeightRef = 128; // 赛道外金币反向权重参考距离(px)，越大越保留真实金币x
    int  goldTrackErrorFixedYMin = 190;   // 赛道内跟随金币 y 达到该值时回退候选/默认 errorCalcY；<=0 关闭
    int  goldOutsideErrorFixedYMin = 190; // 赛道外/边界带跟随金币 y 达到该值时回退候选/默认 errorCalcY；<=0 关闭

    // --- 车辆避让（center_y > carAvoidMinY：进入避让状态并开始拉线）---
    int carAvoidMinY   = 170;
    int avoidOffsetCar = 55;   // 横向偏移 px（相对检测框绕行侧边外缘；Y 取框中心）
    int carAvoidBoundaryOffsetLeft = 0;  // Car 在赛道左侧时的边界外扩(px)
    int carAvoidBoundaryOffsetRight = 0; // Car 在赛道右侧时的边界外扩(px)
    int carAvoidDirectionScanRows = 12; // 车辆方向：底边向上查找单侧 mask 证据的行数（含底边）
    int carDetectMaxY  = 225;  // center_y 大于此值的车辆不参与避让（过远）
    int carAvoidExitY  = 240;  // center_y 大于此值时退出车辆避让
    float carLeavingDistMLeft = 0.5f;  // Car 在赛道左侧时 LEAVING_CAR 保持里程(m)
    float carLeavingDistMRight = 0.5f; // Car 在赛道右侧时 LEAVING_CAR 保持里程(m)
    int carLeavingFarYMax = 135; // 车辆消失前最后 center_y 小于该值时使用远距离保持
    float carLeavingFarDistMLeft = 0.6f;  // 左侧远处车辆消失时 LEAVING_CAR 保持里程(m)
    float carLeavingFarDistMRight = 0.6f; // 右侧远处车辆消失时 LEAVING_CAR 保持里程(m)
    bool carLeavingGoldEnabled = false; // true 时 LEAVING_CAR 遇到金币切 FOLLOW_GOLD 并清空离开态

    // --- 行人避让：检测框 center_y > personAvoidMinY 进入深度区/停车态 ---
    bool personBandVisualEnabled = false; // 调试叠加：显示行人橙色外扩/内收带
    int personAvoidErrorCalcY = 175; // AVOID_PED 独立误差采样行
    int personAvoidMinY   = 170;
    int carAvoidLostMax      = 8;
    int personNearActionXMin = 80;  // 近区坐标算法总作用窗口左界，外侧使用 outer 拉线量
    int personNearActionXMax = 240; // 近区坐标算法总作用窗口右界，外侧使用 outer 拉线量
    int personFarStopXMin = 70;     // 远区停车窗口左界，窗口外拉线
    int personFarStopXMax = 250;    // 远区停车窗口右界，窗口外拉线
    int personNearStopXMin = 110;   // 近区中间停车窗口左界，窗口两侧接近区使用 approach 拉线量
    int personNearStopXMax = 210;   // 近区中间停车窗口右界，窗口两侧接近区使用 approach 拉线量
    int personEmergFarY   = 170;   // 简单避让远区 Y 下限（center_y > 此值）
    int personEmergNearYMax = 170; // 简单避让远区 Y 上限（center_y < 此值；> 为近区）
    int personInstantPassMinY = 148; // 首帧行人 center_y 大于该值且在赛道外时直接拉线通行
    int personTrackWidthAdd      = 15;  // 橙色带向外扩宽（参考行像素数）
    int personTrackWidthInward   = 10;  // 橙色带向内覆盖赛道（参考行像素数）
    int personAvoidBoundaryOffset = 0; // AVOID_PED：按行人方向选左右边界后继续向赛道外侧扩展(px)
    int personXyApproachPullOffset = 60; // AVOID_PED XY路径：接近区固定拉线量(px)
    int personXyOuterPullOffset = 60;    // AVOID_PED XY路径：外侧固定拉线量(px)
    float personPostCarPedDistM = 2.5f;  // car 避让结束后：此距离(m)内行人只向 car 赛道反侧拉线
    bool  personPostCarEnabled  = true;  // PERS-CAR 总开关：car 避让结束后里程窗口内约束行人拉线方向
    int personStopReleaseConfirm = 4;  // Stop->Fast：连续满足释放条件的帧数
    float personAwayMinGrowthRatio = 0.04f; // 赛道内：脚点外侧归一化净增长释放阈值
    int personDetourFastConfirm = 2;   // 拉线有效后连续 N 个新源帧才发 0x02,1,2（FAST）
    bool personFastStopRollbackEnabled = false; // true 时坐标路径 FAST 后普通 STOP 区仍可回 STOP
    int personPullLineHoldFrames = 8;  // 行人消失后维持拉线(Detour)的帧数，0=立即取消
    int carTrackRelationY = 230;       // 本车/赛道关系判断行：使用该行原始中线 error
    int carTrackInsideErrorMin = -80;  // 原始 error 下限，区间内视为本车仍在赛道内
    int carTrackInsideErrorMax = 80;   // 原始 error 上限，区间外进入 FAST_BACK
    int carTrackOutsideEnterConfirmFrames = 3; // IN->OUT 连续确认帧数
    int carTrackInsideEnterConfirmFrames = 3;  // OUT->IN 连续确认帧数
    int forkExitProbeY  = 200;  // 多行探测带中心 y（图像坐标）
    int forkExitProbeBand = 10; // 中心 ±band 行统计蓝段数（弯道单段宽弧抑制）
    int forkProbeMinMultiSegRows = 2; // 探测带内至少多少行有 ≥2 段才视为真分叉
    int forkExitMinSegW = 5;    // 蓝色段最小有效宽度（< 此值视为噪点）
    int forkExitConfirm = 12;    // 连续 N 帧探测带全为单段才确认出岔
    int forkExitMinInForkFrames = 45;  // InFork 内至少停留帧数后才允许 FORK_L/R 单段退出

    // 车头判定线（用于金币/避让反馈）
    int carFrontY    = 225;   // 车头线Y坐标（越大越靠近画面底部）

    // sign 路牌 OCR：在检测框四边外扩像素得到裁剪 ROI（非整幅 320×240）
    int signCenterXOffsetPx = 0;    // SIGN 靠近循迹目标 x = 检测框中心 x + 此偏移；正数向右
    int signSeenXMin = 40;          // SIGN 接近触发窗口：center_x 左界
    int signSeenXMax = 280;         // SIGN 接近触发窗口：center_x 右界
    int signSeenYMax = 98;          // SIGN 接近触发窗口：center_y 上限
    int signOcrXMin = 50;           // SIGN OCR 触发窗口：center_x 左界
    int signOcrXMax = 270;          // SIGN OCR 触发窗口：center_x 右界
    int signOcrYMax = 82;           // SIGN OCR 触发窗口：center_y 上限
    int signOcrWidthMin = 50;      // SIGN OCR 近距离触发：检测框宽度下限，越大越近
    int signOcrRoiMarginX = 24;
    int signOcrRoiMarginY = 20;
    int signOcrMinChars = 2;       // 单次 OCR 有效：合并文本 UTF-8 字数下限
    int signOcrValidSamples = 3;   // 连续有效次数满此值后再送 LLM
    float signOcrMinScore = 0.68f;
    float signOcrHighScore = 0.78f;
    int signOcrMaxAttempts = 8;
    float signOcrIntervalSec = 0.08f; // OCR 推理最小间隔(秒)，越小越快
    int signOcrWarmupFrames = 1;   // 启动 OCR 后首帧可 get 的 put 次数
    int signOcrLostTimeout = 150;  // 路牌丢失后仍继续 OCR 的帧数上限
    int signOcrTriggerCooldownFrames = 90; // 两次 sign OCR 触发之间的最小间隔帧数
    int signOcrErrorCalcYOffset = 25; // sign 识别/入岔后 errorCalcY 下移像素，直到单段赛道或 FORK 退出
    int signLlmWaitMaxFrames = 150; // 等 LLM 最多帧数，超时按默认直行释放停车
    bool signFixedDirectionEnabled = false;
    std::string signFirstDirection = "straight";
    std::string signSecondDirection = "right";
    bool signComplementStrategyEnabled = false;
    float signComplementMinScore = 0.85f;
    bool signDecisionErrGuardEnabled = true;
    int signDecisionErrGuardBoxWidthMin = 120;
    int signDecisionErrGuardReleaseValidFrames = 3;
    float signDecisionRightErrMin = 40.0f;
    float signDecisionRightErrMax = 160.0f;
    float signDecisionRightFallbackErr = 150.0f;
    float signDecisionStraightErrMin = -160.0f;
    float signDecisionStraightErrMax = -40.0f;
    float signDecisionStraightFallbackErr = -150.0f;
};

//=============================================================================
// 应用级开关
//=============================================================================
struct AppParams {
    // 运行模式：
    //   race   = 比赛模式：无窗口，硬件/UART 始终启用，走轻量路径
    //   vision = 可视化调试：开窗口、调试 HUD、键盘快捷键
    std::string runtimeMode = "race";
    int  aiThreadNum        = 3;   // YOLO RKNN worker 数，1..3
    int  aiNpuCoreStart     = 0;   // YOLO worker 从哪个 NPU 核开始轮转，0..2

    // 画面调试叠加总开关；关闭时保留干净视频画面。
    bool debugOverlay         = false;
    bool trackControlEnabled = true;    // 仅作启动时 TrackControl 默认是否开启；
    int  captureReadTimeoutMs = 5;      // 主循环等待新共享内存帧的最长时间，降低 capture 阻塞
    bool shmCaptureVerticalFlip = true; // 共享内存 RGB 帧是否上下翻转后再进入 AI/显示链路

    // 是否在画面上绘制 AI 检测框、置信度、类别名与中心点（关闭可减轻 CPU 开销）
    bool aiShowDetections = true;
    bool perfHudEnabled = true;     // vision 模式画面 PERF 耗时诊断开关
    bool bevEnabled = false;       // 是否在画面右上角显示 BEV 俯视图
    bool carMotionHudEnabled = true; // 是否在画面上方居中显示根据串口推断的车辆运动状态
    bool verboseLogs = false;      // 是否输出调车调试日志；默认仅保留 OCR/LLM 与错误

    // YOLO 检测 fid 对齐：固定使用 ai_frame_fusion 新链路。
    float aiConfThreshold = 0.35f;
    int aiFusionMaxFidDiff = 2;
    int aiFusionMaxTimeDiffMs = 1;
    int aiFusionPredictMaxTimeMs = 66;
    int aiExactPairWaitMs = 80;
    int aiFusionBufferSize = 8;
    bool aiSourceDrivenControlEnabled = true;
    int aiSourceExitConfirmFrames = 2;
    int signOcrSourceMaxAgeMs = 400;

    int ocrDrainPerFrame = 2;
    bool ocrCountEmptyAsAttempt = true;
    int ocrInputScale = 2;
    int ocrDetNpuCore = 0;
    std::array<int, 2> ocrRecNpuCores = {0, 1};
    float ocrDetThreshold = 0.15f;
    float ocrBoxThreshold = 0.20f;
    float ocrDbUnclipRatio = 1.8f;
    bool ocrContrastEnhance = true;
    float ocrContrastClipLimit = 2.0f;
    int ocrContrastTileGrid = 8;
    float ocrRecScoreThreshold = 0.40f;
    float ocrContextRecScoreThreshold = 0.25f;
    float ocrCropExpandRatio = 0.15f;
    int ocrMinBoxArea = 10;
    int ocrMinBoxHeight = 1;
    int ocrMinBoxWidth = 1;
    float ocrMinBoxRatio = 0.12f;
    float ocrMaxBoxRatio = 14.0f;
    int ocrMaxQueueSize = 8;

    // LLM (千帆 API) 凭据
    std::string llmAccessKey;
    std::string llmSecretKey;
    std::string llmModel = "ernie-4.5-turbo-32k";
};

//=============================================================================
// 全局配置
//=============================================================================
struct AppConfig {
    ImgProcessParams   img;
    TrackControlParams tc;
    AppParams          app;
};

AppConfig& config();

bool configLoad(const std::string& path);
bool configSave(const std::string& path);

inline bool appIsVisionMode(const AppParams& app) { return app.runtimeMode == "vision"; }
inline bool appIsRaceMode(const AppParams& app) { return app.runtimeMode == "race"; }
// 画面调试叠加仅由显式 debugOverlay 开关控制。
inline bool appDebugOverlayActive(const AppParams& app) {
    return app.debugOverlay;
}

#endif // CONFIG_H
