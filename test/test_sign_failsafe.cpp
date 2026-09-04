#include "trackcontrol.h"
#include "config.h"
#include "uart.hpp"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

static TrackedObject makeSign(float score = 0.95f, int cx = 215, int cy = 52)
{
    TrackedObject s;
    s.class_id = SIGN;
    s.score = score;
    s.box = cv::Rect(cx - 35, cy - 20, 70, 40);
    s.center_x = cx;
    s.center_y = cy;
    s.frame_id = 7;
    return s;
}

static ControlResult runSignFrame(const std::vector<TrackedObject>& objs)
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    tc_set_track_valid_rows(40);
    return tc_process(mid, left, right, objs, frame, frame, mask, hw);
}

static TcOcrTextResult ocrLine(const char* text, float score, int y)
{
    return {text, score, cv::Rect(0, y, 80, 12), true};
}

int main()
{
    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().app.verboseLogs = false;
    config().tc.signOcrTriggerCooldownFrames = 0;
    config().tc.signLlmWaitMaxFrames = 3;
    config().tc.signOcrYMax = 83;

    Uart::instance().setTransmitEnabled(false);
    tc_reset();
    tc_init(320, 240);

    const std::vector<TrackedObject> sign{makeSign()};
    ControlResult request = runSignFrame(sign);
    const int request_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    const bool request_ok =
        request.ocr_request_class == SIGN && request.ocr_session_id != 0 &&
        request_mode != 1 && !tc_sign_error_control_suppressed();

    tc_notify_ocr_started(request.ocr_session_id, SIGN);
    const int started_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    const bool started_suppressed = tc_sign_error_control_suppressed();
    tc_notify_ocr_stopped(request.ocr_session_id, SIGN);
    const int stopped_before_llm_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    const ControlResult stopped_before_llm = runSignFrame(sign);
    const int stopped_frame_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    const bool stopped_frame_suppressed = tc_sign_error_control_suppressed();
    tc_notify_ocr_started(request.ocr_session_id, SIGN);

    tc_on_ocr_result(request.ocr_session_id, SIGN,
                     std::vector<TcOcrTextResult>{
                         ocrLine("前方路牌文字比较模糊", 1.0f, 10)});
    for (int i = 0; i < 4; ++i)
        (void)runSignFrame(sign);
    const int llm_timeout_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    const bool llm_pending_after_timeout = tc_sign_llm_pending();
    const bool accepted_late_llm =
        tc_on_llm_result(request.ocr_session_id, "turn_right", 1);

    const bool ok = request_ok && started_mode == 1 &&
                    started_suppressed &&
                    stopped_before_llm_mode == 1 &&
                    stopped_frame_mode == 1 &&
                    stopped_frame_suppressed &&
                    std::abs(stopped_before_llm.final_error) < 1.0f &&
                    llm_timeout_mode == 0 &&
                    !tc_sign_error_control_suppressed() &&
                    !llm_pending_after_timeout && !accepted_late_llm;
    std::cout << "request_ocr=" << request.ocr_request_class
              << " request_mode=" << request_mode
              << " started_mode=" << started_mode
              << " started_suppressed=" << started_suppressed
              << " stopped_before_llm_mode=" << stopped_before_llm_mode
              << " stopped_frame_mode=" << stopped_frame_mode
              << " stopped_frame_suppressed=" << stopped_frame_suppressed
              << " stopped_frame_error=" << stopped_before_llm.final_error
              << " llm_timeout_mode=" << llm_timeout_mode
              << " llm_pending_after_timeout=" << llm_pending_after_timeout
              << " accepted_late_llm=" << accepted_late_llm
              << "\n";
    if (!ok)
        return 2;

    tc_reset();
    tc_init(320, 240);
    const ControlResult low_conf_sign = runSignFrame({makeSign(0.64f, 215, 52)});
    tc_reset();
    tc_init(320, 240);
    const ControlResult left_edge_sign = runSignFrame({makeSign(0.95f, 20, 52)});
    tc_reset();
    tc_init(320, 240);
    const ControlResult low_y_sign = runSignFrame({makeSign(0.95f, 215, 87)});
    tc_reset();
    tc_init(320, 240);
    const ControlResult approach_sign = runSignFrame({makeSign(0.66f, 215, 89)});
    const int approach_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    tc_reset();
    tc_init(320, 240);
    const ControlResult ocr_sign = runSignFrame({makeSign(0.76f, 215, 82)});
    const bool sign_gate_ok =
        low_conf_sign.ocr_request_class == 0 &&
        left_edge_sign.ocr_request_class == 0 &&
        low_y_sign.ocr_request_class == 0 &&
        approach_sign.ocr_request_class == 0 &&
        approach_mode == 3 &&
        ocr_sign.ocr_request_class == SIGN;
    std::cout << "sign_gate low_conf=" << low_conf_sign.ocr_request_class
              << " left_edge=" << left_edge_sign.ocr_request_class
              << " y83=" << low_y_sign.ocr_request_class
              << " approach_ocr=" << approach_sign.ocr_request_class
              << " approach_mode=" << approach_mode
              << " ocr=" << ocr_sign.ocr_request_class << "\n";
    if (!sign_gate_ok)
        return 2;

    tc_reset();
    tc_init(320, 240);
    const ControlResult low_score_request = runSignFrame(sign);
    tc_notify_ocr_started(low_score_request.ocr_session_id, SIGN);
    for (int i = 0; i < config().tc.signOcrValidSamples; ++i) {
        tc_on_ocr_result(low_score_request.ocr_session_id, SIGN,
                         std::vector<TcOcrTextResult>{
            ocrLine("右道真的有点崎岖", 0.55f, 10),
            ocrLine("但是直道真的走不了", 0.67f, 30),
        });
    }
    const ControlResult low_score_followup = runSignFrame(sign);
    const bool low_score_keeps_ocr =
        low_score_followup.ocr_request_class == SIGN;
    std::cout << "low_score_followup_ocr="
              << low_score_followup.ocr_request_class << "\n";
    return low_score_keeps_ocr ? 0 : 2;
}
