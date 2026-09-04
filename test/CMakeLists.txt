cmake_minimum_required(VERSION 3.10)
project(XcarTests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ????????
set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/..)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message(STATUS "OpenMP enabled: ${OpenMP_CXX_FLAGS}")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -DNDEBUG")
add_compile_definitions(XCAR_PROJECT_ROOT="${ROOT_DIR}")

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    message(STATUS "OpenCV version: ${OpenCV_VERSION}")
endif()

include_directories(${ROOT_DIR}/include)
include_directories(${ROOT_DIR}/Uart)
include_directories(${ROOT_DIR}/common)
include_directories(${ROOT_DIR}/common/include)
include_directories(${ROOT_DIR}/AI/base)
include_directories(${ROOT_DIR}/AI/base/utils)
include_directories(/usr/include/rga)
include_directories(${ROOT_DIR}/AI/base/third_party/stb)
include_directories(${ROOT_DIR}/AI/PPOCR-1/PPOCR-System/cpp)
include_directories(${ROOT_DIR}/AI/PPOCR-1/PPOCR-System/cpp/rknpu2)
include_directories(${ROOT_DIR}/AI/PPOCR-1/PPOCR-Det/3rdparty/rknpu2/include)

set(TEST_COMMON_SOURCES
    ${ROOT_DIR}/src/perception/imgprocess.cpp
    ${ROOT_DIR}/src/perception/ppseg_infer.cpp
    ${ROOT_DIR}/src/io/config.cpp
    ${ROOT_DIR}/src/perception/camera_model.cpp
    ${ROOT_DIR}/src/io/function.cpp
    ${ROOT_DIR}/AI/base/func.cpp
    ${ROOT_DIR}/AI/base/rknnpool.cpp
    ${ROOT_DIR}/AI/base/postprocess.cc
    ${ROOT_DIR}/AI/base/utils/image_utils.c
    ${ROOT_DIR}/AI/base/utils/file_utils.c
    ${ROOT_DIR}/AI/base/utils/image_drawing.c
)

set(TEST_LIBS
    ${OpenCV_LIBS}
    rknnrt
    pthread
    rt
    rga
    m
    turbojpeg
)

function(add_xcar_test TARGET_NAME TEST_FILE)
    add_executable(${TARGET_NAME}
        ${CMAKE_CURRENT_LIST_DIR}/${TEST_FILE}
        ${TEST_COMMON_SOURCES}
    )
    target_link_libraries(${TARGET_NAME} ${TEST_LIBS})
    set_target_properties(${TARGET_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )
endfunction()

add_xcar_test(test_fork_entry_left test_fork_entry_left.cpp)
add_xcar_test(test_fork_entry_width test_fork_entry_width.cpp)
add_xcar_test(test_fork_entry_early_fork2 test_fork_entry_early_fork2.cpp)
add_xcar_test(test_fork_entry_right_boundary_patch
              test_fork_entry_right_boundary_patch.cpp)
add_xcar_test(test_fork_entry_near_bottom_relation
              test_fork_entry_near_bottom_relation.cpp)
add_xcar_test(test_ped_track_widen test_ped_track_widen.cpp)
add_xcar_test(test_gold_track_band test_gold_track_band.cpp)
add_xcar_test(test_fork_exit_stable test_fork_exit_stable.cpp)
add_xcar_test(test_fork_run_batch test_fork_run_batch.cpp)
add_xcar_test(test_fork_r_frames test_fork_r_frames.cpp)
add_xcar_test(test_fork_scene_samples test_fork_scene_samples.cpp)
add_xcar_test(test_right_fork_exit_left_repair test_right_fork_exit_left_repair.cpp)
add_xcar_test(test_right_fork_exit_153_samples test_right_fork_exit_153_samples.cpp)
add_xcar_test(test_right_fork_exit_video_no_fork_mode
              test_right_fork_exit_video_no_fork_mode.cpp)
add_xcar_test(test_right_fork_exit_left_curve_slope
              test_right_fork_exit_left_curve_slope.cpp)
add_xcar_test(test_right_fork_exit_oblique_samples
              test_right_fork_exit_oblique_samples.cpp)
add_xcar_test(test_right_fork_lifecycle_gate
              test_right_fork_lifecycle_gate.cpp)
add_xcar_test(test_right_fork_entry_lifecycle
              test_right_fork_entry_lifecycle.cpp)
add_xcar_test(test_right_fork_lifecycle_video
              test_right_fork_lifecycle_video.cpp)
add_xcar_test(test_no_hsv_fallback test_no_hsv_fallback.cpp)
add_xcar_test(test_ppseg_crop_roi test_ppseg_crop_roi.cpp)
add_xcar_test(test_ppseg_async_contract test_ppseg_async_contract.cpp)
add_xcar_test(test_ppseg_provided_mask_perf test_ppseg_provided_mask_perf.cpp)
add_xcar_test(test_debug_overlay_config test_debug_overlay_config.cpp)
add_xcar_test(test_track_relation_hud test_track_relation_hud.cpp)
add_xcar_test(test_road_straight_curve_samples test_road_straight_curve_samples.cpp)
add_xcar_test(test_stop_landmark_repair test_stop_landmark_repair.cpp)
target_compile_definitions(test_stop_landmark_repair PRIVATE XCAR_TESTING)
add_xcar_test(test_stop_landmark_lap_gate test_stop_landmark_lap_gate.cpp)

add_executable(test_stop_landmark_not_in_main_flow
    ${CMAKE_CURRENT_LIST_DIR}/test_stop_landmark_not_in_main_flow.cpp
    ${ROOT_DIR}/src/perception/imgprocess.cpp
    ${ROOT_DIR}/src/io/config.cpp
    ${ROOT_DIR}/src/perception/camera_model.cpp
    ${ROOT_DIR}/src/io/function.cpp
    ${ROOT_DIR}/AI/base/func.cpp
    ${ROOT_DIR}/AI/base/rknnpool.cpp
    ${ROOT_DIR}/AI/base/postprocess.cc
    ${ROOT_DIR}/AI/base/utils/image_utils.c
    ${ROOT_DIR}/AI/base/utils/file_utils.c
    ${ROOT_DIR}/AI/base/utils/image_drawing.c
)
target_link_libraries(test_stop_landmark_not_in_main_flow ${TEST_LIBS})
set_target_properties(test_stop_landmark_not_in_main_flow PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_headless_keyboard
    ${CMAKE_CURRENT_LIST_DIR}/test_headless_keyboard.cpp
    ${ROOT_DIR}/src/app/keyboard_input.cpp
)
target_compile_definitions(test_headless_keyboard PRIVATE XCAR_TESTING)
set_target_properties(test_headless_keyboard PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_perf_hud
    ${CMAKE_CURRENT_LIST_DIR}/test_perf_hud.cpp
    ${ROOT_DIR}/src/app/perf_hud.cpp
)
set_target_properties(test_perf_hud PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_pipeline_perf_timing
    ${CMAKE_CURRENT_LIST_DIR}/test_pipeline_perf_timing.cpp
)
set_target_properties(test_pipeline_perf_timing PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_pipeline_text_origin
    ${CMAKE_CURRENT_LIST_DIR}/test_pipeline_text_origin.cpp
)
set_target_properties(test_pipeline_text_origin PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_shm_src_fps_counter
    ${CMAKE_CURRENT_LIST_DIR}/test_shm_src_fps_counter.cpp
)
set_target_properties(test_shm_src_fps_counter PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_ocr_prewarm_lifecycle
    ${CMAKE_CURRENT_LIST_DIR}/test_ocr_prewarm_lifecycle.cpp
)
set_target_properties(test_ocr_prewarm_lifecycle PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_uart_error_send_nonblocking
    ${CMAKE_CURRENT_LIST_DIR}/test_uart_error_send_nonblocking.cpp
    ${ROOT_DIR}/src/io/uart.cpp
)
set_target_properties(test_uart_error_send_nonblocking PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_uart_fail_fast
    ${CMAKE_CURRENT_LIST_DIR}/test_uart_fail_fast.cpp
    ${ROOT_DIR}/src/app/uart_fail_fast.cpp
)
set_target_properties(test_uart_fail_fast PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_terminal_output
    ${CMAKE_CURRENT_LIST_DIR}/test_terminal_output.cpp
    ${ROOT_DIR}/src/io/terminal_output.cpp
)
set_target_properties(test_terminal_output PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_odom_encoder_raw
    ${CMAKE_CURRENT_LIST_DIR}/test_odom_encoder_raw.cpp
    ${ROOT_DIR}/src/io/function.cpp
)
target_link_libraries(test_odom_encoder_raw ${OpenCV_LIBS})
set_target_properties(test_odom_encoder_raw PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_odom_hud_encoder_raw
    ${CMAKE_CURRENT_LIST_DIR}/test_odom_hud_encoder_raw.cpp
    ${ROOT_DIR}/src/app/Hud.cpp
    ${ROOT_DIR}/src/io/function.cpp
)
target_link_libraries(test_odom_hud_encoder_raw ${OpenCV_LIBS})
set_target_properties(test_odom_hud_encoder_raw PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_runtime_resource_paths
    ${CMAKE_CURRENT_LIST_DIR}/test_runtime_resource_paths.cpp
)
set_target_properties(test_runtime_resource_paths PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_ppseg_async_policy
    ${CMAKE_CURRENT_LIST_DIR}/test_ppseg_async_policy.cpp
    ${ROOT_DIR}/src/app/ppseg_async_policy.cpp
)
set_target_properties(test_ppseg_async_policy PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_sign_strategy
    ${CMAKE_CURRENT_LIST_DIR}/test_sign_strategy.cpp
    ${ROOT_DIR}/src/control/sign_strategy.cpp
)
set_target_properties(test_sign_strategy PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_car_avoid_config
    ${CMAKE_CURRENT_LIST_DIR}/test_car_avoid_config.cpp
    ${ROOT_DIR}/src/io/config.cpp
    ${ROOT_DIR}/src/perception/camera_model.cpp
)
target_link_libraries(test_car_avoid_config ${OpenCV_LIBS})
set_target_properties(test_car_avoid_config PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_ped_relative_away
    ${CMAKE_CURRENT_LIST_DIR}/test_ped_relative_away.cpp
    ${ROOT_DIR}/src/control/ped_relative_away.cpp
)
set_target_properties(test_ped_relative_away PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# control 层（drive_control）测试需附带 UartCommander 与 uart 实现
set(TEST_CONTROL_SOURCES
    ${ROOT_DIR}/src/control/drive_control.cpp
    ${ROOT_DIR}/src/control/ped_relative_away.cpp
    ${ROOT_DIR}/src/control/sign_strategy.cpp
    ${ROOT_DIR}/src/perception/sign_ocr_aggregator.cpp
    ${ROOT_DIR}/src/control/UartCommander.cpp
    ${ROOT_DIR}/src/io/uart.cpp
    ${ROOT_DIR}/src/app/Hud.cpp
)

add_executable(test_car_avoid_direction
    ${CMAKE_CURRENT_LIST_DIR}/test_car_avoid_direction.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_car_avoid_direction PRIVATE XCAR_TESTING)
target_link_libraries(test_car_avoid_direction ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_car_avoid_direction PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_ai_score_thresholds
    ${CMAKE_CURRENT_LIST_DIR}/test_ai_score_thresholds.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_ai_score_thresholds PRIVATE XCAR_TESTING)
target_link_libraries(test_ai_score_thresholds ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_ai_score_thresholds PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_obstacle_lap_policy_control
    ${CMAKE_CURRENT_LIST_DIR}/test_obstacle_lap_policy_control.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_obstacle_lap_policy_control PRIVATE XCAR_TESTING)
target_link_libraries(test_obstacle_lap_policy_control ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_obstacle_lap_policy_control PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_ped_car_conflict_patch
    ${CMAKE_CURRENT_LIST_DIR}/test_ped_car_conflict_patch.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_link_libraries(test_ped_car_conflict_patch ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_ped_car_conflict_patch PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_deleted_elements
    ${CMAKE_CURRENT_LIST_DIR}/test_deleted_elements.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_link_libraries(test_deleted_elements ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_deleted_elements PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_stop_landmark_return_track_guard
    ${CMAKE_CURRENT_LIST_DIR}/test_stop_landmark_return_track_guard.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_stop_landmark_return_track_guard PRIVATE XCAR_TESTING)
target_link_libraries(test_stop_landmark_return_track_guard ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_stop_landmark_return_track_guard PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_launch_state
    ${CMAKE_CURRENT_LIST_DIR}/test_launch_state.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_launch_state PRIVATE XCAR_TESTING)
target_link_libraries(test_launch_state ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_launch_state PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_normal_centerline_tracking
    ${CMAKE_CURRENT_LIST_DIR}/test_normal_centerline_tracking.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_normal_centerline_tracking PRIVATE XCAR_TESTING)
target_link_libraries(test_normal_centerline_tracking ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_normal_centerline_tracking PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_gold_slow_band
    ${CMAKE_CURRENT_LIST_DIR}/test_gold_slow_band.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_gold_slow_band PRIVATE XCAR_TESTING)
target_link_libraries(test_gold_slow_band ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_gold_slow_band PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_gold_follow_enabled
    ${CMAKE_CURRENT_LIST_DIR}/test_gold_follow_enabled.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_gold_follow_enabled PRIVATE XCAR_TESTING)
target_link_libraries(test_gold_follow_enabled ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_gold_follow_enabled PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_gold_slow_mode_hysteresis
    ${CMAKE_CURRENT_LIST_DIR}/test_gold_slow_mode_hysteresis.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_gold_slow_mode_hysteresis PRIVATE XCAR_TESTING)
target_link_libraries(test_gold_slow_mode_hysteresis ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_gold_slow_mode_hysteresis PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_gold_band_visual_config
    ${CMAKE_CURRENT_LIST_DIR}/test_gold_band_visual_config.cpp
    ${ROOT_DIR}/src/io/config.cpp
    ${ROOT_DIR}/src/perception/camera_model.cpp
)
target_link_libraries(test_gold_band_visual_config ${OpenCV_LIBS})
set_target_properties(test_gold_band_visual_config PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(tool_gold_yolo_batch
    ${CMAKE_CURRENT_LIST_DIR}/tools/gold_yolo_batch.cpp
    ${TEST_COMMON_SOURCES}
)
target_link_libraries(tool_gold_yolo_batch ${TEST_LIBS})
set_target_properties(tool_gold_yolo_batch PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_config_cleanup
    ${CMAKE_CURRENT_LIST_DIR}/test_config_cleanup.cpp
    ${ROOT_DIR}/src/io/config.cpp
    ${ROOT_DIR}/src/perception/camera_model.cpp
)
target_link_libraries(test_config_cleanup ${OpenCV_LIBS})
set_target_properties(test_config_cleanup PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_ped_relative_config
    ${CMAKE_CURRENT_LIST_DIR}/test_ped_relative_config.cpp
    ${ROOT_DIR}/src/io/config.cpp
    ${ROOT_DIR}/src/perception/camera_model.cpp
)
target_link_libraries(test_ped_relative_config ${OpenCV_LIBS})
set_target_properties(test_ped_relative_config PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_gold_band_visual_overlay
    ${CMAKE_CURRENT_LIST_DIR}/test_gold_band_visual_overlay.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_gold_band_visual_overlay PRIVATE XCAR_TESTING)
target_link_libraries(test_gold_band_visual_overlay ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_gold_band_visual_overlay PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_person_band_visual_overlay
    ${CMAKE_CURRENT_LIST_DIR}/test_person_band_visual_overlay.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_person_band_visual_overlay PRIVATE XCAR_TESTING)
target_link_libraries(test_person_band_visual_overlay ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_person_band_visual_overlay PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_sign_local_decision
    ${CMAKE_CURRENT_LIST_DIR}/test_sign_local_decision.cpp
    ${ROOT_DIR}/src/io/llm_decision.cpp
)
target_link_libraries(test_sign_local_decision curl ssl crypto pthread)
set_target_properties(test_sign_local_decision PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_sign_llm_requests
    ${CMAKE_CURRENT_LIST_DIR}/test_sign_llm_requests.cpp
    ${ROOT_DIR}/src/app/sign_llm_requests.cpp
    ${ROOT_DIR}/src/io/llm_decision.cpp
)
target_link_libraries(test_sign_llm_requests curl ssl crypto pthread)
set_target_properties(test_sign_llm_requests PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_sign_png_replay
    ${CMAKE_CURRENT_LIST_DIR}/test_sign_png_replay.cpp
    ${ROOT_DIR}/src/io/llm_decision.cpp
    ${ROOT_DIR}/src/io/config.cpp
    ${ROOT_DIR}/src/io/videocapture.cpp
    ${ROOT_DIR}/src/perception/sign_ocr_aggregator.cpp
    ${ROOT_DIR}/src/perception/camera_model.cpp
    ${ROOT_DIR}/src/io/function.cpp
    ${ROOT_DIR}/src/io/terminal_output.cpp
    ${ROOT_DIR}/AI/PPOCR-1/PPOCR-System/cpp/main.cc
    ${ROOT_DIR}/AI/PPOCR-1/PPOCR-System/cpp/postprocess.cc
    ${ROOT_DIR}/AI/PPOCR-1/PPOCR-System/cpp/clipper.cc
    ${ROOT_DIR}/AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_system.cc
    ${ROOT_DIR}/AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_det.cc
    ${ROOT_DIR}/AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_rec.cc
    ${ROOT_DIR}/AI/PPOCR-1/PPOCR-Det/utils/file_utils.c
    ${ROOT_DIR}/AI/PPOCR-1/PPOCR-Det/utils/image_utils.c
    ${ROOT_DIR}/AI/PPOCR-1/PPOCR-Det/utils/image_drawing.c
)
target_link_libraries(test_sign_png_replay ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_sign_png_replay PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_sign_text_replay
    ${CMAKE_CURRENT_LIST_DIR}/test_sign_text_replay.cpp
    ${ROOT_DIR}/src/io/llm_decision.cpp
    ${ROOT_DIR}/src/io/config.cpp
    ${ROOT_DIR}/src/perception/camera_model.cpp
)
target_link_libraries(test_sign_text_replay ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_sign_text_replay PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_sign_failsafe
    ${CMAKE_CURRENT_LIST_DIR}/test_sign_failsafe.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_sign_failsafe PRIVATE XCAR_TESTING)
target_link_libraries(test_sign_failsafe ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_sign_failsafe PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_lost_track_steer
    ${CMAKE_CURRENT_LIST_DIR}/test_lost_track_steer.cpp
    ${ROOT_DIR}/src/app/lost_track_steer.cpp
    ${ROOT_DIR}/src/io/config.cpp
    ${ROOT_DIR}/src/perception/camera_model.cpp
)
target_link_libraries(test_lost_track_steer ${OpenCV_LIBS})
set_target_properties(test_lost_track_steer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_ai_inference_mode_config
    ${CMAKE_CURRENT_LIST_DIR}/test_ai_inference_mode_config.cpp
    ${ROOT_DIR}/src/io/config.cpp
    ${ROOT_DIR}/src/perception/camera_model.cpp
)
target_link_libraries(test_ai_inference_mode_config ${OpenCV_LIBS})
set_target_properties(test_ai_inference_mode_config PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_npu_core_config
    ${CMAKE_CURRENT_LIST_DIR}/test_npu_core_config.cpp
    ${ROOT_DIR}/src/io/config.cpp
    ${ROOT_DIR}/src/perception/camera_model.cpp
)
target_link_libraries(test_npu_core_config ${OpenCV_LIBS})
set_target_properties(test_npu_core_config PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_rknnpool_fake
    ${CMAKE_CURRENT_LIST_DIR}/test_rknnpool_fake.cpp
    ${ROOT_DIR}/AI/base/rknnpool.cpp
)
target_link_libraries(test_rknnpool_fake ${TEST_LIBS})
set_target_properties(test_rknnpool_fake PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_ai_frame_fusion
    ${CMAKE_CURRENT_LIST_DIR}/test_ai_frame_fusion.cpp
    ${ROOT_DIR}/src/perception/ai_frame_fusion.cpp
)
target_compile_options(test_ai_frame_fusion PRIVATE -UNDEBUG)
target_link_libraries(test_ai_frame_fusion ${OpenCV_LIBS})
set_target_properties(test_ai_frame_fusion PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_ai_control_evidence
    ${CMAKE_CURRENT_LIST_DIR}/test_ai_control_evidence.cpp
    ${ROOT_DIR}/src/perception/ai_control_evidence.cpp
)
target_link_libraries(test_ai_control_evidence ${OpenCV_LIBS})
set_target_properties(test_ai_control_evidence PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)


add_executable(test_ai_fusion_cache
    ${CMAKE_CURRENT_LIST_DIR}/test_ai_fusion_cache.cpp
    ${ROOT_DIR}/src/perception/ai_frame_fusion.cpp
    ${ROOT_DIR}/src/perception/ai_control_evidence.cpp
)
target_link_libraries(test_ai_fusion_cache ${OpenCV_LIBS})
set_target_properties(test_ai_fusion_cache PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_ai_control_evidence_bridge
    ${CMAKE_CURRENT_LIST_DIR}/test_ai_control_evidence_bridge.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
    ${ROOT_DIR}/src/perception/ai_control_evidence.cpp
)
target_compile_definitions(test_ai_control_evidence_bridge PRIVATE XCAR_TESTING)
target_link_libraries(test_ai_control_evidence_bridge ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_ai_control_evidence_bridge PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_ped_source_driven_control
    ${CMAKE_CURRENT_LIST_DIR}/test_ped_source_driven_control.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
    ${ROOT_DIR}/src/perception/ai_control_evidence.cpp
)
target_compile_definitions(test_ped_source_driven_control PRIVATE XCAR_TESTING)
target_link_libraries(test_ped_source_driven_control ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_ped_source_driven_control PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_ped_coordinate_threshold_scope
    ${CMAKE_CURRENT_LIST_DIR}/test_ped_coordinate_threshold_scope.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_ped_coordinate_threshold_scope PRIVATE XCAR_TESTING)
target_link_libraries(test_ped_coordinate_threshold_scope ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_ped_coordinate_threshold_scope PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_vehicle_gold_source_driven_control
    ${CMAKE_CURRENT_LIST_DIR}/test_vehicle_gold_source_driven_control.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
    ${ROOT_DIR}/src/perception/ai_control_evidence.cpp
)
target_compile_definitions(test_vehicle_gold_source_driven_control PRIVATE XCAR_TESTING)
target_link_libraries(test_vehicle_gold_source_driven_control ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_vehicle_gold_source_driven_control PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_ocr_feed_sample
    ${CMAKE_CURRENT_LIST_DIR}/test_ocr_feed_sample.cpp
    ${ROOT_DIR}/src/perception/ocr_feed_sample.cpp
)
target_link_libraries(test_ocr_feed_sample ${OpenCV_LIBS})
set_target_properties(test_ocr_feed_sample PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_ocr_box_merge
    ${CMAKE_CURRENT_LIST_DIR}/test_ocr_box_merge.cpp
)
set_target_properties(test_ocr_box_merge PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_sign_ocr_aggregator
    ${CMAKE_CURRENT_LIST_DIR}/test_sign_ocr_aggregator.cpp
    ${ROOT_DIR}/src/perception/sign_ocr_aggregator.cpp
)
target_link_libraries(test_sign_ocr_aggregator ${OpenCV_LIBS})
set_target_properties(test_sign_ocr_aggregator PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_sign_ocr_config
    ${CMAKE_CURRENT_LIST_DIR}/test_sign_ocr_config.cpp
    ${ROOT_DIR}/src/io/config.cpp
    ${ROOT_DIR}/src/perception/camera_model.cpp
)
target_link_libraries(test_sign_ocr_config ${OpenCV_LIBS})
set_target_properties(test_sign_ocr_config PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_llm_valid_decision
    ${CMAKE_CURRENT_LIST_DIR}/test_llm_valid_decision.cpp
    ${ROOT_DIR}/src/io/llm_decision.cpp
)
target_link_libraries(test_llm_valid_decision curl ssl crypto pthread)
set_target_properties(test_llm_valid_decision PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_llm_async_transport
    ${CMAKE_CURRENT_LIST_DIR}/test_llm_async_transport.cpp
    ${ROOT_DIR}/src/io/llm_decision.cpp
)
target_link_libraries(test_llm_async_transport ssl crypto pthread)
set_target_properties(test_llm_async_transport PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(test_gold_guidance_weight_split
    ${CMAKE_CURRENT_LIST_DIR}/test_gold_guidance_weight_split.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_gold_guidance_weight_split PRIVATE XCAR_TESTING)
target_link_libraries(test_gold_guidance_weight_split ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_gold_guidance_weight_split PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

message(STATUS "=================================")
message(STATUS "  Xcar Test Build")
message(STATUS "=================================")
message(STATUS "OpenCV:        ${OpenCV_VERSION}")
message(STATUS "Build type:   ${CMAKE_BUILD_TYPE}")
message(STATUS "Output dir:    ${CMAKE_BINARY_DIR}/bin")
