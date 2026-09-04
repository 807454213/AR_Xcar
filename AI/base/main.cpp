// #include <iostream>
// #include <string>
// #include <chrono> 
// #include <iomanip>
// #include <opencv2/opencv.hpp>
// #include "rknnpool.h"
// #include "func.h"
// #include "/home/orangepi/Desktop/setupUI2/dist/Xcar/include/videocapture.h" 

// class AIProcessor {
// private:
//     rknnPoolExecutor* pool;
//     int thread_num;
//     int put_count;
//     int processed_frames;
//     double current_fps;
//     std::chrono::time_point<std::chrono::high_resolution_clock> start_time;

// public:
//     // 构造函数：初始化模型和线程池
//     AIProcessor(const std::string& model_path, int threads) 
//         : thread_num(threads), put_count(0), processed_frames(0), current_fps(0.0) 
//     {
//         pool = new rknnPoolExecutor(model_path, thread_num, run_inference);
//         start_time = std::chrono::high_resolution_clock::now();
//     }

//     // 析构函数：释放资源
//     ~AIProcessor() {
//         if (pool != nullptr) {
//             // 排空流水线中的剩余帧
//             int remaining_frames = put_count - processed_frames;
//             for (int i = 0; i < remaining_frames; ++i) {
//                 pool->get(); 
//             }
//             pool->release();
//             delete pool;
//         }
//     }

//     void process(cv::Mat& frame) 
//     {
//         if (frame.empty()) return;

//         pool->put(frame);
//         put_count++;

//         if (put_count >= thread_num) 
//         {
//             auto result = pool->get();
//             if (std::get<2>(result) && !std::get<0>(result).empty())
//             {
//                 frame = std::get<0>(result); 

//                 // 计算并绘制 FPS
//                 processed_frames++;
//                 auto now = std::chrono::high_resolution_clock::now();
//                 std::chrono::duration<double> elapsed = now - start_time;
                
//                 //0.5秒刷新一次 FPS 数值
//                 if (elapsed.count() >= 0.5) 
//                 { 
//                     current_fps = processed_frames / elapsed.count();
//                     start_time = now;
//                     processed_frames = 0;
//                 }

//                 char fps_text[64];
//                 sprintf(fps_text, "FPS: %.1f", current_fps);
//                 cv::putText(frame, fps_text, cv::Point(15, 30), 
//                             cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
//             } 
//             else 
//             {
//                 frame = cv::Mat(); // 处理失败，返回空帧
//             }
//         } 
//         else 
//         {
//             frame = cv::Mat(); 
//         }
//     }
// };

// int main()
// {
//     std::string model_path = "/home/orangepi/Desktop/setupUI2/dist/Xcar/AI/base/model/rknn_lt.rknn";
//     AIProcessor ai_engine(model_path, 3); 

//     ShmCapture capture("shm_ar_video", 16);
//     capture.start();

//     while (true)
//     {
//         ShmCapture::FrameInfo finfo;
        
//         // 读取帧
//         if (capture.read(finfo, 1000)) 
//         {
//             //共享内存可能传过来空图，先过滤掉
//             if (finfo.frame.empty()) continue;

//             //必须使用 clone() 进行深拷贝
//             // 把图像完完整整复制到属于自己进程的安全内存里，切断和共享内存的联系
//             cv::Mat frame = finfo.frame.clone(); 
            
//             ai_engine.process(frame);
            
//             // 必须判断流水线有没有吐出成品图，才能交给 imshow 显示
//             if (!frame.empty()) 
//             {
//                 cv::imshow("AI 处理结果", frame);
//                 if (cv::waitKey(1) == 27) 
//                     break;
//             }
//         }
//         else
//         {
//             std::cout << "读取超时，程序退出。" << std::endl;
//             break;
//         }
//     }

//     capture.stop();
//     return 0;
// }

#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <opencv2/opencv.hpp>
#include "rknnpool.h"
#include "func.h"

// 这是一个专为实时摄像头设计的取帧器
class RealTimeCamera {
private:
    cv::VideoCapture cap;
    cv::Mat latest_frame;
    std::mutex mtx;
    std::atomic<bool> running;
    std::thread capture_thread;

    void capture_loop() {
        cv::Mat temp_frame;
        while (running) {
            cap >> temp_frame; // 读取，不阻塞
            if (temp_frame.empty()) {
                std::cerr << "Camera disconnected!" << std::endl;
                break;
            }
            
            // 加锁，将最新帧保存下来
            std::lock_guard<std::mutex> lock(mtx);
            // 使用 clone 防止多线程资源冲突
            latest_frame = temp_frame.clone(); 
        }
    }

public:
    RealTimeCamera(const std::string& source = "/dev/video0") : running(false) {
        // 根据传入的是数字(USB摄像头)还是字符串(RTSP流)进行打开
        if (source.length() == 1 && isdigit(source[0])) {
            cap.open(std::stoi(source)); 
        } else {
            cap.open(source);
        }

        if (!cap.isOpened()) {
            std::cerr << "Failed to open camera: " << source << std::endl;
            exit(-1);
        }

        // 启动后台狂奔的取帧线程
        running = true;
        capture_thread = std::thread(&RealTimeCamera::capture_loop, this);
    }

    ~RealTimeCamera() {
        running = false;
        if (capture_thread.joinable()) {
            capture_thread.join();
        }
        cap.release();
    }

    // 供推理线程调用，拿走最新的一帧
    bool get_latest_frame(cv::Mat& frame) {
        std::lock_guard<std::mutex> lock(mtx);
        if (latest_frame.empty()) return false;
        
        frame = latest_frame; 
        latest_frame.release(); // 拿走后清空，避免同一帧被推理两次
        return true;
    }
};

// 主函数示例
int main() {
    // 1. 初始化你的线程池 (假设模型路径为 model.rknn，使用 3 个 NPU 核心)
    int thread_num = 3;
    rknnPoolExecutor pool("model.rknn", thread_num, run_inference);

    // 2. 初始化实时摄像头
    // 如果是USB摄像头传 "0"，如果是网络摄像头传 "rtsp://..."
    RealTimeCamera camera("0"); 
    
    cv::Mat current_frame;
    int put_count = 0;

    std::cout << "Start real-time inference..." << std::endl;

    while (true) {
        // 3. 从摄像头获取
        if (camera.get_latest_frame(current_frame)) {
            // 把最新帧塞进推理池
            if (pool.put(current_frame)) {
                put_count++;
            }
        } else {
            // 如果没拿到新帧，稍微等一下
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        // 4. 控制流水线深度，提取结果并渲染
        // 当塞入的任务数量大于等于线程池大小时，开始取结果，保持流水线转动
        if (put_count >= thread_num) {
            auto result = pool.get(); // 这里阻塞等待是安全的，因为读摄像头的线程还在独立干活！
            cv::Mat res_img = std::get<0>(result);
            bool is_valid = std::get<2>(result);

            if (is_valid && !res_img.empty()) {
                cv::imshow("RKNN Real-Time Detect", res_img);
                if (cv::waitKey(1) == 27) { // 按 ESC 退出
                    break;
                }
            }
        }
    }

    pool.release();
    cv::destroyAllWindows();
    return 0;
}