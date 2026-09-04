#ifndef INFER_WRAP_H
#define INFER_WRAP_H

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <tuple>
#include <cstdint>
#include <vector>

struct DetectResult;
class rknnPoolExecutor; 

struct InferenceTask {
    cv::Mat frame;
};

class InferWrap {
public:
    InferWrap(const std::string& model_dir, int thread_num = 1);
    ~InferWrap();

    bool put(cv::Mat frame, uint64_t fid = 0);
    std::tuple<cv::Mat, std::vector<DetectResult>, bool> get();

    void release();

private:
    std::string get_model_path(const std::string& model_dir);
    
    std::unique_ptr<rknnPoolExecutor> executor; 
    bool stop;
};

#endif
