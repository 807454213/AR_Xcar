// 共享内存视频预览 + PPSeg 语义分割二值图
//
// 独立构建: test/shm_test/build.sh
// 运行:
//   ./bin/shm_test                    # 默认加载 configs/config.json，仅原图
//   ./bin/shm_test --compare          # 左原图 / 右二值对比
//
// 按键: q 退出  s 保存原图  m 保存二值 mask

#include "config.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <ctime>
#include <cerrno>
#include <string>
#include <vector>
#include <filesystem>

using namespace cv;
using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

const string SHM_NAME = "shm_ar_video";
const size_t HEADER_SIZE = 16;

static string g_save_dir;
static string g_config_override;
static bool g_show_compare = false;
static Mat g_seg_mask;
static Mat g_latest_frame;

static filesystem::path sourceRootPath()
{
    return filesystem::path(__FILE__).parent_path().parent_path();
}

static vector<string> defaultConfigCandidates()
{
    const filesystem::path root = sourceRootPath();
    return {
        "configs/config.json",
        "../configs/config.json",
        "../../configs/config.json",
        (root / "configs/config.json").string(),
        "/home/orangepi/Desktop/Xcar2/configs/config.json",
        "/home/orangepi/Desktop/Xcar/configs/config.json",
    };
}

static bool fileExists(const string& path)
{
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

static bool loadConfig(string& loaded_path)
{
    if (!g_config_override.empty()) {
        if (configLoad(g_config_override)) {
            loaded_path = g_config_override;
            return true;
        }
        cerr << "[Config] load failed: " << g_config_override << endl;
        return false;
    }

    for (const string& path : defaultConfigCandidates()) {
        if (!fileExists(path))
            continue;
        if (configLoad(path)) {
            loaded_path = path;
            return true;
        }
    }

    cerr << "[Config] no config.json found (tried configs/config.json and fallbacks)" << endl;
    return false;
}

static bool dirWritable(const string& dir)
{
    return access(dir.c_str(), W_OK) == 0;
}

static string initSaveDir()
{
    const filesystem::path root = sourceRootPath();
    vector<string> candidates = {
        (root / "test/img").string(),
        "/home/orangepi/Desktop/Xcar2/test/img",
        "/home/orangepi/Desktop/Xcar/test/img",
    };
    if (const char* home = getenv("HOME")) {
        candidates.push_back(string(home) + "/Desktop/Xcar/test/img");
        candidates.push_back(string(home) + "/xcar_shm_test");
    }
    candidates.push_back("/tmp/xcar_shm_test");

    for (const string& dir : candidates) {
        error_code ec;
        filesystem::create_directories(dir, ec);
        if (ec) continue;
        if (dirWritable(dir)) {
            cout << "[Save] dir: " << dir << endl;
            return dir;
        }
        cerr << "[Save] skip (not writable): " << dir << endl;
    }
    return {};
}

static bool ensureSaveDir()
{
    if (!g_save_dir.empty() && dirWritable(g_save_dir))
        return true;
    g_save_dir = initSaveDir();
    if (g_save_dir.empty()) {
        cerr << "\n  [Save] no writable save directory" << endl;
        return false;
    }
    return true;
}

static bool saveImage(const Mat& img, const char* prefix)
{
    if (img.empty()) {
        cerr << "\n  [Save] empty image" << endl;
        return false;
    }
    if (!ensureSaveDir())
        return false;

    const auto now = chrono::system_clock::now();
    const time_t sec = chrono::system_clock::to_time_t(now);
    const auto ms = chrono::duration_cast<chrono::milliseconds>(
                        now.time_since_epoch()).count() % 1000;
    struct tm tm_buf;
    localtime_r(&sec, &tm_buf);

    char path[384];
    snprintf(path, sizeof(path),
             "%s/%s_%04d%02d%02d_%02d%02d%02d_%03lld.png",
             g_save_dir.c_str(), prefix,
             tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
             tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec,
             (long long)ms);

    if (img.channels() != 1 && img.channels() != 3 && img.channels() != 4) {
        cerr << "\n  [Save] unsupported channels: " << img.channels() << endl;
        return false;
    }

    if (!imwrite(path, img)) {
        cerr << "\n  [Save] imwrite failed: " << path;
        if (access(g_save_dir.c_str(), W_OK) != 0)
            cerr << " (permission denied on " << g_save_dir << ")";
        cerr << endl;
        return false;
    }
    cout << "\n  [Save] " << path
         << " (" << img.cols << "x" << img.rows
         << " ch=" << img.channels() << ")" << endl;
    return true;
}

static bool saveCurrentFrame()
{
    if (g_latest_frame.empty()) {
        cerr << "\n  [Save] no camera frame yet (wait for SHM video)" << endl;
        return false;
    }
    return saveImage(g_latest_frame, "shm");
}

static bool saveSegMask()
{
    if (g_seg_mask.empty()) {
        cerr << "\n  [Save] no seg mask yet (PPSeg not ready or no inference)" << endl;
        return false;
    }
    Mat bw;
    if (g_seg_mask.channels() == 1)
        threshold(g_seg_mask, bw, 127, 255, THRESH_BINARY);
    else
        cvtColor(g_seg_mask, bw, COLOR_BGR2GRAY);
    return saveImage(bw, "seg");
}

static Mat makeComparePanel(const Mat& frame, const Mat& mask)
{
    Mat right;
    if (mask.channels() == 1)
        cvtColor(mask, right, COLOR_GRAY2BGR);
    else
        right = mask.clone();

    Mat panel;
    hconcat(frame, right, panel);
    putText(panel, "camera", Point(8, 16), FONT_HERSHEY_SIMPLEX, 0.45,
            Scalar(0, 255, 0), 1, LINE_AA);
    putText(panel, "PPSeg binary (255=track)", Point(frame.cols + 8, 16),
            FONT_HERSHEY_SIMPLEX, 0.42, Scalar(0, 255, 255), 1, LINE_AA);
    return panel;
}

static bool inferSegBinary(const Mat& bgr, Mat& outMask)
{
    outMask.release();
    if (!ppsegTrackReady() || bgr.empty())
        return false;
    return ppsegInferTrackMask(bgr, outMask);
}

static void parseArgs(int argc, char* argv[])
{
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--config") && i + 1 < argc) {
            g_config_override = argv[++i];
        } else if (!strcmp(argv[i], "--compare")) {
            g_show_compare = true;
        } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            cout <<
                "usage: shm_test [options]\n"
                "  default config: configs/config.json (with path fallbacks)\n"
                "  --config PATH   override config file\n"
                "  --compare       show camera | binary side-by-side\n"
                "  keys: q quit  s save camera PNG  m save seg mask PNG\n";
            exit(0);
        }
    }
}

int main(int argc, char* argv[])
{
    parseArgs(argc, argv);

    cout << "=== Shared Memory Video + PPSeg Binary Test ===" << endl;
    g_save_dir = initSaveDir();
    if (g_save_dir.empty())
        cerr << "[Save] warning: no writable dir yet; will retry on first save" << endl;

    string config_path;
    if (!loadConfig(config_path)) {
        cerr << "[PPSeg] disabled (config missing)." << endl;
    } else {
        cout << "[Config] loaded: " << config_path << endl;
    }

    bool seg_ready = false;
    if (!config_path.empty()) {
        if (!config().img.usePpSegTrack) {
            cerr << "[PPSeg] usePpSegTrack=false, seg disabled." << endl;
        } else if (!ppsegTrackInit()) {
            cerr << "[PPSeg] init failed." << endl;
        } else {
            seg_ready = true;
            cout << "[PPSeg] ready." << endl;
        }
    }

    cout << "[Step 1] Opening shared memory: " << SHM_NAME << endl;
    int fd = -1;
    while (true) {
        fd = shm_open(SHM_NAME.c_str(), O_RDONLY, 0666);
        if (fd >= 0) break;
        cerr << "  shm_open failed, retrying in 500ms..." << endl;
        sleep_for(milliseconds(500));
    }
    cout << "  shm_open success! fd=" << fd << endl;

    struct stat st;
    if (fstat(fd, &st) < 0) {
        perror("fstat");
        ppsegTrackShutdown();
        return 1;
    }
    cout << "  shm size: " << st.st_size << " bytes" << endl;

    void* shm_ptr = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
    if (shm_ptr == MAP_FAILED) {
        perror("mmap");
        close(fd);
        ppsegTrackShutdown();
        return 1;
    }
    cout << "  mmap success!" << endl;

    cout << "[Step 2] Displaying... (q=quit s=save frame m=save mask)" << endl;
    if (g_show_compare)
        cout << "  [View] --compare: camera | binary in one window" << endl;
    else
        cout << "  [View] camera only (use --compare for binary panel)" << endl;

    namedWindow("Shared Memory Video", WINDOW_NORMAL | WINDOW_FREERATIO);

    uint64_t last_fid = -1;
    Mat display_frame;
    int fps = 0, accum = 0;
    auto fps_tick = steady_clock::now();

    while (true) {
        uint64_t fid;
        uint32_t w, h;
        memcpy(&fid, shm_ptr, 8);
        memcpy(&w,   (uint8_t*)shm_ptr + 8,  4);
        memcpy(&h,   (uint8_t*)shm_ptr + 12, 4);

        if (fid != last_fid && w > 0 && h > 0) {
            last_fid = fid;
            Mat rgb(h, w, CV_8UC3, (uint8_t*)shm_ptr + HEADER_SIZE);
            flip(rgb, display_frame, 0);
            cvtColor(display_frame, display_frame, COLOR_RGB2BGR);
            g_latest_frame = display_frame.clone();

            Mat show_frame = display_frame;
            if (seg_ready)
                inferSegBinary(display_frame, g_seg_mask);

            if (g_show_compare && seg_ready && !g_seg_mask.empty())
                show_frame = makeComparePanel(display_frame, g_seg_mask);

            const int show_w = g_show_compare ? (int)w * 2 : (int)w;
            resizeWindow("Shared Memory Video", show_w, (int)h);
            imshow("Shared Memory Video", show_frame);

            accum++;
            auto now = steady_clock::now();
            double elapsed = duration<double>(now - fps_tick).count();
            if (elapsed >= 1.0) {
                fps = (int)(accum / elapsed + 0.5);
                accum = 0;
                fps_tick = now;
            }
            cout << "\r  FPS: " << fps << "  Frame: " << fid
                 << "  Size: " << w << "x" << h;
            if (seg_ready && ppsegTrackReady()) {
                cout << "  Seg: " << ppsegTrackLastInferMs() << "ms";
                if (!g_seg_mask.empty())
                    cout << "  white=" << countNonZero(g_seg_mask);
            }
            cout << flush;
        }

        int key = waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27) {
            cout << endl << "  Quit key pressed." << endl;
            break;
        }
        if (key == 's' || key == 'S')
            saveCurrentFrame();
        if (key == 'm' || key == 'M')
            saveSegMask();
    }

    munmap(shm_ptr, st.st_size);
    close(fd);
    destroyAllWindows();
    ppsegTrackShutdown();
    cout << "[Done]" << endl;
    return 0;
}
