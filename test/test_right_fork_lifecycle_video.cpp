#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace {

struct Stats {
    int step = 1;
    int offset = 0;
    int right = 0;
    int left = 0;
    int firstRight = -1;
    int samples = 0;
    std::uint64_t midHash = 1469598103934665603ULL;
};

void resetMainRoad()
{
    ppsegResetTemporalMaskState();
    resetTrackRoadMode();
    resetForkPhaseHunt();
    resetForkEntryState();
    resetForkExitSlopeCalib();
    resetForkSideX();
    setForkScanBiasLocked(false);
    setForkScanBias(ForkScanBias::None);
    imgprocess_set_sign_blocks_auto_fork(false);
}

Stats collectMainExit(const std::string& path, int step, int offset)
{
    cv::VideoCapture cap(path);
    Stats out;
    out.step = step;
    out.offset = offset;
    if (!cap.isOpened())
        return out;

    const double fps = cap.get(cv::CAP_PROP_FPS);
    resetMainRoad();
    cv::Mat frame;
    int index = 0;
    while (cap.read(frame)) {
        if (index % step != offset) {
            ++index;
            continue;
        }
        const CenterLineResult result = processFrame(frame);
        const double sec = fps > 0.0 ? (double)index / fps : 0.0;
        if (sec >= 6.6 && sec <= 9.8) {
            ++out.samples;
            const ForkExitRepairState repair = getForkExitRepairState();
            if (repair.active && repair.side == ForkExitRepairSide::Right) {
                if (out.firstRight < 0)
                    out.firstRight = index;
                ++out.right;
            }
            if (repair.active && repair.side == ForkExitRepairSide::Left)
                ++out.left;
            const int mid = result.boundary.mid.size() > 175
                ? result.boundary.mid[175] : -1;
            out.midHash ^= (std::uint64_t)(mid + 2);
            out.midHash *= 1099511628211ULL;
        }
        ++index;
    }
    return out;
}

std::vector<Stats> collectMatrix(const std::string& path)
{
    std::vector<Stats> rows;
    for (int step = 1; step <= 3; ++step)
        for (int offset = 0; offset < step; ++offset)
            rows.push_back(collectMainExit(path, step, offset));
    return rows;
}

bool writeCsv(const std::string& path, const std::vector<Stats>& rows)
{
    std::ofstream out(path);
    if (!out)
        return false;
    out << "step,offset,right,left,first_right,mid175_hash\n";
    for (const Stats& s : rows) {
        if (s.samples <= 0)
            return false;
        out << s.step << ',' << s.offset << ',' << s.right << ',' << s.left
            << ',' << s.firstRight << ',' << s.midHash << '\n';
    }
    return true;
}

bool sameStats(const Stats& a, const Stats& b)
{
    return a.step == b.step &&
           a.offset == b.offset &&
           a.right == b.right &&
           a.left == b.left &&
           a.firstRight == b.firstRight &&
           a.midHash == b.midHash;
}

std::vector<Stats> readCsv(const std::string& path)
{
    std::ifstream in(path);
    std::vector<Stats> rows;
    std::string line;
    std::getline(in, line);
    while (std::getline(in, line)) {
        Stats s;
        unsigned long long hash = 0;
        if (std::sscanf(line.c_str(), "%d,%d,%d,%d,%d,%llu",
                        &s.step, &s.offset, &s.right, &s.left,
                        &s.firstRight, &hash) == 6) {
            s.midHash = (std::uint64_t)hash;
            rows.push_back(s);
        }
    }
    return rows;
}

struct JourneyStats {
    bool reachedBranch = false;
    bool branchAtMainExit = false;
    int totalLeft = 0;
    int entranceLeft = 0;
    int exitLeft = 0;
};

JourneyStats collectJourney(const std::string& path,
                            int step, int offset,
                            bool forceRight,
                            double entranceBegin,
                            double entranceEnd,
                            double exitBegin,
                            double exitEnd)
{
    cv::VideoCapture cap(path);
    JourneyStats out;
    if (!cap.isOpened())
        return out;
    const double fps = cap.get(cv::CAP_PROP_FPS);
    resetMainRoad();
    if (forceRight) {
        setForkScanBiasLocked(true);
        setForkScanBias(ForkScanBias::Right);
    }

    cv::Mat frame;
    int index = 0;
    while (cap.read(frame)) {
        if (index % step != offset) {
            ++index;
            continue;
        }
        (void)processFrame(frame);
        const double sec = fps > 0.0 ? (double)index / fps : 0.0;
        const RightForkJourneyPhase journey =
            getRightForkJourneyPhase();
        if (journey == RightForkJourneyPhase::InRightBranch ||
            journey == RightForkJourneyPhase::RightExitRepair)
            out.reachedBranch = true;
        const ForkExitRepairState repair = getForkExitRepairState();
        const bool left = repair.active &&
            repair.side == ForkExitRepairSide::Left;
        if (left)
            ++out.totalLeft;
        if (sec >= entranceBegin && sec <= entranceEnd && left)
            ++out.entranceLeft;
        if (sec >= exitBegin && sec <= exitEnd) {
            if (left)
                ++out.exitLeft;
            if (journey == RightForkJourneyPhase::InRightBranch ||
                journey == RightForkJourneyPhase::RightExitRepair)
                out.branchAtMainExit = true;
        }
        ++index;
    }
    return out;
}

bool checkLifecycleMatrix(const std::string& rightPath,
                          const std::string& mainPath,
                          bool stabilized)
{
    config().img.ppsegMaskStabilize = stabilized;
    const char* mode = stabilized ? "stabilized" : "unstabilized";
    for (int step = 1; step <= 3; ++step) {
        for (int offset = 0; offset < step; ++offset) {
            const JourneyStats right = collectJourney(
                rightPath, step, offset, true,
                2.4, 5.0, 9.6, 12.8);
            if (!right.reachedBranch ||
                right.entranceLeft != 0 ||
                right.exitLeft == 0) {
                std::printf(
                    "[FAIL] %s right step=%d offset=%d reached=%d entryLeft=%d exitLeft=%d\n",
                    mode, step, offset, right.reachedBranch ? 1 : 0,
                    right.entranceLeft, right.exitLeft);
                return false;
            }

            const JourneyStats main = collectJourney(
                mainPath, step, offset, false,
                21.5, 23.8, 6.6, 9.8);
            if (main.reachedBranch ||
                main.totalLeft != 0 ||
                main.entranceLeft != 0 ||
                main.exitLeft != 0 ||
                main.branchAtMainExit) {
                std::printf(
                    "[FAIL] %s main step=%d offset=%d reached=%d totalLeft=%d entryLeft=%d exitLeft=%d\n",
                    mode, step, offset, main.reachedBranch ? 1 : 0,
                    main.totalLeft, main.entranceLeft, main.exitLeft);
                return false;
            }

            const JourneyStats stress = collectJourney(
                mainPath, step, offset, true,
                21.5, 25.5, 21.5, 25.5);
            if (stress.reachedBranch ||
                stress.totalLeft != 0 ||
                stress.entranceLeft != 0 ||
                stress.exitLeft != 0) {
                std::printf(
                    "[FAIL] %s stress step=%d offset=%d reached=%d totalLeft=%d left=%d\n",
                    mode, step, offset, stress.reachedBranch ? 1 : 0,
                    stress.totalLeft,
                    stress.entranceLeft + stress.exitLeft);
                return false;
            }
        }
    }
    std::printf("%s lifecycle matrix passed: 6 variants x 3 groups\n",
                mode);
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    if (!configLoad(std::string(XCAR_PROJECT_ROOT) + "/configs/config.json"))
        return 1;
    config().img.ppsegMaskStabilize = false;
    if (config().img.usePpSegTrack && !ppsegTrackInit())
        return 1;

    if (argc == 4 && std::string(argv[1]) == "--record-main") {
        const auto rows = collectMatrix(argv[2]);
        return rows.size() == 6 && writeCsv(argv[3], rows) ? 0 : 1;
    }
    if (argc != 5 || std::string(argv[1]) != "--check") {
        std::printf(
            "usage: %s --check <RightFork.mp4> <OneCycle.mp4> <baseline.csv>\n",
            argv[0]);
        return 1;
    }

    const std::vector<Stats> expected = readCsv(argv[4]);
    const std::vector<Stats> actual = collectMatrix(argv[3]);
    if (expected.size() != 6 || actual.size() != expected.size())
        return 2;
    for (size_t i = 0; i < expected.size(); ++i)
        if (!sameStats(expected[i], actual[i])) {
            std::printf("[FAIL] main baseline step=%d offset=%d\n",
                        actual[i].step, actual[i].offset);
            return 2;
        }

    if (!checkLifecycleMatrix(argv[2], argv[3], false))
        return 2;
    if (!checkLifecycleMatrix(argv[2], argv[3], true))
        return 2;

    std::printf("right fork lifecycle video matrices passed\n");
    return 0;
}
