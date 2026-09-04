#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace ocr_box_merge {

struct Options {
    float minVerticalOverlap = 0.60f;
    float maxCenterDistanceHeightRatio = 0.50f;
    float maxHorizontalGapHeightRatio = 2.0f;
    float maxReverseOverlapHeightRatio = 0.30f;
};

struct Segment {
    size_t id;
    int minX, maxX, minY, maxY;
    int height() const { return std::max(1, maxY - minY + 1); }
    double centerY() const { return (minY + maxY) * 0.5; }
};

inline Segment segmentOf(const std::array<int, 8>& box, size_t id)
{
    Segment s{id, box[0], box[0], box[1], box[1]};
    for (int i = 1; i < 4; ++i) {
        s.minX = std::min(s.minX, box[i * 2]);
        s.maxX = std::max(s.maxX, box[i * 2]);
        s.minY = std::min(s.minY, box[i * 2 + 1]);
        s.maxY = std::max(s.maxY, box[i * 2 + 1]);
    }
    return s;
}

inline float verticalOverlap(const Segment& a, const Segment& b)
{
    const int overlap = std::max(0, std::min(a.maxY, b.maxY) -
                                    std::max(a.minY, b.minY) + 1);
    return static_cast<float>(overlap) /
           static_cast<float>(std::min(a.height(), b.height()));
}

inline void mergeBoxesByRow(std::vector<std::array<int, 8>>& boxes,
                            std::vector<float>& scores,
                            const Options& options = {})
{
    if (boxes.empty()) { scores.clear(); return; }
    scores.resize(boxes.size(), 1.0f);
    std::vector<Segment> pending;
    for (size_t i = 0; i < boxes.size(); ++i) pending.push_back(segmentOf(boxes[i], i));
    std::stable_sort(pending.begin(), pending.end(), [](const Segment& a, const Segment& b) {
        return a.centerY() == b.centerY() ? a.minX < b.minX : a.centerY() < b.centerY();
    });

    struct Row { std::vector<Segment> items; double center = 0; double height = 1; };
    std::vector<Row> rows;
    for (const auto& s : pending) {
        size_t best = rows.size();
        double bestDistance = std::numeric_limits<double>::max();
        for (size_t i = 0; i < rows.size(); ++i) {
            const int h = std::max(1, static_cast<int>(std::lround(rows[i].height)));
            const int y0 = static_cast<int>(std::lround(rows[i].center - (h - 1) * .5));
            const Segment representative{0, 0, 0, y0, y0 + h - 1};
            const double distance = std::abs(s.centerY() - rows[i].center);
            if (verticalOverlap(s, representative) < options.minVerticalOverlap ||
                distance > options.maxCenterDistanceHeightRatio *
                           std::min(s.height(), representative.height())) continue;
            if (distance < bestDistance) { best = i; bestDistance = distance; }
        }
        if (best == rows.size()) rows.push_back({{s}, s.centerY(), static_cast<double>(s.height())});
        else {
            Row& row = rows[best];
            const double n = row.items.size();
            row.center = (row.center * n + s.centerY()) / (n + 1);
            row.height = (row.height * n + s.height()) / (n + 1);
            row.items.push_back(s);
        }
    }
    std::stable_sort(rows.begin(), rows.end(),
                     [](const Row& a, const Row& b) { return a.center < b.center; });

    std::vector<std::array<int, 8>> merged;
    std::vector<float> mergedScores;
    for (auto& row : rows) {
        std::stable_sort(row.items.begin(), row.items.end(),
                         [](const Segment& a, const Segment& b) { return a.minX < b.minX; });
        bool active = false;
        Segment group{};
        Segment previous{};
        float scoreSum = 0;
        size_t count = 0;
        auto flush = [&] {
            if (!active) return;
            merged.push_back({group.minX, group.minY, group.maxX, group.minY,
                              group.maxX, group.maxY, group.minX, group.maxY});
            mergedScores.push_back(scoreSum / std::max<size_t>(1, count));
            active = false;
        };
        for (const auto& s : row.items) {
            const int referenceHeight = active ? std::max(1, std::min(previous.height(), s.height())) : 1;
            const int gap = active ? s.minX - previous.maxX : 0;
            const bool join = active &&
                gap <= options.maxHorizontalGapHeightRatio * referenceHeight &&
                gap >= -options.maxReverseOverlapHeightRatio * referenceHeight;
            if (!join) {
                flush(); active = true; group = s; scoreSum = scores[s.id]; count = 1;
            } else {
                group.minX = std::min(group.minX, s.minX); group.maxX = std::max(group.maxX, s.maxX);
                group.minY = std::min(group.minY, s.minY); group.maxY = std::max(group.maxY, s.maxY);
                scoreSum += scores[s.id]; ++count;
            }
            previous = s;
        }
        flush();
    }
    boxes = std::move(merged);
    scores = std::move(mergedScores);
}

} // namespace ocr_box_merge
