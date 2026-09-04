#include "ocr_box_merge.h"
#include <cassert>
#include <cmath>
#include <iostream>

static std::array<int, 8> box(int x0, int y0, int x1, int y1)
{ return {x0, y0, x1, y0, x1, y1, x0, y1}; }

int main()
{
    std::vector<std::array<int, 8>> boxes{box(30,10,50,20), box(5,9,25,20)};
    std::vector<float> scores{.8f,.6f};
    ocr_box_merge::mergeBoxesByRow(boxes, scores);
    assert(boxes.size() == 1 && boxes[0] == box(5,9,50,20));
    assert(std::abs(scores[0] - .7f) < 1e-6f);

    boxes = {box(5,10,15,20), box(50,10,60,20), box(5,35,20,45)};
    scores = {.7f,.8f,.9f};
    ocr_box_merge::mergeBoxesByRow(boxes, scores);
    assert(boxes.size() == 3);

    boxes = {box(5,10,30,20), box(15,10,35,20)};
    scores = {.7f,.8f};
    ocr_box_merge::mergeBoxesByRow(boxes, scores);
    assert(boxes.size() == 2);
    std::cout << "ocr_box_merge tests passed\n";
}
