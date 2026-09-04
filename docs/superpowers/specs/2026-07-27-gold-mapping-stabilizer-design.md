# Gold Mapping Stabilizer Design

**Goal:** Improve gold coin mapped points so `Track` / `Band` / `Outside` / `Reachable` classification stays stable relative to the blue arrow track, while keeping the point close to the visual ground contact.

**Context:** Xcar computes gold guidance through `tcGoldFootPoint()` in `src/control/drive_control.cpp`, which calls `tc_goldMappedYFromBox()` from `include/trackcontrol.h`. The current runtime configuration uses `goldMappedYK1=1`, `goldMappedYHeightRatio=1.81`, and `goldMappedYOffset=0`, so the baseline mapped row is effectively `box.y + box.height * 1.81`. The provided calibration images are 320x240 frames under `test/img/shm_20260727_*.png`.

## Design

Keep the existing box-based mapped point as the raw estimate, then add a small stabilization layer for locked gold targets:

1. Compute `raw_point = (center_x, tc_goldMappedYFromBox(box, img_h))`.
2. When the gold target is locked and the raw mapped row has valid left/right track boundaries, record its relative horizontal position:
   `rel = (raw_x - left[raw_y]) / (right[raw_y] - left[raw_y])`.
   Do not clamp `rel`; values below 0 or above 1 represent gold outside the left/right blue track boundary.
3. On the next matched detection of the same locked target, project the stored `rel` through the current frame's track boundaries at the candidate mapped row:
   `stable_x = left[y] + rel * (right[y] - left[y])`.
4. Use the stable point only if the current boundary row is valid and the correction is plausible. Otherwise, fall back to the raw point.
5. Use the stabilized point for gold zone classification, reachability, lock update, dynamic error row selection, guidance point generation, debug drawing, and outside-gold recording. Do not alter the YOLO box or non-gold detections.

The stabilizer must fail open. During missing boundaries, lost track, fork complexity, large target jumps, or new gold target acquisition, existing raw mapping behavior remains active.

## YOLO Calibration Pass

Before implementing the stabilizer, run the current RKNN YOLO model over the provided images to collect actual detection boxes. The repo does not currently build a ready-made batch image detector, but it has the necessary pieces:

- Model: `AI/base/model/rknn_lt.rknn`
- Runtime: `/usr/lib/librknnrt.so`
- Inference code: `AI/base/ppyoloe.cc`, `AI/base/func.cpp`, `AI/base/postprocess.cc`

Add or use a small local batch detector that writes CSV rows:

```text
image,class_id,score,x,y,w,h,center_x,center_y,raw_mapped_y
```

Optionally write annotated images with the YOLO box, raw mapped point, stabilized point, and current track boundary row. The CSV is a calibration artifact, not production input.

## Tests

Use TDD before production changes:

1. Add a unit-style gold mapping test with synthetic straight-track boundaries where the same locked gold target has two detections with shifted `box.y` / `box.height`. The stabilized point must preserve relative track position and avoid a one-frame zone jump.
2. Add a test where the current row has invalid boundaries. The mapped point must equal the raw existing formula.
3. Add a test with a large target jump. The stabilizer must reject the previous relative position and re-acquire using the raw point.
4. Run focused regressions:

```bash
cmake -S test -B test/build
cmake --build test/build -j$(nproc)
./test/build/bin/test_gold_slow_band
./test/build/bin/test_gold_follow_enabled
./test/build/bin/test_gold_guidance_weight_split
./test/build/bin/test_gold_outside_record_control
./test/build/bin/test_vehicle_gold_source_driven_control
```

## Non-Goals

- Do not change YOLO training, labels, or RKNN model files.
- Do not change pedestrian, vehicle, SIGN, UART, or DriveState priority behavior.
- Do not rely on hard-coded frame timestamps or fixed course positions.
- Do not require camera ground projection for the first implementation; the current camera config has `pitch_deg=0`, so inverse ground projection may be less stable than track-relative geometry.
