# Road Straight/Curve Classifier Design

## Goal

Improve the current ROAD straight/curve separation using the supplied sample frames.
The classifier must keep fork entry/exit detection independent from straight/curve
classification.

## Acceptance Samples

Curve samples:

- `/home/orangepi/xcar_shm_test/shm_20260709_145556_393.png`
- `/home/orangepi/xcar_shm_test/shm_20260709_145544_009.png`
- `/home/orangepi/xcar_shm_test/shm_20260709_145537_610.png`
- `/home/orangepi/xcar_shm_test/shm_20260709_145527_972.png`
- `/home/orangepi/xcar_shm_test/shm_20260709_145512_591.png`
- `/home/orangepi/xcar_shm_test/shm_20260709_145457_769.png`
- `/home/orangepi/xcar_shm_test/shm_20260709_145447_485.png`
- `/home/orangepi/xcar_shm_test/shm_20260709_145434_702.png`
- `/home/orangepi/xcar_shm_test/shm_20260709_145418_620.png`
- `/home/orangepi/xcar_shm_test/shm_20260709_145411_005.png`
- `/home/orangepi/xcar_shm_test/shm_20260709_145346_842.png`
- `/home/orangepi/xcar_shm_test/shm_20260709_145329_669.png`

Straight samples:

- `/home/orangepi/xcar_shm_test/shm_20260710_161217_796.png`
- `/home/orangepi/xcar_shm_test/shm_20260710_161226_556.png`
- `/home/orangepi/xcar_shm_test/shm_20260710_161230_775.png`
- `/home/orangepi/xcar_shm_test/shm_20260710_161238_163.png`
- `/home/orangepi/xcar_shm_test/shm_20260710_161241_440.png`
- `/home/orangepi/xcar_shm_test/shm_20260710_161246_129.png`
- `/home/orangepi/xcar_shm_test/shm_20260710_161249_520.png`
- `/home/orangepi/xcar_shm_test/shm_20260710_161325_159.png`
- `/home/orangepi/xcar_shm_test/shm_20260710_161329_074.png`
- `/home/orangepi/xcar_shm_test/shm_20260710_161333_792.png`
- `/home/orangepi/xcar_shm_test/shm_20260710_161355_660.png`
- `/home/orangepi/xcar_shm_test/shm_20260714_185302_179.png`
- `/home/orangepi/xcar_shm_test/shm_20260714_185305_331.png`
- `/home/orangepi/xcar_shm_test/shm_20260714_185308_199.png`
- `/home/orangepi/xcar_shm_test/shm_20260714_185312_762.png`
- `/home/orangepi/xcar_shm_test/shm_20260714_185316_310.png`

## Expected Behavior

Curve sample frames should classify as `LeftCurve` or `RightCurve` when they are
not in a fork phase. `ForkEntry` and `ForkExit` are not accepted for these curve
samples unless separate fork evidence is genuinely present.

Straight sample frames pass when they classify as `Straight`, `ForkEntry`, or
`ForkExit`. Fork phases are allowed because `FORK_IN` and `FORK_OUT` occur on
the straight road and are judged by their own logic.

Serial output for straight/curve separation must treat `ForkEntry` and
`ForkExit` as straight-road motion for this purpose. Only `LeftCurve` and
`RightCurve` may trigger curve-specific serial behavior.

## Approach

Add a regression test that runs the supplied samples through `processFrame()`.
The test should fail before the classifier changes and then pass after the fix.
It must report per-frame road mode and midline features when a sample fails.

Keep fork phase detection ahead of straight/curve classification. Do not merge
`ForkEntry` or `ForkExit` into the curve detector.

Improve the non-fork classifier in `src/perception/imgprocess.cpp` by adding a
single-path curvature signal beyond global midline variance. The preferred
signal is based on midline displacement across far/mid/near bands, so early
curves with moderate variance can still classify as curves while straight
segments with fork artifacts can remain straight or fork.

Keep configuration changes narrow. Existing `roadCurveVarMin`,
`roadStraightVarMax`, `roadDirDeltaThresh`, and score thresholds may be tuned if
the sample feature distribution supports it. Add new parameters only if the
existing ones cannot express the needed separation.

## Testing

Run the new sample regression test first and confirm it fails for the current
implementation.

After implementation, run:

- The new straight/curve sample test.
- Existing imgprocess/fork tests that cover fork entry and fork exit.
- A focused build of the full test target if time allows.

## Non-Goals

Do not rewrite fork entry/exit repair or OCR/sign decision logic.
Do not change AI detection, UART protocol framing, or TC264 command semantics.
