# Gold Guidance Weight Split Design

## Goal

Give track gold and non-track gold independent guidance-weight reference values. Treat boundary-band gold as non-track gold, and remove the old single runtime parameter after migrating existing configurations.

This change affects only guidance weighting. Gold y mapping, reachability, zone geometry, state transitions, and error-row selection remain unchanged.

## Zone-to-Parameter Mapping

Use the mapped gold point's existing `GoldZone` classification:

| Gold zone | Weight parameter |
|---|---|
| `GoldZone::Track` | `goldTrackGuidanceWeightRef` |
| `GoldZone::Band` | `goldOutsideGuidanceWeightRef` |
| `GoldZone::Outside` | `goldOutsideGuidanceWeightRef` |
| `GoldZone::Unknown` | `goldOutsideGuidanceWeightRef` |

`Band` represents gold near the track boundary and therefore belongs to the outside group. `Unknown` uses the outside value as the conservative fallback.

The existing weighting formula is unchanged:

```cpp
w = min(1, abs(gold_x - track_mid_x) / max(1, weight_ref));
weighted_x = track_mid_x + (gold_x - track_mid_x) * w;
```

A larger reference value keeps the guidance point closer to the track centerline.

## Configuration Schema and Migration

Replace the single runtime field:

```json
"goldGuidanceWeightRef": 128
```

with:

```json
"goldTrackGuidanceWeightRef": 128,
"goldOutsideGuidanceWeightRef": 128
```

Both compiled defaults are `128`. Remove `goldGuidanceWeightRef` from `TrackControlParams`, configuration-save output, active configuration, and tracked configuration presets.

For backward compatibility, configuration loading may still read `goldGuidanceWeightRef` into a local migration value. When an old file contains only that key, assign its value to both new fields. If either new key is also present, that new key overrides the migrated value for its own zone. The old key is never stored as runtime state and is never written back.

Migrate each tracked file under `configs/` that contains the old key. Preserve its existing numerical behavior by copying its old value into both new keys; preserve every unrelated setting in those files.

## Runtime Selection

Before weighting a live gold point, classify its mapped point with the same `goldZone` function already used by reachability and debug rendering, then select the matching reference value.

For a locked gold point reused when the current frame has no eligible gold detections, classify the stored mapped point and apply the same selection. This keeps live and held guidance behavior consistent.

Keep `tcGoldWeightedGuidancePointFromFoot()` parameterized by the selected integer reference value; it remains responsible only for the weighting math, not zone classification.

## Tests

Add or update tests that prove:

1. Both compiled defaults are `128`.
2. A legacy-only `goldGuidanceWeightRef` value populates both new fields.
3. New track/outside keys independently override the legacy migration value.
4. Saving configuration writes both new keys and omits the old key.
5. Track gold uses the track reference value.
6. Boundary-band and outside gold use the outside reference value.
7. Locked-point fallback uses the reference value selected from its stored zone.
8. Existing gold reachability and source-driven control regressions still pass.

## Documentation

Update `Xcar2.md` to describe the two parameters, their zone grouping, and the rule that larger values keep guidance closer to the centerline. Remove documentation of the old single parameter.
