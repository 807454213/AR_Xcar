# Encoder Raw Dynamic Error Y Design

## Goal

Add an optional mode that changes the non-gold control sampling row from the raw encoder tick delta received from TC264. Faster raw encoder movement samples farther ahead in the image, meaning a smaller y coordinate. Gold keeps its existing target-driven dynamic sampling row.

## Inputs

The control signal is the recent raw encoder delta, not converted speed. The runtime value is the average of the latest left and right wheel tick deltas when both sides have been refreshed:

```text
encoder_raw = (abs(left_delta) + abs(right_delta)) / 2
```

If no fresh pair has been received recently, the feature falls back to the existing fixed row rules.

## Configuration

New `tc` keys:

```json
"encoderRawDynamicErrorYEnabled": false,
"encoderRawDynamicErrorYMin": 115,
"encoderRawDynamicErrorYMax": 150,
"encoderRawDynamicErrorRawMin": 0,
"encoderRawDynamicErrorRawMax": 80,
"encoderRawDynamicErrorStaleFrames": 10
```

Mapping:

```text
raw <= RawMin -> y = YMax
raw >= RawMax -> y = YMin
between      -> linear interpolation
```

The names use `YMin/YMax` in image-coordinate terms: `YMin` is farther ahead, `YMax` is closer to the car.

## Control Behavior

When `encoderRawDynamicErrorYEnabled=true`, all states that currently choose a fixed base sampling row use this encoder-derived row instead:

- Normal
- StableSpeed
- AvoidPed
- AvoidCar
- LeavingCar
- FastBack
- sign/OCR/fork fixed-row phases

Gold remains special. If gold follow is active, the existing gold candidate y selection still controls `dynamic_error_y`. The encoder-derived row only acts as the gold base row when the gold logic falls back to the base row.

ReturnTrack remains unchanged because Pipeline overrides the sent error with `LostTrackSteer::fallbackError()`.

## HUD

The debug HUD displays the recent raw encoder value and the row chosen by the encoder dynamic mode. This gives trackside feedback for tuning the raw thresholds without converting units.

## Testing

Tests cover:

- config load/save/defaults for the new keys;
- raw encoder pair tracking and stale fallback;
- enabled mode maps low raw ticks to the lower-speed/nearer row and high raw ticks to the farther row;
- gold follow still keeps target-driven dynamic row behavior.
