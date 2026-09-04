# Position New Map Swap Design

## Goal

Replace the Position module default SDF map with the files in `Position/slam_workspace/new_map` so `robot_core` localizes against the new field map.

## Current State

The Position executable loads `sdf_map.bin` from its current working directory. The normal launcher changes into `Position/slam_workspace/slam_all/build` before running `robot_core`, so the build directory copy is the runtime-critical SDF file.

The current code hard-codes the old map size and origin:

- `MAP_WIDTH = 173`
- `MAP_HEIGHT = 215`
- `MAP_ORIGIN_X = -2.79`
- `MAP_ORIGIN_Y = -6.0`

## New Map Inputs

Use the new map under `Position/slam_workspace/new_map`:

- `mapfn.pgm`: `162 x 156`
- `sdf_map(2).bin`: `101088` bytes, equal to `162 * 156 * sizeof(float)`
- `mapfn.yaml`: `resolution: 0.05`, `origin: [-1.58, -6.05, 0]`

## Design

Make the new map the default map for the existing runtime path.

Update the compile-time map constants in the localization code:

- `MAP_WIDTH = 162`
- `MAP_HEIGHT = 156`
- `MAP_ORIGIN_X = -1.58`
- `MAP_ORIGIN_Y = -6.05`
- Keep `MAP_RESOLUTION = 0.05`

Copy `new_map/sdf_map(2).bin` to both default `sdf_map.bin` locations:

- `Position/slam_workspace/sdf_map.bin`
- `Position/slam_workspace/slam_all/build/sdf_map.bin`

The top-level copy keeps source-tree/manual runs consistent. The build-directory copy is required by `run_all.sh`.

## Validation

Build `robot_core` from `Position/slam_workspace/slam_all/build` with `cmake --build .`. Confirm the SDF file sizes are `101088` bytes in both default locations.
