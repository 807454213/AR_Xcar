# TC264 Menu Typed Parameter Access

## Problem

`Menu.c` routes every non-float menu parameter through `int *`. Several menu
entries actually point to `uint8` or `int16` objects, so menu display reads past
the object and leaving the tuning screen writes four bytes back through the
wrong pointer type.

## Design

Keep the current menu layout, controls, step sizes, and Flash behavior. Add a
small C99-compatible typed integer access interface with three supported kinds:
`uint8`, `int16`, and `int32`. The interface reads into an `int32_t` editing
value and writes back only through the matching pointer type.

Expose one menu callback for each integer kind. Bind every existing menu entry
to the callback matching the declaration in `Global.h`. Update menu-list value
rendering to use the same typed access interface. Float parameters remain on
the existing float path.

The access interface will be header-only so the target build does not require
another generated makefile source entry. Invalid type tags return zero on read
and perform no write; normal menu callbacks only pass valid tags.

## Verification

Host tests place each narrow value between sentinel bytes, then verify reads
return the expected value and writes leave both sentinels unchanged. Existing
TC264 reliability tests must continue to pass. A source audit must confirm that
all integer menu entries use a callback matching the parameter declaration and
that `Menu.c` no longer casts narrow parameters to `int *`.

The Orange Pi cannot perform the final AURIX/TASKING target build because that
toolchain is not installed, so hardware-project compilation remains a separate
verification step.
