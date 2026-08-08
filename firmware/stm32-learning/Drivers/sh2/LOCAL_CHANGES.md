# Local changes to the vendored sh2 driver

Keep this list short and re-apply it deliberately when re-vendoring.

## `sh2.c`: give `getProdIdOp` a timeout

Upstream declares it with no `timeout_us`, which `opProcess()` reads as "wait
forever".  A sensor that never answers therefore hangs the caller instead of
returning an error, and on this robot that stopped the firmware at boot before
the console came up -- taking the servo bus with it, since nothing else runs.

Changed to `.timeout_us = 2000000`.  Two seconds is far longer than a healthy
part needs and still leaves the console usable when the part is silent.
