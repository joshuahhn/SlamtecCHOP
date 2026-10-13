# TouchDesigner Slamtec Lidar CHOP

C++ CHOP targeting **TouchDesigner 2025.33230**, macOS arm64 and Windows x64.
This fork adds the RPLIDAR C1 receive path while retaining the A-series/S-series
motor policies and serial/TCP/UDP transports. Hardware compatibility is still
pending; protocol fixtures are not hardware certification.

## Build

Requires CMake 3.22+, Git, and Xcode Command Line Tools on Mac or Visual Studio
2022 with Desktop development with C++ on Windows. CMake is the maintained build
entry point; the original `.sln`/`.vcxproj` files are historical and reference the
old source/SDK layout.

```sh
git clone --recurse-submodules https://github.com/joshuahhn/SlamtecCHOP.git
cd SlamtecCHOP
git submodule update --init --recursive
```

Mac arm64:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES=arm64 -DBUILD_TESTING=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
cmake --install build --prefix dist
```

Windows x64, from a Visual Studio developer terminal:

```powershell
cmake -S . -B build -A x64 -DBUILD_TESTING=ON
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
cmake --install build --config Release --prefix dist
```

Outputs: `dist/SlamtecCHOP.plugin` on Mac; `dist/SlamtecCHOP.dll` on Windows.
Select that file in a CPlusPlus CHOP, or install it as a custom operator using
TouchDesigner's normal plugin workflow. Keep the accompanying SDK license when
redistributing. The operator type remains `Slamtec`.

For sanitizer checks on Mac, configure a separate build with
`-DCMAKE_BUILD_TYPE=Debug -DSLAMTEC_SANITIZER=address` or
`-DSLAMTEC_SANITIZER=thread`, then build and run CTest as above. These test builds
are for diagnostics; load the Release plugin into TD.

## Use and compatibility

`Active` defaults **off**. Set the device's port and baud rate before enabling it.
For C1, select **460800**, and leave `Standartmode` off to select the device's
reported typical mode. All four baud menu entries share one bounds-checked
mapping: 115200, 256000, 460800, 1000000. Network connections accept numeric IPv4
or IPv6 addresses; hostname resolution is excluded to keep setup bounded.

Parameter names (including `Standartmode`) and channel names remain unchanged:
`angle`, `distance`, `quality`, `flag`, or Cartesian `x`, `y`, `quality`, `flag`.
Distance and Cartesian coordinates are **millimetres**, retaining quarter-mm SDK
conversion. `Precision` 1–4 gives 360–1440 bins at `1 / Precision` degrees.
The `Distance` parameter is in metres; opt-in `Quality` retains the previous
`flag >= 2` and `quality >= 150` thresholds.

Each successful complete sweep replaces the previous sweep and clears empty
bins to zero. Inactive, disconnected, obsolete-generation or stale (>500 ms)
output is -1. This prevents partial/failed reads and old sweeps being mistaken
for fresh measurements. The legacy `Random` Info CHOP channel is retained as 0.

## Receive lifecycle and source

One joinable worker owns SDK setup, read, stop, reconnect and destruction. TD
cook submits requests with `try_lock` and consumes a preallocated three-slot
latest snapshot; it does not wait for SDK I/O. Pending snapshots can be replaced,
with drop diagnostics. Connection changes cancel the previous generation and
retire it before a new session. First-scan/stall deadlines are 5 s/2 s; retries
back off from 250 ms to 4 s. OP destruction requests cancellation and joins the
worker before releasing its resources.

Application waits are finite: SDK queries 500 ms, reads/RX waits 100 ms, network
connect 500 ms and sends 100 ms. Cancellation is checked during setup and socket
connect. Native serial-driver open/ioctl and Windows cancellation completion
still depend on the OS driver; hardware unplug/shutdown tests remain necessary.
C1 reporting no motor-control capability receives no legacy DTR/default motor
commands. Legacy DTR and advertised PWM/RPM startup/shutdown remain supported.

Info CHOP/DAT expose state, sequence, `scanAgeMs`, dropped snapshots, timeouts,
errors, reconnects, capacity hits, invalid nodes, scan modes and motor policy.
`scanAgeMs` measures time since the worker accepted a complete sweep, not a
sensor timestamp or USB transport latency. An 8192-node capacity hit warns of
possible truncation.

The SDK submodule is pinned to
`99478e5fb90de3b4a6db0080acacd373f8b36869`.
[`cmake/sdk-safety.patch`](cmake/sdk-safety.patch) is applied to a **build copy**:
finite timeout forwarding; bounded/NUL-terminated metadata and protocol payloads;
capability-aware motor startup; channel-open/partial-write error propagation;
serial startup/write safety; bounded RX queue; and synchronized SDK
receive/decode/teardown state. The source submodule stays unchanged. TD headers
come from the 2025.33230 CHOP sample (API 10/common 3), with license and ABI checks
retained; trailing-whitespace corrections are cosmetic. `TDHeaders.h` scopes
the explicitly approved vendor-header warning exceptions to those includes.
Our source and tests retain `-Wall -Wextra -Wpedantic -Werror` / `/W4 /WX`.

## Validation status

Local macOS arm64 Release build, CTest, AddressSanitizer and ThreadSanitizer pass.
Tests cover concurrent latest handoff, malformed/bounded scans, generation and
output updates, blocked connect/read cancellation, retry/stall/timeout paths,
worker ownership, real SDK protocol fixtures for C1/legacy DTR/PWM/RPM,
normal-scan decoding/overflow, and TCP/UDP loopback. Bundle metadata/exports and
1000 inactive exported-ABI create/destroy cycles pass outside the TD host.

Mac arm64 and Windows x64 build/test/package jobs both passed in
[GitHub Actions run 38351988988](https://github.com/joshuahhn/SlamtecCHOP/actions/runs/38351988988)
at code revision `aaf3c52`. The Windows job used **MSVC 19.44.35229.0** and ran the
same receiver/protocol/loopback test executable. Both jobs uploaded plugin
artifacts. [The workflow](.github/workflows/build.yml) retains strict checks.
TD-host and all physical-device tests are pending. No real-C1 performance improvement or original crash root cause is
claimed. The uncorrected `f26f13b` baud table is broken and is **not a safe C1
benchmark reference**. Before a physical performance comparison, establish and
record a working reference with a corrected bounds-checked baud mapping and
identical baud/mode/settings; document any additional corrections and the exact
reference revision. An official SDK receiver can provide acquisition-only
comparison, not a TD cook baseline. Alternatively, replay identical recorded
input into both cook paths and label the result **synthetic**. No physical
performance claim is valid until a viable matching reference is established.
Live stability checks cover unplug/replug, Active toggles, OP deletion, plugin
reload and TD shutdown; record cook p95/p99, CPU, host scan age and drop counts.

Original plugin by Vasily Betin. [Support the original author](https://www.buymeacoffee.com/vasilyonl).
