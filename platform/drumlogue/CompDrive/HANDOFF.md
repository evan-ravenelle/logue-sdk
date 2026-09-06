# CompDrive + logue-bench — handoff

**Design doc for the four proposed units:**
https://claude.ai/code/artifact/02abcf16-9069-47cc-b863-f82780009a71

Read that first for *what to build next*. This file is *where things are and how to
build them*.

---

## State

Branch `claude/compdrive-dsp-fixes-and-host-bench`, pushed to `origin`
(github.com/evan-ravenelle/logue-sdk), working tree clean.

    04fe31a  Move the bench out of the CompDrive unit project
    3effc48  Add host-side bench for logue-sdk units
    4d3515c  Fix inverted compression, grainy noise gate and dead RMS meter

`main` is a **protected ref** on the fork — direct pushes are rejected, so work lands
via PR from a branch. PR not opened yet:
https://github.com/evan-ravenelle/logue-sdk/pull/new/claude/compdrive-dsp-fixes-and-host-bench

**CompDrive is hardware-validated.** Compressor, noise gate, drive and sidechain all
confirmed good on the device. It beats the stock drumlogue compressor.

---

## Read these to get oriented

Paths are relative to the repo root (`/Users/evan/logue-sdk`).

### The unit (start here)

| File | Why |
|---|---|
| `platform/drumlogue/CompDrive/masterfx.h` | All the DSP. Compressor gain law, gate, drive, RMS meter. ~700 lines, the only file that really matters. |
| `platform/drumlogue/CompDrive/header.c` | Parameter descriptors. **Read the `frac` column** — see gotchas. |
| `platform/drumlogue/CompDrive/rms.h` | Windowed RMS + envelope. Feeds both the meter and the gate detector. |
| `platform/drumlogue/CompDrive/envelope.h` | Peak envelope follower. Coefficients are passed in, precomputed. |
| `platform/drumlogue/CompDrive/unit.cc` | SDK entry points, plus host-only diagnostics behind `CD_HOST_DIAG`. |

### The bench

Read `cdhost.h` first — it's the contract everything else meets.

| File | Why |
|---|---|
| `tools/logue-bench/host/cdhost.h` | The normalized ABI. One `cdh_vtable` per platform. |
| `tools/logue-bench/host/engine.cc` | Audio thread, transport, WAV I/O, meters, 8 unit slots. Platform-agnostic. |
| `tools/logue-bench/host/plat_drumlogue.c` | drumlogue shim — dlopen + its own SDK headers. |
| `tools/logue-bench/host/plat_nts3.c` | nts-3 shim — SDRAM hooks, genericfx context, touch. |
| `tools/logue-bench/host/unit.mk` | Builds any unit project into a host dylib. |
| `tools/logue-bench/host/compat/arm_math.h` | Portable CMSIS shim. See gotchas. |
| `tools/logue-bench/CompDriveBench/Interop.cs` | P/Invoke surface + parameter display formatting. |
| `tools/logue-bench/CompDriveBench/UnitTabView.cs` | Per-unit tab; builds controls from the unit's own header. |

### SDK reference

| File | Why |
|---|---|
| `platform/drumlogue/common/runtime.h` | `unit_header_t`, `unit_param_t`, `unit_runtime_desc_t`, sample-bank callbacks. |
| `platform/drumlogue/README.md` | Parameter types, bitmap format, module APIs. |
| `platform/nts-3_kaoss/common/unit_genericfx.h` | genericfx header, touch, `get_raw_input`. |
| `platform/nts-3_kaoss/common/runtime.h` | nts-3 structs — **not** binary compatible with drumlogue's. |
| `platform/nts-3_kaoss/common/dsp/` | `biquad.hpp`, `delayline.hpp` (fractional reads), `simplelfo.hpp`. |

---

## Build

**Device unit** — needs Docker (OrbStack); start it first if it isn't running.

    cd /Users/evan/logue-sdk
    docker run --rm -v "$(pwd)/platform:/workspace" -h logue-sdk -i \
      xiashj/logue-sdk:latest /app/cmd_entry build -f --drumlogue drumlogue/CompDrive

Output: `platform/drumlogue/CompDrive/CompDrive.drmlgunit` (~9956 bytes).
Current size: **7764 text / 356 data / 38792 bss**.
Copy *only that file* to `Units/MasterFXs/` on the device.

**Bench**

    cd tools/logue-bench/host && make && make tools
    make -f unit.mk UNIT_DIR=../../../platform/drumlogue/CompDrive PLATFORM=drumlogue
    make -f unit.mk UNIT_DIR=<any project> PLATFORM=nts-3_kaoss

    ./build/selftest 0 build/units/CompDrive_drumlogue.dylib   # 11 checks
    ./build/probe    0 build/units/CompDrive_drumlogue.dylib   # dump params

**UI** — `cd tools/logue-bench/CompDriveBench && dotnet build` (the csproj runs
`make -C ../host` first). **Do not run it** — Evan launches it himself to verify.

---

## Gotchas that will cost you time

- **`frac` scaling.** Parameters declared `frac=1, frac_mode=0` in `header.c` carry one
  fractional bit — the raw integer is **twice** the displayed value. Applies to THold,
  Knee, Makeup, PreGain, NThold. `c_fixedPoint1BitScale` in `masterfx.h` handles it.
  Every threshold was double its label before this was found.

- **Platforms are not binary compatible.** drumlogue and nts-3 disagree on the width of
  `unit_header.target`, parameter count (24 vs 8), name lengths, struct packing, and the
  shape of `unit_runtime_desc_t`. Never share a struct between them — add a shim
  compiled against that platform's own headers.

- **Mach-O rejects the SDK's `section(".unit_header")`.** Host builds compile with
  `-DATTRIBUTES_H_=1` and force-include `harness_attrs.h`. Device builds are untouched.

- **CMSIS on the host.** nts-3 headers reach `arm_math.h` via
  `unit_genericfx.h → osc_api.h → fixed_math.h → cortexm.h`. `compat/arm_math.h`
  implements the 13 reachable intrinsics, including GE-flag emulation for the SDK's
  parallel-subtract-then-`SEL` min/max idiom.

- **Avalonia is pinned to 11.3.x deliberately.** NodeEditorAvalonia and the
  Xaml.Behaviors family have no Avalonia 12 build; mixing them throws `TypeLoadException`
  at runtime. **Do not upgrade Avalonia.**

- **New exports from `unit.cc` need `extern "C"`.** The real entry points get C linkage
  from `unit.h`; anything you add doesn't, and `dlsym` won't find the mangled name.

- **`make clean` in `tools/logue-bench/host` wipes `build/units/` too.** Rebuild units
  after cleaning the host.

- **Docker silently does nothing when OrbStack is stopped.** A "device build" that
  prints no size line did not run. Check the artifact timestamp.

- **The device unpacks units on install** and takes more storage than the file. Deleting
  a unit needs a full USB-storage → init cycle to actually free the space.

---

## Open items

1. **Did the RMS meter animate?** `header.c` param 4 was changed from `{0,0,...}` to
   `{0,15,...}` and `getParameterValue` now returns the live level, on the theory that
   the runtime repaints a parameter when its *value* changes. Untested on hardware.
   If it doesn't animate, revert that descriptor and reclaim the slot.

2. **~82% memory cut available.** `rms.h` holds `float32x4_t circularBuffer[2400]` —
   38,400 bytes, 83% of the unit's 46 KB footprint, just to feed a meter and the gate.
   Replacing the rectangular window with a one-pole exponential RMS
   (`ms += (x*x - ms) * coeff`) removes the buffer entirely: ~46 KB → ~8.5 KB. Better
   DSP too (no window edge artifacts, no 38 KB cache thrash per sample).

3. **Sample-bank emulation in the bench.** `plat_drumlogue.c` stubs
   `get_num_sample_banks` / `get_num_samples_for_bank` / `get_sample` to return zero.
   Wiring them to serve a folder of WAVs unblocks developing the granular synth
   entirely off-device. ~1 hour.

4. **`reference/compdrive_curve.cc` doesn't build.** It targets the old single-unit
   `cd_*` ABI, not slot-addressed `cdh_*`. It's the code that verified the corrected
   compressor curve; kept for reference, not in any build.

5. **PR not opened.**

---

## Working preferences

- **Never run the bench app.** Build it and report; Evan runs it himself to verify.
  Headless tools that print to the terminal (`selftest`, `probe`, the Docker build) are
  fine.
- Measure, don't assert. Every DSP claim in the commit history was verified through the
  real unit code on the bench before being stated.
