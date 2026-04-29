# Repository Guidelines

## Project Structure & Module Organization
`Core/Inc` and `Core/Src` contain the project-owned STM32 firmware, including C startup/HAL glue and C++ control logic such as `PowerManager.cpp` and `SuperCap.cpp`. `Drivers/` and `Middlewares/` are vendor libraries; treat them as third-party code and avoid edits unless updating dependencies. `hardware/` stores KiCad design files, `Calibration/` holds calibration spreadsheets, `Docs/` contains supporting documentation and utilities, and `build/` contains generated firmware outputs and intermediate objects.

## Build, Test, and Development Commands
Use Git Bash on Windows for the shell script workflow documented in `BUILD_GUIDE.md`.

```bash
./build.sh
./build.sh 202
make -j HARDWARE_ID=101
clang-format -i Core/Src/PowerManager.cpp Core/Inc/PowerManager.hpp
```

`./build.sh` configures a temporary CMake + Ninja build and emits `.elf`, `.hex`, and `.bin` files under `build/`. Pass a hardware ID such as `202` when targeting a calibrated board variant. `make -j HARDWARE_ID=101` uses the CubeMX-generated Makefile directly. Rebuild from a clean state by deleting `build/` before compiling.

## Coding Style & Naming Conventions
Formatting is defined by `.clang-format`: 4-space indentation, Allman braces, pointer alignment on the right, and a 150-column limit. Run `clang-format` on touched C/C++ files before submitting. `.clang-tidy` enforces `CamelCase` for types and enums, `camelBack` for functions, methods, parameters, and variables, and `UPPER_CASE` for enum constants and `constexpr` values. Keep user code inside `Core/`; do not mix project logic into generated vendor sources.

## Testing Guidelines
This snapshot does not include a dedicated repository test suite. Validation is build-first: confirm the selected `HARDWARE_ID` compiles cleanly and check the produced firmware artifacts in `build/`. For control-loop changes, document bench or hardware verification in the PR, especially for ADC, CAN, HRTIM, and PID-related paths.

## Commit & Pull Request Guidelines
Git history is not available in this checkout, so no proven commit convention can be derived. Use short imperative commit subjects, for example `Tune current-loop PID limits`. Keep pull requests focused, describe the target hardware ID, list build commands run, and attach screenshots or logs for hardware-facing changes. Call out edits to generated files, linker scripts, or KiCad projects explicitly.

## RM Power Rule Context
This firmware uses a local CAN bridge for referee-side power data rather than parsing the referee UART protocol directly. In [Core/Src/Communication.cpp](D:/RM/xiang_gang_ke_ji/SuperCapacitorController-master(2)/NEWSuperCapacitorController-master/Core/Src/Communication.cpp#L98), CAN ID `0x061` carries:

- `feedbackRefereePowerLimit` -> `PowerManager::ControlData::controlData.refereePowerLimit`
- `feedbackRefereeEnergyBuffer` -> `PowerManager::ControlData::controlData.energyRemain`

This means `refereePowerLimit` is externally provided match data, typically derived upstream from robot class / level / rule state, not a value computed on this STM32 board.

### How This Repo Interprets Power Data
- `refereePowerLimit` is treated as the current chassis power ceiling received from upstream referee data.
- `energyRemain` is treated as the remaining chassis energy buffer available before over-power penalties occur.
- `updateEnergy()` in [Core/Src/PowerManager.cpp](D:/RM/xiang_gang_ke_ji/SuperCapacitorController-master(2)/NEWSuperCapacitorController-master/Core/Src/PowerManager.cpp#L868) converts those two inputs into `baseRefereePower` / `targetRefereePower`, which are then used by `updateVIP()` to decide how aggressively the supercapacitor should charge or discharge.
- When `energyRemain` is high, the code allows a more aggressive target and can float above `refereePowerLimit` within internal clamps.
- When `energyRemain` drops, the code reduces `baseRefereePower`; once energy is low enough it forces a conservative target of `10 W`.

### Official RM Rule Notes
- Official RoboMaster material describes the supercapacitor as a power buffer that stores energy when chassis demand is low and releases energy during bursts.
- Official referee protocol documents expose `chassis_power_limit` and `buffer_energy` in the realtime power packet (`0x0202`, 10 Hz) and also expose newer robot energy / chassis energy related feedback in `0x0204`.
- Official 2025 and 2026 engine/rule materials indicate that current seasons have moved toward broader "bottom/chassis energy" mechanics and, in RMUC, wireless charging interactions.

### Practical Interpretation For This Codebase
- The control logic in this repository is best understood as a buffer-energy-era strategy: stay under the current power ceiling when energy is low, and spend stored energy to permit short bursts when energy is healthy.
- Do not assume this firmware exactly matches the latest official season logic. It is closer to the classic "power limit + buffer energy + supercapacitor burst" model than to the full 2025/2026 chassis-energy system.

### Primary Reference Links
- RoboMaster 2025 rules/resources hub announcement: https://www.robomaster.com/en-US/resource/pages/announcement/1768
- RoboMaster 2025 event engine release notes mentioning bottom/chassis energy and wireless charging updates: https://www.robomaster.com/zh-CN/products/components/detail/6145
- RoboMaster official roundtable article describing supercapacitors as a power buffer unit: https://www.robomaster.com/zh-CN/resource/pages/activities/1004
