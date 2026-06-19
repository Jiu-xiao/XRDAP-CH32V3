# CH32V305 DAP 3-Stage Mainline Clean Engineering Baseline

Purpose:

- keep a clean source-only engineering tree ready for future CH32V305 DAP / CMSIS-DAP work
- preserve the current three-stage / custom-bootloader layout assumptions
- avoid touching bootloader images during ordinary app-side development
- preserve app-side detach / jump-to-bootloader ability as a non-optional capability

Source baseline:

- three-stage DAP project skeleton copied from `/root/codex/work/ch32_dap_fastgpio_x18sys_baseline`
- skeleton copied without `build/`, embedded `libxr/.git`, temporary probe logs, benchmark text dumps, or backup files
- embedded `libxr/` replaced with latest remote `Jiu-xiao/libxr` `master`
- requested remote head:
  - `a07df8e8a41e777c6d8ddc43d5698f28773d9a51`
- intended as a fresh mainline-based DAP working copy for later edits

Key layout facts retained from the source tree:

- app flash base defaults to `0x00002000`
- app flash length defaults to `0x0001E000`
- stage-1 linker script: [Link.stage1.ld](/root/codex/work/ch32v305_dap_3stage_libxr_mainline_clean_20260615/Link.stage1.ld)
- stage-3 bootloader sources:
  - [User/boot_main.c](/root/codex/work/ch32v305_dap_3stage_libxr_mainline_clean_20260615/User/boot_main.c)
  - [User/bootloader_main.cpp](/root/codex/work/ch32v305_dap_3stage_libxr_mainline_clean_20260615/User/bootloader_main.cpp)
  - [User/stage1_jump.S](/root/codex/work/ch32v305_dap_3stage_libxr_mainline_clean_20260615/User/stage1_jump.S)
- app entry:
  - [User/app_main.cpp](/root/codex/work/ch32v305_dap_3stage_libxr_mainline_clean_20260615/User/app_main.cpp)

App detach requirement:

- app-side detach ability is already present in this baseline and must be preserved
- current implementation path in [User/app_main.cpp](/root/codex/work/ch32v305_dap_3stage_libxr_mainline_clean_20260615/User/app_main.cpp):
  - `AppBootloaderPowerManager::JumpToBootloader()`
    - writes the SRAM boot-request mailbox at:
      - `CH32_CUSTOM_BL_REQ_ADDR0`
      - `CH32_CUSTOM_BL_REQ_ADDR1`
    - then calls `Reset()`
  - `JumpToBootloaderThunk(void*)`
  - `LibXR::USB::DfuRuntimeClass dfu_runtime(...)`
  - main loop:
    - `dfu_runtime.Process()`

Recommended workflow for now:

- use the `app-dev` preset for routine DAP-side work
- do not rebuild or flash stage1 / stage3 bootloader unless that is the explicit task
- keep `CH32_APP_USE_CUSTOM_BOOTLOADER=ON` so app-side code still targets the three-stage environment correctly

Presets:

- `app-dev`
  - app-only development
  - does not build stage1 or stage3 targets
- `full-3stage`
  - full layout rebuild including stage1 trampoline and stage3 bootloader

Examples:

```bash
cmake --preset app-dev
cmake --build --preset app-dev
```

```bash
cmake --preset full-3stage
cmake --build --preset full-3stage
```

Artifact path note:

- this tree sets `EXECUTABLE_OUTPUT_PATH=${sourceDir}/build`
- so even when you configure with `build-app-dev` or `build-3stage`, the final:
  - `CH32V305FBP6.elf`
  - `CH32V305FBP6.bin`
  - `CH32V305FBP6.hex`
  are still emitted under `build/`

Live-board safety:

- do not direct-flash an app image to `0x08000000` on a live three-stage board
- default to stage-aware flows only
- treat loss of attach / debug-enable as a regression, not an acceptable shortcut
