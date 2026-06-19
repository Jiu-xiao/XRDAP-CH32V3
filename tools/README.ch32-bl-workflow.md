# CH32 BL-First Workflow

Current board policy:
- `stage1 -> BL`
- power-on/reset defaults to BL (`1209:3072`)
- app in slot does not auto-run
- only explicit `RUN_APP` enters app
- reset after app returns to BL again

Verified safe baseline:
- summary:
  - `/home/xiao/runs/current_safe_bl_baseline_20260320T123530Z/99_summary.txt`
- current flash backup:
  - `/home/xiao/runs/backup_current_bl_baseline_20260320T122839Z/20_sha256.txt`

Tools:
- `ch32_bl_restore_verified.sh`
  - reflash the known-good `stage1 + BL` pair and verify readback
- `ch32_bl_backup_state.sh`
  - dump current `stage1/app/bl/seal` regions and sha256 them
- `ch32_bl_assert_safe.sh`
  - assert the board is still on the verified safe BL baseline before experiments
- `ch32_make_seal.py`
  - generate the standard 256-byte app seal page from an app binary
- `ch32_bl_flash_app_only.sh`
  - flash only `app + seal`, then confirm board still stops in BL
- `ch32_bl_run_app_check.sh`
  - send `RUN_APP`, wait for app USB, then reset and wait for BL again
- `ch32_bl_restore_app_and_seal.sh`
  - restore any saved `app + seal` pair
- `ch32_bl_experiment_cycle.sh`
  - backup current app state, flash candidate app, run app check, optional restore
- `ch32_bl_bulk_in_bench_cycle.sh`
  - backup current app state, flash a raw-bulk style candidate app, benchmark bulk IN, reset back to BL, optional restore
- `ch32_bl_async_in_bench_cycle.sh`
  - generic async-IN benchmark cycle for CDC/CherryUSB-style apps using `libusb_async_in_bench`

Recommended usage:
1. If the board state is unknown, recover BL first.
   - `tools/ch32_bl_restore_verified.sh`
2. Before any experiment, assert the board is still on the safe BL baseline.
   - `tools/ch32_bl_assert_safe.sh`
3. Then save the current app/seal state.
   - `tools/ch32_bl_backup_state.sh`
4. Flash only the candidate app.
   - `tools/ch32_bl_flash_app_only.sh <run_dir> <candidate_app_bin>`
5. Verify manual `RUN_APP` and reset-back-to-BL.
   - `tools/ch32_bl_run_app_check.sh`
6. Or run the whole cycle in one command.
   - `RESTORE_BACKUP=1 tools/ch32_bl_experiment_cycle.sh <run_dir> <candidate_app_bin> <app_vid> <app_pid>`
7. For raw-bulk pure IN throughput checks.
   - `RESTORE_BACKUP=1 tools/ch32_bl_bulk_in_bench_cycle.sh <run_dir> <candidate_app_bin> <app_vid> <app_pid> <ep_in>`
8. For CDC/CherryUSB async-IN checks.
   - `RESTORE_BACKUP=1 tools/ch32_bl_async_in_bench_cycle.sh <run_dir> <candidate_app_bin> <app_vid> <app_pid> <cfg_or_itf> <ep_out> <ep_in>`

Important rule:
- do not touch `stage1` or `BL` for throughput / CherryUSB / protocol experiments
- only modify the app slot unless the task is explicitly about boot flow itself
