# TASK_ANCHOR
repo: /root/codex/work/ch32v305_dap_3stage_libxr_mainline_clean_20260615
branch: n/a (source-only baseline)
module_heads: libxr -> Jiu-xiao/libxr master @ a07df8e8a41e777c6d8ddc43d5698f28773d9a51
current_goal: Create a clean CH32V305 DAP three-stage engineering tree based on the latest remote libxr master and verify it builds on Ubuntu24.
non_goals: Live board flashing, probe-rs benchmarking, SWD tuning, MBS bench app migration.
scenario: CH32V305 custom three-stage DAPLink/CMSIS-DAP baseline with app-only and full-3stage build presets.
data_source: ch32_dap_fastgpio_x18sys_baseline skeleton + remote Jiu-xiao/libxr master.
execution_plane: UK host for tree assembly, Ubuntu24 for build verification.
latest_code_change: assembled fresh DAP three-stage skeleton, embedded libxr mainline tarball, and added clean build presets/docs
expected_artifact: Fresh clean DAP tree + Ubuntu24 build summary under /home/xiao/runs/ch32v305_dap_3stage_mainline_clean_20260615T*/99_summary.txt
acceptance_condition: New tree is assembled, libxr matches current remote master, and Ubuntu24 app-dev build passes.
last_verified_artifact: none
next_action: Sync the new DAP clean tree to Ubuntu24 and verify app-dev/full-3stage builds on the current mainline libxr baseline.
blocked_on: none
updated_at: 2026-06-15T02:27:00+01:00
