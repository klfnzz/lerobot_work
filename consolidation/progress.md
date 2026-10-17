# Consolidation progress

2026-10-17: Read planning-with-files and prune-abstraction skills, ran session catchup (Codex parsing unsupported), read existing task records and inspected Git state. No application source changed yet.

- Migrated 12 hardware-free plugin tests first; confirmed failure for missing native Piper import, then integrated bus/config/follower/leader in native packages. All 12 pass.
- Retained 061 factory fallback, added Piper to CLI imports/async supported list, and added optional piper SDK extra with existing revision. SDK is guarded so generic CLI help works without hardware dependency.
- First uv invocation failed because default cache is read-only; using writable /tmp cache. uv binary is at /home/qwe/.local/bin/uv. Refreshing existing lockfile.
- Independent inventory completed; source review requested. Two prune reviews: existing polling helper is a timed hardware protocol boundary; feature properties/framework lifecycle methods remain required. Config split matches native package layout, not new runtime abstraction.

- 18 Piper smoke/regression tests now pass. A CAN override test exposed required nested motors mapping; fixed it with one dataclass default factory and removed duplicate default map construction.
- Wheel builds; optional dependency lock validates offline. Existing dependency versions are unchanged; only pyagxarm added. pip check passes.
- Approved existing environment update: removed standalone Piper plugin, refreshed 061 editable package, removed incompatible 0.4.4 editable reference from legacy Python 3.10 environment.
- Six actual CLI --help subprocesses pass and advertise Piper: teleoperate, record, rollout, calibrate, replay, async client.
- Archived and byte-compared old source trees (1342 and 29 files), including Git/uncommitted work/logs. Moved outputs intact: 105 files, 10,000,509,778 bytes; all inodes and sizes unchanged.
- Broader tests initially 94 pass / 1 sandbox port-binding failure. Rerunning with localhost socket permission. Full upstream suite also started.

## Completion

- Piper + async + config suite: 95 passed with localhost sockets allowed (18 Piper + 47 async + 30 configs). After review added 2 unsupported-CLI regression tests; both failed before removing unintended registrations. Final Piper + config rerun: 50 passed (20 Piper + 30 configs).
- Covered edges: default/custom CAN and cameras, independent config maps, missing optional SDK, fixed seven-axis ordering, one image prefix, failed enable/calibration/camera cleanup, safe-pose settling, native factory construction, unsupported CLI discovery.
- Full upstream run initially hit a sandbox cache write restriction; reran with writable dataset cache/required execution permission. Result: 91 passed, 13 skipped, first failure OpenCV test reading tests/artifacts/cameras/image_128x128.png, confirmed to be an existing Git LFS pointer. Preexisting gRPC closed-stream logging noise appears after teardown.
- Ruff check passes for all touched Python files; new native modules/tests pass format check. git diff --check passes. Existing lock validates offline; pip check clean; wheel build succeeded.
- All JSON files in moved models parse; 12 safetensors files open and list tensors (including Pi0.5 weights/normalizers). Three ACT configs load with native 061 and retain two expected camera features and seven-dimensional actions. No weight tensors or real robot inference executed.
- Root Git cleanup initially required staging .gitmodules. Staged only that file and old gitlink removal, removed only old submodule local config; did not commit/push. Independent 061 Git history retained.
- Old working source directories moved to temporary recovery directory after durable archive validation. Only 061 remains under workspace lerobot*; old installed package references absent. 105 output files retain original inodes/sizes.
- Independent final review approved, no blockers. Final prune checks: no new wrapper/carrier chain; removed repeated motor-map default copies. Retained enable-state polling as hardware protocol boundary and required framework properties/lifecycle methods. Two structure/readability passes found no further useful inlining.
- Suggested hardware acceptance after this software-only consolidation: supervised low-speed teleoperation with both cameras and each actual CAN channel, then safe-stop check. Hardware was not accessed during this task.

## Git publication follow-up — 2026-10-17

User requested committing and pushing current changes to klfnzz/lerobot_piper_06, branch piper-v0.6.1. No application source changed in this follow-up. Fresh validation: 20 Piper tests and 30 config tests passed; git diff --check passed. Removed inherited ROS PYTHONPATH for pytest after unrelated ROS plugin import failed (missing lark).

A new independent read-only review identified two unresolved shutdown risks in motors/piper/piper.py: disable_gripper exceptions can bypass joint disable/transport cleanup, and joint-disable timeout can be discarded by follower/leader callers. These are not fixed in this snapshot. Suggested regression cases: inject gripper-disable exception and assert joint disable/transport cleanup still occur; simulate joint-disable timeout and assert failure reaches the caller. No hardware executed. Earlier consolidation approval above predates this review.
