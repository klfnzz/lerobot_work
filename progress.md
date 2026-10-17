# Piper task relabel progress

2026-10-09: Inspected dataset read-only; confirmed user boundary at episode 29 inclusive. No dataset edits yet.

Completed: updated five files (task table, info, stats, episode metadata, frame parquet). First task 29 episodes/15990 frames; second task 21 episodes/15857 frames. Boundary smoke checks 0/28/29/49 passed, followed by exhaustive 50-episode verification and non-task frame column equality. Original 50 frame row groups retained. Script: /tmp/piper-task-relabel/relabel.py. Independent review prompted exception rollback and unique backup directory; no helpers introduced.

Backup: /home/qwe/lerobot_exp/data/muti_pick_and_place_task_single_arm_lerobot_30hz_task_backup_20261009_230609_123578
No robot execution, training, video edits or Hub upload performed.
