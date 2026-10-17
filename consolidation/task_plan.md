# Piper directory consolidation — 2026-10-17

Goal: keep one active checkout, lerobot_piper061 (LeRobot 0.6.1), integrating Piper functionality/configuration from lerobot_robot_piper and relevant local assets from lerobot_piper3.

- [x] Inventory code, configuration, unique assets, Git state and installed editable packages.
- [x] Integrate Piper directly into 0.6.1; retain user settings without downgrading core code.
- [x] Run smoke and relevant full tests; independent subagent review and fixes.
- [x] Consolidate old directories with recoverable archives, update paths/install references, verify final tree.

Constraints: preserve existing safety behavior, old history and uncommitted work; no new abstraction layers/hash/contracts/gates; no hardware commands. Old unrelated task records remain separate.

Errors: default uv cache read-only (use /tmp); sandbox GitHub DNS denied (approved lock refresh); initial CLI test incorrectly used a choice registry as root config (corrected to actual TeleoperateConfig); nested CAN override failed because bus motor map was required (regression test added first, fixed default factory); async E2E localhost port binding sandbox denied (approved rerun). Ruff absent, provisioning isolated tool. README patch context did not match; reread and corrected with no partial writes.

Final status: complete. Full upstream pytest remains blocked by preexisting Git LFS pointer image (91 passed, 13 skipped before first failure). No physical robot run. Root .gitmodules and old gitlink removal are staged because git rm required it; no commits or pushes created. All original unrelated Git changes preserved.
