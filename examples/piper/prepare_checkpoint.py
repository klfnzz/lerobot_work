"""Create a canonical Piper view of an existing absolute-action checkpoint."""

import argparse
import json
from pathlib import Path

PIPER_ACTION_NAMES = [*(f"joint_{index}.pos" for index in range(1, 7)), "gripper.pos"]
PIKA_ACTION_NAMES = [
    *(f"arm.jointStatePosition.master.joint{index}" for index in range(6)),
    "gripper.encoderDistance.pikaSensor",
]


def prepare_checkpoint(source: Path, destination: Path) -> Path:
    """Copy compatible config metadata and link the original weights and processors.

    Only the known seven-coordinate Pika/Piper naming convention is translated.
    Joint order, units, weights and normalization statistics stay authoritative.
    Relative-action checkpoints need their own processor migration.
    """
    source = source.resolve(strict=True)
    destination = destination.absolute()
    if destination.resolve().is_relative_to(source):
        raise ValueError("Destination must be outside the original checkpoint directory.")
    original = (source / "config.json").read_text()
    config = json.loads(original)
    if config.get("type") not in {"act", "pi0", "pi05"}:
        raise ValueError("Expected an ACT, Pi0 or Pi0.5 Piper checkpoint.")
    if config.get("output_features", {}).get("action", {}).get("shape") != [7]:
        raise ValueError("Expected seven Piper action coordinates.")
    if config.get("input_features", {}).get("observation.state", {}).get("shape") != [7]:
        raise ValueError("Expected seven Piper state coordinates.")

    names = config.get("action_feature_names")
    if isinstance(names, list) and len(names) == 1 and isinstance(names[0], list):
        names = names[0]
    if names is not None:
        if names not in (PIKA_ACTION_NAMES, PIPER_ACTION_NAMES):
            raise ValueError("Unknown action order; export the checkpoint with verified Piper metadata.")
        if names == PIKA_ACTION_NAMES:
            if config.get("use_relative_actions"):
                raise ValueError("Relative-action checkpoints require a separate processor migration.")
            for filename in ("policy_preprocessor.json", "policy_postprocessor.json"):
                processor_path = source / filename
                if processor_path.exists():
                    processor = json.loads(processor_path.read_text())
                    if any(
                        step.get("registry_name")
                        in {"relative_actions_processor", "absolute_actions_processor"}
                        and step.get("config", {}).get("enabled")
                        for step in processor.get("steps", [])
                    ):
                        raise ValueError("Enabled relative-action processors need a separate migration.")
            names = PIPER_ACTION_NAMES
        config["action_feature_names"] = names

    migrated = json.dumps(config, indent=4, ensure_ascii=False) + "\n"
    if destination.exists():
        # Repeated preparation must never overwrite a checkpoint or user edits.
        if (
            (destination / ".piper-checkpoint-source").is_file()
            and (destination / ".piper-checkpoint-source").read_text() == str(source) + "\n"
            and (destination / "config.json").read_text() == migrated
        ):
            return destination
        raise FileExistsError(f"Destination already exists: {destination}")
    destination.mkdir(parents=True)
    (destination / "config.json").write_text(migrated)
    (destination / ".original-config.json").write_text(original)
    for path in source.iterdir():
        if path.name != "config.json" and not path.name.startswith("."):
            (destination / path.name).symlink_to(path, target_is_directory=path.is_dir())
    (destination / ".piper-checkpoint-source").write_text(str(source) + "\n")
    return destination


def main() -> None:
    """Prepare a local checkpoint without loading weights or accessing hardware."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source", type=Path, required=True, help="Original local pretrained_model directory"
    )
    parser.add_argument(
        "--destination", type=Path, required=True, help="New directory linking the original files"
    )
    args = parser.parse_args()
    print(prepare_checkpoint(args.source, args.destination))


if __name__ == "__main__":
    main()
