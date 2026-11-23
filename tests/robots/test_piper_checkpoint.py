"""Compatibility of existing Pika/Piper checkpoint metadata with remote inference."""

import json

import pytest

from examples.piper.prepare_checkpoint import PIKA_ACTION_NAMES, PIPER_ACTION_NAMES, prepare_checkpoint


@pytest.fixture
def checkpoint(tmp_path):
    source = tmp_path / "original"
    source.mkdir()
    config = {
        "type": "pi05",
        "input_features": {"observation.state": {"shape": [7]}},
        "output_features": {"action": {"shape": [7]}},
        "action_feature_names": [PIKA_ACTION_NAMES],
        "use_relative_actions": False,
    }
    (source / "config.json").write_text(json.dumps(config))
    (source / "model.safetensors").write_bytes(b"original weights")
    (source / "policy_preprocessor.json").write_text('{"steps": []}')
    return source


def test_migration_preserves_original_files_and_links_weights(checkpoint, tmp_path):
    original = (checkpoint / "config.json").read_bytes()
    destination = prepare_checkpoint(checkpoint, tmp_path / "prepared")

    assert json.loads((destination / "config.json").read_text())["action_feature_names"] == PIPER_ACTION_NAMES
    assert (checkpoint / "config.json").read_bytes() == original
    assert (destination / "model.safetensors").is_symlink()
    assert (destination / "model.safetensors").read_bytes() == b"original weights"
    assert prepare_checkpoint(checkpoint, destination) == destination


@pytest.mark.parametrize("change", [{"action_feature_names": ["unknown"]}, {"use_relative_actions": True}])
def test_migration_rejects_unverified_action_mapping(checkpoint, tmp_path, change):
    config = json.loads((checkpoint / "config.json").read_text())
    config.update(change)
    (checkpoint / "config.json").write_text(json.dumps(config))

    with pytest.raises(ValueError):
        prepare_checkpoint(checkpoint, tmp_path / "prepared")
    assert not (tmp_path / "prepared").exists()


def test_migration_never_overwrites_an_existing_directory(checkpoint, tmp_path):
    destination = tmp_path / "existing"
    destination.mkdir()
    (destination / "config.json").write_text("user config")

    with pytest.raises(FileExistsError):
        prepare_checkpoint(checkpoint, destination)
    assert (destination / "config.json").read_text() == "user config"
