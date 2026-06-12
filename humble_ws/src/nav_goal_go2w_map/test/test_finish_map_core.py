from pathlib import Path

import pytest

from nav_goal_go2w_map import finish_map_core

def _stub_prepare(monkeypatch):
    def prepare(argv):
        output = Path(argv[argv.index("--output") + 1])
        for name in ("map.pcd", "viz.pcd", "grid.yaml"):
            (output / name).write_text(name)
        return 0
    import sys
    import types
    monkeypatch.setitem(
        sys.modules, "nav_goal_go2w_map.prepare_map_cli", types.SimpleNamespace(main=prepare)
    )


def test_finish_map_publishes_atomically(tmp_path, monkeypatch):
    _stub_prepare(monkeypatch)
    output = tmp_path / "office"
    statuses = []

    def save(raw_dir, leaf):
        assert leaf == 0.05
        assert not output.exists()
        (raw_dir / "dlio_map.pcd").write_text("pcd")

    result = finish_map_core.finish_map(output, 0.05, save, statuses.append)
    assert result == output
    assert (output / "raw" / "dlio_map.pcd").read_text() == "pcd"
    assert statuses == ["SAVING", "CONVERTING", f"DONE {output}"]
    assert not list(tmp_path.glob(".office.prepare-map.*"))


def test_finish_map_refuses_existing_output(tmp_path):
    output = tmp_path / "office"
    output.mkdir()
    statuses = []
    with pytest.raises(FileExistsError):
        finish_map_core.finish_map(output, 0.05, lambda *_: None, statuses.append)
    assert statuses[-1].startswith("FAILED output already exists")


def test_finish_map_removes_empty_staging_on_failed_save(tmp_path):
    output = tmp_path / "office"
    statuses = []
    with pytest.raises(RuntimeError):
        finish_map_core.finish_map(output, 0.05, lambda *_: None, statuses.append)
    assert not list(tmp_path.glob(".office.prepare-map.*"))
    assert statuses[0] == "SAVING"
    assert statuses[-1].startswith("FAILED ")


def test_finish_map_retains_saved_map_on_failed_conversion(tmp_path, monkeypatch):
    import sys
    import types

    monkeypatch.setitem(
        sys.modules,
        "nav_goal_go2w_map.prepare_map_cli",
        types.SimpleNamespace(main=lambda argv: 1),
    )
    output = tmp_path / "office"
    statuses = []

    def save(raw_dir, leaf):
        (raw_dir / "dlio_map.pcd").write_text("pcd")

    with pytest.raises(RuntimeError):
        finish_map_core.finish_map(output, 0.05, save, statuses.append)
    staging = list(tmp_path.glob(".office.prepare-map.*"))
    assert len(staging) == 1
    assert (staging[0] / "office" / "raw" / "dlio_map.pcd").read_text() == "pcd"
    assert statuses[-1].startswith("FAILED ")
    assert f"partial map retained in {staging[0]}" in statuses[-1]


def test_finish_map_requires_absolute_output():
    with pytest.raises(ValueError):
        finish_map_core.finish_map("relative", 0.05, lambda *_: None)
