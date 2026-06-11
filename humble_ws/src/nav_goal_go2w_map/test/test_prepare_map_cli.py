from nav_goal_go2w_map.provenance import metadata_source


def test_metadata_source_is_relative_for_retained_raw_cloud(tmp_path):
    output = tmp_path / "office"
    raw = output / "raw" / "dlio_map.pcd"
    raw.parent.mkdir(parents=True)
    raw.touch()

    assert metadata_source(raw, output) == "raw/dlio_map.pcd"


def test_metadata_source_is_absolute_for_external_cloud(tmp_path):
    raw = tmp_path / "raw.pcd"
    raw.touch()

    assert metadata_source(raw, tmp_path / "office") == str(raw.resolve())
