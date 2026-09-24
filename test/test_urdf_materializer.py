from pathlib import Path
import xml.etree.ElementTree as ET

from dvrk_pybullet.urdf_materializer import (
    default_generated_root,
    materialize_virtual_robot,
    materialize_virtual_psm,
)


def test_generated_root_is_cache_directory():
    import os
    cache_root = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    assert default_generated_root() == cache_root / "dvrk_pybullet"


def test_virtual_psm1_is_expanded_and_cached(tmp_path):
    first = materialize_virtual_psm(generated_root=tmp_path)
    second = materialize_virtual_psm(generated_root=tmp_path)

    assert first == second
    assert first.urdf_path.is_file()
    assert first.metadata_path.is_file()
    text = first.urdf_path.read_text(encoding="utf-8")
    assert '<robot name="PSM1">' in text
    assert 'joint name="yaw"' in text
    assert "package://dvrk_model/" not in text
    assert 'filename="/' in text


def test_virtual_ecm_is_expanded_and_cached(tmp_path):
    result = materialize_virtual_robot(
        "ECM", endoscope="Si_straight", generated_root=tmp_path
    )
    text = result.urdf_path.read_text(encoding="utf-8")
    assert result.instrument is None
    assert result.endoscope == "Si_straight"
    assert '<robot name="ECM">' in text
    assert 'joint name="insertion"' in text
    assert 'link name="ECM_tip_link"' in text
    assert "package://dvrk_model/" not in text


def test_virtual_large_needle_driver_has_primitive_contact_shapes(tmp_path):
    for instrument in ("400006", "420006"):
        result = materialize_virtual_psm(instrument=instrument, generated_root=tmp_path)
        robot = ET.parse(result.urdf_path).getroot()
        links = {link.attrib["name"]: link for link in robot.findall("link")}
        for suffix in (
            "roll_link",
            "wrist_pitch_link",
            "wrist_yaw_link",
            "jaw_1_link",
            "jaw_2_link",
        ):
            assert links[f"PSM1_{suffix}"].find("collision") is not None
        for suffix in ("jaw_link", "tool_tip_link"):
            assert links[f"PSM1_{suffix}"].find("collision") is None
        cylinder = links["PSM1_roll_link"].find("collision/geometry/cylinder")
        assert cylinder is not None
        assert cylinder.attrib["length"] == "0.5596247"
