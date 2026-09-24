"""Expand and cache PyBullet-ready dVRK Virtual robot URDF files."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import xacro

from .errors import PyBulletBackendError
from dvrk_simulator_base.model_source import locate_dvrk_model


SUPPORTED_PSMS = ("PSM1", "PSM2", "PSM3")
SUPPORTED_ROBOTS = (*SUPPORTED_PSMS, "ECM")
MATERIALIZER_VERSION = 1


@dataclass(frozen=True)
class MaterializedUrdf:
    model: str
    instrument: str | None
    endoscope: str | None
    source_path: Path
    model_root: Path
    urdf_path: Path
    metadata_path: Path
    content_hash: str


def default_generated_root(anchor: str | Path | None = None) -> Path:
    """Return the user cache directory for PyBullet artifacts."""
    cache_root = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    return (cache_root / "dvrk_pybullet").resolve()



def _expand_virtual_robot(
    source: Path,
    parent_link: str,
    instrument: str | None,
    endoscope: str | None,
) -> str:
    mappings = {"parent_link_": parent_link, "show_rcm": "false"}
    if instrument is not None:
        mappings.update({"instrument": instrument, "is_virtual": "true"})
    if endoscope is not None:
        mappings["endoscope"] = endoscope
    try:
        document = xacro.process_file(
            str(source),
            mappings=mappings,
        )
    except Exception as error:
        raise PyBulletBackendError(f"failed to expand {source}: {error}") from error
    return document.toxml()


def _resolved_urdf(urdf_text: str, model_root: Path) -> bytes:
    try:
        robot = ET.fromstring(urdf_text)
    except ET.ParseError as error:
        raise PyBulletBackendError(f"expanded dvrk_model URDF is invalid XML: {error}") from error

    prefix = "package://dvrk_model/"
    for element in robot.iter():
        filename = element.attrib.get("filename")
        if not filename or not filename.startswith(prefix):
            continue
        relative = filename[len(prefix):]
        relative_path = Path(relative)
        if relative_path.is_absolute() or ".." in relative_path.parts:
            raise PyBulletBackendError(f"invalid dvrk_model resource path: {filename}")
        resolved = (model_root / relative).resolve()
        if not resolved.is_file():
            raise PyBulletBackendError(f"dvrk_model resource does not exist: {resolved}")
        element.set("filename", str(resolved))

    return ET.tostring(robot, encoding="utf-8", xml_declaration=True)


def _atomic_write(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def materialize_virtual_robot(
    model: str,
    instrument: str | None = None,
    endoscope: str | None = None,
    parent_link: str = "world",
    generated_root: str | Path | None = None,
) -> MaterializedUrdf:
    """Expand a Virtual PSM or ECM and cache its resolved URDF."""
    model = str(model).upper()
    if model not in SUPPORTED_ROBOTS:
        raise ValueError(f"model must be one of {SUPPORTED_ROBOTS}, got {model!r}")
    if model in SUPPORTED_PSMS:
        instrument = str(instrument or "420006")
        endoscope = None
    else:
        instrument = None
        endoscope = str(endoscope or "Si_straight")

    model_root = locate_dvrk_model()
    source = model_root / "urdf" / "Virtual" / f"{model}.urdf.xacro"
    if not source.is_file():
        raise PyBulletBackendError(f"Virtual PSM Xacro does not exist: {source}")

    expanded = _expand_virtual_robot(source, parent_link, instrument, endoscope)
    resolved = _resolved_urdf(expanded, model_root)
    identity = {
        "materializer_version": MATERIALIZER_VERSION,
        "model": model,
        "instrument": instrument,
        "endoscope": endoscope,
        "parent_link": parent_link,
        "resolved_urdf_sha256": hashlib.sha256(resolved).hexdigest(),
    }
    digest = hashlib.sha256(
        json.dumps(identity, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    output_root = Path(generated_root or default_generated_root()).expanduser().resolve()
    asset_dir = output_root / digest
    urdf_path = asset_dir / "model.urdf"
    metadata_path = asset_dir / "metadata.json"

    metadata = {
        **identity,
        "content_hash": digest,
        "model_root": str(model_root),
        "source_path": str(source),
        "urdf_path": str(urdf_path),
    }
    encoded_metadata = (
        json.dumps(metadata, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")
    if not urdf_path.is_file() or urdf_path.read_bytes() != resolved:
        _atomic_write(urdf_path, resolved)
    if not metadata_path.is_file() or metadata_path.read_bytes() != encoded_metadata:
        _atomic_write(metadata_path, encoded_metadata)

    return MaterializedUrdf(
        model=model,
        instrument=instrument,
        endoscope=endoscope,
        source_path=source,
        model_root=model_root,
        urdf_path=urdf_path,
        metadata_path=metadata_path,
        content_hash=digest,
    )


def materialize_virtual_psm(
    model: str = "PSM1",
    instrument: str = "420006",
    parent_link: str = "world",
    generated_root: str | Path | None = None,
) -> MaterializedUrdf:
    """Compatibility wrapper for the original PSM-only entry point."""
    return materialize_virtual_robot(
        model,
        instrument=instrument,
        parent_link=parent_link,
        generated_root=generated_root,
    )
