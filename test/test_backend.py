import builtins

from dvrk_pybullet.backend import load_pybullet
from dvrk_pybullet.errors import PyBulletDependencyError
import dvrk_pybullet.preview as preview
import pytest


def test_missing_pybullet_has_actionable_error(monkeypatch):
    real_import = builtins.__import__

    def reject_pybullet(name, *args, **kwargs):
        if name == "pybullet":
            raise ImportError("test-controlled missing dependency")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", reject_pybullet)
    with pytest.raises(PyBulletDependencyError) as captured:
        load_pybullet()
    message = str(captured.value)
    assert "bootstrap_venv.sh" in message
    assert "source .venv-pybullet/bin/activate" in message
    assert "colcon build --symlink-install" in message


def test_preview_prints_dependency_recovery_without_traceback(monkeypatch, capsys):
    def missing():
        raise PyBulletDependencyError("recovery instructions")

    monkeypatch.setattr(preview, "load_pybullet", missing)
    assert preview.main([]) == 1
    assert capsys.readouterr().err == "error: recovery instructions\n"
