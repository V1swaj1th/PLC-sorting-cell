"""Runs the check scripts in this folder under pytest, so colcon test reports them."""
import pathlib
import runpy

import pytest

HERE = pathlib.Path(__file__).parent


@pytest.mark.parametrize('script', ['check_keymap'])
def test_check_script(script, monkeypatch):
    monkeypatch.setattr('sys.argv', [script])
    with pytest.raises(SystemExit) as result:
        runpy.run_path(str(HERE / f'{script}.py'), run_name='__main__')
    assert result.value.code == 0, f'{script}.py reported failures (see its output above)'
