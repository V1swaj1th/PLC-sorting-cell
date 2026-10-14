# Files to review before deletion

Moved here during the audit instead of being deleted. `COLCON_IGNORE` keeps colcon from building
anything in this folder.

| Folder | What it is | Why it should go |
|---|---|---|
| `vision_pycache/` | Python bytecode (`*.pyc`) committed by accident | Generated on every run; already in `.gitignore` |
| `openplc_editor_build/` | C++ and ST files the OpenPLC Editor generates when it compiles the ladder | Build output, regenerated on every build. The source of truth is `plc/sorting_cell/pous/programs/main.ld` |
| `sorting_cell_bridge/` | An empty ROS package created from the `ros2 pkg create` template | Never used: the Modbus bridge lives in `sorting_cell_controller/src/plc_bridge.cpp` |

Delete this folder once you have checked it: `git rm -r _to_review`.
| `vision_template_lint_tests/` | `test_copyright.py`, `test_flake8.py`, `test_pep257.py` from the `ros2 pkg create` template | They only ran in the vision package, the copyright one was skipped, and the code was never checked against ament's flake8 rules. Linting now runs on every Python package in CI (`.github/workflows/ci.yml`) with one explicit configuration |
