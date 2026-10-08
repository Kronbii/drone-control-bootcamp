"""Every notebook runs top to bottom.

* the committed notebooks are up to date with ``notebooks/src`` (the build is checked);
* student notebooks contain no solution code;
* solution notebooks execute with no error at all, so every auto-check passes;
* student notebooks execute with errors *only* in the auto-check cells, and every
  auto-check cell fails (each check exercises an unfinished TODO).
"""

from __future__ import annotations

import importlib.util
import os
import sys

import pytest

from conftest import ROOT

spec = importlib.util.spec_from_file_location(
    "build_notebooks", ROOT / "tools" / "build_notebooks.py"
)
build_notebooks = importlib.util.module_from_spec(spec)
sys.modules["build_notebooks"] = build_notebooks
spec.loader.exec_module(build_notebooks)

SOURCES = [t[0] for t in build_notebooks.targets()]
IDS = [s.stem for s in SOURCES]


def test_there_is_one_notebook_per_session():
    sessions = sorted(p.name[:2] for p in (ROOT / "sessions").iterdir() if p.is_dir())
    assert [s.stem[1:3] for s in SOURCES] == sessions


def test_committed_notebooks_are_current():
    assert build_notebooks.main(["--check"]) == 0


@pytest.mark.parametrize("src", SOURCES, ids=IDS)
def test_student_version_has_no_solutions(src):
    nb = build_notebooks.build(src, student=True)
    sol = build_notebooks.build(src, student=False)
    text = "".join("".join(c["source"]) for c in nb["cells"])
    assert "BEGIN SOLUTION" not in text and "END SOLUTION" not in text
    n_ex = sum(1 for c in nb["cells"] if "exercise" in c["metadata"].get("tags", []))
    n_check = sum(1 for c in nb["cells"] if "check" in c["metadata"].get("tags", []))
    assert n_ex >= 4, "each session has at least four auto-checked exercises"
    assert n_check >= n_ex
    assert "NotImplementedError" not in "".join(
        "".join(c["source"]) for c in sol["cells"] if c["cell_type"] == "code"
    )


def _execute(nb: dict, allow_errors: bool):
    nbformat = pytest.importorskip("nbformat")
    nbclient = pytest.importorskip("nbclient")
    node = nbformat.from_dict(nb)
    env_backup = os.environ.get("MPLBACKEND")
    os.environ["MPLBACKEND"] = "Agg"
    try:
        client = nbclient.NotebookClient(
            node,
            timeout=900,
            kernel_name="python3",
            allow_errors=allow_errors,
            resources={"metadata": {"path": str(ROOT / "notebooks")}},
        )
        client.execute()
    finally:
        if env_backup is None:
            os.environ.pop("MPLBACKEND", None)
        else:
            os.environ["MPLBACKEND"] = env_backup
    return node


def _errors(node):
    out = []
    for i, c in enumerate(node.cells):
        if c.cell_type != "code":
            continue
        for o in c.get("outputs", []):
            if o.get("output_type") == "error":
                out.append((i, c, o))
    return out


@pytest.mark.notebooks
@pytest.mark.parametrize("src", SOURCES, ids=IDS)
def test_solution_notebook_passes_every_check(src):
    node = _execute(build_notebooks.build(src, student=False), allow_errors=False)
    passes = sum("[PASS]" in o.get("text", "") for c in node.cells for o in c.get("outputs", []))
    n_check = sum(1 for c in node.cells if "check" in c.metadata.get("tags", []))
    assert passes >= n_check


@pytest.mark.notebooks
@pytest.mark.parametrize("src", SOURCES, ids=IDS)
def test_student_notebook_fails_only_at_the_todos(src):
    node = _execute(build_notebooks.build(src, student=True), allow_errors=True)
    errs = _errors(node)
    failing = {i for i, _, _ in errs}
    checks = {i for i, c in enumerate(node.cells) if "check" in c.metadata.get("tags", [])}
    unexpected = [(i, o["ename"], o["evalue"][:200]) for i, c, o in errs if i not in checks]
    assert not unexpected, f"cells other than auto-checks failed: {unexpected}"
    assert failing == checks, f"auto-check cells that did not fail: {sorted(checks - failing)}"
    for _, _, o in errs:
        assert o["ename"] == "CheckFailed", o["evalue"][:200]
