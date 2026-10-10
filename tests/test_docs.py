"""The course material is complete and consistent.

Every session has its five parts, every relative link resolves, the generated cost tables
are current, every slide deck and figure it references exists, and the repository stays
honest (no claims of having been taught) and private (no employer names).
"""

from __future__ import annotations

import importlib.util
import re
from pathlib import Path

import pytest

from conftest import ROOT

SESSIONS = sorted(p for p in (ROOT / "sessions").iterdir() if p.is_dir())
MD_FILES = [
    p
    for p in ROOT.rglob("*.md")
    if ".venv" not in p.parts and ".pytest_cache" not in p.parts and "node_modules" not in p.parts
]
LINK = re.compile(r"\[[^\]]*\]\(([^)\s]+)\)")


def test_seven_sessions():
    assert [s.name[:2] for s in SESSIONS] == [f"{i:02d}" for i in range(1, 8)]


@pytest.mark.parametrize("session", SESSIONS, ids=[s.name for s in SESSIONS])
def test_session_has_goals_notes_lab_quiz_and_slides(session):
    for name in ("README.md", "notes.md", "lab.md", "quiz.md"):
        assert (session / name).exists(), f"{session.name}/{name} missing"
    readme = (session / "README.md").read_text(encoding="utf-8")
    assert "After this session you can" in readme or session.name.startswith("07")
    deck = ROOT / "slides" / f"{session.name}.md"
    assert deck.exists(), f"slides for {session.name} missing"
    assert (ROOT / "slides" / "pdf" / f"{session.name}.pdf").exists()


def test_every_quiz_has_an_answer_section():
    keys = (ROOT / "instructor" / "quiz-keys.md").read_text(encoding="utf-8")
    for i in range(1, 8):
        assert f"## Quiz {i}" in keys


def _slugs(md: Path) -> set[str]:
    """GitHub-style anchors of the headings in a Markdown file."""
    out = set()
    text = re.sub(r"```.*?```", "", md.read_text(encoding="utf-8"), flags=re.S)
    for h in re.findall(r"^#{1,6}\s+(.+?)\s*$", text, flags=re.M):
        slug = re.sub(r"[^\w\- ]", "", h.strip().lower()).replace(" ", "-")
        out.add(slug)
    return out


@pytest.mark.parametrize("md", MD_FILES, ids=[str(p.relative_to(ROOT)) for p in MD_FILES])
def test_relative_links_resolve(md):
    text = md.read_text(encoding="utf-8")
    text = re.sub(r"```.*?```", "", text, flags=re.S)  # ignore code blocks
    for target in LINK.findall(text):
        if re.match(r"^[a-z]+:", target):
            continue
        path, _, anchor = target.partition("#")
        dest = md if not path else md.parent / path
        assert dest.exists(), f"{md.relative_to(ROOT)}: broken link {target}"
        if anchor and dest.suffix == ".md":
            assert anchor in _slugs(dest), f"{md.relative_to(ROOT)}: no heading for #{anchor}"


def test_slide_images_exist():
    for deck in (ROOT / "slides").glob("*.md"):
        for img in re.findall(r"!\[[^\]]*\]\(([^)]+)\)", deck.read_text(encoding="utf-8")):
            assert (deck.parent / img).exists(), f"{deck.name}: missing image {img}"


def test_kit_cost_tables_are_current():
    spec = importlib.util.spec_from_file_location("kit_cost", ROOT / "tools" / "kit_cost.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert mod.main(["--check"]) == 0


def test_readme_and_portfolio_are_honest_about_status():
    for rel in ("README.md", "docs/portfolio.md", "cohort/pilot-plan.md"):
        text = (ROOT / rel).read_text(encoding="utf-8").lower()
        assert "not been taught" in text or "not yet taught" in text, rel
    readme = (ROOT / "README.md").read_text(encoding="utf-8").lower()
    for claim in ("testimonial", "students loved", "graduates of"):
        assert claim not in readme


def test_no_employer_or_private_names_in_the_repository():
    banned = ["ore" + "yeon"]
    for p in ROOT.rglob("*"):
        if not p.is_file() or {".venv", ".git", ".pytest_cache", ".ruff_cache"} & set(p.parts):
            continue
        if p.suffix.lower() in {".png", ".pdf", ".stl", ".pyc"}:
            continue
        text = p.read_text(encoding="utf-8", errors="ignore").lower()
        for word in banned:
            assert word not in text, f"{p.relative_to(ROOT)} mentions a banned name"


def test_both_licences_present():
    assert "MIT License" in (ROOT / "LICENSE").read_text(encoding="utf-8")
    assert "Attribution 4.0 International" in (ROOT / "LICENSE-CONTENT").read_text(encoding="utf-8")


def test_figures_used_by_the_readme_exist():
    text = (ROOT / "README.md").read_text(encoding="utf-8")
    for img in re.findall(r"!\[[^\]]*\]\(([^)]+)\)", text):
        assert (ROOT / img).exists(), img


def test_example_data_is_labelled_synthetic():
    head = (ROOT / "data" / "example_capstone_rig.csv").read_text(encoding="utf-8")[:300]
    assert "SYNTHETIC" in head
    assert Path(ROOT / "data" / "thrust_template.csv").exists()
