# Drone-control bootcamp -- common tasks (Linux/macOS). See docs/setup.md.

PY ?= .venv/bin/python
PIP ?= .venv/bin/pip
# rotorfault: git URL by default; `make setup ROTORFAULT=../rotor-fault-recovery` for a checkout
ROTORFAULT ?= rotorfault @ git+https://github.com/Kronbii/rotor-fault-recovery
# easyPID sources for the sketches; empty = the library installed with arduino-cli
EASYPID ?=

.PHONY: setup install-rotorfault lint format test test-fast check notebooks figures sketches slides data

setup:
	python3 -m venv .venv
	$(PIP) install --upgrade pip
	$(PIP) install -e ".[dev,notebooks,serial]"
	$(MAKE) install-rotorfault

install-rotorfault:
	@if [ -d "$(ROTORFAULT)" ]; then $(PIP) install -e "$(ROTORFAULT)"; else $(PIP) install "$(ROTORFAULT)"; fi

lint:
	$(PY) -m ruff check .
	$(PY) -m ruff format --check .

format:
	$(PY) -m ruff format .
	$(PY) -m ruff check --fix .

test-fast:
	$(PY) -m pytest -m "not notebooks"

test:
	$(PY) -m pytest

check: lint test
	$(PY) tools/build_notebooks.py --check
	$(PY) tools/sync_sketch_headers.py --check
	$(PY) tools/kit_cost.py --check

notebooks:
	$(PY) tools/build_notebooks.py

figures:
	$(PY) tools/make_figures.py
	$(PY) tools/hero_figure.py

sketches:
	$(PY) tools/compile_sketches.py $(if $(EASYPID),--library $(EASYPID),)

slides:
	npx -y @marp-team/marp-cli@4 --theme-set slides/theme/vneo.css --pdf --allow-local-files \
	  --input-dir slides --output slides/pdf

data:
	$(PY) tools/make_example_data.py
	$(PY) tools/kit_cost.py --write
