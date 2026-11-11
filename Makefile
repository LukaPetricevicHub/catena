# Catena build entry points. `make help` lists them.

SHELL := /bin/sh
VENV := .venv
PY := $(VENV)/bin/python
BUILD := build
STAMP := $(VENV)/.installed

# First Python 3.11+ on PATH. Override with `make setup PYTHON=/path/to/python`.
PYTHON ?= $(shell for p in python3.13 python3.12 python3.11 python3; do \
	if command -v $$p >/dev/null 2>&1 && \
	   $$p -c 'import sys; sys.exit(sys.version_info < (3, 11))' 2>/dev/null; \
	then echo $$p; break; fi; done)

S ?= wander
SEED ?= 1

.PHONY: help setup core test test-full lint sim cad fw clips

help:
	@echo "make setup        create .venv, install Python deps, build core"
	@echo "make core         build core as a shared library plus unit tests (CMake)"
	@echo "make test         C++ unit tests, then fast pytest scenarios"
	@echo "make test-full    all scenarios across the full seed sets"
	@echo "make lint         ruff check on the Python code"
	@echo "make sim S=wander run a scenario with the viewer (SEED=n, ARGS='--speed 4')"
	@echo "make cad          build all parts, run checks, export STEP/STL/3MF (from M5)"
	@echo "make fw           build the firmware with PlatformIO, no upload (from M7)"
	@echo "make clips        render README clips from scenarios (from M4)"

setup: core

$(STAMP): pyproject.toml
	@test -n "$(PYTHON)" || { \
	  echo "error: Python 3.11+ not found on PATH (macOS: brew install python@3.12)"; exit 1; }
	$(PYTHON) -m venv $(VENV)
	$(PY) -m pip install --quiet --upgrade pip
	$(PY) -m pip install --quiet -e ".[dev]"
	@touch $(STAMP)

core: $(STAMP)
	cmake -S . -B $(BUILD) -DCMAKE_BUILD_TYPE=Release -DCATENA_PYTHON=$(CURDIR)/$(PY)
	cmake --build $(BUILD) --parallel 4

test: core
	ctest --test-dir $(BUILD) --output-on-failure
	$(PY) -m pytest -q

test-full: core
	ctest --test-dir $(BUILD) --output-on-failure
	CATENA_FULL=1 $(PY) -m pytest -q

lint: $(STAMP)
	$(PY) -m ruff check sim tests

sim: core
	$(PY) -m sim.runner $(S) --seed $(SEED) --viewer $(ARGS)

cad:
	@echo "make cad: CAD arrives in milestone M5"; exit 1

fw:
	@echo "make fw: the firmware project arrives in milestone M7"; exit 1

clips:
	@echo "make clips: clips arrive in milestone M4"; exit 1
