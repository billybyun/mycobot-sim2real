.PHONY: help venv install-dev fmt lint test

help:
	@echo "Targets: venv install-dev fmt lint test"

venv:
	python3 -m venv .venv && . .venv/bin/activate && pip install -U pip

install-dev:
	. .venv/bin/activate && pip install -e ".[dev]"

fmt:
	. .venv/bin/activate && black .

lint:
	. .venv/bin/activate && ruff check .

test:
	. .venv/bin/activate && pytest -q
