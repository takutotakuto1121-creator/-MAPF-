.DEFAULT_GOAL := help
PYTHON ?= python3
ALGO   ?= cbs
AGENTS ?= 8
SEED   ?= 0
MAP    ?= maps/room.txt

.PHONY: help install test cov lint fmt check demo svg bench clean

help: ## show this help
	@grep -E '^[a-zA-Z_-]+:.*## ' $(MAKEFILE_LIST) | awk -F':.*## ' '{printf "  \033[36m%-10s\033[0m %s\n", $$1, $$2}'
	@echo "\nvariables: ALGO=cbs|pp|pibt  AGENTS=N  SEED=N  MAP=path"

install: ## install the package in editable mode with dev tools
	$(PYTHON) -m pip install -e '.[dev]'

test: ## run the test suite
	$(PYTHON) -m pytest -q

cov: ## run tests with a coverage report
	$(PYTHON) -m pytest -q --cov=mapf --cov-report=term-missing

lint: ## static checks (ruff)
	$(PYTHON) -m ruff check .
	$(PYTHON) -m ruff format --check .

fmt: ## auto-format and fix lint issues
	$(PYTHON) -m ruff check --fix .
	$(PYTHON) -m ruff format .

check: lint test ## lint + test (what CI runs)

demo: ## solve a random instance and print it step by step
	$(PYTHON) -m mapf --random $(MAP) -n $(AGENTS) --seed $(SEED) -a $(ALGO) --ascii

svg: ## write an animated SVG of a random instance to out.svg
	$(PYTHON) -m mapf --random $(MAP) -n $(AGENTS) --seed $(SEED) -a $(ALGO) --svg out.svg

bench: ## compare cbs / pp / pibt on random instances
	$(PYTHON) -m mapf.bench --map $(MAP)

clean: ## remove build and cache files
	rm -rf build dist *.egg-info src/*.egg-info .pytest_cache .ruff_cache .coverage out.svg
	find . -name __pycache__ -type d -prune -exec rm -rf {} +
