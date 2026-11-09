# This file lets you format the code-base with a single command.
FILES := $(shell find . -name '*.lf' -not -path "*/fed-gen/*")

.PHONY: help
help:
	@echo "Available commands:"
	@echo "  make clean         - Clean up build and documentation"
	@echo "  make format        - Format all Lingua Franca files using lff"
	@echo "  make format-dev    - Format all Lingua Franca files using lff-dev"
	@echo "  make build-all     - Build all Lingua Franca files using lfc"
	@echo "  make build-all-dev - Build all Lingua Franca files using lfc-dev"
	
.PHONY: format
format:
	@for file in $(FILES); do \
		echo "========== Formatting $$file =========="; \
		lff $$file; \
	done

.PHONY: format-dev
format-dev:
	@for file in $(FILES); do \
		echo "========== Formatting $$file =========="; \
		lff-dev $$file; \
	done

.PHONY: build-all
build-all:
	@for file in $(FILES); do \
		echo "================================================"; \
		echo "Building $$file"; \
		echo "================================================"; \
		lfc $$file; \
	done

.PHONY: build-all-dev
build-all-dev:
	@for file in $(FILES); do \
		echo "================================================"; \
		echo "Building $$file"; \
		echo "================================================"; \
		lfc-dev $$file; \
	done

.PHONY: clean
clean:
	rm -rf *.lft *.csv *.log src-gen fed-gen include bin
	
# Set help as the default target
.DEFAULT_GOAL := help
