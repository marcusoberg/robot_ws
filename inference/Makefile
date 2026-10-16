PYTHON ?= python3
SCRIPT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))../scripts/manage.py
.PHONY: help serve-local serve-remote up down tunnel proto
help:
	@echo 'serve-local serve-remote up down tunnel proto'
	@echo 'Settings: MODEL DEVICE OFFLINE MODEL_PORT REMOTE_HOST REMOTE_USER REMOTE_PORT REMOTE_DIR'
serve-local serve-remote:
	$(PYTHON) $(SCRIPT) serve $(if $(filter serve-remote,$@),--remote,)
up:
	$(PYTHON) $(SCRIPT) sync --up
down:
	$(PYTHON) $(SCRIPT) sync --down
tunnel:
	$(PYTHON) $(SCRIPT) tunnel
proto:
	protoc --proto_path=$(dir $(SCRIPT))../inference --python_out=$(dir $(SCRIPT))../inference/robot_inference $(dir $(SCRIPT))../inference/protocol.proto
