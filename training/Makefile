PYTHON ?= python3
SCRIPT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))../scripts/manage.py
.PHONY: help setup-local setup-remote up down train-local train-remote resume-local resume-remote export-local export-remote status logs stop-remote
help:
	@echo 'setup-local setup-remote up down train-local train-remote resume-local resume-remote export-local export-remote status logs stop-remote'
	@echo 'Settings: RUN CONFIG DEVICE DATASET_REPO_ID DATASET_ROOT STEPS BATCH_SIZE MODEL CHECKPOINT DEPLOYMENT REMOTE_HOST REMOTE_USER REMOTE_PORT REMOTE_DIR'
setup-local setup-remote:
	$(PYTHON) $(SCRIPT) setup $(if $(filter setup-remote,$@),--remote,)
up:
	$(PYTHON) $(SCRIPT) sync --up
down:
	$(PYTHON) $(SCRIPT) sync --down
train-local train-remote resume-local resume-remote:
	$(PYTHON) $(SCRIPT) train $(if $(findstring remote,$@),--remote,) $(if $(findstring resume,$@),--resume,)
export-local export-remote:
	$(PYTHON) $(SCRIPT) export $(if $(filter export-remote,$@),--remote,)
status logs:
	$(PYTHON) $(SCRIPT) $@
stop-remote:
	$(PYTHON) $(SCRIPT) stop
