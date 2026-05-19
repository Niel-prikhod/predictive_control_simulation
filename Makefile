NAME		= predictive_control
VENV_DIR	= .venv/

PY			= python
PIP			= pip
REQ			= requirements.txt

MAIN		= main.py

install_req: activate
	$(PIP) install $(REQ)

activate: 
	source $(VENV_DIR)/bin/activate

run-pid:
	$(PY) $(MAIN) "pid"
