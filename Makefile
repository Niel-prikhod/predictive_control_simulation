NAME		= predictive_control
VENV_DIR	= .venv/

PY			= python
PIP			= pip
REQ			= requirements.txt

MAIN		= main.py

install_req: activate
	$(PIP) install $(REQ)

run-pid:
	$(PY) $(MAIN) "pid"

run-pole:
	$(PY) $(MAIN) "pole"
