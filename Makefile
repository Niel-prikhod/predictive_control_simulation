NAME		= predictive_control
VENV		= .venv/

PYTHON_SYS	= python3
PIP			= $(VENV)/bin/pip
REQ			= requirements.txt
PY			= $(VENV)/bin/python

MAIN		= main.py

$(VENV)/bin/activate: $(REQ)
	$(PYTHON_SYS) -m venv $(VENV)
	$(PIP) install -r $(REQ)
	@touch $(VENV)/bin/activate

venv: $(VENV)/bin/activate

install: venv

run-pid: venv
	$(PY) $(MAIN) "pid"

run-pole: venv
	$(PY) $(MAIN) "pole"

run-gpc: venv
	$(PY) $(MAIN) "gpc"

clean:
	rm -rf $(VENV)
	find . -name "__pycache__" -exec rm -rf {} + 
	find . -name "*.pyc" -exec rm -rf {} + 
