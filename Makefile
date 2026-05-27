NAME		= predictive_control
VENV		= .venv/

PYTHON_SYS	= python3
PIP			= $(VENV)/bin/pip
REQ			= requirements.txt
PY			= $(VENV)/bin/python

MAIN		= main.py
ARGS		?= 

$(VENV)/bin/activate: $(REQ)
	$(PYTHON_SYS) -m venv $(VENV)
	$(PIP) install -r $(REQ)
	@touch $(VENV)/bin/activate

venv: $(VENV)/bin/activate

install: venv

run-pid: venv
	$(PY) $(MAIN) "pid" $(ARGS)

run-pole: venv
	$(PY) $(MAIN) "pole" $(ARGS)

run-gpc: venv
	$(PY) $(MAIN) "gpc" $(ARGS)

run-gpc-con: venv
	$(PY) $(MAIN) "gpc-constrained" $(ARGS)

run-open: venv
	$(PY) $(MAIN) "null" $(ARGS)

clean:
	rm -rf $(VENV)
	find . -name "__pycache__" -exec rm -rf {} + 
	find . -name "*.pyc" -exec rm -rf {} + 
