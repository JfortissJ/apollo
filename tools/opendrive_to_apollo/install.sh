#!/bin/bash
virtualenv -p python3.7 ./venv
source ./venv/bin/activate && pip3 install -r ./requirements.txt
