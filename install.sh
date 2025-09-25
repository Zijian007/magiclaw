#!/bin/bash
# -*- coding: utf-8 -*-

set -e
set -x

### Create conda environment
if conda env list | grep -q "^magiclaw"; then
    echo "Conda environment 'magiclaw' already exists."
else
    echo "Creating conda environment 'magiclaw' with Python 3.10..."
    conda create -y -n magiclaw python=3.10
fi

### Set default environment
echo "Configuring 'magiclaw' as default environment..."
conda config --set auto_activate_base false
conda init bash
if ! grep -q "conda activate magiclaw" ~/.bashrc; then
    echo "conda activate magiclaw" >> ~/.bashrc
fi
source ~/.bashrc

### Install magiclaw
echo "Activating environment and installing project..."
conda activate magiclaw
cd ~/Documents/magiclaw
pip install -e .

### Get path of run-magiclaw
SCRIPT_PATH=$(which run-magiclaw)
echo "run-magiclaw is installed at: $SCRIPT_PATH"
