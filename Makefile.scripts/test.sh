#!/bin/bash

HERE=$(realpath $(dirname ${BASH_SOURCE[0]}))
source $HERE/__init__.sh
ROOT=$(realpath $HERE/..)

cd $ROOT

source .anaconda/bin/activate

echo_bold "==> Installing pytest"
pip install pytest

echo_bold "==> Testing with pytest"
pytest -v tests
