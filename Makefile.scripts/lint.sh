#!/bin/bash -e

HERE=$(realpath $(dirname ${BASH_SOURCE[0]}))
source $HERE/__init__.sh
ROOT=$(realpath $HERE/..)

cd $ROOT

source .anaconda/bin/activate

echo_bold "==> Installing hacking" 
pip install hacking


# Not avaliable for py2.7
# pip install black mypy
# echo_bold "==> Linting with black and mypy"
# black --check .
# echo_bold "==> Linting with mypy"
# mypy --py2 --package tools --ignore-missing-imports

echo_bold "==> Linting with flake8"
flake8 .


