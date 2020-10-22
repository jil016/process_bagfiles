#!/bin/bash -e

HERE=$(realpath $(dirname ${BASH_SOURCE[0]}))
source $HERE/__init__.sh
ROOT=$(realpath $HERE/..)

source $ROOT/.anaconda/bin/activate

echo_bold "==> Installing requirements with requirements-dev.txt"
pip install -r requirements-dev.txt

echo_bold "==> Installing pre-commit"
pre-commit install

echo_bold "==> Installing the package"
pip install -e .

echo_bold "\nGreat! You can start using this!

  $ source .anaconda/bin/activate
"
