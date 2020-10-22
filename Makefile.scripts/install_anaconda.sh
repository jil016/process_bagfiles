#!/bin/bash -e

HERE=$(realpath $(dirname ${BASH_SOURCE[0]}))
source $HERE/__init__.sh
ROOT=$(realpath $HERE/..)

INSTALL_DIR=$ROOT

if [ -e $INSTALL_DIR/.anaconda ]; then
  echo_bold "==> Anaconda is already installed: $INSTALL_DIR/.anaconda"
  exit 0
fi

echo_bold "==> Installing Anaconda to: $INSTALL_DIR/.anaconda"

TMPDIR=$(mktemp -d)
cd $TMPDIR

if [ "$(uname)" = "Linux" ]; then
  URL='https://repo.continuum.io/miniconda/Miniconda2-latest-Linux-x86_64.sh'
elif [ "$(uname)" = "Darwin" ]; then
  URL='https://repo.continuum.io/miniconda/Miniconda2-latest-MacOSX-x86_64.sh'
else
  echo_warning "==> Unsupported platform: $(uname)"
  exit 1
fi

if which wget &>/dev/null; then
  wget -q $URL -O miniconda2.sh
else
  curl -s -L $URL -o miniconda2.sh
fi

bash ./miniconda2.sh -p $INSTALL_DIR/.anaconda -b
cd -
rm -rf $TMPDIR

source $INSTALL_DIR/.anaconda/bin/activate
conda update -n base -y conda -q
conda deactivate
