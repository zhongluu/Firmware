#! /usr/bin/env bash

brew tap PX4/px4
brew install px4-dev

# Optional, but recommended additional simulation tools:
brew install px4-sim

sudo easy_install pip
# TODO: use requirements.txt
#sudo -H pip install pyserial empy toml numpy pandas jinja2

pip install --user -r requirements.txt
