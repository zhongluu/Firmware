#! /usr/bin/env bash

# check ubuntu version
# instructions for 16.04, 18.04
# otherwise warn and point to docker?
UBUNTU_RELEASE=`lsb_release -rs`

if [[ "${UBUNTU_RELEASE}" == "14.04" ]]
then
	echo "Ubuntu 14.04 unsupported, see docker px4io/px4-dev-base"
	exit 1
elif [[ "${UBUNTU_RELEASE}" == "16.04" ]]
then
	echo "Ubuntu 16.04"
elif [[ "${UBUNTU_RELEASE}" == "18.04" ]]
then
	echo "Ubuntu 18.04"
	echo "WARNING, instructions only tested on Ubuntu 16.04"
fi

export DEBIAN_FRONTEND=noninteractive
sudo apt-get update --quiet
sudo apt-get -y --quiet --no-install-recommends install \
	bzip2 \
	ca-certificates \
	ccache \
	cmake \
	curl \
	default-jre-headless \
	g++ \
	gcc \
	git \
	gosu \
	lcov \
	make \
	ninja-build \
	python-gencpp \
	python-genmsg \
	python-pip \
	rsync \
	unzip \
	wget \
	xsltproc \
	zip

pip install --user -r requirements.txt
