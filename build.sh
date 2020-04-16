#!/bin/bash
DISTRIBUTION=`lsb_release -c | awk '{ print $2}'`
ARCHITECTURE=`gcc -dumpmachine`

reset
echo
echo "Compiling on $DISTRIBUTION for $ARCHITECTURE"
echo
make DISTRO=$DISTRIBUTION ARCH=$ARCHITECTURE clean
make DISTRO=$DISTRIBUTION ARCH=$ARCHITECTURE all
