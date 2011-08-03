#!/bin/bash

source ~/.bashrc
export PATH=$PATH:/opt/TurboVNC/bin

if [ $# -ne 1 ]
then
  echo "Usage: `basename $0` {display}"
  exit $E_BADARGS
fi

rm -f /tmp/.X$1-lock 1>/dev/null 2>&1
rm -f /tmp/.X11-unix/X$1 1>/dev/null 2>&1


HTTPBASE=8080
let "HTTPPORT=$HTTPBASE+$1"

RFBBASE=5900
let "RFBPORT=$RFBBASE+$1"

vncserver :$1   -httpport $HTTPPORT

