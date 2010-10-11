#!/bin/bash

source ~/.bashrc
export PATH=$PATH:/opt/TurboVNC/bin

if [ $# -ne 1 ]
then
  echo "Usage: `basename $0` {display}"
  exit $E_BADARGS
fi


HTTPBASE=8080
let "HTTPPORT=$HTTPBASE+$1"

RFBBASE=5900
let "RFBPORT=$RFBBASE+$1"

vncserver :$1   -httpport $HTTPPORT

