#!/bin/bash


if [ $# -ne 1 ]
then
  echo "Usage: `basename $0` {display}"
  exit $E_BADARGS
fi


HTTPBASE=8080
let "HTTPPORT=$HTTPBASE+$1"

RFBBASE=5900
let "RFBPORT=$RFBBASE+$1"

Xtightvnc :$1 -desktop X -auth /home/uhr/.Xauthority -geometry 1024x768 -depth 24 -rfbwait 120000 -rfbauth /home/uhr/.vnc/passwd -rfbport $RFBPORT -fp /usr/share/fonts/X11/misc/,/usr/share/fonts/X11/Type1/,/usr/share/fonts/X11/75dpi/,/usr/share/fonts/X11/100dpi/ -co /etc/X11/rgb -httpd /usr/share/tightvnc-java -httpport $HTTPPORT

