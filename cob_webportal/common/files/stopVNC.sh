#/bin/bash
echo "Display: $1"
echo "PID: $2"

/opt/TurboVNC/bin/vncserver -kill :$1
kill $2
kill -9 $2
