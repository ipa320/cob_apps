#! /bin/bash
source /opt/ros/cturtle/setup.sh
source /home/uhr/ros/setup.sh


if [ -e $1 ] || [[ $1 != "start"  &&  $1 != "stop" && $1 != "restart" && $1 != "status" ]]; then
  echo "Usage: (start|stop|restart|status)"
  exit -1
fi

if [ $1 = "stop" -o $1 = "restart" ]; then
  echo "Stopping \"roscore\"..."
  pids=`ps ax | grep \/bin\/roscore | grep -v grep | grep -v ros-roscore.sh | sed -e "s/^ *//" | sed -e "s/ .*//"&`
  for pid in $pids; do
    echo " * $pid"
    `kill -9 $pid`
  done

  echo "Stopping \"rosmaster\" processes..."
  pids=`ps ax | grep \/bin\/rosmaster | grep -v grep | grep -v ros-roscore.sh | sed -e "s/^ *//" | sed -e "s/ .*//"&`
  for pid in $pids; do
    echo " * $pid"
    `kill -9 $pid`
  done

  echo "Stopping \"rosout\" processes..."
  pids=`ps ax | grep rosout | grep -v grep | grep -v ros-roscore.sh | sed -e "s/^ *//" | sed -e "s/ .*//"&`
  for pid in $pids; do
    echo " * $pid"
    `kill -9 $pid`
  done
fi

if [ $1 = "start" -o $1 = "restart" ]; then
  echo "Starting \"RosCore\"..."
  python -u /opt/ros/cturtle/ros/bin/roscore &

  echo "Waiting for \"roscore\" to be initialized"

  timeout=10
  sleepPerPeriod=1
  time=0
  while [ $time -lt $timeout ]; do
    result=$(rosnode list 2>&1)

    if [ "$result" == "ERROR: Unable to communicate with master!" ]; then
      time=$(($time+$sleepPerPeriod))
      echo "Wait: $sleepPerPeriod, Timeout: $timeout, time: $time"
      sleep $sleepPerPeriod
    
    else
      break
    fi
  done
fi

if [ $1 = "status" ]; then
  echo "Status not implemeted yet."
fi

echo "done"
exit 0
