#!/bin/bash
export ROBOT=cob3-1
source /opt/ros/cturtle/setup.sh
source /home/brics/git/care-o-bot/setup.sh /home/brics/git/care-o-bot
#source /home/brics/git/cob3_intern/setup.sh /home/uhr/git/cob3_intern
#source /home/uhr/git/robocup/setup.sh /home/uhr/git/robocup

VIRTUALGL_PATH=/opt/VirtualGL/bin/vglrun


status() {
  local package=$cmd
  local launchfile=$name

  status=1
  nodeRunning=0
  runningNodes=`rosnode list`
  nodes=`roslaunch --nodes $package $launchfile`

  for node in $nodes; do
    running=0
    for rNode in $runningNodes; do
      if [ $node == $rNode ]; then
        nodeRunning=1
      fi
    done

    if [ $nodeRunning == 0 ]; then
     status=0
      break
    fi
  done

  echo $status;
}


if [ -e $1 ] || [ -e $2 ] || [ -e $3 ] || [ -e $4 ] || [ -e $5 ] || [[ $1 != "start" && $1 != "stop" && $1 != "restart" && $1 != "status" ]] || [[ $4 != "rosstart" && $4 != "roslaunch" && $4 != "rosrun" ]]; then
  echo "Usage: (start|stop|restart|status) name searchname (rosstart|roslaunch) launchfile args [DISPLAY] [vgl]"
  exit -1
fi

# Store the parameters in variables
cmd=$1
name=$2
searchname=$3
rostype=$4
launchfile=$5
disp=$6
vgl=$7

if [ -z $disp ]; then
	export DISPLAY=:$disp
fi

unset args[0]
unset args[1]
unset args[2]
unset args[3]
unset args[4]

argStr=${args[@]}  



if [ $cmd == "stop" -o $cmd == "restart" ]; then
  if [ $rostype == "roslaunch" ]; then
    echo "Stopping  \"$name\"'s nodes in launch-file \"$launchfile\"..."
    nodes=`roslaunch --nodes $name $launchfile`
    for node in $nodes; do
      echo " * $node"
      a=`rosnode kill $node 2>/dev/null`
    done

    sleep 1 # Wait for nodes to shut down
  fi

  echo "Stopping proccess containing \"$searchname\" ..."
  pids=`ps ax | grep $searchname | grep -v grep | grep -v ros-roscomp.sh | sed -e "s/^ *//" | sed -e "s/ .*//"&`
  for pid in $pids; do
    echo " * $pid"
    `kill -9 $pid 2>/dev/null`
  done
fi

if [ $cmd = "start" -o $cmd = "restart" ]; then
  echo "Starting \"$name\"..."

  echo " * $rostype $name $launchfile $argStr"
  #$rostype $name $argStr &
   if [ -e $vgl ]; then 
   	/opt/ros/cturtle/ros/bin/$rostype $name $launchfile $argStr &
	echo "/opt/ros/cturtle/ros/bin/$rostype $name $launchfile $argStr "
   else
	echo "$VIRTUALGL_PATH /opt/ros/cturtle/ros/bin/$rostype $name $launchfile $argStr &"
   	$VIRTUALGL_PATH  /opt/ros/cturtle/ros/bin/$rostype $name $launchfile $argStr &
   fi

  echo "Waiting for \"$name\" to be initialized"

  timeout=10
  sleepPerPeriod=0.2
  time=0
  running=0
  while [ $time -lt $timeout ]; do
    result=$(status "$name" "$launchfile")
    if [ $result == 1 ]; then
      running=1
    fi

    if [ $result == 0 ]; then
      time=$(($time+$sleepPerPeriod))
      echo "Wait: $sleepPerPeriod, Timeout: $timeout, time: $time"
      sleep $sleepPerPeriod
    
    else
      break
    fi
  done
fi

if [ $cmd = "status" ]; then
  result=$(status "$name" "$launchfile")

  if [ $result == 0 ]; then
    echo "not_running"
  elif [ $result == 1 ]; then
    echo "running"
  else
    echo "unexpected_result"
  fi

fi

exit 0
