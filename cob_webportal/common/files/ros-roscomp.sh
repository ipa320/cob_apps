#!/bin/bash
export ROBOT=cob3-1
source /opt/ros/cturtle/setup.sh
#source /home/brics/git/care-o-bot/setup.sh /home/brics/git/care-o-bot
#source /home/brics/git/cob3_intern/setup.sh /home/brics/git/cob3_intern
source /opt/ros/diamondback/setup.bash
#source /home/uhr/git/robocup/setup.sh /home/uhr/git/robocup

echo $ROS_PACKAGE_PATH_ADD
if [[ $ROS_PACKAGE_PATH_ADD == 'overlay' ]]; then
	export ROS_PACKAGE_PATH=~/git/care-o-bot:$ROS_PACKAGE_PATH
fi
echo $ROS_PACKAGE_PATH

VIRTUALGL_PATH=/opt/VirtualGL/bin/vglrun
IFS=$'\n'

status() {
  local searchname=$searchname
  grep=`ps ax | grep $searchname | grep -v grep | grep -v ros-roscomp.sh`
  if [[ $grep == '' ]]; then
    echo 0
  else
    echo 1
  fi
}


if [ -e $1 ] || [ -e $2 ] || [ -e $3 ] || [ -e $4 ] || [ -e $5 ] || [[ $1 != "start" && $1 != "stop" && $1 != "restart" && $1 != "status" ]] || [[ $4 != "rosstart" && $4 != "roslaunch" && $4 != "rosrun" ]]; then
  echo "Usage: (start|stop|restart|status) name searchname (rosstart|roslaunch) launchfile args [vgl]"
  exit -1
fi

# Store the parameters in variables
cmd=$1
name=$2
searchname=$3
rostype=$4
launchfile=$5

if [[ $6 == "vgl" ]]; then
  vgl=$6
  unset args[6]
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
      eval "timeout -9 5 rosnode kill $node 1>/dev/null 2>&1"
#      `timeout -9 5 rosnode kill $node`
    done

    sleep 1 # Wait for nodes to shut down
  fi

  echo "Stopping proccess containing \"$searchname\" ..."
  pids=`ps ax | grep $searchname | grep -v grep | grep -v ros-roscomp.sh | sed -e "s/^ *//" | sed -e "s/ .*//"&`
  for pid in $pids; do
    echo " * $pid"
    `kill -9 $pid 1>/dev/null 2>&1`
  done

  sleep 1
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
   	$VIRTUALGL_PATH  /opt/ros/cturtle/ros/bin/$rostype $name $launchfile $argStr  &     
  fi

  echo "Waiting for \"$name\" to be initialized"
  
  sleepPerPeriod=1
  time=0
  timeout=6
  while [ $time -lt $timeout ]; do
    result=$(status "$name" "$launchfile")

    if [[ $result == 0 ]]; then
      time=$(($time+$sleepPerPeriod))
      echo "Waiting for all rosnodes to start. Timeout: $timeout, time: $time"
      sleep $sleepPerPeriod
    
    else
      break
    fi
  done
  echo "* All rosnodes started successfully!"


fi

if [ $cmd = "status" ]; then
  ps ax | grep $searchname | grep -v grep | grep -v ros-roscomp.sh

  echo "roslaunch --nodes $name $launchfile"

  runningNodes=`rosnode list`
  nodes=`roslaunch --nodes $name $launchfile`

  for node in $nodes; do
    nodeRunning=0
    echo " * $node"

    # skip $(anon nodes, their names are ramodomly assigned
    if [[ ${node:0:6} == '$(anon' ]]; then
       continue
    fi
    if [[ $node == 'cob3_gazebo_model' ]]; then
       continue
    fi

    for rNode in $runningNodes; do
      if [ $node == $rNode ]; then
        nodeRunning=1
      fi
    done

    if [[ $nodeRunnin == 1 ]]; then
       echo "Node Running"
    else
      echo "Node NOT Running"
    fi

  done


fi

exit 0
