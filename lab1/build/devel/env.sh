#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/viki/catkin_ws/src/robotics-hoverboard-project/lab1/build/devel', type 'exit' to leave"
  . "/home/viki/catkin_ws/src/robotics-hoverboard-project/lab1/build/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/viki/catkin_ws/src/robotics-hoverboard-project/lab1/build/devel'"
else
  . "/home/viki/catkin_ws/src/robotics-hoverboard-project/lab1/build/devel/setup.sh"
  exec "$@"
fi
