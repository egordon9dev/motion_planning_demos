if [ $# == 1 ]
then
  catkin_make
  . devel/setup.bash
  rosrun hello $1
fi
