execute_process(COMMAND "/home/ubuntu/ros_mbrb/build/src/geometry2/tf2_sensor_msgs/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ubuntu/ros_mbrb/build/src/geometry2/tf2_sensor_msgs/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
