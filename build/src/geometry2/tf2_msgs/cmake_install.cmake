# Install script for directory: /home/ubuntu/ros_mbrb/src/src/geometry2/tf2_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ubuntu/ros_mbrb/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf2_msgs/msg" TYPE FILE FILES
    "/home/ubuntu/ros_mbrb/src/src/geometry2/tf2_msgs/msg/TF2Error.msg"
    "/home/ubuntu/ros_mbrb/src/src/geometry2/tf2_msgs/msg/TFMessage.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf2_msgs/srv" TYPE FILE FILES "/home/ubuntu/ros_mbrb/src/src/geometry2/tf2_msgs/srv/FrameGraph.srv")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf2_msgs/action" TYPE FILE FILES "/home/ubuntu/ros_mbrb/src/src/geometry2/tf2_msgs/action/LookupTransform.action")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf2_msgs/msg" TYPE FILE FILES
    "/home/ubuntu/ros_mbrb/devel/share/tf2_msgs/msg/LookupTransformAction.msg"
    "/home/ubuntu/ros_mbrb/devel/share/tf2_msgs/msg/LookupTransformActionGoal.msg"
    "/home/ubuntu/ros_mbrb/devel/share/tf2_msgs/msg/LookupTransformActionResult.msg"
    "/home/ubuntu/ros_mbrb/devel/share/tf2_msgs/msg/LookupTransformActionFeedback.msg"
    "/home/ubuntu/ros_mbrb/devel/share/tf2_msgs/msg/LookupTransformGoal.msg"
    "/home/ubuntu/ros_mbrb/devel/share/tf2_msgs/msg/LookupTransformResult.msg"
    "/home/ubuntu/ros_mbrb/devel/share/tf2_msgs/msg/LookupTransformFeedback.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf2_msgs/cmake" TYPE FILE FILES "/home/ubuntu/ros_mbrb/build/src/geometry2/tf2_msgs/catkin_generated/installspace/tf2_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ubuntu/ros_mbrb/devel/include/tf2_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ubuntu/ros_mbrb/devel/share/roseus/ros/tf2_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ubuntu/ros_mbrb/devel/share/common-lisp/ros/tf2_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ubuntu/ros_mbrb/devel/share/gennodejs/ros/tf2_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/ubuntu/ros_mbrb/devel/lib/python3/dist-packages/tf2_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ubuntu/ros_mbrb/devel/lib/python3/dist-packages/tf2_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ubuntu/ros_mbrb/build/src/geometry2/tf2_msgs/catkin_generated/installspace/tf2_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf2_msgs/cmake" TYPE FILE FILES "/home/ubuntu/ros_mbrb/build/src/geometry2/tf2_msgs/catkin_generated/installspace/tf2_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf2_msgs/cmake" TYPE FILE FILES
    "/home/ubuntu/ros_mbrb/build/src/geometry2/tf2_msgs/catkin_generated/installspace/tf2_msgsConfig.cmake"
    "/home/ubuntu/ros_mbrb/build/src/geometry2/tf2_msgs/catkin_generated/installspace/tf2_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf2_msgs" TYPE FILE FILES "/home/ubuntu/ros_mbrb/src/src/geometry2/tf2_msgs/package.xml")
endif()

