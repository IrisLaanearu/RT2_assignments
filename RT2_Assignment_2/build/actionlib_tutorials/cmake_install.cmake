# Install script for directory: /root/RT1_Assignment_2/src/actionlib_tutorials

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/root/RT1_Assignment_2/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_tutorials/action" TYPE FILE FILES
    "/root/RT1_Assignment_2/src/actionlib_tutorials/action/Fibonacci.action"
    "/root/RT1_Assignment_2/src/actionlib_tutorials/action/Averaging.action"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_tutorials/msg" TYPE FILE FILES
    "/root/RT1_Assignment_2/devel/share/actionlib_tutorials/msg/FibonacciAction.msg"
    "/root/RT1_Assignment_2/devel/share/actionlib_tutorials/msg/FibonacciActionGoal.msg"
    "/root/RT1_Assignment_2/devel/share/actionlib_tutorials/msg/FibonacciActionResult.msg"
    "/root/RT1_Assignment_2/devel/share/actionlib_tutorials/msg/FibonacciActionFeedback.msg"
    "/root/RT1_Assignment_2/devel/share/actionlib_tutorials/msg/FibonacciGoal.msg"
    "/root/RT1_Assignment_2/devel/share/actionlib_tutorials/msg/FibonacciResult.msg"
    "/root/RT1_Assignment_2/devel/share/actionlib_tutorials/msg/FibonacciFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_tutorials/msg" TYPE FILE FILES
    "/root/RT1_Assignment_2/devel/share/actionlib_tutorials/msg/AveragingAction.msg"
    "/root/RT1_Assignment_2/devel/share/actionlib_tutorials/msg/AveragingActionGoal.msg"
    "/root/RT1_Assignment_2/devel/share/actionlib_tutorials/msg/AveragingActionResult.msg"
    "/root/RT1_Assignment_2/devel/share/actionlib_tutorials/msg/AveragingActionFeedback.msg"
    "/root/RT1_Assignment_2/devel/share/actionlib_tutorials/msg/AveragingGoal.msg"
    "/root/RT1_Assignment_2/devel/share/actionlib_tutorials/msg/AveragingResult.msg"
    "/root/RT1_Assignment_2/devel/share/actionlib_tutorials/msg/AveragingFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_tutorials/cmake" TYPE FILE FILES "/root/RT1_Assignment_2/build/actionlib_tutorials/catkin_generated/installspace/actionlib_tutorials-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/root/RT1_Assignment_2/devel/include/actionlib_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/root/RT1_Assignment_2/devel/share/roseus/ros/actionlib_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/root/RT1_Assignment_2/devel/share/common-lisp/ros/actionlib_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/root/RT1_Assignment_2/devel/share/gennodejs/ros/actionlib_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/root/RT1_Assignment_2/devel/lib/python3/dist-packages/actionlib_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/root/RT1_Assignment_2/devel/lib/python3/dist-packages/actionlib_tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/root/RT1_Assignment_2/build/actionlib_tutorials/catkin_generated/installspace/actionlib_tutorials.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_tutorials/cmake" TYPE FILE FILES "/root/RT1_Assignment_2/build/actionlib_tutorials/catkin_generated/installspace/actionlib_tutorials-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_tutorials/cmake" TYPE FILE FILES
    "/root/RT1_Assignment_2/build/actionlib_tutorials/catkin_generated/installspace/actionlib_tutorialsConfig.cmake"
    "/root/RT1_Assignment_2/build/actionlib_tutorials/catkin_generated/installspace/actionlib_tutorialsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_tutorials" TYPE FILE FILES "/root/RT1_Assignment_2/src/actionlib_tutorials/package.xml")
endif()

