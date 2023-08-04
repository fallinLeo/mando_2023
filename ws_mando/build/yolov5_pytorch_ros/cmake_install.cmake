# Install script for directory: /home/leo/Documents/GitHub/mando_2023/ws_mando/src/yolov5_pytorch_ros

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/leo/Documents/GitHub/mando_2023/ws_mando/install")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yolov5_pytorch_ros/msg" TYPE FILE FILES
    "/home/leo/Documents/GitHub/mando_2023/ws_mando/src/yolov5_pytorch_ros/msg/BoundingBox.msg"
    "/home/leo/Documents/GitHub/mando_2023/ws_mando/src/yolov5_pytorch_ros/msg/BoundingBoxes.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yolov5_pytorch_ros/cmake" TYPE FILE FILES "/home/leo/Documents/GitHub/mando_2023/ws_mando/build/yolov5_pytorch_ros/catkin_generated/installspace/yolov5_pytorch_ros-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/include/yolov5_pytorch_ros")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/share/roseus/ros/yolov5_pytorch_ros")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/share/common-lisp/ros/yolov5_pytorch_ros")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/share/gennodejs/ros/yolov5_pytorch_ros")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/python3/dist-packages/yolov5_pytorch_ros")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/python3/dist-packages/yolov5_pytorch_ros")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/leo/Documents/GitHub/mando_2023/ws_mando/build/yolov5_pytorch_ros/catkin_generated/installspace/yolov5_pytorch_ros.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yolov5_pytorch_ros/cmake" TYPE FILE FILES "/home/leo/Documents/GitHub/mando_2023/ws_mando/build/yolov5_pytorch_ros/catkin_generated/installspace/yolov5_pytorch_ros-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yolov5_pytorch_ros/cmake" TYPE FILE FILES
    "/home/leo/Documents/GitHub/mando_2023/ws_mando/build/yolov5_pytorch_ros/catkin_generated/installspace/yolov5_pytorch_rosConfig.cmake"
    "/home/leo/Documents/GitHub/mando_2023/ws_mando/build/yolov5_pytorch_ros/catkin_generated/installspace/yolov5_pytorch_rosConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yolov5_pytorch_ros" TYPE FILE FILES "/home/leo/Documents/GitHub/mando_2023/ws_mando/src/yolov5_pytorch_ros/package.xml")
endif()

