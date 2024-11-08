# Install script for directory: /home/zgc/learnRos2/zgc_ws/src/Stage/worlds

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zgc/learnRos2/zgc_ws/install/Stage")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RELEASE")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stage/worlds" TYPE FILE FILES
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/amcl-sonar.cfg"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/autolab.cfg"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/camera.cfg"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/everything.cfg"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/lsp_test.cfg"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/mbicp.cfg"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/nd.cfg"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/roomba.cfg"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/simple.cfg"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/test.cfg"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/uoa_robotics_lab.cfg"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/vfh.cfg"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/wavefront-remote.cfg"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/wavefront.cfg"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/wifi.cfg"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/SFU.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/autolab.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/camera.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/circuit.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/everything.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/fasr.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/fasr2.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/fasr_plan.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/large.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/lsp_test.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/mbicp.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/pioneer_flocking.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/pioneer_follow.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/pioneer_walle.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/roomba.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/sensor_noise_demo.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/sensor_noise_module_demo.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/simple.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/uoa_robotics_lab.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/wifi.world"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/beacons.inc"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/chatterbox.inc"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/hokuyo.inc"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/irobot.inc"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/map.inc"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/objects.inc"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/pantilt.inc"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/pioneer.inc"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/sick.inc"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/ubot.inc"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/uoa_robotics_lab_models.inc"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/walle.inc"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/cfggen.sh"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/test.sh"
    "/home/zgc/learnRos2/zgc_ws/src/Stage/worlds/worldgen.sh"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/zgc/learnRos2/zgc_ws/build/Stage/worlds/benchmark/cmake_install.cmake")
  include("/home/zgc/learnRos2/zgc_ws/build/Stage/worlds/bitmaps/cmake_install.cmake")
  include("/home/zgc/learnRos2/zgc_ws/build/Stage/worlds/wifi/cmake_install.cmake")

endif()

