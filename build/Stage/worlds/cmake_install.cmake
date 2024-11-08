# Install script for directory: /home/zgc/mr_ws/Stage/worlds

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zgc/mr_ws/install/Stage")
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
    "/home/zgc/mr_ws/Stage/worlds/amcl-sonar.cfg"
    "/home/zgc/mr_ws/Stage/worlds/autolab.cfg"
    "/home/zgc/mr_ws/Stage/worlds/camera.cfg"
    "/home/zgc/mr_ws/Stage/worlds/everything.cfg"
    "/home/zgc/mr_ws/Stage/worlds/lsp_test.cfg"
    "/home/zgc/mr_ws/Stage/worlds/mbicp.cfg"
    "/home/zgc/mr_ws/Stage/worlds/nd.cfg"
    "/home/zgc/mr_ws/Stage/worlds/roomba.cfg"
    "/home/zgc/mr_ws/Stage/worlds/simple.cfg"
    "/home/zgc/mr_ws/Stage/worlds/test.cfg"
    "/home/zgc/mr_ws/Stage/worlds/uoa_robotics_lab.cfg"
    "/home/zgc/mr_ws/Stage/worlds/vfh.cfg"
    "/home/zgc/mr_ws/Stage/worlds/wavefront-remote.cfg"
    "/home/zgc/mr_ws/Stage/worlds/wavefront.cfg"
    "/home/zgc/mr_ws/Stage/worlds/wifi.cfg"
    "/home/zgc/mr_ws/Stage/worlds/SFU.world"
    "/home/zgc/mr_ws/Stage/worlds/autolab.world"
    "/home/zgc/mr_ws/Stage/worlds/camera.world"
    "/home/zgc/mr_ws/Stage/worlds/circuit.world"
    "/home/zgc/mr_ws/Stage/worlds/everything.world"
    "/home/zgc/mr_ws/Stage/worlds/fasr.world"
    "/home/zgc/mr_ws/Stage/worlds/fasr2.world"
    "/home/zgc/mr_ws/Stage/worlds/fasr_plan.world"
    "/home/zgc/mr_ws/Stage/worlds/large.world"
    "/home/zgc/mr_ws/Stage/worlds/lsp_test.world"
    "/home/zgc/mr_ws/Stage/worlds/mbicp.world"
    "/home/zgc/mr_ws/Stage/worlds/pioneer_flocking.world"
    "/home/zgc/mr_ws/Stage/worlds/pioneer_follow.world"
    "/home/zgc/mr_ws/Stage/worlds/pioneer_walle.world"
    "/home/zgc/mr_ws/Stage/worlds/roomba.world"
    "/home/zgc/mr_ws/Stage/worlds/sensor_noise_demo.world"
    "/home/zgc/mr_ws/Stage/worlds/sensor_noise_module_demo.world"
    "/home/zgc/mr_ws/Stage/worlds/simple.world"
    "/home/zgc/mr_ws/Stage/worlds/uoa_robotics_lab.world"
    "/home/zgc/mr_ws/Stage/worlds/wifi.world"
    "/home/zgc/mr_ws/Stage/worlds/beacons.inc"
    "/home/zgc/mr_ws/Stage/worlds/chatterbox.inc"
    "/home/zgc/mr_ws/Stage/worlds/hokuyo.inc"
    "/home/zgc/mr_ws/Stage/worlds/irobot.inc"
    "/home/zgc/mr_ws/Stage/worlds/map.inc"
    "/home/zgc/mr_ws/Stage/worlds/objects.inc"
    "/home/zgc/mr_ws/Stage/worlds/pantilt.inc"
    "/home/zgc/mr_ws/Stage/worlds/pioneer.inc"
    "/home/zgc/mr_ws/Stage/worlds/sick.inc"
    "/home/zgc/mr_ws/Stage/worlds/ubot.inc"
    "/home/zgc/mr_ws/Stage/worlds/uoa_robotics_lab_models.inc"
    "/home/zgc/mr_ws/Stage/worlds/walle.inc"
    "/home/zgc/mr_ws/Stage/worlds/cfggen.sh"
    "/home/zgc/mr_ws/Stage/worlds/test.sh"
    "/home/zgc/mr_ws/Stage/worlds/worldgen.sh"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/zgc/mr_ws/build/Stage/worlds/benchmark/cmake_install.cmake")
  include("/home/zgc/mr_ws/build/Stage/worlds/bitmaps/cmake_install.cmake")
  include("/home/zgc/mr_ws/build/Stage/worlds/wifi/cmake_install.cmake")

endif()

