# Install script for directory: /home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/src/platform_gui

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/platform_gui/platform_gui" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/platform_gui/platform_gui")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/platform_gui/platform_gui"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/platform_gui" TYPE EXECUTABLE FILES "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/src/platform_gui/qt_build/platform_gui")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/platform_gui/platform_gui" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/platform_gui/platform_gui")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/platform_gui/platform_gui"
         OLD_RPATH "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/rclcpp/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/sensor_msgs/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/builtin_interfaces/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/geometry_msgs/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/std_msgs/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/rosidl_typesupport_cpp/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/rosidl_typesupport_introspection_cpp/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/rcpputils/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/rosidl_typesupport_c/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/rosidl_typesupport_introspection_c/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/rcutils/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/rosidl_runtime_c/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/install/cv_bridge/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/ament_index_cpp/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/libstatistics_collector/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/rcl/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/rcl_interfaces/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/rmw_implementation/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/rcl_logging_spdlog/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/rcl_logging_interface/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/rcl_yaml_param_parser/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/rmw/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/libyaml_vendor/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/rosgraph_msgs/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/statistics_msgs/lib:/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/tracetools/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/platform_gui/platform_gui")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/src/platform_gui/qt_build/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/platform_gui")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/src/platform_gui/qt_build/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/platform_gui")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/platform_gui/environment" TYPE FILE FILES "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/platform_gui/environment" TYPE FILE FILES "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/src/platform_gui/qt_build/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/platform_gui/environment" TYPE FILE FILES "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/ros2_galactic/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/platform_gui/environment" TYPE FILE FILES "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/src/platform_gui/qt_build/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/platform_gui" TYPE FILE FILES "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/src/platform_gui/qt_build/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/platform_gui" TYPE FILE FILES "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/src/platform_gui/qt_build/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/platform_gui" TYPE FILE FILES "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/src/platform_gui/qt_build/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/platform_gui" TYPE FILE FILES "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/src/platform_gui/qt_build/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/platform_gui" TYPE FILE FILES "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/src/platform_gui/qt_build/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/src/platform_gui/qt_build/ament_cmake_index/share/ament_index/resource_index/packages/platform_gui")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/platform_gui/cmake" TYPE FILE FILES
    "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/src/platform_gui/qt_build/ament_cmake_core/platform_guiConfig.cmake"
    "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/src/platform_gui/qt_build/ament_cmake_core/platform_guiConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/platform_gui" TYPE FILE FILES "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/src/platform_gui/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/park/Documents/GitHub/Platform_ROS2_ws/workspaces/self_drive_ws/src/platform_gui/qt_build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
