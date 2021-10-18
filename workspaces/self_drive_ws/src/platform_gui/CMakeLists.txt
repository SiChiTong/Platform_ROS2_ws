cmake_minimum_required(VERSION 3.5)

project(platform_gui)

set(CMAKE_CXX_STANDARD 14)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif ()

IF (NOT DEFINED BUILD_VERSION)
	SET(BUILD_VERSION "not set")
ENDIF ()
ADD_DEFINITIONS(-DBUILD_VERSION="${BUILD_VERSION}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5SerialPort REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(QT_QMAKE_EXECUTABLE src/main.cpp)

include_directories(
	${rclcpp_INCLUDE_DIRS}
	${std_msgs_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${cv_bridge_INCLUDE_DIRS}
	${sensor_msgs_INCLUDE_DIRS}
	${image_transport_INCLUDE_DIRS}

	${Qt5Network_INCLUDE_DIRS}
	${Qt5SerialPort_INCLUDE_DIRS}
	${Qt5Core_INCLUDE_DIRS}
	${Qt5Quick_INCLUDE_DIRS}
)

file(GLOB SOURCE_FILES
	src/*.ui
	src/*.h
	src/*.hpp
	src/*.cpp
	src/*.qrc
	)

add_executable(${PROJECT_NAME} ${SOURCE_FILES} src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}
	rclcpp
	std_msgs
	cv_bridge
	sensor_msgs
	)
target_link_libraries(${PROJECT_NAME}
	Qt5::Core
	Qt5::Widgets
	Qt5::Network
	Qt5::SerialPort
	${OpenCV_LIBRARIES}
	)

install(TARGETS ${PROJECT_NAME}
	DESTINATION lib/${PROJECT_NAME})

ament_package()

