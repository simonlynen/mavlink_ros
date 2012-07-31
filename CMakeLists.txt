cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

##########    CHOOSE PLATFORM   #############
set(PLATFORM sensesoar)
#set(PLATFORM common)
#set(PLATFORM ardupilotmega)
#set(PLATFORM matrixpilot)
#set(PLATFORM minimal)
#set(PLATFORM pixhawk)
#set(PLATFORM sensesoar)
#set(PLATFORM slugs)
#set(PLATFORM test)
#set(PLATFORM ualberta)

rosbuild_init()

add_definitions("-DPLATFORM=${PLATFORM}")

include_directories("${CMAKE_CURRENT_BINARY_DIR}/include/v1.0/include")
include_directories("${CMAKE_SOURCE_DIR}/msg_gen/cpp/include")
include_directories("${CMAKE_SOURCE_DIR}/include/mavlink/v1.0/${PLATFORM}")
include_directories("${CMAKE_SOURCE_DIR}/include")
include_directories("${CMAKE_SOURCE_DIR}/include/mavlink/v1.0")

SET(
  CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake
)
include(cmake/FindMAVLINK.cmake)
include(cmake/FindGLIB2.cmake)
include(cmake/FindGTHREAD2.cmake)

FIND_PACKAGE(MAVLINK  REQUIRED)
FIND_PACKAGE(GLIB2  REQUIRED)
FIND_PACKAGE(GTHREAD2  REQUIRED)


INCLUDE_DIRECTORIES(
  ${MAVLINK_INCLUDE_DIR}
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
  ../include/
)

#messages generated with ros
rosbuild_genmsg()

# mavlink generation
macro(generateMavlink version definitions)
    foreach(definition ${definitions})
        set(targetName ${definition}-v${version})
        set(definitionAbsPath ${CMAKE_SOURCE_DIR}/message_definitions/v${version}/${definition})
        message(STATUS "processing: ${definitionAbsPath}")
        add_custom_command( 
            OUTPUT ${targetName}-stamp
            COMMAND ${PYTHON_EXECUTABLE} ${mavgen} --lang=ROS --wire-protocol=${version}
                --output=include/v${version} ${definitionAbsPath}
	    
	    COMMAND cp -rf ${CMAKE_CURRENT_BINARY_DIR}/include/v${version}/msg ${CMAKE_SOURCE_DIR}
            COMMAND touch ${targetName}-stamp
            DEPENDS ${definitionAbsPath} ${mavgen}
            )
        add_custom_target(${targetName} ALL DEPENDS ${targetName}-stamp)
	add_dependencies(mavlink_ros_serial ${targetName})
    endforeach()
endmacro()

#the main executable that passes the data from the serial port to ros
rosbuild_add_executable(mavlink_ros_serial src/mavlink_ros_serial.cpp)
target_link_libraries(mavlink_ros_serial
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
)

# build
set(mavgen ${CMAKE_SOURCE_DIR}/pymavlink/generator/mavgen.py)
set(v0.9Definitions
    ardupilotmega.xml
    common.xml
    minimal.xml
    pixhawk.xml
    slugs.xml
    test.xml
    ualberta.xml
    )
#generateMavlink("0.9" "${v0.9Definitions}")
set(v1.0Definitions
    ardupilotmega.xml
    common.xml
    matrixpilot.xml
    minimal.xml
    pixhawk.xml
    slugs.xml
    test.xml
    ualberta.xml
    sensesoar.xml
    )
generateMavlink("1.0" "${v1.0Definitions}")


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# On mac use clang++
IF(${CMAKE_SYSTEM} MATCHES "Darwin-11.*")
    # Use clang++
    set(CMAKE_CXX_COMPILER /usr/bin/clang++)
    # Use clang
    set(CMAKE_C_COMPILER /usr/bin/clang)
ENDIF(${CMAKE_SYSTEM} MATCHES "Darwin-11.*")

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)



