cmake_minimum_required(VERSION 2.8)
project(SlamLoaderPlugin)
if (POLICY CMP0020)
    cmake_policy(SET CMP0020 OLD)
endif ()

find_package(CloudCompare)
#requires PCL
find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(nlohmann_json 3.2.0 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)

#As we are going to re-use several components of the main qCC project
#we define its sources path relatively to the plugin's dir
set(CloudCompare_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../qCC)

#include the whole CC directory (easier)
include_directories(${CloudCompare_SOURCE_DIR})
include_directories(${CloudCompareProjects_SOURCE_DIR})

option(INSTALL_SLAMLOADER_PLUGIN "Check to install SLAMLOADER plugin" ON)

#if (INSTALL_SLAMLOADER_PLUGIN)
#
#
#    #include(../../CMakePluginTpl.cmake)
#
#    #set dependencies to necessary libraries (see qPCV for an example)
#    include_directories(${PCL_INCLUDE_DIRS})
#    include_directories(${QPCL_PLUGIN_UTILS_LIB_SOURCE_DIR}/filters)
#    include_directories(${QPCL_PLUGIN_UTILS_LIB_SOURCE_DIR}/utils)
#
#    link_directories(${PCL_LIBRARY_DIRS})
#
#    target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${PCL_LIBRARIES} QPCL_PLUGIN_UTILS_LIB nlohmann_json::nlohmann_json)
#
#endif ()