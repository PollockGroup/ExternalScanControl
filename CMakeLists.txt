cmake_minimum_required(VERSION 3.1)
project(ExternalScanControl)

#find nidaq header/library
find_path(NIDAQmx_INCLUDE_DIR NIDAQmx.h ${CMAKE_CURRENT_SOURCE_DIR})
find_library(NIDAQmx_LIBRARY NIDAQmx ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${NIDAQmx_INCLUDE_DIR})

#set SFML_ROOT for module to work
set(SFML_ROOT "SFML_ROOT-NOTFOUND" CACHE FILEPATH "sfml install location")
set(CMAKE_MODULE_PATH "${SFML_ROOT}/cmake/Modules" ${CMAKE_MODULE_PATH})
find_package(SFML 2.3 REQUIRED system window graphics)
include_directories(${SFML_INCLUDE_DIR})

add_executable (ExternalScan main.cpp)
set_property(TARGET ExternalScan PROPERTY CXX_STANDARD 11)
target_link_libraries(ExternalScan ${NIDAQmx_LIBRARY})

add_executable (ExternalScanGui gui.cpp)
set_property(TARGET ExternalScanGui PROPERTY CXX_STANDARD 11)
target_link_libraries(ScanControlTest ${NIDAQmx_LIBRARY} ${SFML_LIBRARIES})