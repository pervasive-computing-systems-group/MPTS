#
# This very appreciated template was provided by Matthias Miltenberger.
# To build:
#  > cmake -H. -Bbuild
#  > cmake --build build
#

cmake_minimum_required(VERSION 3.10)

project(assignTasks)

option(CXX "enable C++ compilation" ON)
if(CXX)
	enable_language(CXX)
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
set(CMAKE_CXX_FLAGS "-O0 -ggdb -g -Wall -Werror -pthread")

include_directories(
	${CMAKE_CURRENT_LIST_DIR}/inc
)

# list source files here
set(sources src/driver.cpp)

add_executable(${CMAKE_PROJECT_NAME}
	${sources}
	src/GradientSolver.cpp
	src/HungGSSolver.cpp
	src/MsgSender.cpp
	src/MsgServer.cpp
	src/Node.cpp
	src/P2P.cpp
	src/PoissonBinomial.cpp
	src/Solver.cpp
	src/Utilities.cpp
)
