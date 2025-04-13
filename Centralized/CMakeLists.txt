#
# This very appreciated template was provided by Matthias Miltenberger.
# To build:
#  > cmake -H. -Bbuild
#  > cmake --build build
#

cmake_minimum_required(VERSION 3.10)

project(find-assignment)

option(CXX "enable C++ compilation" ON)
if(CXX)
	enable_language(CXX)
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
set(CMAKE_CXX_FLAGS "-O3 -ggdb -g -Wall -Werror")

# Visual Studio compiler with static runtime libraries
if(MSVC AND MT)
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT -O0 -ggdb -g -Wall -Werror")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd -O0 -ggdb -g -Wall -Werror")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT -O0 -ggdb -g -Wall -Werror")
	set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd -O0 -ggdb -g -Wall -Werror")
endif()

include_directories(
	${CMAKE_CURRENT_LIST_DIR}/inc
)

# list source files here
set(sources src/main.cpp)

add_executable(${CMAKE_PROJECT_NAME}
	${sources}
	src/Hungarian.cpp
	src/I_solution.cpp
	src/Input.cpp
	src/MASP_BalMatch.cpp
	src/MASP_BranchAndBound.cpp
	src/MASP_BranchAndMatch.cpp
	src/MASP_comp.cpp
	src/MASP_EdgeCutting.cpp
	src/MASP_FastComp.cpp
	src/MASP_GradientSearch.cpp
	src/MASP_LogBalMatch.cpp
	src/MASP_MatchAct.cpp
	src/MASP_MatchGS.cpp
	src/MASP_MinDist.cpp
	src/MASP_TMatch.cpp
	src/MASPInput.cpp
	src/MASPSolver.cpp
	src/PoissonBinomial.cpp
	src/Solver.cpp
	src/Utilities.cpp
	src/MASP_Swap.cpp
)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
	include(FeatureSummary)
	feature_summary(WHAT ALL)
endif()
