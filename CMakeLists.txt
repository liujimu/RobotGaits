cmake_minimum_required(VERSION 2.8)

# add prefix to every string in a variable
FUNCTION(PREPEND var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND)

FUNCTION(PREPEND_CPP var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}.cpp")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND_CPP)

FUNCTION(PREPEND_H var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}.h")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND_H)

project(RobotGaits_by_LJM)

if(UNIX)
	set(CMAKE_C_COMPILER "/usr/bin/gcc")
	set(CMAKE_CXX_COMPILER "/usr/bin/g++")
#	set(CMAKE_C_COMPILER "/usr/bin/clang")
#	set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
endif(UNIX)

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# set debug and release options
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CONFIGURATION_TYPES Debug Release)

# set compile options
if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(UNIX)

# set include and lib folders
if(UNIX)
set(GENERAL_INCLUDE_DIR "/usr/Aris_Dependent")
set(SYSTEM_LINK_LIBS pthread gfortran)
set(ROBOTS_INCLUDE_DIR "/usr/Robots/include")
set(ROBOTS_LINK_DIR "/usr/Robots/lib")
set(ROBOTS_LINK_LIBS Robot_Type_I Robot_Type_II Robot_Base Robot_Server Robot_Client)
set(ARIS_INCLUDE_DIR "/usr/Aris/include")
set(ARIS_LINK_DIR "/usr/Aris/lib")
set(ARIS_LINK_LIBS Aris_Control Aris_Plan Aris_Sensor Aris_DynKer Aris_ExpCal Aris_Core)
set(XENOMAI_INCLUDE_DIR "/usr/xenomai/include")
set(XENOMAI_LINK_DIR "/usr/xenomai/lib")
set(XENOMAI_LINK_LIBS native rtdm xenomai)
set(ETHERLAB_INCLUDE_DIR "/opt/etherlab/include")
set(ETHERLAB_LINK_DIR "/opt/etherlab/lib")
set(ETHERLAB_LINK_LIBS ethercat rt)
set(XSENS_INCLUDE_DIR "/usr/Aris_Dependent/xsens/include")
set(XSENS_LINK_LIBS xdeviceclass xcommunication xstypes)
set(XSENS_LINK_DIR "/usr/Aris_Dependent/xsens/lib")

set(ALL_INCLUDE_DIR ${ROBOTS_INCLUDE_DIR} ${GENERAL_INCLUDE_DIR} ${ARIS_INCLUDE_DIR} ${LAPACK_INCLUDE_DIR} ${XSENS_INCLUDE_DIR} ${ETHERLAB_INCLUDE_DIR} ${XENOMAI_INCLUDE_DIR})
set(ALL_LINK_DIR ${ROBOTS_LINK_DIR} ${ARIS_LINK_DIR} ${XSENS_LINK_DIR} ${ETHERLAB_LINK_DIR} ${XENOMAI_LINK_DIR})
set(ALL_LINK_LIBS ${ROBOTS_LINK_LIBS} ${ARIS_LINK_LIBS} ${XSENS_LINK_LIBS} ${ETHERLAB_LINK_LIBS} ${XENOMAI_LINK_LIBS} ${SYSTEM_LINK_LIBS})
endif(UNIX)
if(WIN32)
set(GENERAL_INCLUDE_DIR "C:/Aris_Dependent")
set(SYSTEM_LINK_LIBS ws2_32 setupapi)
set(ROBOTS_INCLUDE_DIR "C:/Robots/include")
set(ROBOTS_LINK_DIR "C:/Robots/lib")
set(ROBOTS_LINK_LIBS Robot_Type_I Robot_Type_II Robot_Base Robot_Server Robot_Client)
set(ARIS_INCLUDE_DIR "C:/Aris/include")
set(ARIS_LINK_DIR "C:/Aris/lib")
set(ARIS_LINK_LIBS Aris_Plan Aris_Sensor Aris_DynKer Aris_ExpCal Aris_Core)
set(XSENS_INCLUDE_DIR "C:/Aris_Dependent/xsens/include")
set(XSENS_LINK_DIR "C:/Aris_Dependent/xsens/lib")
set(XSENS_LINK_LIBS xdeviceclass)

set(ALL_INCLUDE_DIR ${ROBOTS_INCLUDE_DIR} ${GENERAL_INCLUDE_DIR} ${ARIS_INCLUDE_DIR} ${ETHERLAB_INCLUDE_DIR} ${XENOMAI_INCLUDE_DIR} ${LAPACK_INCLUDE_DIR} ${XSENS_INCLUDE_DIR})
set(ALL_LINK_DIR ${ROBOTS_LINK_DIR} ${ARIS_LINK_DIR} ${XSENS_LINK_DIR} ${LAPACK_LINK_DIR})
set(ALL_LINK_LIBS ${ROBOTS_LINK_LIBS} ${ARIS_LINK_LIBS} ${LAPACK_LINK_LIBS} ${XSENS_LINK_LIBS} ${SYSTEM_LINK_LIBS})
endif(WIN32)

include_directories(${ALL_INCLUDE_DIR})
link_directories(${ALL_LINK_DIR})

################################### build server ####################################
set(SOURCE_FILES main.cpp force_guided_walk.cpp)
PREPEND(FULL_SRC Server ${SOURCE_FILES})
add_executable(Server ${FULL_SRC})
target_link_libraries(Server ${ALL_LINK_LIBS})

################################### build client ####################################
add_executable(Client Client/client.cpp)
target_link_libraries(Client ${ALL_LINK_LIBS})

################################### install ####################################
if(UNIX)
	set(CMAKE_INSTALL_PREFIX "/usr/Robots/CMakeDemo/")
endif(UNIX)
if(WIN32)
	set(CMAKE_INSTALL_PREFIX "C:/Robots/CMakeDemo/")
endif(WIN32)

install(TARGETS Server Client RUNTIME DESTINATION bin)


























