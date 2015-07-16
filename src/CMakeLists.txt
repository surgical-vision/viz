# HEADERS - contains a list of our application header files
# SOURCES - contains a list of our application src files
# LIB_INC_DIR - list of dirs we have header files in 
# USER_INC - all of the external library header directories i.e. stuff that's in /usr/inc/
# LINK_LIBS - same as USER_INC but for the built libs

#######################################################
## Library sources

set( INCDIR "../include")
set( BINARY_NAME "viz")
set( MAIN_FILE "main.cpp" )

## Header only includes 
set(
  HEADERS
  ${INCDIR}/camera.hpp ${INCDIR}/config_reader.hpp
  ${INCDIR}/pose_grabber.hpp ${INCDIR}/davinci.hpp
  ${INCDIR}/resources.hpp
  ${INCDIR}/vizApp.hpp  ${INCDIR}/video.hpp
  ${INCDIR}/model.hpp ${INCDIR}/sub_window.hpp
)

## Store list of source files
set( SOURCES camera.cpp davinci.cpp pose_grabber.cpp video.cpp vizApp.cpp model.cpp sub_window.cpp )


#######################################################
## Setup required includes / link info

# Project headers trump everything (including any potentially installed Pangolin)
list(APPEND LIB_INC_DIR  "${PROJECT_SOURCE_DIR}/include;${CMAKE_CURRENT_BINARY_DIR}/include" )

#Boost
set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF)
#set(Boost_BOOST_ALL_DYN_LINK)
find_package(Boost REQUIRED COMPONENTS regex chrono date_time system filesystem thread)
if(Boost_FOUND)
    list(APPEND USER_INC "${Boost_INCLUDE_DIRS}")
    list(APPEND LINK_LIBS "${Boost_LIBRARIES}")
endif()

find_package(OpenCV REQUIRED)
list(APPEND USER_INC "${OpenCV_INCLUDE_DIRS}" )
list(APPEND LINK_LIBS "${OpenCV_LIBRARIES}" )

find_package(Cinder REQUIRED)
list(APPEND USER_INC "${CINDER_INCLUDE_DIRS}" )
list(APPEND LINK_LIBS "${CINDER_LIBRARIES}" )

#######################################################
## Add Libraries / Include Directories / Link directories

include_directories( ${LIB_INC_DIR} )
include_directories( ${USER_INC} )
link_directories( ${USER_LIB} )

if(_WIN_)
add_executable(${BINARY_NAME} WIN32 "../resources/Resources.rc" ${MAIN_FILE} ${SOURCES} ${HEADERS} )
else(_WIN_)
add_executable(${BINARY_NAME} ${MAIN_FILE} ${SOURCES} ${HEADERS} )
endif()

target_link_libraries(${BINARY_NAME} ${LINK_LIBS})



