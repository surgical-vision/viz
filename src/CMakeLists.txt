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
#set( TTRACK_INCDIR "z:/phd/ttrack/include/" )
#set( TTRACK_REL_LIBDIR "z:/phd/ttrack/build/Release" )
#set( TTRACK_DEB_LIBDIR "z:/phd/ttrack/build/Debug" )
#set( TTRACK_REL_LIBS ${TTRACK_REL_LIBDIR}/articulated_model.obj
#                     ${TTRACK_REL_LIBDIR}/handler.obj
#                     ${TTRACK_REL_LIBDIR}/model.obj
#                     )
                     
#set( TTRACK_DEB_LIBS ${TTRACK_DEB_LIBDIR}/articulated_model.obj
#                     ${TTRACK_DEB_LIBDIR}/handler.obj
#                     ${TTRACK_DEB_LIBDIR}/model.obj
#                     )

## Header only includes 
set(
  HEADERS
  ${INCDIR}/camera.hpp ${INCDIR}/config_reader.hpp
  ${INCDIR}/pose_grabber.hpp ${INCDIR}/davinci.hpp
  ${INCDIR}/resources.hpp ${INCDIR}/snippets.hpp ${INCDIR}/trackable.hpp
  ${INCDIR}/vizApp.hpp 
  ${INCDIR}/model.hpp
)

## Store list of source files
set( SOURCES camera.cpp davinci.cpp pose_grabber.cpp trackable.cpp snippets.cpp vizApp.cpp model.cpp )

#######################################################
## User build options

#example
#option(BUILD_PANGOLIN_GUI "Build support for Pangolin GUI" ON)
#if(BUILD_PANGOLIN_GUI)
#  list(APPEND HEADERS
#    ${INCDIR}/display.h ${INCDIR}/display_internal.h 
#    ${INCDIR}/view.h ${INCDIR}/viewport.h
#    ${INCDIR}/handler.h ${INCDIR}/handler_enums.h
#    ${INCDIR}/handler_glbuffer.h
#    ${INCDIR}/opengl_render_state.h
#    ${INCDIR}/datalog.h
#    ${INCDIR}/glfont.h ${INCDIR}/glpangoglu.h
#  )
#  list(APPEND SOURCES
#    display.cpp view.cpp viewport.cpp
#   handler.cpp handler_glbuffer.cpp
#    opengl_render_state.cpp
#    datalog.cpp
#    glfont.cpp glpangoglu.cpp
#  )
#  if(NOT HAVE_GLES OR HAVE_GLES_2)
#    list(APPEND HEADERS ${INCDIR}/plotter.h)
#    list(APPEND SOURCES plotter.cpp)
#  endif()
#endif()

#######################################################
## Setup required includes / link info

# Project headers trump everything (including any potentially installed Pangolin)
list(APPEND LIB_INC_DIR  "${PROJECT_SOURCE_DIR}/include;${CMAKE_CURRENT_BINARY_DIR}/include" )

#Boost
set(Boost_USE_STATIC_LIBS OFF) # only find static libs
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(BOOST_ALL_DYN_LINK ON)
find_package(Boost REQUIRED QUIET)
list(APPEND USER_INC  "${Boost_INCLUDE_DIRS}" )
list(APPEND LINK_LIBS "${Boost_LIBRARIES}" )
list(APPEND USER_LIB "${Boost_LIBRARY_DIR}" )

find_package(OpenCV REQUIRED)
list(APPEND USER_INC "${OpenCV_INCLUDE_DIRS}" )
list(APPEND LINK_LIBS "${OpenCV_LIBRARIES}" )

find_package(Cinder REQUIRED)
list(APPEND USER_INC "${CINDER_INCLUDE_DIRS}" )
list(APPEND LINK_LIBS "${CINDER_LIBRARIES}" )

#list(APPEND USER_INC "${TTRACK_INCDIR}" )
#list(APPEND LINK_LIBS "${TTRACK_DEB_LIBS}" )

#######################################################
## Add Libraries / Include Directories / Link directories

include_directories( ${LIB_INC_DIR} )
include_directories( ${USER_INC} )
link_directories( ${USER_LIB} )

#if(_WIN_)
add_executable(${BINARY_NAME} WIN32 ${MAIN_FILE} ${SOURCES} ${HEADERS} )
#else(_WIN_)
#add_executable(${BINARY_NAME} ${MAIN_FILE} ${SOURCES} ${HEADERS} )
#endif

#if(MSVC)
#    set(VS_IGNORE_LIBRARY_FLAGS "/NODEFAULTLIB:msvcprt.lib #/NODEFAULTLIB:msvcrt.lib")
    #SET_TARGET_PROPERTIES(${BINARY_NAME} PROPERTIES LINK_FLAGS ${VS_IGNORE_LIBRARY_FLAGS})
    #SET_TARGET_PROPERTIES(${BINARY_NAME} PROPERTIES ENABLE_EXPORTS true)
#endif()

target_link_libraries(${BINARY_NAME} ${LINK_LIBS})



