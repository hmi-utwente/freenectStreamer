CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)

if(WIN32 AND NOT MINGW)
  if(NOT DEFINED CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX "d")
  endif()
endif()


IF(NOT DEFINED CMAKE_BUILD_TYPE)
  # No effect for multi-configuration generators (e.g. for Visual Studio)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose: RelWithDebInfo Release Debug MinSizeRel None")
ENDIF()

PROJECT(freenectStreamer_project)
SET(CMAKE_CXX_STANDARD 11)

SET(MY_DIR ${PROJECT_SOURCE_DIR})
SET(DEPENDS_DIR "${MY_DIR}/../libfreenect2/depends" CACHE STRING "libfreenect2 Dependency directory")
SET(STREAMER_DEPENDS_DIR "${MY_DIR}/depends" CACHE STRING "freenectStreamer Dependency directory")

OPTION(ENABLE_OPENGL "Enable OpenGL support" ON)

# The example build system is standalone and will work out-of-tree with these files copied
SET(freenect2_ROOT_DIR ${MY_DIR}/../libfreenect2)
SET(flextGL_SOURCES ${freenect2_ROOT_DIR}/src/flextGL.cpp)
SET(flextGL_INCLUDE_DIRS ${freenect2_ROOT_DIR}/src) # for flextGL.h

FIND_PACKAGE(PkgConfig)
LIST(APPEND CMAKE_MODULE_PATH "${freenect2_ROOT_DIR}/cmake_modules" ) # freenect2 Modules
LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules" ) # freenectStreamer Modules

FIND_PACKAGE(freenect2 REQUIRED)
FIND_PACKAGE(ASIO REQUIRED)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DASIO_STANDALONE=YES" )

INCLUDE_DIRECTORIES(${freenect2_INCLUDE_DIR})    
INCLUDE_DIRECTORIES(${ASIO_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/headers)      

SET(freenectStreamer_src ${PROJECT_SOURCE_DIR}/src/freenectStreamer.cpp)

if(WIN32)
  SET(freenectStreamer_LIBRARIES optimized ${freenect2_LIBRARIES} debug ${freenect2_LIBRARIES_D})
  SET(freenectStreamer_DLLS optimized ${freenect2_DLLS} debug ${freenect2_DLLS_D})
ELSE()
  SET(freenectStreamer_LIBRARIES ${freenect2_LIBRARIES})
  SET(freenectStreamer_DLLS ${freenect2_DLLS})
ENDIF()

IF(ENABLE_OPENGL)
  FIND_PACKAGE(GLFW3)
  FIND_PACKAGE(OpenGL)
  IF(GLFW3_FOUND AND OPENGL_FOUND)
    INCLUDE_DIRECTORIES(
      ${GLFW3_INCLUDE_DIRS}
      ${flextGL_INCLUDE_DIRS}
    )

    LIST(APPEND freenectStreamer_DLLS ${GLFW3_DLL})
    LIST(APPEND freenectStreamer_src
      ${PROJECT_SOURCE_DIR}/src/viewer.cpp
      ${flextGL_SOURCES}
    )
    LIST(APPEND freenectStreamer_LIBRARIES
      ${GLFW3_LIBRARIES}
      ${OPENGL_gl_LIBRARY}
    )
    ADD_DEFINITIONS(-DEXAMPLES_WITH_OPENGL_SUPPORT=1)
  ENDIF()
ENDIF(ENABLE_OPENGL)

ADD_EXECUTABLE(freenectStreamer
  ${freenectStreamer_src}# ${HEADERS}
)

TARGET_LINK_LIBRARIES(freenectStreamer
  ${freenectStreamer_LIBRARIES}
)

#SET(CMAKE_CXX_FLAGS "-Wno-error=deprecated-declarations")