# FindFreenect2.cmake
# - freenect2_FOUND
# - freenect2_LIBRARIES
# - freenect2_LIBRARIES_D
# - freenect2_DLLS
# - freenect2_DLLS_D
# - freenect2_INCLUDE_DIRS
  
MESSAGE(STATUS "FindFreenect2 ...")
  
FIND_LIBRARY(freenect2_LIBRARY freenect2
  DOC "Found freenect2 library"
  PATHS ${freenect2_ROOT_DIR}/build
	"/usr/local"
	"/opt/local"
	NO_DEFAULT_PATH 
  PATH_SUFFIXES
    "lib/Release" "lib"
)

FIND_LIBRARY(freenect2_LIBRARY_D freenect2d
  DOC "Found freenect2d library"
  PATHS ${freenect2_ROOT_DIR}/build
	"/usr/local"
	"/opt/local"
	NO_DEFAULT_PATH
  PATH_SUFFIXES
    "lib/Debug" "lib"
)

SET(freenect2_LIBRARIES ${freenect2_LIBRARY} )
SET(freenect2_LIBRARIES_D ${freenect2_LIBRARY_D} )


IF(WIN32)
FIND_FILE(freenect2_DLL freenect2.dll
  DOC "Found freenect2 dll"
  PATHS ${freenect2_ROOT_DIR}/build
	NO_DEFAULT_PATH
  PATH_SUFFIXES
    "bin/Release" "bin"
)

FIND_FILE(freenect2_DLL_D freenect2d.dll
  DOC "Found freenect2d dll"
  PATHS ${freenect2_ROOT_DIR}/build
	NO_DEFAULT_PATH
  PATH_SUFFIXES
    "bin/Debug" "bin"
)

FILE(COPY ${freenect2_DLL} DESTINATION ${CMAKE_BINARY_DIR}/Release)
SET(freenect2_DLL ${CMAKE_BINARY_DIR}/Release/freenect2.dll)
FILE(COPY ${freenect2_DLL_D} DESTINATION ${CMAKE_BINARY_DIR}/Debug)
SET(freenect2_DLL ${CMAKE_BINARY_DIR}/Debug/freenect2d.dll)

SET(freenect2_DLLS ${freenect2_DLL} )
SET(freenect2_DLLS_D ${freenect2_DLL_D} )
ENDIF()

FIND_PATH(freenect2_INCLUDE_DIR libfreenect2/libfreenect2.hpp
  DOC "Found freenect2 include directory"
  PATHS ${freenect2_ROOT_DIR}/build/install/include
	"/usr/include" "/usr/local"
	"/opt/local"
)

SET(freenect2_INCLUDE_DIRS ${freenect2_INCLUDE_DIR})

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(freenect2 FOUND_VAR freenect2_FOUND
  REQUIRED_VARS freenect2_LIBRARIES freenect2_INCLUDE_DIRS)