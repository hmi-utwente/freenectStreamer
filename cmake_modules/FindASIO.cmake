# FindASIO.cmake
# - ASIO_INCLUDE_DIRS_FOUND
# - ASIO_INCLUDE_DIRS_INCLUDE_DIRS

find_path(ASIO_INCLUDE_DIRS
  asio.hpp
  DOC "Found asio include directory"
  PATHS
    "${STREAMER_DEPENDS_DIR}/asio/asio"
	"/usr/include" "/usr/local"
	"/opt/local"
  PATH_SUFFIXES
    include
)

IF(ASIO_INCLUDE_DIRS)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DASIO_STANDALONE=YES" )	
#INCLUDE(CheckCSourceCompiles)
#set(CMAKE_REQUIRED_INCLUDES ${asio_INCLUDE_DIRS})
#check_c_source_compiles("#include <asio.hpp>\nint main(void) { return 0; }" ASIO_WORKS)
#set(CMAKE_REQUIRED_DEFINITIONS)
#set(CMAKE_REQUIRED_INCLUDES)
ENDIF()
  
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(ASIO FOUND_VAR ASIO_FOUND
  REQUIRED_VARS ASIO_INCLUDE_DIRS)
  
  