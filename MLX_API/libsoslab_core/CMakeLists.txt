cmake_minimum_required (VERSION 3.10)

# project name and version
project (libsoslab_core VERSION 230426)

# configuration and make file create
set(CMAKE_VERBOSE_MAKEFILE true)

# directory variable
set(CORE_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(CORE_INCLUDE_DIR ${CORE_ROOT_DIR}/include)
set(ASIO_INCLUDE_DIR ${CORE_ROOT_DIR}/include/core)
set(CORE_SRC_DIR ${CORE_ROOT_DIR}/src)

# output name variable
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
	set(OUTPUT_NAME soslab_core)
else()
	set(OUTPUT_NAME libsoslab_core)
endif()

# Append "d" to the name of the compiled library
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Set debug library postfix")
# file variable
set(HEADER_FILES
	${CORE_INCLUDE_DIR}/soslab_typedef.h
	${CORE_INCLUDE_DIR}/core/asio.hpp
	${CORE_INCLUDE_DIR}/core/ext_interfaces.h
	${CORE_INCLUDE_DIR}/core/fifo.hpp
	${CORE_INCLUDE_DIR}/core/json.hpp
	${CORE_INCLUDE_DIR}/core/logger.h
	)
set(SRC_FILES
	${CORE_SRC_DIR}/ext_interfaces.cpp
	${CORE_SRC_DIR}/logger.cpp
	)

# output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CORE_ROOT_DIR}/../output/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CORE_ROOT_DIR}/../output/${CMAKE_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CORE_ROOT_DIR}/../output/${CMAKE_BUILD_TYPE})

# add_library
if(NOT STATIC)
	add_library(${OUTPUT_NAME} SHARED ${HEADER_FILES} ${SRC_FILES})
else()
	add_library(${OUTPUT_NAME} STATIC ${HEADER_FILES} ${SRC_FILES})
endif()

# include
target_include_directories(${OUTPUT_NAME} PUBLIC ${CORE_INCLUDE_DIR})
target_include_directories(${OUTPUT_NAME} PUBLIC ${ASIO_INCLUDE_DIR})
target_compile_definitions(${OUTPUT_NAME} PRIVATE SOSLABAPI_EXPORTS)

# Install library
install(TARGETS ${OUTPUT_NAME}
  LIBRARY DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
  ARCHIVE DESTINATION ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
  RUNTIME DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
  )
