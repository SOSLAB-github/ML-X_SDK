cmake_minimum_required (VERSION 3.10)

# project name and version
project (libsoslab_ml VERSION 230426)

# configuration and make file create
set(CMAKE_VERBOSE_MAKEFILE true)

# directory variable
set(ML_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(ML_INCLUDE_DIR ${ML_ROOT_DIR}/include)
set(ML_SRC_DIR ${ML_ROOT_DIR}/src)
set(CORE_INCLUDE_DIR ${ML_ROOT_DIR}/../libsoslab_core/include)
set(ASIO_INCLUDE_DIR ${ML_ROOT_DIR}/../libsoslab_core/include/core)
set(OUT_DIR ${ML_ROOT_DIR}/../output/${CMAKE_BUILD_TYPE})

# output name variable
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
	set(OUTPUT_NAME soslab_ml)
else()
	set(OUTPUT_NAME libsoslab_ml)
endif()

# Append "d" to the name of the compiled library
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Set debug library postfix")
# file variable
set(HEADER_FILES
	${ML_INCLUDE_DIR}/ml/libsoslab_ml.h
	)
set(SRC_FILES
	${ML_SRC_DIR}/libsoslab_ml.cpp
	)

# output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${ML_ROOT_DIR}/../output/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${ML_ROOT_DIR}/../output/${CMAKE_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ML_ROOT_DIR}/../output/${CMAKE_BUILD_TYPE})

# library dir
link_directories (${OUT_DIR})

# add_library
if(NOT STATIC)
	add_library(${OUTPUT_NAME} SHARED ${HEADER_FILES} ${SRC_FILES})
else()
	add_library(${OUTPUT_NAME} STATIC ${HEADER_FILES} ${SRC_FILES})
endif()

# include
target_include_directories(${OUTPUT_NAME} PUBLIC ${CORE_INCLUDE_DIR})
target_include_directories(${OUTPUT_NAME} PUBLIC ${ASIO_INCLUDE_DIR})
target_include_directories(${OUTPUT_NAME} PUBLIC ${ML_INCLUDE_DIR})

# library
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
	target_link_libraries(${OUTPUT_NAME} 
		debug soslab_cored
		optimized soslab_core)
else()
	target_link_libraries(${OUTPUT_NAME} 
		debug libsoslab_cored
		optimized libsoslab_core)
endif()

# pre-definition
target_compile_definitions(${OUTPUT_NAME} PRIVATE SOSLABAPI_EXPORTS)

# Install library
install(TARGETS ${OUTPUT_NAME}
  LIBRARY DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
  ARCHIVE DESTINATION ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
  RUNTIME DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
  )
