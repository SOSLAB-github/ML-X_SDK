cmake_minimum_required (VERSION 3.10)

# project name and version
project (test_ml VERSION 230426)

# configuration and make file create
set(CMAKE_VERBOSE_MAKEFILE true)

# directory variable
set(TEST_ML_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(TEST_ML_SRC_DIR ${TEST_ML_ROOT_DIR})
set(CORE_INCLUDE_DIR ${TEST_ML_ROOT_DIR}/../../libsoslab_core/include)
set(ASIO_INCLUDE_DIR ${TEST_ML_ROOT_DIR}/../../libsoslab_core/include/core)
set(ML_INCLUDE_DIR ${TEST_ML_ROOT_DIR}/../../libsoslab_ml/include)
set(OUT_DIR ${TEST_ML_ROOT_DIR}/../../output/${CMAKE_BUILD_TYPE})

# output name variable
set(OUTPUT_NAME test_ml)
# file variable
set(SRC_FILES
	${TEST_ML_SRC_DIR}/test_ml.cpp
	)

# output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${TEST_ML_ROOT_DIR}/../../output/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${TEST_ML_ROOT_DIR}/../../output/${CMAKE_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${TEST_ML_ROOT_DIR}/../../output/${CMAKE_BUILD_TYPE})

find_package(Threads REQUIRED)

# library dir
link_directories (${OUT_DIR})

# add_executable
add_executable(${OUTPUT_NAME} ${SRC_FILES})

# include
target_include_directories(${OUTPUT_NAME} PUBLIC ${CORE_INCLUDE_DIR})
target_include_directories(${OUTPUT_NAME} PUBLIC ${ASIO_INCLUDE_DIR})
target_include_directories(${OUTPUT_NAME} PUBLIC ${ML_INCLUDE_DIR})

# library
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
	target_link_libraries(${OUTPUT_NAME} 
		debug soslab_cored
		debug soslab_mld
		optimized soslab_core
		optimized soslab_ml
		Threads::Threads)
else()
	target_link_libraries(${OUTPUT_NAME} 
		debug libsoslab_cored
		debug libsoslab_mld
		optimized libsoslab_core
		optimized libsoslab_ml
		Threads::Threads)
endif()

# Install
install(TARGETS ${OUTPUT_NAME}
LIBRARY DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
ARCHIVE DESTINATION ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
RUNTIME DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)