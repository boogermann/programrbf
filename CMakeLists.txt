cmake_minimum_required(VERSION 3.10)

project(rbfloader
		VERSION "0.1.0"
		DESCRIPTION "FPGA RBF Loader for ARM Linux"
		HOMEPAGE_URL "https://github.com/raetro-sigs/rbfloader"
		)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

configure_file(include/version.h.in version.h)

include_directories(${THREAD_INLCUDE_DIRS} include ${PROJECT_BINARY_DIR})

FILE(GLOB Sources src/*.cpp src/*.c) #Assume we want to include everything from src/

add_executable(${PROJECT_NAME}
			   ${Sources}
			   pkg/rbfloader.conf src/jtag.cpp include/jtag.h)

set_target_properties(${PROJECT_NAME} PROPERTIES
					  CXX_STANDARD 14
					  C_STANDARD 99
					  )

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} m)

install(
	TARGETS ${PROJECT_NAME}
	COMPONENT binaries
	RUNTIME DESTINATION "bin/"
	LIBRARY DESTINATION "lib/"
)

install(
	FILES pkg/rbfloader.conf
	COMPONENT config
	DESTINATION "etc/"
)

SET(CPACK_GENERATOR "DEB")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "sig-packages@raetro.org") #required
INCLUDE(CPack)