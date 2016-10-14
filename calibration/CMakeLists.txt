cmake_minimum_required (VERSION 2.6)
project(nikipilot)
find_package(Eigen3 REQUIRED)
SET(CMAKE_C_COMPILER /usr/bin/arm-linux-gnueabi-gcc)
SET(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabi-g++) 

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
	${EIGEN3_INCLUDE_DIR}
	rosserial
        
        
)

file(GLOB SRC_FILES
	kalman.cpp
        control/*.cpp
        PCA9685/*.cpp
	MPU6050/*.cpp
	pevents/*.cpp
	I2C/*.cpp
	threads/*.cpp
        quad_threads.cpp
        )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libstdc++ -O3 -Wextra -pthread") 


add_definitions(-DPOSIX_ENV)
add_definitions(-DSUPPRESS_RDWR_ERROR_CHECKING)
add_definitions(-DSUPPRESS_TIMER_SLIP_WARNINGS)
add_executable(calib.out ${SRC_FILES})
target_link_libraries(calib.out pthread)
target_link_libraries(calib.out rt)

