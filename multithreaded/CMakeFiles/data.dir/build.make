# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sahabi/AGNC/Quad/multithreaded

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sahabi/AGNC/Quad/multithreaded

# Include any dependencies generated for this target.
include CMakeFiles/data.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/data.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/data.dir/flags.make

CMakeFiles/data.dir/kalman.cpp.o: CMakeFiles/data.dir/flags.make
CMakeFiles/data.dir/kalman.cpp.o: kalman.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sahabi/AGNC/Quad/multithreaded/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/data.dir/kalman.cpp.o"
	/usr/bin/arm-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data.dir/kalman.cpp.o -c /home/sahabi/AGNC/Quad/multithreaded/kalman.cpp

CMakeFiles/data.dir/kalman.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data.dir/kalman.cpp.i"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sahabi/AGNC/Quad/multithreaded/kalman.cpp > CMakeFiles/data.dir/kalman.cpp.i

CMakeFiles/data.dir/kalman.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data.dir/kalman.cpp.s"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sahabi/AGNC/Quad/multithreaded/kalman.cpp -o CMakeFiles/data.dir/kalman.cpp.s

CMakeFiles/data.dir/kalman.cpp.o.requires:
.PHONY : CMakeFiles/data.dir/kalman.cpp.o.requires

CMakeFiles/data.dir/kalman.cpp.o.provides: CMakeFiles/data.dir/kalman.cpp.o.requires
	$(MAKE) -f CMakeFiles/data.dir/build.make CMakeFiles/data.dir/kalman.cpp.o.provides.build
.PHONY : CMakeFiles/data.dir/kalman.cpp.o.provides

CMakeFiles/data.dir/kalman.cpp.o.provides.build: CMakeFiles/data.dir/kalman.cpp.o

CMakeFiles/data.dir/control/MathFuncs.cpp.o: CMakeFiles/data.dir/flags.make
CMakeFiles/data.dir/control/MathFuncs.cpp.o: control/MathFuncs.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sahabi/AGNC/Quad/multithreaded/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/data.dir/control/MathFuncs.cpp.o"
	/usr/bin/arm-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data.dir/control/MathFuncs.cpp.o -c /home/sahabi/AGNC/Quad/multithreaded/control/MathFuncs.cpp

CMakeFiles/data.dir/control/MathFuncs.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data.dir/control/MathFuncs.cpp.i"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sahabi/AGNC/Quad/multithreaded/control/MathFuncs.cpp > CMakeFiles/data.dir/control/MathFuncs.cpp.i

CMakeFiles/data.dir/control/MathFuncs.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data.dir/control/MathFuncs.cpp.s"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sahabi/AGNC/Quad/multithreaded/control/MathFuncs.cpp -o CMakeFiles/data.dir/control/MathFuncs.cpp.s

CMakeFiles/data.dir/control/MathFuncs.cpp.o.requires:
.PHONY : CMakeFiles/data.dir/control/MathFuncs.cpp.o.requires

CMakeFiles/data.dir/control/MathFuncs.cpp.o.provides: CMakeFiles/data.dir/control/MathFuncs.cpp.o.requires
	$(MAKE) -f CMakeFiles/data.dir/build.make CMakeFiles/data.dir/control/MathFuncs.cpp.o.provides.build
.PHONY : CMakeFiles/data.dir/control/MathFuncs.cpp.o.provides

CMakeFiles/data.dir/control/MathFuncs.cpp.o.provides.build: CMakeFiles/data.dir/control/MathFuncs.cpp.o

CMakeFiles/data.dir/control/MatricesAndVectors.cpp.o: CMakeFiles/data.dir/flags.make
CMakeFiles/data.dir/control/MatricesAndVectors.cpp.o: control/MatricesAndVectors.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sahabi/AGNC/Quad/multithreaded/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/data.dir/control/MatricesAndVectors.cpp.o"
	/usr/bin/arm-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data.dir/control/MatricesAndVectors.cpp.o -c /home/sahabi/AGNC/Quad/multithreaded/control/MatricesAndVectors.cpp

CMakeFiles/data.dir/control/MatricesAndVectors.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data.dir/control/MatricesAndVectors.cpp.i"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sahabi/AGNC/Quad/multithreaded/control/MatricesAndVectors.cpp > CMakeFiles/data.dir/control/MatricesAndVectors.cpp.i

CMakeFiles/data.dir/control/MatricesAndVectors.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data.dir/control/MatricesAndVectors.cpp.s"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sahabi/AGNC/Quad/multithreaded/control/MatricesAndVectors.cpp -o CMakeFiles/data.dir/control/MatricesAndVectors.cpp.s

CMakeFiles/data.dir/control/MatricesAndVectors.cpp.o.requires:
.PHONY : CMakeFiles/data.dir/control/MatricesAndVectors.cpp.o.requires

CMakeFiles/data.dir/control/MatricesAndVectors.cpp.o.provides: CMakeFiles/data.dir/control/MatricesAndVectors.cpp.o.requires
	$(MAKE) -f CMakeFiles/data.dir/build.make CMakeFiles/data.dir/control/MatricesAndVectors.cpp.o.provides.build
.PHONY : CMakeFiles/data.dir/control/MatricesAndVectors.cpp.o.provides

CMakeFiles/data.dir/control/MatricesAndVectors.cpp.o.provides.build: CMakeFiles/data.dir/control/MatricesAndVectors.cpp.o

CMakeFiles/data.dir/control/AttitudeControl.cpp.o: CMakeFiles/data.dir/flags.make
CMakeFiles/data.dir/control/AttitudeControl.cpp.o: control/AttitudeControl.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sahabi/AGNC/Quad/multithreaded/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/data.dir/control/AttitudeControl.cpp.o"
	/usr/bin/arm-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data.dir/control/AttitudeControl.cpp.o -c /home/sahabi/AGNC/Quad/multithreaded/control/AttitudeControl.cpp

CMakeFiles/data.dir/control/AttitudeControl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data.dir/control/AttitudeControl.cpp.i"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sahabi/AGNC/Quad/multithreaded/control/AttitudeControl.cpp > CMakeFiles/data.dir/control/AttitudeControl.cpp.i

CMakeFiles/data.dir/control/AttitudeControl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data.dir/control/AttitudeControl.cpp.s"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sahabi/AGNC/Quad/multithreaded/control/AttitudeControl.cpp -o CMakeFiles/data.dir/control/AttitudeControl.cpp.s

CMakeFiles/data.dir/control/AttitudeControl.cpp.o.requires:
.PHONY : CMakeFiles/data.dir/control/AttitudeControl.cpp.o.requires

CMakeFiles/data.dir/control/AttitudeControl.cpp.o.provides: CMakeFiles/data.dir/control/AttitudeControl.cpp.o.requires
	$(MAKE) -f CMakeFiles/data.dir/build.make CMakeFiles/data.dir/control/AttitudeControl.cpp.o.provides.build
.PHONY : CMakeFiles/data.dir/control/AttitudeControl.cpp.o.provides

CMakeFiles/data.dir/control/AttitudeControl.cpp.o.provides.build: CMakeFiles/data.dir/control/AttitudeControl.cpp.o

CMakeFiles/data.dir/control/QuatRotEuler.cpp.o: CMakeFiles/data.dir/flags.make
CMakeFiles/data.dir/control/QuatRotEuler.cpp.o: control/QuatRotEuler.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sahabi/AGNC/Quad/multithreaded/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/data.dir/control/QuatRotEuler.cpp.o"
	/usr/bin/arm-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data.dir/control/QuatRotEuler.cpp.o -c /home/sahabi/AGNC/Quad/multithreaded/control/QuatRotEuler.cpp

CMakeFiles/data.dir/control/QuatRotEuler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data.dir/control/QuatRotEuler.cpp.i"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sahabi/AGNC/Quad/multithreaded/control/QuatRotEuler.cpp > CMakeFiles/data.dir/control/QuatRotEuler.cpp.i

CMakeFiles/data.dir/control/QuatRotEuler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data.dir/control/QuatRotEuler.cpp.s"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sahabi/AGNC/Quad/multithreaded/control/QuatRotEuler.cpp -o CMakeFiles/data.dir/control/QuatRotEuler.cpp.s

CMakeFiles/data.dir/control/QuatRotEuler.cpp.o.requires:
.PHONY : CMakeFiles/data.dir/control/QuatRotEuler.cpp.o.requires

CMakeFiles/data.dir/control/QuatRotEuler.cpp.o.provides: CMakeFiles/data.dir/control/QuatRotEuler.cpp.o.requires
	$(MAKE) -f CMakeFiles/data.dir/build.make CMakeFiles/data.dir/control/QuatRotEuler.cpp.o.provides.build
.PHONY : CMakeFiles/data.dir/control/QuatRotEuler.cpp.o.provides

CMakeFiles/data.dir/control/QuatRotEuler.cpp.o.provides.build: CMakeFiles/data.dir/control/QuatRotEuler.cpp.o

CMakeFiles/data.dir/control/PID_3DOF.cpp.o: CMakeFiles/data.dir/flags.make
CMakeFiles/data.dir/control/PID_3DOF.cpp.o: control/PID_3DOF.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sahabi/AGNC/Quad/multithreaded/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/data.dir/control/PID_3DOF.cpp.o"
	/usr/bin/arm-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data.dir/control/PID_3DOF.cpp.o -c /home/sahabi/AGNC/Quad/multithreaded/control/PID_3DOF.cpp

CMakeFiles/data.dir/control/PID_3DOF.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data.dir/control/PID_3DOF.cpp.i"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sahabi/AGNC/Quad/multithreaded/control/PID_3DOF.cpp > CMakeFiles/data.dir/control/PID_3DOF.cpp.i

CMakeFiles/data.dir/control/PID_3DOF.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data.dir/control/PID_3DOF.cpp.s"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sahabi/AGNC/Quad/multithreaded/control/PID_3DOF.cpp -o CMakeFiles/data.dir/control/PID_3DOF.cpp.s

CMakeFiles/data.dir/control/PID_3DOF.cpp.o.requires:
.PHONY : CMakeFiles/data.dir/control/PID_3DOF.cpp.o.requires

CMakeFiles/data.dir/control/PID_3DOF.cpp.o.provides: CMakeFiles/data.dir/control/PID_3DOF.cpp.o.requires
	$(MAKE) -f CMakeFiles/data.dir/build.make CMakeFiles/data.dir/control/PID_3DOF.cpp.o.provides.build
.PHONY : CMakeFiles/data.dir/control/PID_3DOF.cpp.o.provides

CMakeFiles/data.dir/control/PID_3DOF.cpp.o.provides.build: CMakeFiles/data.dir/control/PID_3DOF.cpp.o

CMakeFiles/data.dir/PCA9685/pca9685.cpp.o: CMakeFiles/data.dir/flags.make
CMakeFiles/data.dir/PCA9685/pca9685.cpp.o: PCA9685/pca9685.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sahabi/AGNC/Quad/multithreaded/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/data.dir/PCA9685/pca9685.cpp.o"
	/usr/bin/arm-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data.dir/PCA9685/pca9685.cpp.o -c /home/sahabi/AGNC/Quad/multithreaded/PCA9685/pca9685.cpp

CMakeFiles/data.dir/PCA9685/pca9685.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data.dir/PCA9685/pca9685.cpp.i"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sahabi/AGNC/Quad/multithreaded/PCA9685/pca9685.cpp > CMakeFiles/data.dir/PCA9685/pca9685.cpp.i

CMakeFiles/data.dir/PCA9685/pca9685.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data.dir/PCA9685/pca9685.cpp.s"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sahabi/AGNC/Quad/multithreaded/PCA9685/pca9685.cpp -o CMakeFiles/data.dir/PCA9685/pca9685.cpp.s

CMakeFiles/data.dir/PCA9685/pca9685.cpp.o.requires:
.PHONY : CMakeFiles/data.dir/PCA9685/pca9685.cpp.o.requires

CMakeFiles/data.dir/PCA9685/pca9685.cpp.o.provides: CMakeFiles/data.dir/PCA9685/pca9685.cpp.o.requires
	$(MAKE) -f CMakeFiles/data.dir/build.make CMakeFiles/data.dir/PCA9685/pca9685.cpp.o.provides.build
.PHONY : CMakeFiles/data.dir/PCA9685/pca9685.cpp.o.provides

CMakeFiles/data.dir/PCA9685/pca9685.cpp.o.provides.build: CMakeFiles/data.dir/PCA9685/pca9685.cpp.o

CMakeFiles/data.dir/MPU6050/MPU6050.cpp.o: CMakeFiles/data.dir/flags.make
CMakeFiles/data.dir/MPU6050/MPU6050.cpp.o: MPU6050/MPU6050.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sahabi/AGNC/Quad/multithreaded/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/data.dir/MPU6050/MPU6050.cpp.o"
	/usr/bin/arm-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data.dir/MPU6050/MPU6050.cpp.o -c /home/sahabi/AGNC/Quad/multithreaded/MPU6050/MPU6050.cpp

CMakeFiles/data.dir/MPU6050/MPU6050.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data.dir/MPU6050/MPU6050.cpp.i"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sahabi/AGNC/Quad/multithreaded/MPU6050/MPU6050.cpp > CMakeFiles/data.dir/MPU6050/MPU6050.cpp.i

CMakeFiles/data.dir/MPU6050/MPU6050.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data.dir/MPU6050/MPU6050.cpp.s"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sahabi/AGNC/Quad/multithreaded/MPU6050/MPU6050.cpp -o CMakeFiles/data.dir/MPU6050/MPU6050.cpp.s

CMakeFiles/data.dir/MPU6050/MPU6050.cpp.o.requires:
.PHONY : CMakeFiles/data.dir/MPU6050/MPU6050.cpp.o.requires

CMakeFiles/data.dir/MPU6050/MPU6050.cpp.o.provides: CMakeFiles/data.dir/MPU6050/MPU6050.cpp.o.requires
	$(MAKE) -f CMakeFiles/data.dir/build.make CMakeFiles/data.dir/MPU6050/MPU6050.cpp.o.provides.build
.PHONY : CMakeFiles/data.dir/MPU6050/MPU6050.cpp.o.provides

CMakeFiles/data.dir/MPU6050/MPU6050.cpp.o.provides.build: CMakeFiles/data.dir/MPU6050/MPU6050.cpp.o

CMakeFiles/data.dir/pevents/pevents.cpp.o: CMakeFiles/data.dir/flags.make
CMakeFiles/data.dir/pevents/pevents.cpp.o: pevents/pevents.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sahabi/AGNC/Quad/multithreaded/CMakeFiles $(CMAKE_PROGRESS_9)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/data.dir/pevents/pevents.cpp.o"
	/usr/bin/arm-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data.dir/pevents/pevents.cpp.o -c /home/sahabi/AGNC/Quad/multithreaded/pevents/pevents.cpp

CMakeFiles/data.dir/pevents/pevents.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data.dir/pevents/pevents.cpp.i"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sahabi/AGNC/Quad/multithreaded/pevents/pevents.cpp > CMakeFiles/data.dir/pevents/pevents.cpp.i

CMakeFiles/data.dir/pevents/pevents.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data.dir/pevents/pevents.cpp.s"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sahabi/AGNC/Quad/multithreaded/pevents/pevents.cpp -o CMakeFiles/data.dir/pevents/pevents.cpp.s

CMakeFiles/data.dir/pevents/pevents.cpp.o.requires:
.PHONY : CMakeFiles/data.dir/pevents/pevents.cpp.o.requires

CMakeFiles/data.dir/pevents/pevents.cpp.o.provides: CMakeFiles/data.dir/pevents/pevents.cpp.o.requires
	$(MAKE) -f CMakeFiles/data.dir/build.make CMakeFiles/data.dir/pevents/pevents.cpp.o.provides.build
.PHONY : CMakeFiles/data.dir/pevents/pevents.cpp.o.provides

CMakeFiles/data.dir/pevents/pevents.cpp.o.provides.build: CMakeFiles/data.dir/pevents/pevents.cpp.o

CMakeFiles/data.dir/I2C/i2c.cpp.o: CMakeFiles/data.dir/flags.make
CMakeFiles/data.dir/I2C/i2c.cpp.o: I2C/i2c.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sahabi/AGNC/Quad/multithreaded/CMakeFiles $(CMAKE_PROGRESS_10)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/data.dir/I2C/i2c.cpp.o"
	/usr/bin/arm-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data.dir/I2C/i2c.cpp.o -c /home/sahabi/AGNC/Quad/multithreaded/I2C/i2c.cpp

CMakeFiles/data.dir/I2C/i2c.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data.dir/I2C/i2c.cpp.i"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sahabi/AGNC/Quad/multithreaded/I2C/i2c.cpp > CMakeFiles/data.dir/I2C/i2c.cpp.i

CMakeFiles/data.dir/I2C/i2c.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data.dir/I2C/i2c.cpp.s"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sahabi/AGNC/Quad/multithreaded/I2C/i2c.cpp -o CMakeFiles/data.dir/I2C/i2c.cpp.s

CMakeFiles/data.dir/I2C/i2c.cpp.o.requires:
.PHONY : CMakeFiles/data.dir/I2C/i2c.cpp.o.requires

CMakeFiles/data.dir/I2C/i2c.cpp.o.provides: CMakeFiles/data.dir/I2C/i2c.cpp.o.requires
	$(MAKE) -f CMakeFiles/data.dir/build.make CMakeFiles/data.dir/I2C/i2c.cpp.o.provides.build
.PHONY : CMakeFiles/data.dir/I2C/i2c.cpp.o.provides

CMakeFiles/data.dir/I2C/i2c.cpp.o.provides.build: CMakeFiles/data.dir/I2C/i2c.cpp.o

CMakeFiles/data.dir/threads/keyboard_thread.cpp.o: CMakeFiles/data.dir/flags.make
CMakeFiles/data.dir/threads/keyboard_thread.cpp.o: threads/keyboard_thread.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sahabi/AGNC/Quad/multithreaded/CMakeFiles $(CMAKE_PROGRESS_11)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/data.dir/threads/keyboard_thread.cpp.o"
	/usr/bin/arm-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data.dir/threads/keyboard_thread.cpp.o -c /home/sahabi/AGNC/Quad/multithreaded/threads/keyboard_thread.cpp

CMakeFiles/data.dir/threads/keyboard_thread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data.dir/threads/keyboard_thread.cpp.i"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sahabi/AGNC/Quad/multithreaded/threads/keyboard_thread.cpp > CMakeFiles/data.dir/threads/keyboard_thread.cpp.i

CMakeFiles/data.dir/threads/keyboard_thread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data.dir/threads/keyboard_thread.cpp.s"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sahabi/AGNC/Quad/multithreaded/threads/keyboard_thread.cpp -o CMakeFiles/data.dir/threads/keyboard_thread.cpp.s

CMakeFiles/data.dir/threads/keyboard_thread.cpp.o.requires:
.PHONY : CMakeFiles/data.dir/threads/keyboard_thread.cpp.o.requires

CMakeFiles/data.dir/threads/keyboard_thread.cpp.o.provides: CMakeFiles/data.dir/threads/keyboard_thread.cpp.o.requires
	$(MAKE) -f CMakeFiles/data.dir/build.make CMakeFiles/data.dir/threads/keyboard_thread.cpp.o.provides.build
.PHONY : CMakeFiles/data.dir/threads/keyboard_thread.cpp.o.provides

CMakeFiles/data.dir/threads/keyboard_thread.cpp.o.provides.build: CMakeFiles/data.dir/threads/keyboard_thread.cpp.o

CMakeFiles/data.dir/threads/mpu_thread.cpp.o: CMakeFiles/data.dir/flags.make
CMakeFiles/data.dir/threads/mpu_thread.cpp.o: threads/mpu_thread.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sahabi/AGNC/Quad/multithreaded/CMakeFiles $(CMAKE_PROGRESS_12)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/data.dir/threads/mpu_thread.cpp.o"
	/usr/bin/arm-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data.dir/threads/mpu_thread.cpp.o -c /home/sahabi/AGNC/Quad/multithreaded/threads/mpu_thread.cpp

CMakeFiles/data.dir/threads/mpu_thread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data.dir/threads/mpu_thread.cpp.i"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sahabi/AGNC/Quad/multithreaded/threads/mpu_thread.cpp > CMakeFiles/data.dir/threads/mpu_thread.cpp.i

CMakeFiles/data.dir/threads/mpu_thread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data.dir/threads/mpu_thread.cpp.s"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sahabi/AGNC/Quad/multithreaded/threads/mpu_thread.cpp -o CMakeFiles/data.dir/threads/mpu_thread.cpp.s

CMakeFiles/data.dir/threads/mpu_thread.cpp.o.requires:
.PHONY : CMakeFiles/data.dir/threads/mpu_thread.cpp.o.requires

CMakeFiles/data.dir/threads/mpu_thread.cpp.o.provides: CMakeFiles/data.dir/threads/mpu_thread.cpp.o.requires
	$(MAKE) -f CMakeFiles/data.dir/build.make CMakeFiles/data.dir/threads/mpu_thread.cpp.o.provides.build
.PHONY : CMakeFiles/data.dir/threads/mpu_thread.cpp.o.provides

CMakeFiles/data.dir/threads/mpu_thread.cpp.o.provides.build: CMakeFiles/data.dir/threads/mpu_thread.cpp.o

CMakeFiles/data.dir/threads/print_thread.cpp.o: CMakeFiles/data.dir/flags.make
CMakeFiles/data.dir/threads/print_thread.cpp.o: threads/print_thread.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sahabi/AGNC/Quad/multithreaded/CMakeFiles $(CMAKE_PROGRESS_13)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/data.dir/threads/print_thread.cpp.o"
	/usr/bin/arm-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data.dir/threads/print_thread.cpp.o -c /home/sahabi/AGNC/Quad/multithreaded/threads/print_thread.cpp

CMakeFiles/data.dir/threads/print_thread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data.dir/threads/print_thread.cpp.i"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sahabi/AGNC/Quad/multithreaded/threads/print_thread.cpp > CMakeFiles/data.dir/threads/print_thread.cpp.i

CMakeFiles/data.dir/threads/print_thread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data.dir/threads/print_thread.cpp.s"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sahabi/AGNC/Quad/multithreaded/threads/print_thread.cpp -o CMakeFiles/data.dir/threads/print_thread.cpp.s

CMakeFiles/data.dir/threads/print_thread.cpp.o.requires:
.PHONY : CMakeFiles/data.dir/threads/print_thread.cpp.o.requires

CMakeFiles/data.dir/threads/print_thread.cpp.o.provides: CMakeFiles/data.dir/threads/print_thread.cpp.o.requires
	$(MAKE) -f CMakeFiles/data.dir/build.make CMakeFiles/data.dir/threads/print_thread.cpp.o.provides.build
.PHONY : CMakeFiles/data.dir/threads/print_thread.cpp.o.provides

CMakeFiles/data.dir/threads/print_thread.cpp.o.provides.build: CMakeFiles/data.dir/threads/print_thread.cpp.o

CMakeFiles/data.dir/threads/pca_thread.cpp.o: CMakeFiles/data.dir/flags.make
CMakeFiles/data.dir/threads/pca_thread.cpp.o: threads/pca_thread.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sahabi/AGNC/Quad/multithreaded/CMakeFiles $(CMAKE_PROGRESS_14)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/data.dir/threads/pca_thread.cpp.o"
	/usr/bin/arm-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data.dir/threads/pca_thread.cpp.o -c /home/sahabi/AGNC/Quad/multithreaded/threads/pca_thread.cpp

CMakeFiles/data.dir/threads/pca_thread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data.dir/threads/pca_thread.cpp.i"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sahabi/AGNC/Quad/multithreaded/threads/pca_thread.cpp > CMakeFiles/data.dir/threads/pca_thread.cpp.i

CMakeFiles/data.dir/threads/pca_thread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data.dir/threads/pca_thread.cpp.s"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sahabi/AGNC/Quad/multithreaded/threads/pca_thread.cpp -o CMakeFiles/data.dir/threads/pca_thread.cpp.s

CMakeFiles/data.dir/threads/pca_thread.cpp.o.requires:
.PHONY : CMakeFiles/data.dir/threads/pca_thread.cpp.o.requires

CMakeFiles/data.dir/threads/pca_thread.cpp.o.provides: CMakeFiles/data.dir/threads/pca_thread.cpp.o.requires
	$(MAKE) -f CMakeFiles/data.dir/build.make CMakeFiles/data.dir/threads/pca_thread.cpp.o.provides.build
.PHONY : CMakeFiles/data.dir/threads/pca_thread.cpp.o.provides

CMakeFiles/data.dir/threads/pca_thread.cpp.o.provides.build: CMakeFiles/data.dir/threads/pca_thread.cpp.o

CMakeFiles/data.dir/quad_threads.cpp.o: CMakeFiles/data.dir/flags.make
CMakeFiles/data.dir/quad_threads.cpp.o: quad_threads.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sahabi/AGNC/Quad/multithreaded/CMakeFiles $(CMAKE_PROGRESS_15)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/data.dir/quad_threads.cpp.o"
	/usr/bin/arm-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data.dir/quad_threads.cpp.o -c /home/sahabi/AGNC/Quad/multithreaded/quad_threads.cpp

CMakeFiles/data.dir/quad_threads.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data.dir/quad_threads.cpp.i"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sahabi/AGNC/Quad/multithreaded/quad_threads.cpp > CMakeFiles/data.dir/quad_threads.cpp.i

CMakeFiles/data.dir/quad_threads.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data.dir/quad_threads.cpp.s"
	/usr/bin/arm-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sahabi/AGNC/Quad/multithreaded/quad_threads.cpp -o CMakeFiles/data.dir/quad_threads.cpp.s

CMakeFiles/data.dir/quad_threads.cpp.o.requires:
.PHONY : CMakeFiles/data.dir/quad_threads.cpp.o.requires

CMakeFiles/data.dir/quad_threads.cpp.o.provides: CMakeFiles/data.dir/quad_threads.cpp.o.requires
	$(MAKE) -f CMakeFiles/data.dir/build.make CMakeFiles/data.dir/quad_threads.cpp.o.provides.build
.PHONY : CMakeFiles/data.dir/quad_threads.cpp.o.provides

CMakeFiles/data.dir/quad_threads.cpp.o.provides.build: CMakeFiles/data.dir/quad_threads.cpp.o

# Object files for target data
data_OBJECTS = \
"CMakeFiles/data.dir/kalman.cpp.o" \
"CMakeFiles/data.dir/control/MathFuncs.cpp.o" \
"CMakeFiles/data.dir/control/MatricesAndVectors.cpp.o" \
"CMakeFiles/data.dir/control/AttitudeControl.cpp.o" \
"CMakeFiles/data.dir/control/QuatRotEuler.cpp.o" \
"CMakeFiles/data.dir/control/PID_3DOF.cpp.o" \
"CMakeFiles/data.dir/PCA9685/pca9685.cpp.o" \
"CMakeFiles/data.dir/MPU6050/MPU6050.cpp.o" \
"CMakeFiles/data.dir/pevents/pevents.cpp.o" \
"CMakeFiles/data.dir/I2C/i2c.cpp.o" \
"CMakeFiles/data.dir/threads/keyboard_thread.cpp.o" \
"CMakeFiles/data.dir/threads/mpu_thread.cpp.o" \
"CMakeFiles/data.dir/threads/print_thread.cpp.o" \
"CMakeFiles/data.dir/threads/pca_thread.cpp.o" \
"CMakeFiles/data.dir/quad_threads.cpp.o"

# External object files for target data
data_EXTERNAL_OBJECTS =

data: CMakeFiles/data.dir/kalman.cpp.o
data: CMakeFiles/data.dir/control/MathFuncs.cpp.o
data: CMakeFiles/data.dir/control/MatricesAndVectors.cpp.o
data: CMakeFiles/data.dir/control/AttitudeControl.cpp.o
data: CMakeFiles/data.dir/control/QuatRotEuler.cpp.o
data: CMakeFiles/data.dir/control/PID_3DOF.cpp.o
data: CMakeFiles/data.dir/PCA9685/pca9685.cpp.o
data: CMakeFiles/data.dir/MPU6050/MPU6050.cpp.o
data: CMakeFiles/data.dir/pevents/pevents.cpp.o
data: CMakeFiles/data.dir/I2C/i2c.cpp.o
data: CMakeFiles/data.dir/threads/keyboard_thread.cpp.o
data: CMakeFiles/data.dir/threads/mpu_thread.cpp.o
data: CMakeFiles/data.dir/threads/print_thread.cpp.o
data: CMakeFiles/data.dir/threads/pca_thread.cpp.o
data: CMakeFiles/data.dir/quad_threads.cpp.o
data: CMakeFiles/data.dir/build.make
data: CMakeFiles/data.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable data"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/data.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/data.dir/build: data
.PHONY : CMakeFiles/data.dir/build

CMakeFiles/data.dir/requires: CMakeFiles/data.dir/kalman.cpp.o.requires
CMakeFiles/data.dir/requires: CMakeFiles/data.dir/control/MathFuncs.cpp.o.requires
CMakeFiles/data.dir/requires: CMakeFiles/data.dir/control/MatricesAndVectors.cpp.o.requires
CMakeFiles/data.dir/requires: CMakeFiles/data.dir/control/AttitudeControl.cpp.o.requires
CMakeFiles/data.dir/requires: CMakeFiles/data.dir/control/QuatRotEuler.cpp.o.requires
CMakeFiles/data.dir/requires: CMakeFiles/data.dir/control/PID_3DOF.cpp.o.requires
CMakeFiles/data.dir/requires: CMakeFiles/data.dir/PCA9685/pca9685.cpp.o.requires
CMakeFiles/data.dir/requires: CMakeFiles/data.dir/MPU6050/MPU6050.cpp.o.requires
CMakeFiles/data.dir/requires: CMakeFiles/data.dir/pevents/pevents.cpp.o.requires
CMakeFiles/data.dir/requires: CMakeFiles/data.dir/I2C/i2c.cpp.o.requires
CMakeFiles/data.dir/requires: CMakeFiles/data.dir/threads/keyboard_thread.cpp.o.requires
CMakeFiles/data.dir/requires: CMakeFiles/data.dir/threads/mpu_thread.cpp.o.requires
CMakeFiles/data.dir/requires: CMakeFiles/data.dir/threads/print_thread.cpp.o.requires
CMakeFiles/data.dir/requires: CMakeFiles/data.dir/threads/pca_thread.cpp.o.requires
CMakeFiles/data.dir/requires: CMakeFiles/data.dir/quad_threads.cpp.o.requires
.PHONY : CMakeFiles/data.dir/requires

CMakeFiles/data.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/data.dir/cmake_clean.cmake
.PHONY : CMakeFiles/data.dir/clean

CMakeFiles/data.dir/depend:
	cd /home/sahabi/AGNC/Quad/multithreaded && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sahabi/AGNC/Quad/multithreaded /home/sahabi/AGNC/Quad/multithreaded /home/sahabi/AGNC/Quad/multithreaded /home/sahabi/AGNC/Quad/multithreaded /home/sahabi/AGNC/Quad/multithreaded/CMakeFiles/data.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/data.dir/depend

