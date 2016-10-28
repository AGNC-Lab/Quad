####################################################################################################

ROOT = ..
FSW = $(ROOT)/fsw
EIGEN = $(ROOT)/..
UTILITIES = $(ROOT)/../utilities
COMM = $(ROOT)/../comm

EXEC = $(FSW)/exec
GLOBALS = $(FSW)/globals
HARDWARE = $(FSW)/hardware
####################################################################################################

CC = arm-linux-gnueabi-g++
CPP_FLAGS += -static-libstdc++
CPP_FLAGS += -O3
CPP_FLAGS += -Wall
CPP_FLAGS += -Wextra
CPP_FLAGS += -D POSIX_ENV
CPP_FLAGS += -D PRECOMPILE_COMM_LIB
CPP_FLAGS += -I $(ROOT)
CPP_FLAGS += -I $(EIGEN)
CPP_FLAGS += -I $(UTILITIES)
CPP_FLAGS += -I $(COMM)

####################################################################################################

FSW_EXEC_SOURCES += red2green.cpp

FSW_GLOBALS_SOURCES += globals.cpp
FSW_GLOBALS_SOURCES += module.cpp
FSW_GLOBALS_SOURCES += object.cpp

FSW_HARDWARE_SOURCES += hmc5883l.cpp
#FSW_HARDWARE_SOURCES += max11644eua.cpp
FSW_HARDWARE_SOURCES += mp_ap_board.cpp
FSW_HARDWARE_SOURCES += mpu6050.cpp
FSW_HARDWARE_SOURCES += overo.cpp
FSW_HARDWARE_SOURCES += pca9685.cpp
FSW_HARDWARE_SOURCES += sc18is602b.cpp

COMM_LIB = $(COMM)/build/lib_mikipilot_comm.a
UTILITIES_LIB = $(UTILITIES)/build/lib_mikipilot_utilities.a

FSW_VERSION_SOURCES += version.cpp


####################################################################################################

EXEC_OBJS = $(addprefix exec_,$(FSW_EXEC_SOURCES:.cpp=.o))
FSW_GLOBALS_OBJS = $(addprefix fsw_globals_,$(FSW_GLOBALS_SOURCES:.cpp=.o))
FSW_HARDWARE_OBJS = $(addprefix fsw_hardware_,$(FSW_HARDWARE_SOURCES:.cpp=.o))
FSW_VERSION_OBJS = $(addprefix fsw_,$(FSW_VERSION_SOURCES:.cpp=.o))

DEPS_OBJS += $(FSW_GLOBALS_OBJS)
DEPS_OBJS += $(FSW_HARDWARE_OBJS)
DEPS_OBJS += $(FSW_VERSION_OBJS)

ALL_OBJS += $(EXEC_OBJS)
ALL_OBJS += $(DEPS_OBJS)

####################################################################################################

# Targets.
default: red2green
all: red2green
deps: globals hardware version
exec: $(EXEC_OBJS)
globals: $(FSW_GLOBALS_OBJS)
hardware: $(FSW_HARDWARE_OBJS)
version: $(FSW_VERSION_OBJS)

lib_comm:
	@ make -s -C $(COMM)/build
lib_utilities:
	@ make -s -C $(UTILITIES)/build

####################################################################################################

# Build MIKIPILOT_FSW EXEC.
red2green: lib_utilities lib_comm deps red2green.o
	@ echo
	@ echo " BUILDING EXEC:" $@
	@ $(CC) $(CPP_FLAGS) -o $@ $@.o $(DEPS_OBJS) $(COMM_LIB) $(UTILITIES_LIB) -lrt -lpthread
	@ echo " DONE:" $@
	@ echo

####################################################################################################

# Build EXEC object files.
red2green.o: $(EXEC)/red2green.cpp
	@ echo " building obj:" $@
	@ $(CC) $(CPP_FLAGS) -c $< -o $@

####################################################################################################

# Build FSW GLOBALS object files.
fsw_globals_%.o: $(GLOBALS)/%.cpp $(GLOBALS)/%.h
	@ echo " building obj:" $@
	@ $(CC) $(CPP_FLAGS) -c $< -o $@

####################################################################################################

# Build FSW HARDWARE object files.
fsw_hardware_%.o: $(HARDWARE)/%.cpp $(HARDWARE)/%.h
	@ echo " building obj:" $@
	@ $(CC) $(CPP_FLAGS) -c $< -o $@

####################################################################################################

# Build FSW VERSION object files.
fsw_version.o: fsw_version.cpp
	@ echo " building obj:" fsw_version.o
	@ $(CC) $(CPP_FLAGS) -c $< -o $@
	@ rm -f $<
	
# Generate FSW VERSION source files.
fsw_version.cpp:
	@ printf '%s\n%s\nconst string fsw::globals::version::string = "%s";' \
	  '#include "fsw/globals/globals.h"' \
		'using std::string;' \
		N/A \
		>> $@

####################################################################################################

# Clean all files.
clean:
	@ rm -f red2green
	@ rm -f $(ALL_OBJS)
	@ make -s clean -C $(COMM)/build
	@ make -s clean -C $(UTILITIES)/build

####################################################################################################
