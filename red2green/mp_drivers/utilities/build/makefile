####################################################################################################

MP = ../..
EIGEN = $(MP)
UTILITIES = $(MP)/utilities
SOURCE = $(UTILITIES)/utilities

####################################################################################################

CC = arm-linux-gnueabi-g++
CPP_FLAGS += -O3
CPP_FLAGS += -Wall
CPP_FLAGS += -Wextra
CPP_FLAGS += -D POSIX_ENV
CPP_FLAGS += -I $(EIGEN)
CPP_FLAGS += -I $(UTILITIES)

####################################################################################################

UTILITIES_LIB = lib_mikipilot_utilities.a

UTILITIES_SOURCES += cv.cpp
UTILITIES_SOURCES += formatting.cpp
UTILITIES_SOURCES += globals.cpp
UTILITIES_SOURCES += i2c_port.cpp
UTILITIES_SOURCES += ifile.cpp
UTILITIES_SOURCES += ipc_socket.cpp
UTILITIES_SOURCES += module.cpp
UTILITIES_SOURCES += mutex.cpp
UTILITIES_SOURCES += object.cpp
UTILITIES_SOURCES += ofile.cpp
UTILITIES_SOURCES += thread.cpp
UTILITIES_SOURCES += timer.cpp
UTILITIES_SOURCES += tob.cpp
UTILITIES_SOURCES += uart_port.cpp
UTILITIES_SOURCES += udp_socket.cpp

####################################################################################################

UTILITIES_OBJS = $(addprefix utilities_,$(UTILITIES_SOURCES:.cpp=.o))

####################################################################################################

# Targets.
default: mode1
lib: $(UTILITIES_LIB)
mode0: lib
mode1: lib
mode2: lib
mode3: lib
mode4: lib

# Conditional Rules.
mode1: OPT_FLAGS += -D SUPPRESS_TIMER_SLIP_WARNINGS
mode1: OPT_FLAGS += -D SUPPRESS_RDWR_ERROR_CHECKING
mode2: OPT_FLAGS += -D SUPPRESS_TIMER_SLIP_WARNINGS
mode3: OPT_FLAGS += -D SUPPRESS_RDWR_ERROR_CHECKING
mode4: OPT_FLAGS += -D SUPPRESS_STATE_MACHINE_MESSAGES

####################################################################################################

# Build library file.
$(UTILITIES_LIB): $(UTILITIES_OBJS)
	@ echo " building lib:" $@
	@ ar rcs $(UTILITIES_LIB) $(UTILITIES_OBJS)
	@ echo
	@ echo " FLAGS:" $(OPT_FLAGS)
	@ echo " DONE:" $(UTILITIES_LIB)
	@ echo

####################################################################################################

# Build UTILITIES object files.
utilities_%.o: $(SOURCE)/%.cpp $(SOURCE)/%.h
	@ echo " building obj:" $@
	@ $(CC) $(CPP_FLAGS) $(OPT_FLAGS) -c $< -o $@

####################################################################################################

# Clean all files.
clean:
	@ rm -f $(UTILITIES_LIB)
	@ rm -f $(UTILITIES_OBJS)

####################################################################################################
