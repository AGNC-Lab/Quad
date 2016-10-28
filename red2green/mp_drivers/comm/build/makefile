####################################################################################################

ROOT = ..
COMM = $(ROOT)/comm
PACKETS = $(ROOT)/packets
PARAMS = $(ROOT)/params
STATES = $(ROOT)/states
EIGEN = $(ROOT)/..
UTILITIES = $(ROOT)/../utilities

####################################################################################################

CC = arm-linux-gnueabi-g++
CPP_FLAGS += -O3
CPP_FLAGS += -Wall
CPP_FLAGS += -Wextra
CPP_FLAGS += -D POSIX_ENV
CPP_FLAGS += -D PRECOMPILE_COMM_LIB
CPP_FLAGS += -I $(ROOT)
CPP_FLAGS += -I $(EIGEN)
CPP_FLAGS += -I $(UTILITIES)

####################################################################################################

COMM_LIB = lib_mikipilot_comm.a

COMM_SOURCES += $(shell cd $(COMM); ls *.cpp)
PACKETS_SOURCES += $(shell cd $(PACKETS); ls *.cpp)
PARAMS_SOURCES += $(shell cd $(PARAMS); ls *.cpp)
STATES_SOURCES += $(shell cd $(STATES); ls *.cpp)

####################################################################################################

COMM_OBJS = $(addprefix comm_,$(COMM_SOURCES:.cpp=.o))
PACKETS_OBJS = $(addprefix packets_,$(PACKETS_SOURCES:.cpp=.o))
PARAMS_OBJS = $(addprefix params_,$(PARAMS_SOURCES:.cpp=.o))
STATES_OBJS = $(addprefix states_,$(STATES_SOURCES:.cpp=.o))

####################################################################################################

# Targets.
default: lib
lib: $(COMM_LIB)
comm: $(COMM_OBJS)
packets: $(PACKETS_OBJS)
params: $(PARAMS_OBJS)
states: $(STATES_OBJS)

####################################################################################################

# Build library file.
$(COMM_LIB): comm packets params states
	@ echo " building lib:" $@
	@ ar rcs $(COMM_LIB) $(COMM_OBJS) $(PACKETS_OBJS) $(PARAMS_OBJS) $(STATES_OBJS)
	@ echo
	@ echo " DONE:" $(COMM_LIB)
	@ echo

####################################################################################################

# Build COMM object files.
comm_%.o: $(COMM)/%.cpp $(COMM)/%.h
	@ echo " building obj:" $@
	@ $(CC) $(CPP_FLAGS) -c $< -o $@

####################################################################################################

# Build COMM object files.
packets_%.o: $(PACKETS)/%.cpp $(PACKETS)/%.h
	@ echo " building obj:" $@
	@ $(CC) $(CPP_FLAGS) -c $< -o $@

####################################################################################################

# Build COMM object files.
params_%.o: $(PARAMS)/%.cpp $(PARAMS)/%.h
	@ echo " building obj:" $@
	@ $(CC) $(CPP_FLAGS) -c $< -o $@

####################################################################################################

# Build STATES object files.
states_%.o: $(STATES)/%.cpp $(STATES)/%.h
	@ echo " building obj:" $@
	@ $(CC) $(CPP_FLAGS) -c $< -o $@

####################################################################################################

# Clean all files.
clean:
	@ rm -f $(COMM_LIB)
	@ rm -f $(COMM_OBJS)
	@ rm -f $(PACKETS_OBJS)
	@ rm -f $(PARAMS_OBJS)
	@ rm -f $(STATES_OBJS)

####################################################################################################
