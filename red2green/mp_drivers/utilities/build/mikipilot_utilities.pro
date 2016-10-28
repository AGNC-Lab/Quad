#-------------------------------------------------
#
# Project created by QtCreator 2015-11-12T11:35:56
#
#-------------------------------------------------

# Define project target and template.
TARGET = _mikipilot_utilities
TEMPLATE = lib

# Configure build.
CONFIG += staticlib
CONFIG += release

# Add Qt libraries.
QT += core
QT -= gui
QT += network

# Add defines flags.
DEFINES += QT_ENV
DEFINES += SUPPRESS_TIMER_SLIP_WARNINGS
DEFINES += SUPPRESS_RDWR_ERROR_CHECKING

# Add include paths.
INCLUDEPATH += ../..
INCLUDEPATH += ..

# Add project header files.
HEADERS += ../utilities/consumer.h
HEADERS += ../utilities/cv.h
HEADERS += ../utilities/dcm.h
HEADERS += ../utilities/filter.h
HEADERS += ../utilities/formatting.h
HEADERS += ../utilities/globals.h
HEADERS += ../utilities/i2c_port.h
HEADERS += ../utilities/ifile.h
HEADERS += ../utilities/ipc_socket.h
HEADERS += ../utilities/module.h
HEADERS += ../utilities/mutex.h
HEADERS += ../utilities/object.h
HEADERS += ../utilities/ofile.h
HEADERS += ../utilities/packet.h
HEADERS += ../utilities/params.h
HEADERS += ../utilities/producer.h
HEADERS += ../utilities/quat.h
HEADERS += ../utilities/serializable.h
HEADERS += ../utilities/socket.h
HEADERS += ../utilities/state.h
HEADERS += ../utilities/state_machine.h
HEADERS += ../utilities/thread.h
HEADERS += ../utilities/timer.h
HEADERS += ../utilities/tob.h
HEADERS += ../utilities/uart_port.h
HEADERS += ../utilities/udp_socket.h

# Add project source files.
SOURCES += ../utilities/cv.cpp
SOURCES += ../utilities/formatting.cpp
SOURCES += ../utilities/globals.cpp
SOURCES += ../utilities/i2c_port.cpp
SOURCES += ../utilities/ifile.cpp
SOURCES += ../utilities/ipc_socket.cpp
SOURCES += ../utilities/module.cpp
SOURCES += ../utilities/mutex.cpp
SOURCES += ../utilities/object.cpp
SOURCES += ../utilities/ofile.cpp
SOURCES += ../utilities/thread.cpp
SOURCES += ../utilities/timer.cpp
SOURCES += ../utilities/tob.cpp
SOURCES += ../utilities/uart_port.cpp
SOURCES += ../utilities/udp_socket.cpp
