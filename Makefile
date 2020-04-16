#
# $Id: Makefile,v 1.8 2016/05/10 15:44:18 gianluca Exp $
#
DEFINES ?=
DISTRO  ?= wheezy
ARCH    ?= x86_64-linux-gnu

CC            = gcc
CXX           = g++
CFLAGS        = -pipe -O2 -Wall -W -D_REENTRANT $(DEFINES)
CXXFLAGS      = -pipe -O2 -Wall -W -D_REENTRANT $(DEFINES)
INCPATH       = -I./inc
LINK          = gcc
LFLAGS        = -Wl,-O1 -g
LIBS          = $(SUBLIBS) -lpthread -lm -lrt
AR            = ar cqs
RANLIB        =
TAR           = tar -cf
COMPRESS      = gzip -9f
COPY          = cp -f
SED           = sed
COPY_FILE     = $(COPY)
COPY_DIR      = $(COPY) -r
STRIP         = strip
INSTALL_FILE  = install -m 644 -p
INSTALL_DIR   = $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p

###### On stretch use capabilities
ifeq ($(DISTRO),stretch)
endif

ifneq ($(ARCH),x86_64-linux-gnu)
	###### EXTRA OPTIMIZATION
	ifeq ($(DISTRO),stretch)
		CFLAGS += -O3 -mcpu=arm926ej-s -march=armv5te -marm -msoft-float -funroll-loops -Wno-narrowing -fomit-frame-pointer 
		CXXFLAGS += -O3 -std=c++0x -std=gnu++0x -mcpu=arm926ej-s -march=armv5te -marm -msoft-float -funroll-loops -Wno-narrowing -fomit-frame-pointer
		LFLAGS += -O3 -mcpu=arm926ej-s -march=armv5te -marm -msoft-float -funroll-loops -Wno-narrowing -fomit-frame-pointer
	else
		CFLAGS += -O3 -mcpu=arm926ej-s -march=armv5te -marm -msoft-float -funroll-loops -Wno-narrowing -fomit-frame-pointer
		CXXFLAGS += -O3 -mcpu=arm926ej-s -march=armv5te -marm -msoft-float -funroll-loops -Wno-narrowing -fomit-frame-pointer
		LFLAGS += -O3 -mcpu=arm926ej-s -march=armv5te -marm -msoft-float -funroll-loops -Wno-narrowing -fomit-frame-pointer
	endif
endif

####### Output directory

OBJECTS_DIR   = src/

####### Files

OBJECTS       = \
	src/testunit.o \
	src/serial.o \
	src/version.o \


DESTDIR       = bin/

TARGET        = $(DESTDIR)testunit

first: all
####### Implicit rules

.SUFFIXES: .o .c .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o "$@" "$<"

####### Build rules

all: Makefile $(TARGET)

$(TARGET):  $(OBJECTS)
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)

clean:
	$(DEL_FILE) $(OBJECTS)
	$(DEL_FILE) *~ core *.core


####### Sub-libraries

distclean: clean
	$(DEL_FILE) $(TARGET)


####### Compile

####### Install

install:   FORCE

uninstall:   FORCE

FORCE:
