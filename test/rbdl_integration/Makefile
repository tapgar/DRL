RBDLDIR := /home/tapgar/rbdl
MJDIR   := /home/tapgar/libcassie-master/mjpro150
HEADDIR := /home/tapgar/cuda-workspace/DRL
SRCDIR := $(HEADDIR)/src
SIMDIR := $(SRCDIR)/Simulation
ROBOTDIR := $(SIMDIR)/Robots
CASSIEDIR := $(ROBOTDIR)/Cassie
SLEGDIR := $(ROBOTDIR)/SingleLeg
CTRLDIR := $(SRCDIR)/Control
HELPDIR := $(SRCDIR)/HelperFuncs
PLANDIR := $(SRCDIR)/Planning
NLPDIR := $(PLANDIR)/NLP/S_LIP
UIDIR := $(SRCDIR)/UserInterface
COMMDIR := $(SRCDIR)/Communication

# CHANGEME: Here is the name of all object files corresponding to the source
#           code that you wrote in order to define the problem statement
TESTOBJS = main.o \
	DynamicModel.o \
	OSC_RBDL.o \
        $(UIDIR)/Visualizer.o \
	~/Documents/OOQP-0.99.26/src/CInterface/cBounds.o

OBJS = $(TESTOBJS)

# CHANGEME: Additional libraries
ADDLIBS = -L$(MJDIR)/bin -Wl,-rpath,'$(MJDIR)/bin',-L/home/tapgar/Documents/OOQP-0.99.26 -L/usr/lib/gcc/x86_64-linux-gnu/5 -L$(RBDLDIR)/build -lmujoco150 -lglew -lGL -l:libglfw.so.3 -lOOQP -lblas -llapack -lgfortran -lrbdl

# CHANGEME: Additional flags for compilation (e.g., include flags)
ADDINCFLAGS = -I. -I$(COMMDIR) -I$(HELPDIR) -I$(ROBOTDIR) -I$(SLEGDIR) -I$(UIDIR) -I$(MJDIR)/include -I/home/tapgar/eigen -I/usr/local/include/ooqp -I/home/tapgar/Documents/OOQP-0.99.26 -I$(RBDLDIR)/src -I$(RBDLDIR)/include -I$(RBDLDIR)/build/include

##########################################################################
#  Usually, you don't have to change anything below.  Note that if you   #
#  change certain compiler options, you might have to recompile Ipopt.   #
##########################################################################

# C++ Compiler command
CXX = g++

# C++ Compiler options
CXXFLAGS = -std=c++11 -O3 -pipe -DNDEBUG -Wparentheses -Wreturn-type -Wcast-qual -Wall -Wpointer-arith -Wwrite-strings -Wno-unknown-pragmas -Wno-long-long   -DIPOPT_BUILD -fopenmp

# additional C++ Compiler options for linking
CXXLINKFLAGS =  -Wl,--rpath -Wl,/home/tapgar/Documents/Ipopt-3.12.8/build/lib

# Include directories (we use the CYGPATH_W variables to allow compilation with Windows compilers)
INCL = `PKG_CONFIG_PATH=/home/tapgar/Documents/Ipopt-3.12.8/build/lib64/pkgconfig:/home/tapgar/Documents/Ipopt-3.12.8/build/lib/pkgconfig:/home/tapgar/Documents/Ipopt-3.12.8/build/share/pkgconfig:/opt/ros/kinetic/lib/pkgconfig pkg-config --cflags ipopt` $(ADDINCFLAGS)
#INCL = -I`$(CYGPATH_W) /home/tapgar/Documents/Ipopt-3.12.8/build/include/coin`  $(ADDINCFLAGS)

# Linker flags
LIBS = `PKG_CONFIG_PATH=/home/tapgar/Documents/Ipopt-3.12.8/build/lib64/pkgconfig:/home/tapgar/Documents/Ipopt-3.12.8/build/lib/pkgconfig:/home/tapgar/Documents/Ipopt-3.12.8/build/share/pkgconfig:/opt/ros/kinetic/lib/pkgconfig pkg-config --libs ipopt`
##LIBS = -link -libpath:`$(CYGPATH_W) /home/tapgar/Documents/Ipopt-3.12.8/build/lib` libipopt.lib -llapack -lblas -lm  -ldl
#LIBS = -L/home/tapgar/Documents/Ipopt-3.12.8/build/lib -lipopt -llapack -lblas -lm  -ldl

# The following is necessary under cygwin, if native compilers are used
CYGPATH_W = echo

all: main

.SUFFIXES: .cpp .c .o .obj

main: $(TESTOBJS)
	bla=;\
	for file in $(TESTOBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(CXXLINKFLAGS) $(CXXFLAGS) -o $@ $$bla $(ADDLIBS) $(LIBS)

clean:
	rm -rf planner sim $(OBJS)

.cpp.o:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ $<


.cpp.obj:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ `$(CYGPATH_W) '$<'`
