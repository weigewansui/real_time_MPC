# Compiling
CC	      = gcc
INCPATH       = -I/usr/include/
CFLAGS=-c -Wall -ansi `pkg-config --cflags lcm` -std=c99
LCM_TYPES = $(shell ls lcmtypes/*.lcm)
LCM_O = $(LCM_TYPES:%.lcm=%.o)
LDFLAGS = `pkg-config --libs lcm`

# Linking
LINK          = g++
LFLAGS        = -Wl,-O1 
LIBS          = -lm -lpthread

# Source & Target
SOURCES     = src/asctecCommIntf.c src/util.c src/filter_util.c main.c logger/logger.c navigation/read_imu.c \
	navigation/nav.c navigation/hummingBirdState.c logger/imu_logger.c LLP_control/main.c
CPP_SOURCES = control/cmmd_gen.cpp
EXECUTABLE  = bin/aci_recvar bin/logger bin/transmit_log bin/control_test bin/logger
ASCLIB      = src/asctecCommIntf.o
UTILS		= src/filter_util.o src/util.o
#===========================================================================
OBJECTS=$(SOURCES:%.c=%.o)

.PHONY: all clean Vicon lcmtypes mpc_test simu_position navigation_cpp
all: lcmtypes Vicon $(OBJECTS) $(EXECUTABLE) mpc_test simu_position navigation_cpp

lcmtypes:
	@$(MAKE) -C lcmtypes

clean: 	
	rm -f *.o bin/* include/*.o logger/*.o src/*.o
	rm -f control/*.o navigation/*.o LLP_control/*.o
	@$(MAKE) -C lcmtypes clean


Vicon: $(UTILS)
	@$(MAKE) -C Vicon

$(EXECUTABLE): $(LCM_O) $(OBJECTS) 
	$(CC) $(LFLAGS)-o bin/control_test $(LCM_O) main.o $(ASCLIB) $(UTILS) $(LIBS) $(LDFLAGS)
	$(CC) $(LFLAGS)-o bin/logger $(LCM_O) logger/logger.o $(ASCLIB) $(UTILS) $(LIBS) $(LDFLAGS)
	$(CC) $(LFLAGS)-o bin/read_imu $(LCM_O) navigation/read_imu.o $(ASCLIB) $(UTILS) $(LIBS) $(LDFLAGS)
	$(CC) $(LFLAGS)-o bin/navigation $(LCM_O) navigation/nav.o $(ASCLIB) $(UTILS) $(LIBS) $(LDFLAGS)
	$(CC) $(LFLAGS)-o bin/HLP_state $(LCM_O) navigation/hummingBirdState.o $(ASCLIB) $(UTILS) $(LIBS) $(LDFLAGS)
	$(CC) $(LFLAGS)-o bin/imu_logger $(LCM_O) logger/imu_logger.o $(ASCLIB) $(UTILS) $(LIBS) $(LDFLAGS)
	$(CC) $(LFLAGS)-o bin/LLP_control $(LCM_O) LLP_control/main.c $(ASCLIB) $(UTILS) $(LIBS) $(LDFLAGS)
	
.o: $(SOURCES)
	$(CC) $(CFLAGS) $(INCPATH) $(LCMFLAGS) $< -o %(SOURCES) #$@


mpc_test:
	g++ mpc_test.cpp -o bin/mpctest -std=c++11 -larmadillo -pthread $(LDFLAGS)
simu_position:
	g++ simu_position.cpp -o bin/simu_position -llcm
navigation_cpp:
	g++ navigation/nav.cpp -o bin/navigation_cpp -llcm -pthread -std=c++11 -larmadillo
	