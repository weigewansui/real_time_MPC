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
SOURCES     = src/asctecCommIntf.c src/util.c src/filter_util.c main.c
 # logger/logger.c logger/vicon_logger.c
CPP_SOURCES = control/cmmd_gen.cpp
EXECUTABLE  = bin/aci_recvar bin/logger bin/transmit_log bin/control_test
ASCLIB      = src/asctecCommIntf.o
UTILS		= src/filter_util.o src/util.o
#===========================================================================
OBJECTS=$(SOURCES:%.c=%.o)

.PHONY: all clean Vicon lcmtypes Vision
all: lcmtypes Vicon Vision $(OBJECTS) $(EXECUTABLE)

lcmtypes:
	@$(MAKE) -C lcmtypes

clean: 	
	rm -f *.o bin/* include/*.o logger/*.o src/*.o
	rm -f control/*.o navigation/*.o
	@$(MAKE) -C lcmtypes clean

$(EXECUTABLE): $(LCM_O) $(OBJECTS) 

	# $(CC) $(LFLAGS)-o bin/logger $(LCM_O) logger/logger.o $(LIBS) $(LDFLAGS)
	# $(CC) $(LFLAGS)-o bin/transmit_log $(LCM_O) logger/transmit_log.o $(LIBS) $(LDFLAGS)
	# $(CC) $(LFLAGS)-o bin/vicon_logger $(LCM_O) logger/vicon_logger.o $(LIBS) $(LDFLAGS)
	$(CC) $(LFLAGS)-o bin/control_test $(LCM_O) main.o $(ASCLIB) $(UTILS) $(LIBS) $(LDFLAGS)
	# $(CC) $(LFLAGS)-o bin/vision_control $(LCM_O) control/main_Vision.o $(ASCLIB) $(UTILS) $(LIBS) $(LDFLAGS)
	# g++ -pthread $(LFLAGS)-o bin/cmmd_gen $(UTILS)
	
.o: $(SOURCES)
	$(CC) $(CFLAGS) $(INCPATH) $(LCMFLAGS) $< -o %(SOURCES) #$@
	# g++ -std=c++0x -c -o $(CPP_SOURCES) $(LDFLAGS)

	