TARGET = earth stage1 stage2
TEST = earth_run stage1_run stage2_run
CC = gcc
LIBS = -lm
CFLAGS = -O3 -g -W -Wall
INCL = ../include/
DEPS = $(INCL)initialise.h $(INCL)io.h $(INCL)functions.h $(INCL)motion.h $(INCL)events.h $(INCL)landing.h

default: 1
1: earth earth_run stage1 stage1_run
2: earth earth_run stage2 stage2_run
all: $(TARGET) $(TEST) 

###########################################################

%.o: %.c $(DEPS)
	$(CC) -I$(INCL) $(CFLAGS) $< -o $@ -c

earth: earth.c
	$(CC) $(CFLAGS) $< -o $@ $(LIBS)

earth_run:
	./earth

stage1: return.o
	$(CC) -I$(INCL) $(CFLAGS) $< -o $@ $(LIBS)

stage1_run:
	./stage1 -s 0 -f profile.txt

stage2: orbit.o
	$(CC) -I$(INCL) $(CFLAGS) $< -o $@ $(LIBS)

stage2_run:
	./stage2 -s 1 -f profile.txt

###########################################################

.PHONY: default all clean

clean:
	@-rm -f *.o
	@-rm -f *.dat
	@-rm -f $(TARGET)




