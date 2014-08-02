TARGET = earth stage1 stage2
TEST = earth_run stage1_run stage2_run
CC = gcc
LIBS = -lm
CFLAGS = -O3 -g -W -Wall
DEPS = common.h

default: 1
1: stage1 earth earth_run stage1_run
2: stage2 earth earth_run stage2_run
all: $(TARGET) $(TEST) 

###########################################################

%.o: %.c $(DEPS)
	@$(CC) $(CFLAGS) -o $@ $< -c

earth: earth.c
	@$(CC) $(CFLAGS) $< -o $@ $(LIBS)

earth_run:
	@./earth

stage1: return.o
	@$(CC) $(CFLAGS) $< -o $@ $(LIBS)

stage1_run:
	@./stage1 -s 0 -f profile.txt

stage2: orbit.o
	@$(CC) $(CFLAGS) $< -o $@ $(LIBS)

stage2_run:
	@./stage2 -s 1 -f profile.txt

###########################################################

.PHONY: default all clean

clean:
	@-rm -f *.o
	@-rm -f *.dat
	@-rm -f $(TARGET)




