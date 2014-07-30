TARGET = earth stage1 stage2
TEST = earth_run stage1_run stage2_run
CC = mpicc
LIBS = -lm
CFLAGS = -O3 -g
DEPS = common.h

default: 1
1: stage1 earth earth_run stage1_run
2: stage2 earth earth_run stage2_run
all: $(TARGET) $(TEST) 

###########################################################

%.o: %.c $(DEPS)
	@mpicc $(CFLAGS) -o $@ $< -c

earth: earth.c
	@gcc $(CFLAGS) $< -o $@ $(LIBS)

earth_run:
	@./earth

stage1: return.o
	@mpicc $(CFLAGS) $< -o $@ $(LIBS)

stage1_run:
	@mpirun -np 2 ./stage1

stage2: orbit.o
	@mpicc $(CFLAGS) $< -o $@ $(LIBS)

stage2_run:
	@mpirun -np 2 ./stage2

###########################################################

.PHONY: default all clean

clean:
	@-rm -f *.o
	@-rm -f *.dat
	@-rm -f $(TARGET)




