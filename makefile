TARGET = earth stage1 stage2
TEST = earth_run stage1_run stage2_run
CC = mpicc
LIBS = -lm
CFLAGS = -O3 -g -W -Wall -pedantic

default: 2
1: clean stage1 earth stage1_run earth_run
2: clean stage2 earth stage2_run earth_run
all: $(TARGET) $(TEST) 

###########################################################

%.o: %.c $(DEPS)
	@gcc $(CFLAGS) -o $@ $< -c

earth: earth.c
	@gcc $(CFLAGS) $< -o $@ $(LIBS)

earth_run:
	@./earth

stage1: default.o
	@mpicc $(CFLAGS) $< -o $@ $(LIBS)

stage1_run:
	@mpirun -np 2 ./stage1

stage2: coriolis.o
	@mpicc $(CFLAGS) $< -o $@ $(LIBS)

stage2_run:
	@mpirun -np 2 ./stage2

###########################################################

.PHONY: default all clean

clean:
	@-rm -f *.o
	@-rm -f *.dat
	@-rm -f $(TARGET)




