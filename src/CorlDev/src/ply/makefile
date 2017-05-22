# PLY geometry file filters

COPTIONS = -g
CFLAGS = -I. $(COPTIONS)
CP_FLAGS = $(CFLAGS) -woff 3262
LIBS = -lm

.C.o:
	CC $(CP_FLAGS) -c $*.C

all: ply2ascii ply2binary xformply ply2iv sphereply \
     platoply boundply obj2ply flipply normalsply headply 

headply: headply.o ply.o makefile
	cc -o headply headply.o ply.o $(LIBS)

flipply: flipply.o ply.o makefile
	cc -o flipply flipply.o ply.o $(LIBS)

normalsply: normalsply.o ply.o makefile
	cc -o normalsply normalsply.o ply.o $(LIBS)

obj2ply: obj2ply.o ply.o makefile
	cc -o obj2ply obj2ply.o ply.o $(LIBS)

boundply: boundply.o ply.o makefile
	cc -o boundply boundply.o ply.o $(LIBS)

platoply: platoply.o ply.o makefile
	cc -o platoply platoply.o ply.o $(LIBS)

sphereply: sphereply.o ply.o makefile
	cc -o sphereply sphereply.o ply.o $(LIBS)

ply2iv: ply2iv.o ply.o makefile
	cc -o ply2iv ply2iv.o ply.o $(LIBS)

xformply: xformply.o ply.o makefile
	cc -o xformply xformply.o ply.o $(LIBS)

ply.o: ply.c ply.h
	cc $(CFLAGS) -c ply.c

ply2ascii.o: convertply.c ply.h
	cc $(CFLAGS) -c convertply.c -DWRITE_ASCII
	mv convertply.o ply2ascii.o

ply2binary.o: convertply.c ply.h
	cc $(CFLAGS) -c convertply.c -DWRITE_BINARY
	mv convertply.o ply2binary.o

ply2ascii: ply2ascii.o ply.o makefile
	cc -o ply2ascii ply2ascii.o ply.o $(LIBS)

ply2binary: ply2binary.o ply.o makefile
	cc -o ply2binary ply2binary.o ply.o $(LIBS)

