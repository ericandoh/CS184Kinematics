CC = g++
ifeq ($(shell sw_vers 2>/dev/null | grep Mac | awk '{ print $$2}'),Mac)
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -I./include/ -I/usr/X11/include -DOSX -I./eigen-lib
	LDFLAGS = -framework GLUT -framework OpenGL \
    	-L"/System/Library/Frameworks/OpenGL.framework/Libraries" \
    	-lGL -lGLU -lm -lstdc++
else
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -Iglut-3.7.6-bin -I ./eigen-lib
	LDFLAGS = -lglut -lGLU -lGL
endif
CFLAGS += -std=c++11
	
RM = /bin/rm -f 
all: main 
main: ass_04.o 
	$(CC) $(CFLAGS) -o as4 ass_04.o $(LDFLAGS) 
ass_04.o: ass_04.cpp
	$(CC) $(CFLAGS) -c ass_04.cpp -o ass_04.o
clean: 
	$(RM) *.o as4
 


