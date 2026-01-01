CC=gcc

# debugging on, source compiled into exe currently

CFLAGS  = -Wall -g $(shell pkg-config --cflags freetype2) -m64
LFLAGS = -g

# CFLAGS   = -m64 -pipe

SDL_LDFLAGS = $(shell sdl2-config --libs)

GLEW_LDFLAGS = $(shell pkg-config glew --static --libs)


LIBS=  $(SDL_LDFLAGS) $(GLEW_LDFLAGS) -lfreetype -lpng -lz -lm


SOURCES=main.c image_loader.c math.c shaders.c window.c mesh.c \
bounding_box.c octree.c world.c skybox.c hash_table.c object.c deflate.c log.c memory.c lcp.c physics.c


OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=main

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) $(LFLAGS) $(LIBS) -o $@

.c.o:
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	rm *.o

# remember readelf -d main
# then just sudo find / -name 'whaever.so.0'