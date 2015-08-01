CC = g++
LD = g++

DEGUB = -g

CFLAGS = -Wall -Wextra -Wpointer-arith -Wredundant-decls
# CFLAGS += -Wcast-align -Wcast-qual
CFLAGS += -I/usr/include/opencv/
# CFLAGS += -I../opencv-2.4.10/include/opencv/
# CFLAGS += -I../opencv-2.4.10/modules/*/include
# CFLAGS += -I../opencv-2.4.10/modules/imgproc/include
# CFLAGS += -I/usr/local/include/SDL
CFLAGS += $(DEBUG)

LDFLAGS = 
# LDFLAGS = -lSDL
# LDFLAGS = `pkg-config opencv --cflags` `pkg-config opencv --libs`
# LDFLAGS += -ljpeg
LDFLAGS += -lopencv_core
# LDFLAGS += -lopencv_calib3d
# LDFLAGS += -lopencv_contrib
# LDFLAGS += -lopencv_features2d
# LDFLAGS += -lopencv_flann
# LDFLAGS += -lopencv_gpu
LDFLAGS += -lopencv_highgui
LDFLAGS += -lopencv_imgproc
# LDFLAGS += -lopencv_legacy
# LDFLAGS += -lopencv_ml
# LDFLAGS += -lopencv_objdetect
# LDFLAGS += -lopencv_ocl
# LDFLAGS += -lopencv_photo
# LDFLAGS += -lopencv_stitching
# LDFLAGS += -lopencv_superres
# LDFLAGS += -lopencv_ts
# LDFLAGS += -lopencv_video
# LDFLAGS += -lopencv_videostab

EXEC = bin.bin

OBJ_DIR = objects

# SRC = sampler.cpp cvplot.cpp
SRC = $(wildcard *.cpp)
_OBJ = $(SRC:.cpp=.o)
OBJ = $(patsubst %,$(OBJ_DIR)/%,$(_OBJ))


all: $(OBJ)
	$(LD) $(LDFLAGS) $(OBJ) -o $(EXEC)

$(OBJ_DIR)/%.o: %.cpp
	$(CC) -c $(CFLAGS) $< -o $@

.PHONY: clean

clean:
	rm -f $(OBJ_DIR)/*.o
	rm $(EXEC)
