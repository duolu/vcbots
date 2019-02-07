IDIR = ../include
CC = g++
CFLAGS = -I$(IDIR) -g


LDIR = 

LIBS = 


TARGET = test_js



OBJ = test_js.o


%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $< 

%.o: %.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

all : $(TARGET)
	

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^  $(LIBS)

clean:
	rm -f *.o
	rm -f $(TARGET)