IDIR = -I../include -I/usr/local/include/
CC = g++
CFLAGS = $(IDIR) -g


LDIR = 

LIBS = -lpthread


TARGET = vehicle_ctl



OBJ = vehicle_ctl.o serial.o sprotocol.o joystick.o


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