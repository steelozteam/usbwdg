TARGET = usbwdg
OBJS = usbwdg.o hid.o
CFLAGS = -O3 -s
LDFLAGS = -ludev

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $(TARGET)

clean:
	rm -f $(TARGET) $(OBJS)
