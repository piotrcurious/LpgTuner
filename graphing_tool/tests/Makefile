CC = g++
CFLAGS = -Wall -I. -DUNIT_TEST

SRCS = mock_arduino.cpp test_graphing.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = test_graphing

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS)

%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
