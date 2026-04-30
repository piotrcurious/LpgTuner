CXX = g++
CXXFLAGS = -std=c++11 -Wall -I. -I./include

SRCS = test_lambda_simulator.cpp mock_arduino.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = test_runner

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) *.o
