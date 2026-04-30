CXX = g++
CXXFLAGS = -std=c++11 -I.

SRCS = test_logic.cpp mock_arduino.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = test_logic

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: all
	./$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
