CXX = g++
CXXFLAGS = -std=c++11 -I. -DUNIT_TEST

SRCS = test_kalman.cpp mock_arduino.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = test_kalman

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
