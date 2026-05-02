CXX = g++
CXXFLAGS = -std=c++11 -I. -DUNIT_TEST

SRCS = test_simple_lambda.cpp mock_arduino.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = test_simple_lambda

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
