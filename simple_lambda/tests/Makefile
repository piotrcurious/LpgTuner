CXX = g++
CXXFLAGS = -std=c++11 -I. -DUNIT_TEST

SRCS = test_simple_lambda.cpp mock_arduino.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = test_simple_lambda

SRCS_ESP32 = test_simple_lambda_esp32.cpp mock_arduino.cpp
OBJS_ESP32 = $(SRCS_ESP32:.cpp=.o)
TARGET_ESP32 = test_simple_lambda_esp32

all: $(TARGET) $(TARGET_ESP32)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(TARGET_ESP32): $(OBJS_ESP32)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

run: $(TARGET) $(TARGET_ESP32)
	./$(TARGET)
	./$(TARGET_ESP32)

clean:
	rm -f *.o $(TARGET) $(TARGET_ESP32)
