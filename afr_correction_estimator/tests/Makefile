CXX = g++
CXXFLAGS = -std=c++11 -I. -DUNIT_TEST

SRCS = test_afr_correction_estimator.cpp mock_arduino.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = test_afr_correction_estimator

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
