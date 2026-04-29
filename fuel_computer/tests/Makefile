CXX = g++
CXXFLAGS = -Wall -I. -std=c++11

TEST_TARGET = test_fuel_computer
TEST_SRCS = test_fuel_computer.cpp mock_arduino.cpp

all: $(TEST_TARGET)

$(TEST_TARGET): $(TEST_SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^

run: $(TEST_TARGET)
	./$(TEST_TARGET)

clean:
	rm -f $(TEST_TARGET)
