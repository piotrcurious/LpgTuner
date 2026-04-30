CXX = g++
CXXFLAGS = -std=c++11 -I./include -D"IRAM_ATTR=" -DUNIT_TEST

all: test_ultra test_ultra_asm

test_ultra: test_ultra.cpp mock_arduino.cpp
	$(CXX) $(CXXFLAGS) test_ultra.cpp mock_arduino.cpp -o $@

test_ultra_asm: test_ultra_asm.cpp mock_arduino.cpp
	$(CXX) $(CXXFLAGS) test_ultra_asm.cpp mock_arduino.cpp -o $@

run: test_ultra test_ultra_asm
	./test_ultra
	./test_ultra_asm

clean:
	rm -f test_ultra test_ultra_asm
