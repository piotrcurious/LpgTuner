#include <iostream>
#include <vector>
#include <cstdint>
#include <cassert>

// Mocking logic for Cam Wheel Triggering
enum TriggerMode {
  MODE_INDEPENDENT,
  MODE_CAM_WHEEL
};

TriggerMode currentMode = MODE_INDEPENDENT;
bool trigger_occurred[4] = {false, false, false, false};
uint8_t cam_wheel_target = 0;

void handleTrigger0() {
  if (currentMode == MODE_INDEPENDENT) {
    trigger_occurred[0] = true;
  } else {
    trigger_occurred[cam_wheel_target] = true;
    cam_wheel_target = (cam_wheel_target + 1) % 4;
  }
}

void test_independent_mode() {
    currentMode = MODE_INDEPENDENT;
    for(int i=0; i<4; i++) trigger_occurred[i] = false;

    handleTrigger0();
    assert(trigger_occurred[0] == true);
    assert(trigger_occurred[1] == false);
    std::cout << "Test Independent Mode Passed" << std::endl;
}

void test_cam_wheel_mode() {
    currentMode = MODE_CAM_WHEEL;
    cam_wheel_target = 0;
    for(int i=0; i<4; i++) trigger_occurred[i] = false;

    handleTrigger0(); // Should target CH0
    assert(trigger_occurred[0] == true);
    assert(cam_wheel_target == 1);

    trigger_occurred[0] = false;
    handleTrigger0(); // Should target CH1
    assert(trigger_occurred[1] == true);
    assert(cam_wheel_target == 2);

    trigger_occurred[1] = false;
    handleTrigger0(); // Should target CH2
    assert(trigger_occurred[2] == true);
    assert(cam_wheel_target == 3);

    trigger_occurred[2] = false;
    handleTrigger0(); // Should target CH3
    assert(trigger_occurred[3] == true);
    assert(cam_wheel_target == 0);

    std::cout << "Test Cam Wheel Mode Passed" << std::endl;
}

int main() {
    test_independent_mode();
    test_cam_wheel_mode();
    return 0;
}
