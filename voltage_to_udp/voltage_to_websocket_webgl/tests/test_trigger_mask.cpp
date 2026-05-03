#include <iostream>
#include <vector>
#include <cstdint>
#include <cassert>

// Mocking logic for Triggering with Mask
enum TriggerMode {
  MODE_INDEPENDENT,
  MODE_CAM_WHEEL
};

TriggerMode currentMode = MODE_INDEPENDENT;
bool trigger_occurred[4] = {false, false, false, false};
uint8_t cam_wheel_target = 0;
uint8_t trigger_mask = 0x0F;

void handleTrigger(int idx) {
  if (idx == 0 && currentMode == MODE_CAM_WHEEL) {
    trigger_occurred[cam_wheel_target] = true;
    cam_wheel_target = (cam_wheel_target + 1) % 4;
  } else if (currentMode == MODE_INDEPENDENT) {
    trigger_occurred[idx] = true;
  }
}

void test_trigger_mask() {
    // Mode Auto/Normal logic
    auto check_any_trigger = [](bool* occurred, uint8_t mask) {
        bool any = false;
        for (int t = 0; t < 4; t++) {
            if (occurred[t]) {
                if (mask & (1 << t)) any = true;
                occurred[t] = false;
            }
        }
        return any;
    };

    // Scenario 1: All triggers active, mask allows all
    for(int i=0; i<4; i++) trigger_occurred[i] = true;
    trigger_mask = 0x0F;
    assert(check_any_trigger(trigger_occurred, trigger_mask) == true);

    // Scenario 2: Only CH3 occurred, mask only allows CH0
    for(int i=0; i<4; i++) trigger_occurred[i] = false;
    trigger_occurred[3] = true;
    trigger_mask = 0x01; // Only CH0
    assert(check_any_trigger(trigger_occurred, trigger_mask) == false);

    // Scenario 3: CH3 occurred, mask allows CH3
    trigger_occurred[3] = true;
    trigger_mask = 0x08; // Only CH3
    assert(check_any_trigger(trigger_occurred, trigger_mask) == true);

    std::cout << "Test Trigger Mask Passed" << std::endl;
}

int main() {
    test_trigger_mask();
    return 0;
}
