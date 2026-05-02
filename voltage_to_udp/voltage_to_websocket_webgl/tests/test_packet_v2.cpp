#include <iostream>
#include <vector>
#include <cstdint>
#include <cassert>
#include <cstring>

#define SAMPLES_PER_PACKET 512
#define ADC_CHANNELS_COUNT 4

#pragma pack(push, 1)
struct ScopePacket {
  uint32_t sequence;
  uint32_t timestamp;
  uint8_t trigger_flags;
  uint32_t free_heap;
  uint16_t data[SAMPLES_PER_PACKET * ADC_CHANNELS_COUNT];
};
#pragma pack(pop)

void test_packet_v2_structure() {
    std::cout << "Testing ScopePacket V2 structure..." << std::endl;
    // seq(4) + ts(4) + trig(1) + heap(4) + data(512*4*2)
    size_t expected = 4 + 4 + 1 + 4 + (SAMPLES_PER_PACKET * ADC_CHANNELS_COUNT * 2);
    assert(sizeof(ScopePacket) == expected);

    ScopePacket pkt;
    pkt.trigger_flags = 0x80; // Cam mode bit
    assert(((uint8_t*)&pkt)[8] == 0x80);

    std::cout << "ScopePacket V2 test passed." << std::endl;
}

int main() {
    test_packet_v2_structure();
    return 0;
}
