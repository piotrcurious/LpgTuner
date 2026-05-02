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
  uint16_t data[SAMPLES_PER_PACKET * ADC_CHANNELS_COUNT];
};
#pragma pack(pop)

void test_packet_structure() {
    std::cout << "Testing ScopePacket structure..." << std::endl;
    assert(sizeof(ScopePacket) == 4 + 4 + 1 + (SAMPLES_PER_PACKET * ADC_CHANNELS_COUNT * 2));

    ScopePacket pkt;
    pkt.sequence = 0x12345678;
    pkt.timestamp = 0xABCDEF01;
    pkt.trigger_flags = 0x0F;

    uint8_t* ptr = (uint8_t*)&pkt;
    assert(ptr[0] == 0x78); // Little endian
    assert(ptr[4] == 0x01);
    assert(ptr[8] == 0x0F);

    std::cout << "ScopePacket structure test passed." << std::endl;
}

void test_interleaving_logic() {
    std::cout << "Testing channel interleaving simulation..." << std::endl;
    ScopePacket pkt;
    int sample_idx = 0;

    // Simulate filling the packet
    for(int i=0; i<SAMPLES_PER_PACKET; i++) {
        for(int ch=0; ch<ADC_CHANNELS_COUNT; ch++) {
            pkt.data[i * ADC_CHANNELS_COUNT + ch] = (i << 4) | ch;
        }
    }

    // Verify
    for(int i=0; i<SAMPLES_PER_PACKET; i++) {
        for(int ch=0; ch<ADC_CHANNELS_COUNT; ch++) {
            uint16_t val = pkt.data[i * ADC_CHANNELS_COUNT + ch];
            assert((val & 0xF) == ch);
            assert((val >> 4) == i);
        }
    }
    std::cout << "Channel interleaving test passed." << std::endl;
}

int main() {
    test_packet_structure();
    test_interleaving_logic();
    return 0;
}
