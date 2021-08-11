#include "tests.h"
#include "dma.h"

void test_streams()
{
    const int8_t noStreams = 8;

    for (uint32_t i = 0; i < 0x10*noStreams; ++i) {
        REG(BA_MEM_DMA+i) = i;
    }

    DmaStream stream[noStreams];
    for (int i = 0; i < noStreams; ++i) {
        dma_init(&stream[i], i, 0x10000000*i, 0x10*i, dirMemToMem, 16);
    }
    for (int i = 0; i < noStreams; ++i) {
        dma_config(&stream[i]);
    }
}
