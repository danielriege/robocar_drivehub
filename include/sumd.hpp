#include <cstdint>
#include "RingBuffer.hpp"
#include "serial.hpp"

#define MAX_CHAN_COUNT (16)
#define SUMD_PAKET_MINSIZE (3 + 24 + 2)
#define MAN_ID (0xA8)
#define STATE_NORMAL (0x01)
#define STATE_FS (0x81)

struct SumD_Paket
{
    uint8_t manufactureId;
    uint8_t state;
    uint8_t numChannels;
    uint16_t channel[MAX_CHAN_COUNT] = {12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000};
    uint16_t crc;
};

class SumD_Receiver
{
public:
    MyRingBuffer<64> buffer_;
    uint64_t timeLastPaket;
    uint32_t recvNumChannels = 0;
    int recvcount = 0;

    SumD_Paket paket;
    SumD_Receiver();
    void start();
    void process();
    bool timeout();
    int analyzePaket();
    void init();
    uint64_t micros();
    
    inline float getPercent(int chan) { return (float)(paket.channel[chan] - 12000) / 3200; }
    inline uint16_t getRaw(int chan) { return paket.channel[chan]; }
    inline uint16_t getPPM(int chan) { return paket.channel[chan] >> 3; }

private:
    static int on_uart_recv(void *);
};
