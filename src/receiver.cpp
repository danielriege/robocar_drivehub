#include "receiver.hpp"

//#define DEBUGGING
#ifdef DEBUGGING
#define DBG_PRINT(x...) printf(x)
#else
#define DBG_PRINT(x...) //
#endif

Receiver::Receiver(std::string dev, uint32_t baud) : packet{0}, ser(std::make_unique<Serial>(dev, baud)) {
}

/// starts the thread for async read on serial
void Receiver::start() {
    ser->startAsync(std::bind(&Receiver::uartReceive, this, std::placeholders::_1, std::placeholders::_2));
}

void Receiver::uartReceive(uint8_t* data, size_t size) {
    for (int i = 0; i < size; i++) {
        buffer_.push(data[i]);
    }
    if (analyzePacket() > 0) {
        packetReceivedCallback(this->packet);
    }
}

// sets callback so program can be notified on new packet
void Receiver::setPacketReceivedCallback(std::function<void(SumD_Packet data)> callback) {
    packetReceivedCallback = callback;
}

uint16_t Receiver::sumd_crc16(int len) {
    unsigned int i;
	unsigned short cksum = 0;
	for (i = 0; i < len; i++) {
		cksum = crc16_tab[(((cksum >> 8) ^ buffer_[i]) & 0xFF)] ^ (cksum << 8);
	}
	return cksum;
}

int Receiver::analyzePacket()
{
    int retCount = 0;
    while(buffer_.available() >= SUMD_PAKET_MINSIZE)
    {
        // check for paket type
        if(buffer_[0] != MAN_ID) {
            buffer_.pop(1); // throw away until first byte is right
            continue;
        }
        recvNumChannels = buffer_[2];
        if(recvNumChannels > MAX_CHAN_COUNT)
        {
            buffer_.pop(1);
            continue;
        }
        DBG_PRINT("NUM CHANNELS: d\n", recvNumChannels);
        int payloadSize = 3 + 2 * recvNumChannels;
        // check crc
        uint16_t crc = sumd_crc16(payloadSize + 2); // crc bytes included
        if(crc != 0)
        {
            DBG_PRINT("CRC failed");
            buffer_.pop(1);
            continue;
        }

        // copy paket
        uint8_t *ptr = (uint8_t *)&packet;
        ptr[0] = buffer_[0];
        ptr[1] = buffer_[1];
        ptr[2] = buffer_[2];
        for(int i = 3; i < payloadSize + 2; i += 2)
        {
            ptr[i] = buffer_[i];
            ptr[i + 1] = buffer_[i+1];
        }
        buffer_.pop(payloadSize + 2);
        retCount++;
        DBG_PRINT("Got Paket: %d", paket.state);
    }
    return retCount;
}
