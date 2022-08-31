#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#undef min
#undef max
#include <array>

template <size_t SIZE_ = 256>
class RingBuffer
{
public:
    std::array<unsigned char, SIZE_> buffer;
    int writePos;
    int readPos;

public:
    /**
     * @brief Constructor
     **/
    RingBuffer() : writePos(0), readPos(0) {}

    /**
     * @brief retrieves the object on position pos in the RingBuffer
     * @param pos - position in buffer relative to readPosition
     * @return char - the char at the pos
     **/
    unsigned char operator[](int pos)
    {
        if(pos < available())
        {
            return buffer[(readPos + pos) % SIZE_];
        } else
        {
            return -1; // could be normal value
        }
    }

    /**
     * @brief pops items from the buffer
     * @param count - number of items to pop
     **/
    void pop(int count = 1)
    {
        if(count > available())
        {
            readPos = writePos;
        } else
        {
            readPos = (readPos + count) % SIZE_;
        }
    }

    /**
     * @brief pushes one item to buffer
     * @param c - next item
     **/
    void push(char c)
    {
        buffer[writePos] = c;
        writePos = (writePos + 1) % SIZE_;
    }

    /**
     * @brief checks if buffer has at least one item
     * @return number of items in buffer
     **/
    int available() { return (SIZE_ + writePos - readPos) % SIZE_; }
};

#endif
