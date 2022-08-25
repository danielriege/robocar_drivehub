#include "serial.hpp"
#include "sumd.hpp"

int main(int argc, char** argv) {
    SumD_Receiver receiver;
    Serial serial("/dev/cu.usbserial-0001", 115200);
    
    serial.startAsync([&](uint8_t* data, size_t size) {
        for (int i = 0; i < size; i++) {
            receiver.buffer_.push(data[i]);
        }
        if (receiver.buffer_.available() >= 29) {
            if (receiver.analyzePaket() > 0) {

                float speed = receiver.getPercent(1);
                float steer = receiver.getPercent(2);
                printf("speed: %5.2f steer: %5.2f\n", speed, steer);
            }
        }
    });
    
    printf("test\n");
    
//    while (1) {
//        while (serial.read() != 0xa8) {}
//        receiver.buffer_.push(0xa8);
//        for (int i = 0; i < 28; i++) {
//            receiver.buffer_.push(serial.read());
//        }
//        receiver.process();
//        float speed = receiver.getPercent(2);
//        float steer = receiver.getPercent(3);
//        printf("speed: %5.2f steer: %5.2f\n", speed, steer);
//    }
    
    while (1) {
       
//        while(serial.available()) receiver.buffer_.push(serial.read());
//        receiver.process();
//
//        float speed = receiver.getPercent(2);
//        float steer = receiver.getPercent(3);
//        printf("speed: %5.2f steer: %5.2f\n", speed, steer);
    }
}
