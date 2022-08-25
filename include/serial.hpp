#pragma once

#include <vector>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#define BUF_LEN 64

class Serial
{
public:
    Serial(const std::string& port, const unsigned int baud_rate)
    : io(), serial(io,port)
    {
        using namespace boost::asio;
        serial.set_option(serial_port_base::baud_rate(baud_rate));
    }

    uint8_t read()
    {
        using namespace boost;
        uint8_t c;
        asio::read(serial,asio::buffer(&c,1));
        return c;
    }

    std::vector<uint8_t> read(const size_t size){
        using namespace boost;
        std::vector<uint8_t> buf(size);
        asio::read(serial, asio::buffer(buf.data(),size));
        return buf;
    }

    template<typename T>
    void readInto(T& obj){
        boost::asio::read(serial, boost::asio::buffer(&obj, sizeof(T)));
    }
    
    size_t available() {
        int value = 0;
        if (::ioctl(serial.lowest_layer().native_handle(), FIONREAD, &value) == 0) {
            boost::system::error_code error = boost::system::error_code(errno, boost::asio::error::get_system_category());
            boost::throw_exception((boost::system::system_error(error)));
        }
        printf("%d\n", value);
        return static_cast<size_t>(value);
    }
    
    void handleRecieve(const boost::system::error_code& error, size_t bytes_transferred) {
        if (bytes_transferred > 0) {
            callback_(buf, bytes_transferred);
        }
//        boost::asio::async_read(serial, boost::asio::buffer(buf,BUF_LEN), boost::bind(&Serial::handleRecieve,
//                                                                              this, boost::asio::placeholders::error,
//                                                                              boost::asio::placeholders::bytes_transferred));
        serial.async_read_some(boost::asio::buffer(buf,BUF_LEN), boost::bind(&Serial::handleRecieve,
                                                                       this, boost::asio::placeholders::error,
                                                                       boost::asio::placeholders::bytes_transferred));
    }
    
    void startAsync(std::function<void(uint8_t* data, size_t size)> callback) {
        callback_ = callback;
        boost::system::error_code error;
//        boost::asio::async_read(serial, boost::asio::buffer(buf,BUF_LEN), boost::bind(&Serial::handleRecieve,
//                                                                              this, boost::asio::placeholders::error,
//                                                                              boost::asio::placeholders::bytes_transferred));
        serial.async_read_some(boost::asio::buffer(buf,BUF_LEN), boost::bind(&Serial::handleRecieve,
                                                                       this, boost::asio::placeholders::error,
                                                                       boost::asio::placeholders::bytes_transferred));
        boost::thread t(boost::bind(&boost::asio::io_service::run, &io));
        //io.run();
    }
    
    void writeByte(uint8_t data) {
        
    }
    
    void writeBytes(uint8_t* data, int len) {
        
    }

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
    uint8_t buf[BUF_LEN];
    std::function<void(uint8_t* data, size_t size)> callback_;
};
