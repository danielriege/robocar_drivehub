#ifndef TIMER_HPP
#define TIMER_HPP

#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <functional>
#include <mutex>
#include <condition_variable>

class Timer {
    private:
        std::atomic<bool> active{true};
        std::thread t;
        std::mutex m;
        std::condition_variable cv;
	
    public:
        void setTimeout(std::function<void(void)> function, int delay);
        void setInterval(std::function<void(void)> function, int interval);
        void stop();

};

#endif