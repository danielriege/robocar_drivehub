#ifndef TIMER_HPP
#define TIMER_HPP

#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <functional>

class Timer {
	std::atomic<bool> active{true};
    std::thread t;
	
    public:
        void setTimeout(std::function<void(void)> function, int delay);
        void setInterval(std::function<void(void)> function, int interval);
        void stop();

};

#endif