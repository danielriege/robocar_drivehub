#include "timer.hpp"

void Timer::setTimeout(std::function<void(void)> function, int delay) {
    stop();

    active = true;
    t = std::thread([=]() {
        if(!active.load()) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        if(!active.load()) return;
        function();
    });
}

void Timer::setInterval(std::function<void(void)> function, int interval) {
    stop();

    active = true;
    t = std::thread([=]() {
        while(active.load()) {
            //std::this_thread::sleep_for(std::chrono::milliseconds(interval));
            std::unique_lock<std::mutex> lock(m);
            if (cv.wait_for(lock, std::chrono::milliseconds(interval), [&]{return !active.load();})) {
                return;
            }
            function();
        }
    });
}

void Timer::stop() {
    if (active.load()) {
        {
        std::unique_lock<std::mutex> lock(m);
        active = false;
        cv.notify_all();
        }
        if (t.joinable() && std::this_thread::get_id() != t.get_id()) {
          t.join();
        }
    }
}
