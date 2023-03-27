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
            std::this_thread::sleep_for(std::chrono::milliseconds(interval));
            if(!active.load()) return;
            function();
        }
    });
}

void Timer::stop() {
    if (active) {
        active = false;
        if (t.joinable() && std::this_thread::get_id() != t.get_id()) {
          t.join();
        }
    }
}
