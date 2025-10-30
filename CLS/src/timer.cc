#include "timer.h"
#include <chrono>

Timer::Timer() : running(false), singleShot(false) {}

Timer::~Timer() {
    stop();
}

void Timer::setSingleShot(bool single) {
    singleShot = single;
}

void Timer::start() {
    if (running) {
        return;
    }

    running = true;
    timerThread = std::thread([this]() {
        auto nextTick = std::chrono::steady_clock::now();
        auto interval = std::chrono::milliseconds(intervalMs);

        do {
            nextTick += interval;
            std::this_thread::sleep_until(nextTick);

            if (running && callback) {
                callback();
            }
        } while (running && !singleShot);

        running = false;
    });
}

void Timer::start(int msInterval) {
    setmsInterval(msInterval);
    start();
}

void Timer::stop() {
    if (running) {
        running = false;
        if (timerThread.joinable()) {
            timerThread.join();
        }
    }
}
