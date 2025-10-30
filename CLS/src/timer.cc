#include "timer.h"
#include <chrono>

Timer::Timer() : running(false), singleShot(false) {}

Timer::~Timer() {
    stop();
}

void Timer::setCallback(std::function<void()> func) {
    callback = func;
}

void Timer::setInterval(int ms) {
    intervalMs = ms;
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

void Timer::start(int ms) {
    setInterval(ms);
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

bool Timer::isActive() const {
    return running;
}
