#ifndef TIMER_H
#define TIMER_H

#include <functional>
#include <thread>
#include <atomic>

class Timer {
public:
    Timer();
    ~Timer();

    // Set the callback function
    void setCallback(std::function<void()> func) { callback = func; };
    // Set interval in milliseconds
    void setmsInterval(int ms) { intervalMs = ms; };

    // Set single shot mode (call once vs repeat)
    void setSingleShot(bool single);

    // Start the timer
    void start();

    // Start with specific interval
    void start(int msInterval);

    // Stop the timer
    void stop();

    // Check if timer is running
    bool isActive() const { return running; };

private:
    std::function<void()> callback;
    std::thread timerThread;
    std::atomic<bool> running;
    int intervalMs = 1000;
    bool singleShot;
};

#endif // TIMER_H
