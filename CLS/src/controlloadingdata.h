#ifndef CONTROLLOADINGDATA_H
#define CONTROLLOADINGDATA_H

#include <cstdint>
#include <array>

// Maximum number of control axes
constexpr size_t MAX_CONTROL_AXES = 4;

struct ControlLoadingData
{
    uint8_t axis;
    float position;
    float velocity;
    float force;

    ControlLoadingData()
        : axis(0), position(0.0f), velocity(0.0f), force(0.0f) {}
};

// Global buffer for control loading data, accessible by both MarshConnection and CLSInterface
// Indexed by axis number (0=ROLL, 1=PITCH, 2=THRUST, 3=YAW)
extern std::array<ControlLoadingData, MAX_CONTROL_AXES> g_controlLoadingData;

#endif // CONTROLLOADINGDATA_H
