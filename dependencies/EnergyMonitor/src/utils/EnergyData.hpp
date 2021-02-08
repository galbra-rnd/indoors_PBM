#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include "Missions.hpp"
#include "Components.hpp"

#define THRESH_INIT_VALUE -1
enum DataMonitoringType
{
    BatteryTimeLeft = 0,
    TimeToHome,
};

/**
 * @brief Contains thresh values per color.
 * This needs to be set via LoadThresholdValues, via IEnergyMonitorMediator
 * 
 */
struct ThresholdValues
{
    float red = THRESH_INIT_VALUE, orange = THRESH_INIT_VALUE, green = THRESH_INIT_VALUE;

    friend std::ostream &operator<<(std::ostream &o, const ThresholdValues &thresh)
    {
        return o << "red: " << thresh.red << "\torange: " << thresh.orange << "\tgreen: " << thresh.green << std::endl;
    }
};

struct MissionPossible
{
    MissionsAvailable mission;
    MISSION state{MISSION::GREEN};
};

/**
 * @brief Missions contain MissionPossible, a summary about each mission
 * Each mission can receive MISSION status.
 * 
 */
struct Missions
{
    std::vector<MissionPossible> missions;
    std::unordered_map<Components, ThresholdValues> mission_thresholds;
};
