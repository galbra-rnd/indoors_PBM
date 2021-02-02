#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include "Missions.hpp"

enum DataMonitoringType
{
    BatteryTimeLeft = 0,
    TimeToHome,
};

/**
 * @brief EnergyData contains data used for monitoring on mission availableness
 * 
 */
// struct EnergyData
// {
    
// };

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
};
