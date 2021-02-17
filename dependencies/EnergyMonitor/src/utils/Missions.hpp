#pragma once
#include <string>

enum MISSION
{
    GREEN = 0,
    ORANGE,
    RED
};

static const std::string mission_state_to_string(MISSION state)
{
    switch (state)
    {
    case MISSION::GREEN:
        return "GREEN";
    case MISSION::ORANGE:
        return "ORANGE";
    case MISSION::RED:
        return "RED";
    default:
        return "NOT_IMPLEMENTED";
    }
};

enum MissionsAvailable
{
    NOT_IMPLEMENTED = 0,
    CAN_FLY,
    GO_HOME
};

/**
 * @brief Convert mission names to MissionsAvailable
 * 
 * @param loaded_mission - loaded from parameters.
 * @return const MissionsAvailable 
 */
static const MissionsAvailable string_to_available_mission(std::string loaded_mission)
{
    if (loaded_mission == "CAN_FLY")
        return MissionsAvailable::CAN_FLY;
    if (loaded_mission == "GO_HOME")
        return MissionsAvailable::GO_HOME;

    return MissionsAvailable::NOT_IMPLEMENTED;
}

static const std::string mission_to_string(MissionsAvailable mission)
{
    switch (mission)
    {
    case MissionsAvailable::CAN_FLY:
        return "CAN_FLY";
    case MissionsAvailable::GO_HOME:
        return "GO_HOME";
    default:
        return "NOT_IMPLEMENTED";
    }
}
