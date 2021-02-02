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
    FLY = 0,
    GO_HOME
};

static const std::string mission_to_string(MissionsAvailable mission)
{
    switch (mission)
    {
    case MissionsAvailable::FLY:
        return "FLY";
    case MissionsAvailable::GO_HOME:
        return "GO_HOME";
    default:
        return "NOT_IMPLEMENTED";
    }
}
