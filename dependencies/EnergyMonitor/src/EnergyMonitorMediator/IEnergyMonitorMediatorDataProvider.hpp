#pragma once
#include "iostream"
#include "utils/EnergyData.hpp"

/**
 * @brief Interface exposing relevant information for Behavior-tree nodes.
 * 
 */
class IEnergyMonitorMediatorDataProvider
{
public:
    virtual float GetTimeEstimation(DataMonitoringType data_type) = 0;
    virtual void SetMissionAvailability(MissionsAvailable mission, MISSION status) = 0;
    virtual ~IEnergyMonitorMediatorDataProvider(){};
};