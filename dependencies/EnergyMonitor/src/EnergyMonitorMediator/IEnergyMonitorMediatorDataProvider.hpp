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
    virtual const ThresholdValues &GetComponentThresholds(const Components component) = 0;
    virtual float GetTimeEstimation(DataMonitoringType data_type) = 0;
    virtual bool SetMissionAvailability(MissionsAvailable mission, MISSION status) = 0;
    virtual ~IEnergyMonitorMediatorDataProvider(){};
};