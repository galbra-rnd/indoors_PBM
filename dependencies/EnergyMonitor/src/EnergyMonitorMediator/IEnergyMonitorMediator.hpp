#pragma once
#include "iostream"
#include "utils/EnergyData.hpp"

/**
 * @brief Interface exposing relevant information for ROS node.
 * 
 */
class IEnergyMonitorMediator
{
public:
    virtual void LoadThresholdValues(const std::unordered_map<Components, ThresholdValues> &mission_thresholds) = 0;
    virtual void LoadMissions(const std::vector<MissionsAvailable> &missions_to_load) = 0;
    virtual Missions &GetMissionsData() = 0;
    virtual bool SetBatteryEstimation(float estimation) = 0;
    virtual bool SetTimeToHomeEstimation(float estimation) = 0;
    virtual ~IEnergyMonitorMediator(){};
};