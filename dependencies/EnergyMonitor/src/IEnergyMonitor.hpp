#pragma once
#include <memory>
#include "EnergyMonitorConfig.hpp"
#include "EnergyMonitorMediator/EnergyMonitorMediator.hpp"
#include "utils/EnergyData.hpp"
class IEnergyMonitor
{
public:
    virtual void LoadThresholdValues(const std::unordered_map<Components, ThresholdValues> &mission_thresholds) = 0;
    virtual void LoadMissions(const std::vector<MissionsAvailable> &missions_to_load) = 0;
    virtual Missions &GetMissions() = 0;
    virtual bool SetBatteryEstimation(float estimation) = 0;
    virtual bool SetTimeToHome(float estimation) = 0;
    ~IEnergyMonitor(){};
};
