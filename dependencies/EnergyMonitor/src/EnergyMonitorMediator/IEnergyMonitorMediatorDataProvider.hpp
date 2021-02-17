#pragma once
#include <memory>
#include "iostream"
#include "utils/EnergyData.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
/**
 * @brief Interface exposing relevant information for Behavior-tree nodes.
 * 
 */
class IEnergyMonitorMediatorDataProvider
{
public:
    virtual std::shared_ptr<spdlog::logger> GetLogger() = 0;
    virtual const ThresholdValues &GetComponentThresholds(const Components component) = 0;
    virtual float GetTimeEstimation(DataMonitoringType data_type) = 0;
    virtual bool SetMissionAvailability(MissionsAvailable mission, MISSION status) = 0;
    virtual ~IEnergyMonitorMediatorDataProvider(){};
};