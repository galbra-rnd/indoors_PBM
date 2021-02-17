#pragma once
#include "behaviortree_cpp_v3/condition_node.h"
#include "EnergyMonitorMediator/IEnergyMonitorMediatorDataProvider.hpp"
// Example of custom SyncActionNode (synchronous action)
// without ports.

/**
 * @brief ConditionNode for testing if BatteryEstimation is above threshold
 * 
 */
class IsBatteryOk : public BT::ConditionNode
{
public:
    IsBatteryOk(const std::string &name, std::shared_ptr<IEnergyMonitorMediatorDataProvider> DataProvider);

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

private:
    bool setNewMissionAvailability(MISSION mission_status);
    /**
     * @brief Thresholds might change if User change policy in runtime.
     * 
     */
    void load_thresh();
    MissionsAvailable m_ThisMission{MissionsAvailable::CAN_FLY};
    ThresholdValues m_Thresh;
    std::shared_ptr<IEnergyMonitorMediatorDataProvider> m_DataProvider;
};
