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
    ThresholdValues m_Thresh;
    std::shared_ptr<IEnergyMonitorMediatorDataProvider> m_DataProvider;
};
