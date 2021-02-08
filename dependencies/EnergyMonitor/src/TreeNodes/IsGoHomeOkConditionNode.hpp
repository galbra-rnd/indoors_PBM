#pragma once
#include "behaviortree_cpp_v3/condition_node.h"
#include "EnergyMonitorMediator/IEnergyMonitorMediatorDataProvider.hpp"

/**
 * @brief ConditionNode for testing if GoHome mission is RED/ORANGE/GREEN \n
 * * Assuming \f$BatteryLeft != 0\f$. \n
 * * \f$TTH\f$ is TimeToHome received from PathPlanner \n
 * GoHome possiple colors: \n
 * * RED    = 0.    It means that if \f$1 - \frac{TTH}{BatteryLeft} <= 0\f$, we cant go home. \n
 * * ORANGE = 0.5.  It means that if \f$1 - \frac{TTH}{BatteryLeft} <= 0.5\f$, we can go home. But battery is running out. \n
 * * GREEN  = 1.    It means that if \f$1 - \frac{TTH}{BatteryLeft} <= 1\f$, we can go home. \n
 */
class IsGoHomeOkConditionNode : public BT::ConditionNode
{
public:
    IsGoHomeOkConditionNode(const std::string &name, std::shared_ptr<IEnergyMonitorMediatorDataProvider> DataProvider);

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

private:
    bool setNewMissionAvailability(MISSION mission_status);
    MissionsAvailable m_ThisMission{MissionsAvailable::GO_HOME};
    ThresholdValues m_Thresh;
    std::shared_ptr<IEnergyMonitorMediatorDataProvider> m_DataProvider;
};
