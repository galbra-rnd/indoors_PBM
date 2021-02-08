#include "IsBatteryOkConditionNode.hpp"

IsBatteryOk::IsBatteryOk(const std::string &name, std::shared_ptr<IEnergyMonitorMediatorDataProvider> DataProvider)
    : BT::ConditionNode(name, {}),
      m_DataProvider(DataProvider)
{
    m_Thresh = m_DataProvider->GetComponentThresholds(Components::BATTERY_TIME);
    std::cout << "Initiating: " << this->name() << ". Loaded threshes of component "
              << component_to_string(Components::BATTERY_TIME) << " are: " << m_Thresh << "\n";
}

// You must override the virtual function tick()
BT::NodeStatus IsBatteryOk::tick()
{
    if (m_Thresh.red == THRESH_INIT_VALUE)
    {
        std::cerr << this->name() << " Thresh values are not set. Cant estimate!!!!!!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto est = m_DataProvider->GetTimeEstimation(DataMonitoringType::BatteryTimeLeft); /** @brief Get new estimation  */
    if (est < m_Thresh.red)
    {
        std::cerr << this->name() << " Battery is dead!!!!!!" << std::endl;
        m_DataProvider->SetMissionAvailability(MissionsAvailable::FLY, MISSION::RED);
        return BT::NodeStatus::FAILURE;
    }
    else if (est < m_Thresh.orange)
    {
        std::cout << this->name() << " Warning battery depleting" << std::endl;
        m_DataProvider->SetMissionAvailability(MissionsAvailable::FLY, MISSION::ORANGE);
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        std::cout << "IsBatteryOk: " << this->name() << " Battery is ok" << std::endl;
        m_DataProvider->SetMissionAvailability(MissionsAvailable::FLY, MISSION::GREEN);
        return BT::NodeStatus::SUCCESS;
    }
}
