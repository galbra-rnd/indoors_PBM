#include "IsBatteryOkConditionNode.hpp"

IsBatteryOk::IsBatteryOk(const std::string &name, std::shared_ptr<IEnergyMonitorMediatorDataProvider> DataProvider)
    : BT::ConditionNode(name, {}),
      m_DataProvider(DataProvider)
{
}

// You must override the virtual function tick()
BT::NodeStatus IsBatteryOk::tick()
{

    auto est = m_DataProvider->GetTimeEstimation(DataMonitoringType::BatteryTimeLeft); /** @brief Get new estimation  */
    if (est < MIN_BATTERY_ESTIMATION_THRESH)
    {
        std::cerr << this->name() << " Battery is dead!!!!!!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "IsBatteryOk: " << this->name() << " Battery is ok" << std::endl;
    return BT::NodeStatus::SUCCESS;
}
