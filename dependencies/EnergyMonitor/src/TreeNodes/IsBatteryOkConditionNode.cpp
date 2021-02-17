#include "IsBatteryOkConditionNode.hpp"

IsBatteryOk::IsBatteryOk(const std::string &name, std::shared_ptr<IEnergyMonitorMediatorDataProvider> DataProvider)
    : BT::ConditionNode(name, {}),
      m_DataProvider(DataProvider)
{
    // m_Thresh = m_DataProvider->GetComponentThresholds(Components::BATTERY_TIME);
}

bool IsBatteryOk::setNewMissionAvailability(MISSION mission_status)
{
    auto is_succeed = m_DataProvider->SetMissionAvailability(m_ThisMission, mission_status);
    if (!is_succeed)
    {
        m_DataProvider->GetLogger()->warn("{} Could not set mission availability: {}", this->name(), mission_state_to_string(mission_status));
        // std::cerr << this->name() << " Could not set mission availability: " << mission_state_to_string(mission_status);
        return is_succeed;
    }

    return is_succeed;
}

void IsBatteryOk::load_thresh()
{
    m_Thresh = m_DataProvider->GetComponentThresholds(Components::BATTERY_TIME);
    
    std::stringstream ss;
    ss << this->name() << ". Loaded threshes of component " << component_to_string(Components::BATTERY_TIME) << " are: " << m_Thresh << "\n";
    m_DataProvider->GetLogger()->info(ss.str());
}

// You must override the virtual function tick()
BT::NodeStatus IsBatteryOk::tick()
{
    load_thresh();

    if (m_Thresh.red == THRESH_INIT_VALUE)
    {
        m_DataProvider->GetLogger()->warn(this->name() + " Thresh values are not set. Cant estimate!!!!!!");
        return BT::NodeStatus::FAILURE;
    }

    auto est = m_DataProvider->GetTimeEstimation(DataMonitoringType::BatteryTimeLeft); /** @brief Get new estimation  */
    
    if (est < m_Thresh.red)
    {
        m_DataProvider->GetLogger()->warn(this->name() + " Battery is dead!!!!!!");
        if (!setNewMissionAvailability(MISSION::RED))
        {
            m_DataProvider->GetLogger()->warn("Could not set New Mission Availability. Is loaded policy configured behavior tree to support this: {}", this->name());
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::FAILURE;
    }
    else if (est < m_Thresh.orange)
    {
        m_DataProvider->GetLogger()->warn("{} Warning battery depleting", this->name());
        // std::cout << this->name() << " Warning battery depleting" << std::endl;
        if (!setNewMissionAvailability(MISSION::ORANGE))
        {
            m_DataProvider->GetLogger()->warn("Could not set New Mission Availability. Is loaded policy configured behavior tree to support this: {}", this->name());
            // std::cerr << "Could not set set New Mission Availability. Is loaded policy configured behavior tree to support this: " << this->name() << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        // m_DataProvider->GetLogger()->info("IsBatteryOk: {} Battery is ok", this->name());
        std::cout << "IsBatteryOk: " << this->name() << " Battery is ok" << std::endl;
        if (!setNewMissionAvailability(MISSION::GREEN))
        {
            m_DataProvider->GetLogger()->warn("Could not set New Mission Availability. Is loaded policy configured behavior tree to support this: {}", this->name());
            // std::cerr << "Could not set set New Mission Availability. Is loaded policy configured behavior tree to support this: " << this->name() << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
}
