#include "IsGoHomeOkConditionNode.hpp"

IsGoHomeOkConditionNode::IsGoHomeOkConditionNode(const std::string &name, std::shared_ptr<IEnergyMonitorMediatorDataProvider> DataProvider)
    : BT::ConditionNode(name, {}),
      m_DataProvider(DataProvider)
{
}

bool IsGoHomeOkConditionNode::setNewMissionAvailability(MISSION mission_status)
{
    auto is_succeed = m_DataProvider->SetMissionAvailability(m_ThisMission, mission_status);
    if (!is_succeed)
    {
        std::cerr << this->name() << " Could not set mission availability: " << mission_state_to_string(mission_status) << "\n";
        return is_succeed;
    }

    return is_succeed;
}

// You must override the virtual function tick()
BT::NodeStatus IsGoHomeOkConditionNode::tick()
{
    /**
     * @brief Thresholds might change in run time.
     * 
     */
    m_Thresh = m_DataProvider->GetComponentThresholds(Components::TIME_TO_HOME);

    if (m_Thresh.red == THRESH_INIT_VALUE)
    {
        std::cerr << this->name() << " Thresh values are not set. Cant estimate!!!!!!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    /** @brief Get new estimation  */
    auto tth_est = m_DataProvider->GetTimeEstimation(DataMonitoringType::TimeToHome);
    auto bat_left_est = m_DataProvider->GetTimeEstimation(DataMonitoringType::BatteryTimeLeft);

    if (bat_left_est == 0)
    {
        std::cerr << this->name() << " Battery time left received from estimator is: 0. Cant estimate!!!!!!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto est = 1 - (tth_est / (bat_left_est));
    // std::cerr << this->name() << " Current est: " << est << "\n";
    m_DataProvider->GetLogger()->info("Current estimation: {}",est);
    if (est <= m_Thresh.red)
    {
        m_DataProvider->GetLogger()->warn("{} Warning! mission: {} cant be completed.\n", this->name(), mission_to_string(m_ThisMission));
        if (!setNewMissionAvailability(MISSION::RED))
        {
            m_DataProvider->GetLogger()->warn("Could not set New Mission Availability. Is loaded policy configured behavior tree to support this: {}", this->name());
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::FAILURE;
    }
    else if (est <= m_Thresh.orange)
    {
        // std::cout << this->name() << " Warning! mission " << mission_to_string(m_ThisMission) << " is jeopardised.\n";
        m_DataProvider->GetLogger()->warn("{} Warning! mission: {} is jeopardised.\n", this->name(), mission_to_string(m_ThisMission));
        if (!setNewMissionAvailability(MISSION::ORANGE))
        {
            m_DataProvider->GetLogger()->warn("Could not set New Mission Availability. Is loaded policy configured behavior tree to support this: {}", this->name());
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        // std::cout << this->name() << " Mission " << mission_to_string(m_ThisMission) << " is Ok.\n";
        m_DataProvider->GetLogger()->info("{} Mission: {} is Ok.\n", this->name(), mission_to_string(m_ThisMission));
        if (!setNewMissionAvailability(MISSION::GREEN))
        {
            m_DataProvider->GetLogger()->warn("Could not set New Mission Availability. Is loaded policy configured behavior tree to support this: {}", this->name());
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
}
