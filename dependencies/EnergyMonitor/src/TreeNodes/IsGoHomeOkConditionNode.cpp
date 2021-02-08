#include "IsGoHomeOkConditionNode.hpp"

IsGoHomeOkConditionNode::IsGoHomeOkConditionNode(const std::string &name, std::shared_ptr<IEnergyMonitorMediatorDataProvider> DataProvider)
    : BT::ConditionNode(name, {}),
      m_DataProvider(DataProvider)
{
    m_Thresh = m_DataProvider->GetComponentThresholds(Components::TIME_TO_HOME);
    std::cout << "Initiating: " << this->name() << ". Loaded threshes of component "
              << component_to_string(Components::TIME_TO_HOME) << " are: " << m_Thresh << "\n";
}

bool IsGoHomeOkConditionNode::setNewMissionAvailability(MISSION mission_status)
{
    auto is_succeed = m_DataProvider->SetMissionAvailability(m_ThisMission, mission_status);
    if (!is_succeed)
    {
        std::cerr << this->name() << " Could not set mission availability: " << mission_state_to_string(mission_status);
        return is_succeed;
    }

    return is_succeed;
}

// You must override the virtual function tick()
BT::NodeStatus IsGoHomeOkConditionNode::tick()
{
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
        std::cerr << this->name() << " Battery time left is 0. Cant estimate!!!!!!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto est = 1 - (tth_est / (bat_left_est));
    std::cerr << this->name() << " Current est: " << est << "\n";

    if (est <= m_Thresh.red)
    {
        std::cerr << this->name() << " Warning! mission " << mission_to_string(m_ThisMission) << " cant be completed.\n";
        setNewMissionAvailability(MISSION::RED);

        return BT::NodeStatus::FAILURE;
    }
    else if (est <= m_Thresh.orange)
    {
        std::cout << this->name() << " Warning! mission " << mission_to_string(m_ThisMission) << " is jeopardised.\n";
        setNewMissionAvailability(MISSION::ORANGE);
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        std::cout << this->name() << " Mission " << mission_to_string(m_ThisMission) << " is Ok.\n";
        setNewMissionAvailability(MISSION::GREEN);
        return BT::NodeStatus::SUCCESS;
    }
}
